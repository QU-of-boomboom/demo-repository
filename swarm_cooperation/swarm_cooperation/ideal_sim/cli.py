import argparse
import json
from pathlib import Path
import queue
import threading
import time

from .artifacts import write_artifacts
from .config import ExperimentConfig
from .simulation import IdealSimulation


def main(argv=None):
    defaults = ExperimentConfig()
    parser = argparse.ArgumentParser(description="五轮理想中继调度：随机移箱或首轮返程手动窗口")
    parser.add_argument("--mode", choices=("random", "window", "none"), default=defaults.mode)
    parser.add_argument("--seed", type=int, default=defaults.seed)
    parser.add_argument("--rounds", type=int, default=defaults.rounds)
    parser.add_argument("--cell-m", type=float, default=defaults.cell_m)
    parser.add_argument("--window-start", type=float, default=defaults.window_start_s)
    parser.add_argument("--window-end", type=float, default=defaults.window_end_s)
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--interactive", action="store_true", help="实时运行，终端输入remove 1,1 2,2")
    parser.add_argument("--command-file", type=Path, help="窗口内读取JSON {id,cells}；可在运行中编辑")
    parser.add_argument("--scripted-command", type=Path, help="可复现测试脚本JSON {at_return_s,id,cells}，非手动交互")
    parser.add_argument("--model-plugin", default="", help="Python模块:模型工厂")
    parser.add_argument("--predictor", default="", help="V2 fixed-batch ONNX path; requires deployment.json")
    parser.add_argument("--strategy", choices=("fixed", "reactive", "predeploy", "predictive"),
                        default=defaults.strategy)
    parser.add_argument("--map-observation", choices=("ideal", "static"), default=defaults.map_observation,
                        help="static uses only the preflight map; no automatic observation of removed boxes")
    parser.add_argument("--noise-db", type=float, default=0.)
    parser.add_argument("--drop-rate", type=float, default=0.)
    parser.add_argument("--delay-s", type=float, default=0.)
    parser.add_argument("--ack-drop-rate", type=float, default=0.)
    parser.add_argument("--export-telemetry", action="store_true")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    cfg = ExperimentConfig(rounds=args.rounds, mode=args.mode, seed=args.seed, cell_m=args.cell_m,
                           window_start_s=args.window_start, window_end_s=args.window_end,
                           model_plugin=args.model_plugin, predictor_path=args.predictor, strategy=args.strategy,
                           map_observation=args.map_observation, noise_sigma_db=args.noise_db,
                           measurement_drop_rate=args.drop_rate, measurement_delay_s=args.delay_s,
                           ack_drop_rate=args.ack_drop_rate, record_learning=args.export_telemetry)
    if args.predictor and args.strategy != "predictive":
        parser.error("--predictor requires --strategy predictive for an explicit AI comparison")
    if (args.interactive or args.command_file or args.scripted_command) and args.mode != "window":
        parser.error("手动/脚本命令入口只适用于 --mode window")
    if args.mode == "window" and not (args.interactive or args.command_file or args.scripted_command):
        parser.error("窗口模式请指定 --interactive、--command-file 或 --scripted-command")
    scripted = json.loads(args.scripted_command.read_text(encoding="utf-8")) if args.scripted_command else None
    if scripted and not cfg.window_start_s <= float(scripted["at_return_s"]) <= cfg.window_end_s:
        parser.error("脚本提交时刻必须在窗口内")

    def output(event):
        if not args.quiet:
            print(json.dumps(event, ensure_ascii=False), flush=True)

    sim = IdealSimulation(cfg, output=output)
    sim.run_context = {"entrypoint": "cli", "window_input": "scripted-test" if scripted else
                       "terminal" if args.interactive else "command-file" if args.command_file else "none"}
    commands = queue.Queue()
    if args.interactive:
        print("首轮返程窗口开放后输入：remove 1,1 2,2（货箱为1基坐标，一次2–4块）", flush=True)

        def reader():
            sequence = 0
            while True:
                try:
                    line = input().strip()
                except EOFError:
                    return
                try:
                    parts = line.split()
                    if not parts or parts[0] != "remove":
                        raise ValueError("格式：remove 1,1 2,2")
                    sequence += 1
                    commands.put({"id": f"terminal-{sequence}", "cells": [list(map(int, p.split(','))) for p in parts[1:]]})
                except ValueError as exc:
                    print(str(exc), flush=True)
        threading.Thread(target=reader, daemon=True).start()
    scripted_done, last_file = False, None
    while not sim.finished:
        start = time.monotonic()
        sim.step()
        if scripted and not scripted_done and sim.window_open and sim.return_elapsed + 1e-9 >= scripted["at_return_s"]:
            commands.put(scripted)
            scripted_done = True
        if args.command_file and sim.window_open and args.command_file.exists():
            raw = args.command_file.read_text(encoding="utf-8")
            if raw != last_file:
                try:
                    commands.put(json.loads(raw))
                    last_file = raw
                except json.JSONDecodeError:
                    pass  # 文件正在写入，下一步重读；未解析成功不执行。
        while not commands.empty():
            command = commands.get_nowait()
            if not isinstance(command, dict):
                print("命令必须为JSON对象", flush=True)
                continue
            result = sim.submit_removal(command.get("cells", []), command.get("id", "manual-1"))
            if not args.quiet:
                print(json.dumps(result, ensure_ascii=False), flush=True)
        if args.realtime or args.interactive or args.command_file:
            time.sleep(max(0., cfg.dt_s - (time.monotonic() - start)))
    directory = args.output or Path("outputs_v2") / f"{args.strategy}_{args.mode}_seed{args.seed}"
    out = write_artifacts(sim, directory)
    report = sim.report()
    if sim.predictor is not None:
        sim.predictor.close()
    print(f"完成 {report['completed_rounds']} 轮/{report['single_legs']} 单程；"
          f"主机 {report['main_distance_m']} m；全阶段断链 {report['all_phase_outage_s']} s；"
          f"移除任务 {'已执行' if report['change_executed'] else '未执行'}；结果：{out}", flush=True)
    if not report["success"]:
        print(f"任务未完成：{report['failure_reason']}", flush=True)
        return 3
    if args.mode == "window" and not report["change_executed"]:
        print("窗口内未接受移除任务；本次完成巡线，但不算移箱场景验收通过。", flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
