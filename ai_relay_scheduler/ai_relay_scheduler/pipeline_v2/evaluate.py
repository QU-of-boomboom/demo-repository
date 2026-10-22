"""Pair all policies by scenario, recomputing links from their own moving positions."""
from dataclasses import replace
import json
import time

from ideal_sim.artifacts import write_artifacts
from ideal_sim.simulation import IdealSimulation
from ideal_sim.contract import VERSION
from .common import load_config, output_path, parser, write_json
from .dataset import simulation_config


def evaluate(c):
    out = output_path(c)
    rows = []
    model_path = out / "model_fp32_batch1.onnx"
    if not model_path.exists():
        raise FileNotFoundError("Export the v2 model before policy comparison")
    for index in range(c["evaluation_voyages"]):
        base = simulation_config(c, index, evaluation=True)
        for strategy in ("fixed", "reactive", "predeploy", "predictive"):
            cfg = replace(base, strategy=strategy,
                          predictor_path=str(model_path.resolve()) if strategy == "predictive" else "")
            started = time.perf_counter()
            sim = IdealSimulation(cfg)
            try:
                result = sim.run()
            finally:
                if sim.predictor is not None:
                    sim.predictor.close()
            row = {key: result[key] for key in (
                "success", "failure_reason", "completed_rounds", "main_distance_m",
                "relay_distance_m", "all_phase_outage_s", "longest_outage_s", "outage_events",
                "total_switches", "deployment_count", "deployment_successes", "total_time_s",
                "inference_calls", "inference_fallbacks", "planning_timing",
                "inference_cycle_timing", "relay_hop_s", "pingpong_count")}
            row.update(scenario=index, strategy=strategy, wall_seconds=time.perf_counter() - started)
            rows.append(row)
            if index == 0:
                write_artifacts(sim, out / "replays" / strategy)
            print(f"scenario={index} {strategy}: success={row['success']} outage={row['all_phase_outage_s']}s", flush=True)
    paired = []
    for index in range(c["evaluation_voyages"]):
        policies = {r["strategy"]: r for r in rows if r["scenario"] == index}
        paired.append(dict(scenario=index,
                           predictive_minus_reactive_outage_s=policies["predictive"]["all_phase_outage_s"]
                           - policies["reactive"]["all_phase_outage_s"],
                           predictive_minus_predeploy_outage_s=policies["predictive"]["all_phase_outage_s"]
                           - policies["predeploy"]["all_phase_outage_s"],
                           predictive_success=policies["predictive"]["success"],
                           reactive_success=policies["reactive"]["success"],
                           baseline_success=policies["predeploy"]["success"]))
    report = dict(stage="simulation comparison; not real flight or NPU validation",
                  simulation_contract_version=VERSION,
                  rule_baselines=["reactive", "predeploy"],
                  model_data_source=c["data_source"], evaluation_seed=c["evaluation_seed"],
                  disturbance_contract="same seed, radio parameters and mission-relative removal rule; "
                                      "links recomputed at policy-specific positions and times",
                  caution="Aborted runs are failures; shorter outage on an aborted run is not an improvement.",
                  rows=rows, paired=paired)
    write_json(out / "comparison_report.json", report)
    summary(c)
    return report


def summary(c):
    out = output_path(c)
    training = json.loads((out / "training_report.json").read_text(encoding="utf-8"))
    quant = json.loads((out / "quantization_report.json").read_text(encoding="utf-8"))
    comparison_path = out / "comparison_report.json"
    lines = ["# V2 模板运行记录", "", f"- 数据来源：`{c['data_source']}`。",
             "- 仿真数据仅验证训练、导出和集成流程，不代表真实无线预测效果。",
             "- 尚未运行 Icraft 或 NPU，不代表三机实飞完成。",
             f"- 模型参数：{training['parameter_count']}；输入 `[1,96]`，输出 `[1,2]`。",
             f"- FP32 导出最大绝对误差：{quant['export_max_abs_error']:.6g}。",
             f"- PC INT8 两个时域的召回下降（百分点）：{quant['recall_drop_pp']}。",
             "", "| 时域 | 测试召回 | 精确率 | 阈值 |", "|---|---:|---:|---:|"]
    for horizon, item, threshold in zip(c["feature_spec"]["horizons_s"], training["test"], training["thresholds"]):
        lines.append(f"| {horizon}s | {item['recall']:.4f} | {item['precision']:.4f} | {threshold:.3f} |")
    if comparison_path.exists():
        report = json.loads(comparison_path.read_text(encoding="utf-8"))
        lines += ["", "| 场景 | 策略 | 完成 | 断链/s | 移动/m | 切换 |",
                  "|---|---|---|---:|---:|---:|"]
        for row in report["rows"]:
            lines.append(f"| {row['scenario']} | {row['strategy']} | {row['success']} | "
                         f"{row['all_phase_outage_s']} | {row['relay_distance_m']} | {row['total_switches']} |")
        lines += ["", "未完成的任务必须作为失败报告，不能因为提前停止后累计断链更少就宣称改善。",
                  "地图输入为配置指定的 ideal/static；静态模式不会从 RSSI 推断哪一块货箱移除。"]
    (out / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    evaluate(load_config(parser(__doc__).parse_args().config))


def report_main():
    summary(load_config(parser("Regenerate the v2 results document").parse_args().config))
