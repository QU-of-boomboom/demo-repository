"""Paired rule baselines without requiring trained weights.

Run from the B directory:

    python -m ideal_sim.compare_baselines

The default archive is the repository-level ``outputs_v2/p0/baselines`` and
uses the five-round, five-seed settings in ``mission_profile.json``.
"""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from .config import ExperimentConfig
from .contract import VERSION, manifest
from .profile import PROFILE, PROFILE_SHA256
from .simulation import IdealSimulation
from .artifacts import write_artifacts

ROOT = Path(__file__).resolve().parents[3]
_BASELINES = PROFILE["baselines"]


def _ratio(numerator, denominator):
    return round(numerator / denominator, 6) if denominator else None


def compare(configs, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows, paired = [], []
    for index, base in enumerate(configs):
        reports = {}
        for strategy in ("reactive", "predeploy"):
            cfg = replace(base, strategy=strategy, predictor_path="", model_plugin="", record_learning=False)
            sim = IdealSimulation(cfg)
            result = sim.run()
            row = {key: result[key] for key in (
                "success", "failure_reason", "completed_rounds", "main_distance_m", "relay_distance_m",
                "all_phase_outage_s", "longest_outage_s", "outage_events", "total_switches",
                "total_time_s", "phase_time_s", "deployment_count", "deployment_successes",
                "relay_hop_s", "pingpong_count")}
            row.update(
                scenario=index,
                strategy=strategy,
                rounds=cfg.rounds,
                outage_count=result["outage_events"],
                outage_s=result["all_phase_outage_s"],
                relay_occupancy_ratio=_ratio(result["relay_hop_s"], result["total_time_s"]),
                deployment_success_rate=_ratio(result["deployment_successes"], result["deployment_count"]),
                config=asdict(cfg),
                contract=manifest(cfg),
            )
            rows.append(row)
            reports[strategy] = row
            if index == 0:
                write_artifacts(sim, output / strategy)
        paired.append(dict(scenario=index,
                           both_completed=all(r["success"] for r in reports.values()),
                           predeploy_minus_reactive_outage_s=reports["predeploy"]["all_phase_outage_s"]
                           - reports["reactive"]["all_phase_outage_s"],
                           predeploy_minus_reactive_outage_count=reports["predeploy"]["outage_events"]
                           - reports["reactive"]["outage_events"],
                           predeploy_minus_reactive_time_s=reports["predeploy"]["total_time_s"]
                           - reports["reactive"]["total_time_s"]))
    report = dict(simulation_contract_version=VERSION,
                  profile_version=PROFILE["schema_version"],
                  profile_sha256=PROFILE_SHA256,
                  rounds_per_run=configs[0].rounds if configs else _BASELINES["rounds"],
                  seeds=[cfg.seed for cfg in configs],
                  required_metrics=["outage_count", "outage_s", "total_switches",
                                    "relay_occupancy_ratio", "deployment_success_rate"],
                  rows=rows, paired=paired,
                  policies={"reactive": "current measured threshold; current-position planning only, every leg",
                            "predeploy": "known route and observed map; preflight, each leg and periodic planning"},
                  fairness="same scenario parameters, time/link keyed noise/drop and mission-relative map change; "
                           "same motion, route ACK and observation gates; no future true RSSI",
                  caution="Failures are failures even if accumulated outage is lower. No AI superiority claim.")
    (output / "comparison_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    lines = ["# 两条规则基线配对结果", "",
             f"契约：`{VERSION}`。未使用 AI 权重。每策略每场景跑 **{configs[0].rounds if configs else _BASELINES['rounds']} 个完整往返**。",
             "",
             "| 场景 | 策略 | 完成 | 轮数 | 主机/m | 中继/m | 断链次数 | 断链/s | 最长/s | 切换 | 中继占用率 | 到位成功率 | 总时长/s |",
             "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        occupancy = "—" if r["relay_occupancy_ratio"] is None else f"{r['relay_occupancy_ratio']:.1%}"
        success = "—" if r["deployment_success_rate"] is None else f"{r['deployment_success_rate']:.1%}"
        lines.append(f"| {r['scenario']} | {r['strategy']} | {r['success']} | {r['rounds']} | "
                     f"{r['main_distance_m']} | {r['relay_distance_m']} | {r['outage_events']} | "
                     f"{r['all_phase_outage_s']} | {r['longest_outage_s']} | {r['total_switches']} | "
                     f"{occupancy} | {success} | {r['total_time_s']} |")
    lines += ["", "中继占用率 = 中继转发跳秒 / 任务总时长；到位成功率 = 到位成功次数 / 部署次数。",
              "相同场景参数、扰动种子和执行约束；策略造成的实际位置与时间不同，链路按各自状态重新计算。",
              "未完成任务不能按累计断链较少解释成收益。与 AI 比较时须同时报告两条基线。"]
    (output / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=_BASELINES["seeds"])
    parser.add_argument("--rounds", type=int, default=_BASELINES["rounds"])
    parser.add_argument("--output", type=Path, default=ROOT / _BASELINES["archive"])
    args = parser.parse_args()
    compare([ExperimentConfig(rounds=args.rounds, seed=s,
                              noise_sigma_db=_BASELINES["noise_sigma_db"],
                              measurement_delay_s=_BASELINES["measurement_delay_s"],
                              measurement_drop_rate=_BASELINES["measurement_drop_rate"],
                              map_observation=_BASELINES["map_observation"]) for s in args.seeds],
            args.output)
    print(args.output.resolve() / "comparison_report.json")


if __name__ == "__main__":
    main()
