"""Strict P0 exit gate: A/B/C regressions, six-hop diff, rule baselines, frozen-scope hashes.

All tests must run: a skip is a failure of this exit gate. No training or export.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / "swarm_cooperation/swarm_cooperation"),
                str(ROOT / "ai_relay_scheduler/ai_relay_scheduler")]
SUITES = {
    "A": ("warehouse_relay_sim/warehouse_relay_sim", "test_simulation.py"),
    "B": ("swarm_cooperation/swarm_cooperation", "tests"),
    "C": ("ai_relay_scheduler/ai_relay_scheduler", "test_core.py"),
    "A_legacy": ("warehouse_relay_sim/warehouse_relay_sim/legacy_v1", "test_simulation.py"),
    "B_legacy": ("swarm_cooperation/swarm_cooperation/legacy_v1", "test_legacy_snapshot.py"),
    "C_legacy": ("ai_relay_scheduler/ai_relay_scheduler/legacy_v1", "test_core.py"),
}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def worker(name, output):
    relative, target = SUITES[name]
    directory = ROOT / relative
    os.chdir(directory)
    sys.path.insert(0, str(directory))
    suite = (unittest.defaultTestLoader.discover(str(directory / "tests")) if target == "tests"
             else unittest.defaultTestLoader.loadTestsFromName(Path(target).stem))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(tests=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  skipped=len(result.skipped), passed=result.wasSuccessful() and not result.skipped,
                  failure_details=[(str(test), detail) for test, detail in result.failures + result.errors],
                  skipped_details=[(str(test), detail) for test, detail in result.skipped])
    write_json(output / f"tests_{name}.json", report)
    return 0 if report["passed"] else 1


def profile_checks():
    from ideal_sim.config import ExperimentConfig
    from ideal_sim.contract import CELL_M, DT_S, OUTWARD, PROFILE_SHA256, VERSION
    from ideal_sim.profile import PROFILE, PROFILE_PATH

    cfg = ExperimentConfig()
    profile = PROFILE
    expected = dict(
        rounds=int(profile["mission"]["rounds"]),
        cell_m=float(profile["map"]["cell_m"]),
        dt_s=float(profile["timing"]["dt_s"]),
        main_speed_mps=float(profile["timing"]["main_speed_mps"]),
        relay_speed_mps=float(profile["timing"]["relay_speed_mps"]),
        terminal_dwell_s=float(profile["timing"]["terminal_dwell_s"]),
        home_dwell_s=float(profile["timing"]["home_dwell_s"]),
        service_dbm=float(profile["thresholds"]["service_dbm"]),
        disconnect_dbm=float(profile["thresholds"]["disconnect_dbm"]),
        hysteresis_db=float(profile["thresholds"]["hysteresis_db"]),
        p0_dbm=float(profile["radio"]["p0_dbm"]),
        path_loss_n=float(profile["radio"]["path_loss_n"]),
        shelf_loss_db=float(profile["radio"]["shelf_loss_db"]),
        takeoff_s=float(profile["timing"]["takeoff_s"]),
        settle_s=float(profile["timing"]["settle_s"]),
        probe_s=float(profile["timing"]["probe_s"]),
        route_ack_s=float(profile["timing"]["route_ack_s"]),
        route_timeout_s=float(profile["timing"]["route_timeout_s"]),
        measurement_max_age_s=float(profile["timing"]["measurement_max_age_s"]),
        failsafe_hold_s=float(profile["timing"]["failsafe_hold_s"]),
        initial_stations=tuple(tuple(float(v) for v in p) for p in profile["stations"]["initial"]),
    )
    actual = {key: getattr(cfg, key) for key in expected}
    return dict(
        loaded=PROFILE_PATH.name == "mission_profile.json",
        config_matches=actual == expected,
        contract_matches=(VERSION == profile["contract_version"] and CELL_M == cfg.cell_m
                          and DT_S == cfg.dt_s and OUTWARD == tuple(
                              tuple(float(v) for v in p) for p in profile["map"]["route"])),
        gain_retired=float(profile["radio"]["gain_db"]) == 0.0,
        profile_sha256=PROFILE_SHA256,
        profile_path=str(PROFILE_PATH),
    )


def baseline_checks(report, profile):
    required = {"outage_count", "outage_s", "total_switches",
                "relay_occupancy_ratio", "deployment_success_rate"}
    rows = report.get("rows", [])
    expected_rows = len(profile["baselines"]["seeds"]) * 2
    return dict(
        five_round_runs=report.get("rounds_per_run") == profile["baselines"]["rounds"] == 5,
        row_count=len(rows) == expected_rows,
        all_completed=all(row.get("success") and row.get("completed_rounds") == 5 for row in rows),
        metrics_present=bool(rows) and all(required <= set(row) for row in rows),
        both_strategies={row.get("strategy") for row in rows} == {"reactive", "predeploy"},
        contract=report.get("simulation_contract_version") == profile["contract_version"],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs_v2/p0")
    parser.add_argument("--suite", choices=SUITES)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.suite:
        return worker(args.suite, output)
    # The canonical lock ships with the repository; custom output folders start empty.
    lock_path = ROOT / "outputs_v2/p0/scope_lock_before.json"
    baseline_lock = json.loads(lock_path.read_text(encoding="utf-8"))
    if output != lock_path.parent:
        write_json(output / "scope_lock_before.json", baseline_lock)
    temporary = ROOT / ".cache/abc-test-tmp"
    temporary.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TMP=str(temporary), TEMP=str(temporary), PYTHONUTF8="1")

    def run_suite(name):
        with (output / f"tests_{name}.log").open("w", encoding="utf-8") as log:
            completed = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--suite", name,
                                        "--output", str(output)], cwd=ROOT, env=env,
                                       stdout=log, stderr=subprocess.STDOUT)
        result_path = output / f"tests_{name}.json"
        result = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else {"passed": False}
        result["returncode"] = completed.returncode
        result["passed"] = result["passed"] and completed.returncode == 0
        print(f"{name}: {result}", flush=True)
        return name, result

    with ThreadPoolExecutor(max_workers=3) as pool:
        suites = dict(pool.map(run_suite, SUITES))
    from diff_abc import run as diff_run
    from ideal_sim.compare_baselines import compare
    from ideal_sim.config import ExperimentConfig
    from ideal_sim.profile import PROFILE
    from relay_learning.features import FeatureSpec
    diff = diff_run(output / "diff")
    baseline_cfg = PROFILE["baselines"]
    baselines = compare([ExperimentConfig(rounds=int(baseline_cfg["rounds"]), seed=s,
                                          noise_sigma_db=float(baseline_cfg["noise_sigma_db"]),
                                          measurement_delay_s=float(baseline_cfg["measurement_delay_s"]),
                                          measurement_drop_rate=float(baseline_cfg["measurement_drop_rate"]),
                                          map_observation=baseline_cfg["map_observation"])
                         for s in baseline_cfg["seeds"]], output / "baselines")
    # Archived lock keys use Windows separators; resolve them on Linux as well.
    after = {name: hashlib.sha256((ROOT / name.replace("\\", "/")).read_bytes()).hexdigest()
             for name in baseline_lock["protected_sha256"]}
    feature_manifest = json.loads(json.dumps(FeatureSpec().manifest()))
    scope = dict(feature_manifest_unchanged=feature_manifest == baseline_lock["feature_manifest"],
                 protected_files_unchanged=after == baseline_lock["protected_sha256"],
                 protected_sha256=after)
    write_json(output / "scope_lock_after.json", scope)
    profile = profile_checks()
    baseline = baseline_checks(baselines, PROFILE)
    readme = (ROOT / "README-无人机中继巡检.md").read_text(encoding="utf-8")
    checks = dict(trajectory_six_hop_diff=diff["passed"],
                  motion_eta_and_observation_tests=suites["A"]["passed"],
                  rule_baselines_archived=all(baseline.values()),
                  all_module_and_legacy_tests_no_skips=all(s["passed"] for s in suites.values()),
                  mission_profile_single_source=all(profile[key] for key in
                                                    ("loaded", "config_matches", "contract_matches", "gain_retired")),
                  readme_contract_and_gain_retirement="warehouse-abc-v3" in readme and "固定 +6dB 假设作废" in readme,
                  feature_96_and_weights_frozen=scope["feature_manifest_unchanged"] and scope["protected_files_unchanged"])
    report = dict(passed=all(checks.values()), checks=checks, tests=suites,
                  baseline_checks=baseline, profile=profile,
                  environment={name: importlib.metadata.version(name) for name in ("pymavlink", "lxml", "fastcrc")},
                  feature_count=FeatureSpec().input_size,
                  note="A includes the cross-module acceptance suite; archived A/C tests are compatibility checks only.")
    write_json(output / "dod_report.json", report)
    print(json.dumps({"passed": report["passed"], "checks": checks}, ensure_ascii=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
