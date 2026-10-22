"""Reproduce A/B/C acceptance and save machine-readable evidence. No AI weights needed."""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parent


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs_v2/p0")
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("abc_acceptance", ROOT / "tests/test_abc_alignment.py")
    tests = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tests)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(tests))
    cfg = tests.ExperimentConfig(rounds=1, mode="none", noise_sigma_db=.5, measurement_delay_s=.1)
    summaries = {}
    for name, factory in (("A", tests.A.create_simulation), ("B", tests.IdealSimulation),
                          ("C", tests.C.create_simulation)):
        sim = factory(cfg)
        report = sim.run()
        summaries[name] = dict(success=report["success"], contract=report["contract"],
                               trace_sha256=digest(sim.samples), event_sha256=digest(sim.events),
                               sample_count=len(sim.samples), distance_m=report["main_distance_m"],
                               total_time_s=report["total_time_s"], phase_time_s=report["phase_time_s"])
    equal = len({s["trace_sha256"] for s in summaries.values()}) == 1
    source_paths = [ROOT / "swarm_cooperation/swarm_cooperation/ideal_sim" / name
                    for name in ("contract.py", "config.py", "world.py", "scheduler.py", "simulation.py")]
    source_paths += [ROOT / "swarm_cooperation/swarm_cooperation/relay_learning" / name
                     for name in ("features.py", "monitor.py", "routing.py")]
    report = dict(passed=result.wasSuccessful() and equal and all(s["success"] for s in summaries.values()),
                  tests_run=result.testsRun,
                  failures=len(result.failures), errors=len(result.errors), skipped=len(result.skipped),
                  scenario=asdict(cfg), modules=summaries, exact_trace_match=equal,
                  source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in source_paths})
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output / "acceptance.json"
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(target.resolve())
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
