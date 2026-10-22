"""Run the P0 gate and ground-car regression from a fresh checkout."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / ".cache/validation")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    temp = output / "tmp"
    temp.mkdir(exist_ok=True)
    env = dict(os.environ, PYTHONUTF8="1", TEMP=str(temp), TMP=str(temp))
    env.pop("MISSION_PROFILE", None)
    steps = [
        ("dialect", ["swarm_cooperation/swarm_cooperation/generate_dialect.py"]),
        ("p0", ["check_dod.py", "--output", str(output / "p0")]),
        ("ground_car", ["-m", "unittest", "discover", "-s", "tests", "-p", "test_car_profile.py", "-v"]),
    ]
    results = {}
    for name, command in steps:
        log_path = output / f"{name}.log"
        print(f"Running {name}; log: {log_path}", flush=True)
        with log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run([sys.executable, *command], cwd=ROOT, env=env,
                                    stdout=log, stderr=subprocess.STDOUT)
        results[name] = {"passed": result.returncode == 0, "returncode": result.returncode,
                         "log": log_path.name}
        print(f"{name}: {'PASS' if result.returncode == 0 else 'FAIL'}", flush=True)
        if result.returncode:
            print(log_path.read_text(encoding="utf-8")[-6000:], flush=True)
    report = {"passed": all(row["passed"] for row in results.values()), "steps": results,
              "python": sys.version.split()[0], "platform": sys.platform,
              "checked_at_utc": datetime.now(timezone.utc).isoformat()}
    p0_path = output / "p0/dod_report.json"
    if results["p0"]["passed"] and p0_path.is_file():
        p0 = json.loads(p0_path.read_text(encoding="utf-8"))
        report["p0_tests"] = sum(suite["tests"] for suite in p0["tests"].values())
        report["p0_skipped"] = sum(suite["skipped"] for suite in p0["tests"].values())
    (output / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
