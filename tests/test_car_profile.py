"""Smoke test for the one-main-drone + two-ground-cars profile.

This is intentionally outside the P0 relay-drone gate; it checks the new
vehicle-kind variant end to end in a subprocess so the profile is loaded before
``ideal_sim`` imports.
"""
import ast
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
B = ROOT / "swarm_cooperation" / "swarm_cooperation"
PROFILE = ROOT / "mission_profile_1drone_2cars.json"


class GroundCarProfileTests(unittest.TestCase):
    def test_one_drone_two_cars_has_no_takeoff_or_settling(self):
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ, MISSION_PROFILE=str(PROFILE), PYTHONUTF8="1")
            completed = subprocess.run(
                [sys.executable, "demo.py", "--mode", "none", "--rounds", "1",
                 "--quiet", "--output", directory],
                cwd=B, env=env, capture_output=True, text=True, timeout=120)
            self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
            report = json.loads((Path(directory) / "report.json").read_text(encoding="utf-8"))
            self.assertTrue(report["success"])
            self.assertEqual(report["config"]["vehicle_kind"], "ground_car")
            self.assertEqual(report["config"]["takeoff_s"], 0.0)
            self.assertEqual(report["config"]["settle_s"], 0.0)
            self.assertEqual(report["main_distance_m"], 46.0)
            events = [json.loads(line) for line in
                      (Path(directory) / "events.jsonl").read_text(encoding="utf-8").splitlines()
                      if line.strip()]
            self.assertFalse(any(event["kind"] in ("takeoff", "settling") for event in events))
            trace_path = Path(directory) / "trace.csv"
            with trace_path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            statuses = [status for row in rows for status in ast.literal_eval(row["relay_status"])]
            self.assertIn("MOVING", statuses)
            self.assertIn("READY", statuses)
            self.assertFalse({"TAKEOFF", "SETTLING"}.intersection(statuses))


if __name__ == "__main__":
    unittest.main()
