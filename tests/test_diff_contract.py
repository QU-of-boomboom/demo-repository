"""Pytest-compatible P0 contract gate.

This module is intentionally a ``unittest`` case: pytest collects it without
extra dependencies and ``python check_dod.py`` runs it through the A suite.
It drives the standard 23-cell, 1 m/s outbound leg through A, B and C and
diffs every 0.1 s slot (position, crossed columns/cells, all six physical RSSI
hops).  Re-run it whenever the contract or mission profile changes.
"""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "swarm_cooperation/swarm_cooperation"),
                str(ROOT / "ai_relay_scheduler/ai_relay_scheduler")]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from diff_abc import run
from ideal_sim.config import ExperimentConfig
from ideal_sim.contract import LINKS, OUTWARD, PROFILE_SHA256, VERSION
from ideal_sim.profile import PROFILE, PROFILE_PATH


class ProfileContractTests(unittest.TestCase):
    def test_mission_profile_is_single_source_and_retires_gain(self):
        self.assertEqual(PROFILE_PATH.name, "mission_profile.json")
        self.assertEqual(VERSION, PROFILE["contract_version"])
        self.assertEqual(PROFILE["map"]["cell_m"], 1.0)
        self.assertEqual(PROFILE["timing"]["dt_s"], 0.1)
        self.assertEqual(PROFILE["radio"]["gain_db"], 0.0)
        self.assertEqual(len(PROFILE_SHA256), 64)
        self.assertEqual(OUTWARD, tuple(tuple(float(v) for v in p) for p in PROFILE["map"]["route"]))
        cfg = ExperimentConfig()
        self.assertEqual(cfg.cell_m, PROFILE["map"]["cell_m"])
        self.assertEqual(cfg.main_speed_mps, PROFILE["timing"]["main_speed_mps"])
        self.assertEqual(cfg.relay_speed_mps, PROFILE["timing"]["relay_speed_mps"])
        self.assertEqual(cfg.terminal_dwell_s, PROFILE["timing"]["terminal_dwell_s"])
        self.assertEqual(cfg.home_dwell_s, PROFILE["timing"]["home_dwell_s"])
        self.assertEqual(cfg.reactive_on_slots, PROFILE["thresholds"]["reactive_on_slots"])
        self.assertEqual(PROFILE["baselines"]["rounds"], 5)
        self.assertEqual(PROFILE["baselines"]["seeds"], [42, 43, 44, 45, 46])


class StandardOutboundDiff(unittest.TestCase):
    def test_standard_outbound_23_cells_matches_across_abc(self):
        cfg = ExperimentConfig(rounds=1, mode="none", strategy="fixed",
                               noise_sigma_db=0., measurement_delay_s=0.,
                               measurement_drop_rate=0., record_learning=True)
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "diff"
            report = run(out, cfg)
            self.assertTrue(report["passed"], report["first_mismatches"])
            self.assertEqual(report["mismatch_count"], {"B": 0, "C": 0})
            self.assertEqual(report["contract_version"], "warehouse-abc-v3")
            for name in ("A", "B", "C"):
                path = out / f"{name}_six_hops.csv"
                with path.open(encoding="utf-8-sig", newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertTrue(rows)
                self.assertEqual({row["link"] for row in rows}, set(LINKS))
                fly = [row for row in rows if row["phase"] == "FLY"
                       and row["direction"] == "outbound"]
                times = sorted({float(row["time_s"]) for row in fly})
                progress = sorted({float(row["progress_m"]) for row in fly})
                all_times = sorted({float(row["time_s"]) for row in rows})
                self.assertEqual(len(times), 230)
                self.assertEqual(len(progress), 230)
                self.assertGreater(times[0], 0.)
                self.assertAlmostEqual(progress[0], 0.1)
                self.assertAlmostEqual(progress[-1], 23.0)
                self.assertAlmostEqual(progress[-1] - progress[0], 22.9)
                self.assertTrue(all(round(b - a, 9) == 0.1
                                    for a, b in zip(progress, progress[1:])))
                self.assertTrue(all(round(b - a, 9) == 0.1
                                    for a, b in zip(all_times, all_times[1:])))
                self.assertTrue(all(float(row["distance_m"]) >= 0. for row in rows))
                # Each slot has exactly the six physical links.
                self.assertEqual(len(fly), 230 * len(LINKS))

    def test_standard_outbound_is_deterministic_not_noise_dependent(self):
        cfg = ExperimentConfig(rounds=1, mode="none", strategy="fixed")
        with tempfile.TemporaryDirectory() as directory:
            first = run(Path(directory) / "a", cfg)
            second = run(Path(directory) / "b", cfg)
            self.assertTrue(first["passed"])
            self.assertTrue(second["passed"])
            self.assertEqual(first["mismatch_count"], second["mismatch_count"])
            self.assertEqual(first["modules"]["A"]["samples"], second["modules"]["A"]["samples"])


if __name__ == "__main__":
    unittest.main()
