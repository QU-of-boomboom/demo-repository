"""运行 python -m unittest -v；无需图形界面或硬件。"""

import math
import unittest

from warehouse_relay_sim import (
    BASE, Config, build_route, crossed_shelf_columns, hop_rssi,
    link_candidates, make_pickups, segment_enters_cell, select_link, simulate,
)


class RadioGeometryTests(unittest.TestCase):
    def test_grid_lines_and_corner_touches_do_not_penetrate_shelves(self):
        for a, b in [((0, 0), (3, 0)), ((0, 0), (0, 4)), ((1, 0), (1, 4)), ((0, 2), (3, 2))]:
            with self.subTest(a=a, b=b):
                self.assertEqual(crossed_shelf_columns(a, b), 0)
        self.assertFalse(segment_enters_cell((0, 1), (1, 0), 2, 1))
        self.assertFalse(segment_enters_cell((0, 0), (1, 1), 1, 2))

    def test_deduplicates_columns_and_is_symmetric(self):
        for a, b, expected in [((0, 0), (1, 4), 1), ((0, 0), (2, 4), 2), ((0, 0), (3, 4), 3), ((1, 2), (3, 4), 2)]:
            with self.subTest(a=a, b=b):
                self.assertEqual(crossed_shelf_columns(a, b), expected)
                self.assertEqual(crossed_shelf_columns(b, a), expected)

    def test_reference_and_known_rssi(self):
        cfg = Config()
        self.assertEqual(hop_rssi(BASE, BASE, cfg), -40.0)
        self.assertEqual(hop_rssi(BASE, (0, 0.1), cfg), -40.0)
        # 独立手算值：地面站至中继1穿1列，中继2穿2列，终点穿3列。
        self.assertAlmostEqual(hop_rssi(BASE, (1, 2), cfg), -59.6886700477, places=8)
        self.assertAlmostEqual(hop_rssi(BASE, (2, 4), cfg), -78.3113299523, places=8)
        self.assertAlmostEqual(hop_rssi(BASE, (3, 4), cfg), -91.3773400954, places=8)
        values = link_candidates((3, 4), cfg)
        self.assertAlmostEqual(values["relay_1"], -67.9339898569, places=8)
        self.assertAlmostEqual(values["relay_2"], -72.3113299523, places=8)


class SchedulerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_trigger_is_strict_and_selects_best_relay(self):
        values = {"direct": -75.0, "relay_1": -69.0, "relay_2": -65.0}
        self.assertEqual(select_link("direct", values, self.cfg)[0], "direct")
        values["direct"] = -75.1
        self.assertEqual(select_link("direct", values, self.cfg)[0], "relay_2")

    def test_three_db_return_hysteresis_rejects_threshold_noise(self):
        mode = "relay_1"
        for direct in [-74.9, -75.1, -74.7, -73.0, -72.01]:
            mode = select_link(mode, {"direct": direct, "relay_1": -65, "relay_2": -69}, self.cfg)[0]
            self.assertEqual(mode, "relay_1")
        self.assertEqual(select_link(mode, {"direct": -72, "relay_1": -65, "relay_2": -69}, self.cfg)[0], "direct")

    def test_relay_to_relay_hysteresis(self):
        values = {"direct": -80, "relay_1": -70, "relay_2": -67.01}
        self.assertEqual(select_link("relay_1", values, self.cfg)[0], "relay_1")
        values["relay_2"] = -67
        self.assertEqual(select_link("relay_1", values, self.cfg)[0], "relay_2")
        values["relay_1"] = -66.9
        self.assertEqual(select_link("relay_2", values, self.cfg)[0], "relay_2")

    def test_emergency_uses_available_link_even_without_three_db_gain(self):
        values = {"direct": -88.1, "relay_1": -87, "relay_2": -89}
        self.assertEqual(select_link("direct", values, self.cfg)[0], "relay_1")


class MissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = simulate(Config())

    def test_geometry_and_accepted_duration(self):
        route = build_route("upper-right")
        self.assertEqual(route, [(0, 0), (0, 4), (1, 4), (1, 0), (2, 0), (2, 4), (3, 4), (3, 0), (3, 4)])
        self.assertEqual(sum(math.dist(a, b) for a, b in zip(route, route[1:])), 23)
        self.assertEqual(self.result["arrival_time_s"], 23)
        self.assertEqual(self.result["turnaround_time_s"], 24)
        self.assertEqual(self.result["total_time_s"], 47)
        self.assertEqual(self.result["distance_m"], 46)

    def test_every_sample_has_valid_motion_and_reverse_replay(self):
        rows = self.result["samples"]
        self.assertEqual(len(rows), 471)
        for i, row in enumerate(rows):
            self.assertAlmostEqual(row.time_s, i * 0.1)
            self.assertTrue(row.x_m.is_integer() or row.y_m.is_integer())
            self.assertGreaterEqual(row.rssi_dbm, -88)
            if i:
                distance = math.dist((row.x_m, row.y_m), (rows[i - 1].x_m, rows[i - 1].y_m))
                self.assertAlmostEqual(distance, 0 if 231 <= i <= 240 else 0.1)
        outbound = [(s.x_m, s.y_m) for s in rows[:231]]
        returning = [(s.x_m, s.y_m) for s in rows[240:]]
        self.assertEqual(outbound, returning[::-1])
        self.assertEqual(returning[-1], (0, 0))

    def test_switch_evidence_and_takeover_quality(self):
        switches = [e for e in self.result["events"] if e.kind == "switch"]
        self.assertGreaterEqual(len(switches), 4)
        self.assertEqual(len(switches), sum(a.mode != b.mode for a, b in zip(self.result["samples"], self.result["samples"][1:])))
        for event in switches:
            d = event.details
            candidates = link_candidates(tuple(d["position"]), Config())
            self.assertAlmostEqual(d["before_dbm"], candidates[d["old_mode"]])
            self.assertAlmostEqual(d["after_dbm"], candidates[d["new_mode"]])
            if d["new_mode"] != "direct":
                self.assertGreater(d["after_dbm"], -75)
                self.assertEqual(d["after_dbm"], max(candidates["relay_1"], candidates["relay_2"]))
        self.assertTrue(self.result["validation"]["all_acceptance_checks_pass"])

    def test_seeded_pickups_are_reproducible_unique_and_reachable(self):
        for seed in range(20):
            with self.subTest(seed=seed):
                targets = make_pickups(seed)
                self.assertEqual(targets, make_pickups(seed))
                self.assertIn(len(targets), (2, 3))
                self.assertEqual(len(set(targets)), len(targets))
                result = simulate(Config(seed=seed))
                self.assertEqual(sorted(targets), sorted(tuple(item["cell"]) for item in result["picked"]))

    def test_optional_39s_variant_has_explicit_different_endpoint(self):
        result = simulate(Config(endpoint="lower-right"))
        self.assertEqual(result["route"][-1], (3, 0))
        self.assertEqual(result["total_time_s"], 39)
        self.assertTrue(result["validation"]["all_acceptance_checks_pass"])


if __name__ == "__main__":
    unittest.main()
