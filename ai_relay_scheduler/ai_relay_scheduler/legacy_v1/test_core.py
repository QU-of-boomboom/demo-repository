"""关键正确性检查：几何、标签时序、因果性、配对公平性与切换延迟。"""
import unittest

import runtime_compat
import numpy as np

from common import load_config, normalize, select_threshold
from compare import schedule
from simulator import build_route, crossed_shelf_columns, features, future_labels, hop_rssi, simulate_voyage


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.c = load_config()

    def test_nominal_route_and_endpoints(self):
        v = simulate_voyage(0, self.c, randomized=False)
        route = build_route(self.c)
        self.assertEqual(np.linalg.norm(np.diff(route, axis=0), axis=1).sum(), 23)
        np.testing.assert_allclose(v["xy"][[0, 230, -1]], [[0, 0], [3, 4], [0, 0]], atol=1e-8)
        np.testing.assert_allclose(v["xy"][:231], v["xy"][230:][::-1], atol=1e-8)
        self.assertEqual(len(v["xy"]), 461)
        self.assertAlmostEqual(v["time"][-1], 46)
        np.testing.assert_allclose(np.diff(v["time"]), .1)
        # 每个采样坐标至少位于一条棋盘格线上。
        on_grid = np.isclose(v["xy"], np.round(v["xy"]), atol=1e-8)
        self.assertTrue(np.all(on_grid.any(axis=1)))

    def test_shelf_geometry_and_reference_distance(self):
        points = np.array([[0, 4], [3, 0], [3, 4], [1, 2], [2, 4], [0, 0]])
        np.testing.assert_array_equal(crossed_shelf_columns([0, 0], points, self.c), [0, 0, 3, 1, 2, 0])
        self.assertEqual(crossed_shelf_columns([1, 0], [[1, 4]], self.c)[0], 0)
        self.assertEqual(hop_rssi([0, 0], [0, 0], 12, self.c)[0], -40)
        expected = -40 - 22 * np.log10(5) - 36
        self.assertAlmostEqual(hop_rssi([0, 0], [3, 4], 12, self.c)[0], expected)

    def test_independent_scalar_geometry_oracle(self):
        def oracle(a, b):
            total = 0
            for col in range(3):
                hit = False
                for row in range(4):
                    enter, leave = 0., 1.
                    for axis, lo in ((0, col), (1, row)):
                        delta = b[axis] - a[axis]
                        if abs(delta) < 1e-9:
                            if not lo + 1e-9 < a[axis] < lo + 1 - 1e-9:
                                leave = -1
                                break
                        else:
                            low, high = sorted(((lo - a[axis]) / delta, (lo + 1 - a[axis]) / delta))
                            enter, leave = max(enter, low), min(leave, high)
                    hit |= leave - enter > 1e-9
                total += hit
            return total
        rng = np.random.default_rng(31)
        a, b = rng.uniform([0, 0], [3, 4], (2, 100, 2))
        np.testing.assert_array_equal(crossed_shelf_columns(a, b, self.c), [oracle(p, q) for p, q in zip(a, b)])

    def test_future_labels_exclude_present_and_require_full_horizon(self):
        rssi = np.full(40, -60.)
        rssi[20] = -76
        y = future_labels(rssi, self.c)
        self.assertEqual(len(y), 25)
        np.testing.assert_array_equal(np.flatnonzero(y), np.arange(5, 20))
        rssi[20] = -75
        self.assertFalse(future_labels(rssi, self.c).any())

    def test_history_causality_and_padding(self):
        rssi = np.arange(50, dtype=float) - 90
        xy, speed = np.zeros((50, 2)), np.ones(50)
        x1 = features(rssi, xy, speed, self.c)
        rssi[31:] -= 50
        x2 = features(rssi, xy, speed, self.c)
        np.testing.assert_array_equal(x1[:31], x2[:31])
        np.testing.assert_array_equal(x1[0, :20], np.full(20, -90))
        np.testing.assert_array_equal(x1[20, :20], np.arange(1, 21) - 90)
        self.assertEqual(x1.shape, (50, 25))
        self.assertEqual(normalize(x1, self.c).dtype, np.float32)

    def test_seed_streams_and_physical_ranges(self):
        a = simulate_voyage(42, self.c)
        b = simulate_voyage(42, self.c)
        e = simulate_voyage(42, self.c, stream="evaluation")
        np.testing.assert_array_equal(a["links"], b["links"])
        self.assertFalse(np.array_equal(a["links"], e["links"]))
        self.assertTrue(8 <= a["shelf_loss_db"] <= 16 and 1 <= a["sigma_db"] <= 3)
        progress = a["progress"]
        interior = ~np.isclose(progress, 23) & ~np.isclose(progress, 46)
        self.assertTrue(np.all(a["speed"][interior] >= .7 - 1e-8))
        self.assertTrue(np.all(a["speed"] <= 1.3 + 1e-8))

    def test_anticipation_and_no_artificial_zero_delay_outage(self):
        links = np.tile([-65., -60., -80.], (30, 1))
        links[10:, 0] = -93
        p = np.zeros(30)
        p[5:] = 1
        rule, _ = schedule(links, p, .5, self.c, "rule")
        ai, trace = schedule(links, p, .5, self.c, "ai")
        zero, _ = schedule(links, p, .5, self.c, "rule", delay_s=0)
        self.assertEqual(rule["outage_samples"], 5)
        self.assertEqual(ai["outage_samples"], 0)
        self.assertEqual(zero["outage_samples"], 0)
        np.testing.assert_array_equal(trace["events"], [[10, 0, 1]])

    def test_hysteresis_and_episode_definition(self):
        links = np.tile([-76., -60., -80.], (20, 1))
        links[2:8, 0] = -73
        links[8:, 0] = -71
        _, trace = schedule(links, np.zeros(20), .5, self.c, "rule", delay_s=0)
        np.testing.assert_array_equal(trace["events"], [[0, 0, 1], [8, 1, 0]])
        links[:] = -95
        links[[0, 4, 8], 0] = -60
        m, _ = schedule(links, np.zeros(20), .5, self.c, "rule", delay_s=0)
        self.assertEqual(m["outage_events"], 3)
        self.assertEqual(m["outage_samples"], 17)

    def test_threshold_selection_matches_direct_counts(self):
        y = np.array([0, 0, 0, 1, 1, 1], dtype=np.uint8)
        p = np.array([.01, .03, .4, .3, .8, .9])
        m = select_threshold(y, p, self.c)
        self.assertEqual(m["recall"], 1)
        self.assertEqual(m["tp"], 3)
        self.assertEqual(m["fp"], 1)

    def test_scheduler_does_not_look_ahead(self):
        v = simulate_voyage(2, self.c)
        p = np.linspace(0, 1, len(v["links"]))
        _, complete = schedule(v["links"], p, .5, self.c, "ai")
        _, prefix = schedule(v["links"][:150], p[:150], .5, self.c, "ai")
        np.testing.assert_array_equal(complete["mode"][:150], prefix["mode"])
        np.testing.assert_array_equal(complete["effective"][:150], prefix["effective"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
