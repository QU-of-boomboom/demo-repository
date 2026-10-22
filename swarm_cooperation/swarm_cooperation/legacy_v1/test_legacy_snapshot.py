"""Smoke test for the frozen B legacy_v1 snapshot (2 m/cell, 3 s dwell)."""
import unittest

from config import CONFIG
from demo import SwarmSimulation
from state_machine import MainState, RelayState


class LegacySnapshotTests(unittest.TestCase):
    def test_frozen_parameters_and_single_round_mission(self):
        self.assertEqual(CONFIG.cell_m, 2.0)
        self.assertEqual(CONFIG.pick_s, 3.0)
        sim = SwarmSimulation(output=None)
        sim.run()
        self.assertTrue(sim.finished)
        self.assertEqual(sim.main.state, MainState.LAND)
        self.assertTrue(sim.main.picked)
        self.assertTrue(all(r.home_reached or r.state == RelayState.STANDBY for r in sim.relays))
        self.assertGreaterEqual(len(sim.log.events), 6)


if __name__ == "__main__":
    unittest.main()
