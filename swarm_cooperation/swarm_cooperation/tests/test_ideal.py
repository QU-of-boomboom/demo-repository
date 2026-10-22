import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from ideal_sim.artifacts import write_artifacts
from ideal_sim.config import ExperimentConfig
from ideal_sim.gazebo_transport import pose_request
from ideal_sim.gazebo_world import world_sdf
from ideal_sim.scheduler import CandidateScheduler
from ideal_sim.simulation import IdealSimulation
from ideal_sim.world import hop_rssi, outward_route


class IdealExperimentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.random = IdealSimulation()
        cls.random.run()
        cls.manual = IdealSimulation(ExperimentConfig(mode="window"))
        while not cls.manual.window_open:
            cls.manual.step()
        cls.command_time = cls.manual.now
        cls.accepted = cls.manual.submit_removal([[1, 1], [2, 2], [3, 4]], "test-command")
        cls.manual.run()

    def test_five_complete_rounds_and_preflight_graph(self):
        sim = self.random
        report = sim.report()
        self.assertTrue(sim.finished)
        self.assertEqual((report["completed_rounds"], report["single_legs"]), (5, 10))
        self.assertEqual(report["main_distance_m"], 230)
        self.assertEqual(sim.position, (0, 0))
        self.assertEqual(sim.relay_positions, [(0, 0), (0, 0)])
        self.assertEqual(sim.events[0]["time_s"], 0)
        self.assertEqual((sim.events[0]["nodes"], sim.events[0]["edges"], sim.events[0]["boxes"]), (20, 31, 12))
        survey = next(e for e in sim.events if e["kind"] == "leg_complete")
        self.assertEqual(survey["surveyed_nodes"], 20)

    def test_each_return_retraces_full_outbound(self):
        for round_index in range(1, 6):
            points = lambda direction: [(s["x"], s["y"]) for s in self.random.samples
                                        if s["round"] == round_index and s["direction"] == direction and s["phase"] == "FLY"]
            outbound = [(0., 0.)] + points("outbound")
            returning = points("return")
            self.assertEqual(len(returning), len(outbound) - 1)
            for a, b in zip(reversed(outbound[:-1]), returning):
                self.assertLess(math.dist(a, b), 1e-8)
            self.assertTrue(any(math.dist(p, (3, 0)) < 1e-8 for p in outbound))

    def test_random_change_is_two_to_four_during_first_return(self):
        removed = self.random.removal
        self.assertIn(len(removed["cells"]), (2, 3, 4))
        self.assertEqual(len(set(removed["cells"])), len(removed["cells"]))
        self.assertTrue(2 <= removed["return_elapsed_s"] <= 12.1)
        event = next(e for e in self.random.events if e["kind"] == "boxes_removed")
        self.assertEqual((event["round"], event["direction"]), (1, "return"))

    def test_changes_are_observed_before_second_forward_plan(self):
        for sim in (self.random, self.manual):
            update = next(e for e in sim.events if e["kind"] == "first_return_map")
            self.assertTrue(update["removed_observed"])
            self.assertGreater(update["map_version"], 0)
            first, second = sim.decisions[0], sim.decisions[2]
            self.assertEqual(len(first["state"]["known_occupied"]), 12)
            self.assertEqual(len(second["state"]["known_occupied"]), 12 - len(sim.removal["cells"]))
            self.assertNotIn("truth", second["state"])
            self.assertNotIn("removal", second["state"])
            self.assertGreaterEqual(second["state"]["time_s"], update["time_s"])
            self.assertEqual(second["chosen"]["stations"], sim.legs[2]["stations"])

    def test_manual_idempotency_and_wrong_payload(self):
        self.assertTrue(self.accepted["accepted"])
        self.assertTrue(self.manual.submit_removal([[1, 1], [2, 2], [3, 4]], "test-command")["accepted"])
        self.assertFalse(self.manual.submit_removal([[1, 2], [2, 3]], "test-command")["accepted"])
        self.assertEqual(sum(e["kind"] == "boxes_removed" for e in self.manual.events), 1)

    def test_window_boundaries_validation_and_observation_separation(self):
        sim = IdealSimulation(ExperimentConfig(mode="window", rounds=2))
        self.assertFalse(sim.submit_removal([[1, 1], [2, 2]])["accepted"])
        while not sim.window_open:
            sim.step()
        for cells in ([[1, 1]], [[1, 1]] * 2, [[0, 1], [2, 2]], [[1.5, 1], [2, 2]],
                      [[1, 1], [1, 2], [1, 3], [1, 4], [2, 1]], [None]):
            self.assertFalse(sim.submit_removal(cells)["accepted"])
        before = set(sim.world.known)
        self.assertTrue(sim.submit_removal([[1, 1], [2, 2]])["accepted"])
        self.assertEqual(sim.world.known, before)
        self.assertNotEqual(sim.world.known, sim.world.truth)
        self.assertFalse(sim.submit_removal([[1, 2], [2, 3]], "second")["accepted"])

    def test_missed_window_is_not_faked_as_success(self):
        sim = IdealSimulation(ExperimentConfig(mode="window", rounds=2))
        sim.run()
        self.assertFalse(sim.report()["change_executed"])
        self.assertEqual(sim.world.known, sim.world.initial)
        self.assertFalse(sim.submit_removal([[1, 1], [2, 2]])["accepted"])
        self.assertTrue(any(e["kind"] == "window_closed" for e in sim.events))

    def test_removed_box_reduces_physical_attenuation(self):
        cfg = ExperimentConfig()
        self.assertAlmostEqual(hop_rssi((0, 0), (1, 1), {(1, 1)}, cfg) + 12,
                               hop_rssi((0, 0), (1, 1), set(), cfg))
        self.assertEqual(hop_rssi((0, 0), (0, 1), {(1, 1)}, cfg), -40)

    def test_model_invalid_output_falls_back_and_reliability_constraint_holds(self):
        state = self.random.scheduling_state()
        baseline = CandidateScheduler(self.random.cfg, self.random.world.nodes)
        expected = baseline.plan(state)

        class Invalid:
            def score(self, state, candidates):
                return [float("nan")] * len(candidates)

        scheduler = CandidateScheduler(self.random.cfg, self.random.world.nodes, Invalid())
        self.assertEqual(scheduler.plan(state), expected)
        self.assertIn("ValueError", scheduler.last_fallback)

        class BadPreference:
            def score(self, state, candidates):
                return [-10000 if c.candidate_id == 0 else c.candidate_id for c in candidates]

        scheduler.model = BadPreference()
        selected = scheduler.plan(state)
        self.assertEqual(selected.cost[:2], expected.cost[:2])

    def test_same_scene_comparison_and_candidate_export(self):
        comparison = self.random.optimization
        self.assertEqual(set(comparison["records"]), {"first_pass_fixed", "frozen_map", "updated_map", "selected"})
        self.assertEqual(len(self.random.decisions[2]["candidates"]), 421)
        delta = comparison["delta_updated_minus_initial"]
        for key in delta:
            self.assertEqual(delta[key], comparison["records"]["updated_map"][key] - comparison["records"]["first_pass_fixed"][key])

    def test_artifacts_and_gazebo_world(self):
        with tempfile.TemporaryDirectory() as directory:
            out = write_artifacts(self.random, directory)
            report = json.loads((out / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["completed_rounds"], 5)
            self.assertIn("requestAnimationFrame", (out / "replay.html").read_text(encoding="utf-8"))
        world = ET.fromstring(world_sdf()).find("world")
        names = {model.attrib["name"] for model in world.findall("model")}
        self.assertTrue({"main", "relay1", "relay2", "box_1_1", "box_3_4"} <= names)
        request = pose_request(self.random.snapshot(), 1.)
        self.assertIn('name: "main"', request)
        self.assertIn("z: -10.0", request)
        self.assertEqual(request.count("pose {"), 15)

    def test_config_rejects_invalid_window(self):
        for start, end in ((5, 3), (-1, 2), (0, 23), (1, float("nan"))):
            with self.assertRaises(ValueError):
                ExperimentConfig(window_start_s=start, window_end_s=end)


if __name__ == "__main__":
    unittest.main()
