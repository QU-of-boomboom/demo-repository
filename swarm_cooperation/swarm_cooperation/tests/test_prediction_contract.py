import math
import unittest

from ideal_sim.config import ExperimentConfig
from ideal_sim.simulation import IdealSimulation
from ideal_sim.scheduler import CandidateScheduler
from dataclasses import replace
from relay_learning.features import FeatureSpec, HistoryBuffer, LINKS
from relay_learning.monitor import LinkMonitor
from relay_learning.routing import RouteController, path_quality


def state():
    return dict(main_position=(0., 1.), relay_positions=((1., 2.), (2., 4.)),
                relay_targets=((1., 2.), (2., 4.)), relay_ready=(True, True),
                relay_eta_s=(0., 0.), relay_battery=(1., 1.),
                direction="outbound", progress_cells=1., phase="FLY",
                main_speed_mps=1., cell_m=1.)


def measurements(now, values=None):
    return [dict(link_id=link, rssi_dbm=(values or {}).get(link, -60. - i),
                 sample_time_s=now, valid=True) for i, link in enumerate(LINKS)]


class FeatureContractTests(unittest.TestCase):
    def test_missing_stale_nonfinite_and_future_samples_are_invalid(self):
        buffer = HistoryBuffer()
        data = measurements(1.)
        data[0]["sample_time_s"] = 0.
        data[1]["sample_time_s"] = 2.
        data[2]["rssi_dbm"] = math.nan
        data[3]["valid"] = False
        buffer.push(1., data)
        for link in LINKS[:4]:
            self.assertFalse(buffer.latest_valid(link))
            self.assertEqual(buffer.encode(state(), link)[57:60], [0., 0., 1.])

    def test_each_physical_link_uses_its_own_window_and_direction(self):
        buffer = HistoryBuffer()
        buffer.push(.1, measurements(.1))
        direct, backbone = buffer.encode(state(), "G-M"), buffer.encode(state(), "R1-R2")
        self.assertEqual(len(direct), 96)
        self.assertNotEqual(direct[57], backbone[57])
        s = state()
        s["direction"], s["progress_cells"] = "return", 22.
        returning = buffer.encode(s, "G-M")
        self.assertNotEqual(direct, returning)

    def test_gaps_insert_missing_slots_and_duplicates_fail(self):
        buffer = HistoryBuffer()
        buffer.push(.1, measurements(.1))
        buffer.push(.4, measurements(.4))
        x = buffer.encode(state(), "G-M")
        self.assertEqual(x[51:57], [0., 0., 1.] * 2)
        with self.assertRaises(ValueError):
            buffer.push(.4, measurements(.4))
        with self.assertRaises(ValueError):
            buffer.push(.45, measurements(.45))

    def test_snapshot_copy_does_not_depend_on_truth_or_future(self):
        buffer = HistoryBuffer()
        buffer.push(.1, measurements(.1))
        s = state()
        expected = buffer.encode(s, "G-M")
        s["future_rssi"], s["truth_map"] = [-1.] * 100, [(1, 1)]
        self.assertEqual(buffer.encode(s, "G-M"), expected)


class RouteTransactionTests(unittest.TestCase):
    def test_prepare_commit_wrong_ack_and_confirm_preserve_old_route(self):
        ctl = RouteController()
        good, ready = dict.fromkeys(LINKS, -60.), (True, True)
        ctl.request((1, 2), 0.)
        ctl.advance(.2, good, ready)
        self.assertEqual(ctl.active, ())
        self.assertFalse(ctl.acknowledge(99, .5, good, ready))
        self.assertFalse(ctl.acknowledge(1, .4, good, ready))
        self.assertEqual(ctl.active, ())
        self.assertTrue(ctl.acknowledge(1, .5, good, ready))
        self.assertEqual(ctl.active, (1, 2))
        self.assertFalse(ctl.acknowledge(1, .6, good, ready))

    def test_failed_or_missing_backbone_prevents_activation(self):
        ctl = RouteController()
        good = dict.fromkeys(LINKS, -60.)
        good["G-R1"] = -95.
        self.assertEqual(path_quality((1, 2), good, (True, True)), -95.)
        ctl.request((1, 2), 0.)
        ctl.advance(.2, good, (True, True))
        self.assertIsNone(ctl.pending)
        self.assertEqual(ctl.active, ())
        del good["G-R1"]
        self.assertEqual(path_quality((1,), good, (True, True)), -math.inf)

    def test_timeout_never_releases_old_path(self):
        ctl = RouteController()
        good = dict.fromkeys(LINKS, -60.)
        ctl.request((1,), 0.)
        ctl.advance(.2, good, (True, True))
        ctl.advance(2.1, good, (True, True))
        self.assertIsNone(ctl.pending)
        self.assertEqual(ctl.active, ())

    def test_prediction_can_switch_before_current_rssi_is_weak(self):
        ctl = RouteController()
        good = dict.fromkeys(LINKS, -65.)
        risks = dict.fromkeys(LINKS, .01)
        risks["G-M"] = .99
        self.assertEqual(ctl.propose(2., good, (True, True)), ())
        self.assertNotEqual(ctl.propose(2., good, (True, True), risks), ())

    def test_failure_overrides_dwell_and_unready_candidate_is_rejected(self):
        ctl = RouteController(dwell=100.)
        ctl.last_switch = 0.
        good = dict.fromkeys(LINKS, -65.)
        good["G-M"] = -95.
        self.assertEqual(ctl.propose(.1, good, (False, True)), (2,))


class IntegrationContractTests(unittest.TestCase):
    def test_model_error_falls_back_and_never_marks_stale_as_good(self):
        class Failing:
            spec = FeatureSpec()
            thresholds = [.5, .5]

            def predict(self, rows):
                raise TimeoutError("injected")

        monitor = LinkMonitor(ExperimentConfig(), Failing())
        monitor.update(.1, [-60.] * 6, state(), 1)
        self.assertIsNone(monitor.risks)
        self.assertEqual(monitor.fallback_count, 1)
        self.assertIn("TimeoutError", monitor.last_error)

    def test_static_map_does_not_observe_removed_boxes(self):
        sim = IdealSimulation(ExperimentConfig(rounds=2, map_observation="static"))
        report = sim.run()
        self.assertTrue(report["change_executed"])
        self.assertEqual(sim.world.known, sim.world.initial)
        self.assertFalse(report["final_map_matches_truth"])
        self.assertFalse(any(e["kind"] == "map_updated" for e in sim.events))

    def test_active_relays_never_move_without_handover(self):
        sim = IdealSimulation(ExperimentConfig(rounds=2))
        sim.run()
        previous = None
        for sample in sim.samples:
            if previous is not None:
                for relay in previous["route"]:
                    if relay in sample["route"]:
                        self.assertEqual((previous[f"r{relay}_x"], previous[f"r{relay}_y"]),
                                         (sample[f"r{relay}_x"], sample[f"r{relay}_y"]))
            previous = sample
        commits = {e["epoch"]: e["time_s"] for e in sim.events if e["kind"] == "route_commit"}
        for event in sim.events:
            if event["kind"] == "route_confirmed":
                self.assertGreaterEqual(event["time_s"] - commits[event["epoch"]] + 1e-8,
                                        sim.cfg.route_ack_s)

    def test_scheduler_rejects_removing_only_working_upstream(self):
        sim = IdealSimulation(ExperimentConfig(rounds=2))
        state_ = replace(sim.scheduling_state(), relay_positions=((1., 2.), (2., 4.)),
                         active_route=(1, 2), relay_ready=(True, True),
                         measured_links=(("G-M", -95.), ("G-R1", -60.), ("G-R2", -95.),
                                         ("R1-M", -95.), ("R2-M", -60.), ("R1-R2", -60.)))
        scheduler = CandidateScheduler(sim.cfg, sim.world.nodes)
        selected = scheduler.plan(state_)
        self.assertEqual(selected.stations, state_.relay_positions)
        self.assertTrue(any(not c.feasible and c.rejection_reason == "active upstream dependency"
                            for c in scheduler.last_candidates))

    def test_persistent_missing_telemetry_holds_then_returns_onboard(self):
        sim = IdealSimulation(ExperimentConfig(rounds=2, mode="none"))
        while sim.phase != "FLY" or sim.progress < 1.:
            sim.step()
        # Inject the radio adapter fault; local navigation still works.
        sim.monitor.cfg = replace(sim.cfg, measurement_delay_s=10.)
        report = sim.run()
        self.assertFalse(report["success"])
        self.assertEqual(sim.position, (0., 0.))
        holds = [e for e in sim.events if e["kind"] == "onboard_link_hold"]
        returns = [e for e in sim.events if e["kind"] == "onboard_emergency_return"]
        self.assertEqual(len(returns), 1)
        self.assertGreaterEqual(returns[0]["time_s"] - holds[-1]["time_s"], sim.cfg.failsafe_hold_s)


if __name__ == "__main__":
    unittest.main()
