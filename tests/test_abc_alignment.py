"""Independent acceptance of the three maintained module entry points."""
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path[:0] = [str(ROOT / "swarm_cooperation/swarm_cooperation"),
               str(ROOT / "ai_relay_scheduler/ai_relay_scheduler")]

from ideal_sim.config import ExperimentConfig
from ideal_sim.contract import OCCUPIED, ROUTE_LINKS, LINKS, route_position
from ideal_sim.simulation import IdealSimulation
from ideal_sim.world import hop_rssi, intersects_cell, outward_route
from ideal_sim.scheduler import CandidateScheduler, route_qualities
from relay_learning.routing import path_quality, RouteController
from relay_learning.features import HistoryBuffer
from relay_learning.monitor import LinkMonitor
from geometry import link_rssi as legacy_b_hop
from config import CONFIG as LEGACY_B_CONFIG


def load_entry(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


A = load_entry("module_a", "warehouse_relay_sim/warehouse_relay_sim/warehouse_relay_sim.py")
C = load_entry("module_c", "ai_relay_scheduler/ai_relay_scheduler/simulator.py")


class GeometryAcceptance(unittest.TestCase):
    def test_six_hop_diff_detects_non_bottleneck_errors_and_columns(self):
        from diff_abc import geometry_rows, differences
        from types import SimpleNamespace
        cfg = ExperimentConfig()
        sample = dict(time_s=1., phase="FLY", direction="outbound", progress_m=1.,
                      x=3., y=4., r1_x=1., r1_y=2., r2_x=2., r2_y=4.)
        rows = geometry_rows(A, sample, OCCUPIED, cfg)
        self.assertEqual([r["link"] for r in rows], list(LINKS))
        self.assertEqual((rows[0]["crossed_column_count"], rows[0]["crossed_cell_count"]), (3, 6))
        broken = SimpleNamespace(intersects_cell=A.intersects_cell,
                                 hop_rssi=lambda a, b, occupied, cfg: A.hop_rssi(a, b, occupied, cfg) +
                                 (1. if (a, b) == ((1., 2.), (2., 4.)) else 0.))
        diff = differences(rows, geometry_rows(broken, sample, OCCUPIED, cfg))
        self.assertEqual(len(diff), 1)
        self.assertEqual((diff[0]["link"], diff[0]["field"]), ("R1-R2", "physical_rssi_dbm"))
        bad_columns = [dict(r) for r in rows]
        bad_columns[0]["crossed_column_count"] = 2
        self.assertEqual(differences(rows, bad_columns)[0]["field"], "crossed_column_count")

    def test_independent_occlusion_and_path_loss_oracle(self):
        cases = [((0, 0), (3, 4), 6), ((0, 0), (1, 4), 4),
                 ((0, 0), (2, 4), 4), ((0, 0), (1, 2), 2),
                 ((0, 2), (3, 2), 0), ((0, 0), (0, 4), 0),
                 ((0, 0), (2, 2), 2), ((1, 2), (1, 2), 0)]
        for cell_m in (1., 2.):
            cfg = ExperimentConfig(cell_m=cell_m)
            for a, b, count in cases:
                with self.subTest(cell_m=cell_m, a=a, b=b):
                    expected = -40 - 22 * math.log10(max(math.dist(a, b) * cell_m, 1)) - 12 * count
                    for module in (A, C):
                        self.assertEqual(sum(module.intersects_cell(a, b, c) for c in OCCUPIED), count)
                        self.assertAlmostEqual(module.hop_rssi(a, b, OCCUPIED, cfg), expected)
                    self.assertAlmostEqual(hop_rssi(b, a, OCCUPIED, cfg), expected)
                    self.assertAlmostEqual(legacy_b_hop(a, b, replace(LEGACY_B_CONFIG, cell_m=cell_m)), expected)
        self.assertEqual(LEGACY_B_CONFIG.cell_m, 1.)
        self.assertEqual(LEGACY_B_CONFIG.pick_s, 1.)

    def test_every_route_sample_all_five_topologies_and_removed_cells(self):
        cfg = ExperimentConfig()
        stations = ((1., 2.), (2., 4.))
        for occupied in (OCCUPIED, OCCUPIED - {(1, 1), (2, 3)}):
            for direction in ("outbound", "return"):
                for tick in range(231):
                    point, segment = route_position(direction, tick / 10)
                    expected = route_qualities(point, stations, occupied, cfg)
                    self.assertEqual(set(expected), set(ROUTE_LINKS))
                    for module in (A, C):
                        self.assertEqual(module.route_position(direction, tick / 10), (point, segment))
                        self.assertEqual(module.route_qualities(point, stations, occupied, cfg), expected)
                    for route, quality in expected.items():
                        nodes = [(0., 0.), *[stations[i - 1] for i in route], point]
                        self.assertEqual(quality, min(hop_rssi(a, b, occupied, cfg)
                                                     for a, b in zip(nodes, nodes[1:])))


class ExecutionAcceptance(unittest.TestCase):
    def test_eta_is_a_causal_snapshot_not_a_future_arrival(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="fixed"))
        sim.now = 2.
        sim.relay_positions = [(1., 0.), (0., 0.)]
        sim.relay_status = ["MOVING", "STANDBY"]
        sim.targets = [(1., 2.), (2., 4.)]
        before = sim.feature_state()
        encoded_before = json.dumps(before, sort_keys=True)
        self.assertEqual(before["relay_eta_estimated_at_s"], 2.)
        self.assertAlmostEqual(before["relay_eta_s"][0], 1.5)
        self.assertFalse(before["relay_ready"][0])
        # A later hold, changed speed and goal must not rewrite this earlier estimate.
        sim.now = 20.
        sim.cfg = replace(sim.cfg, relay_speed_mps=.5)
        sim.targets = [(2., 4.), (1., 3.)]
        sim.events.append(dict(kind="hypothetical_future_arrival", time_s=100.))
        after = sim.feature_state()
        self.assertEqual(json.dumps(before, sort_keys=True), encoded_before)
        self.assertEqual(after["relay_eta_estimated_at_s"], 20.)
        self.assertNotEqual(after["relay_eta_s"], before["relay_eta_s"])
        sim.controller.active = (1,)
        self.assertTrue(sim.feature_state()["relay_eta_blocked"][0])

    def test_complete_trace_matches_across_abc_with_delay_and_noise(self):
        cfg = ExperimentConfig(rounds=1, mode="none", strategy="predeploy",
                               measurement_delay_s=.1, noise_sigma_db=.5, record_learning=True)
        sims = [A.create_simulation(cfg), IdealSimulation(cfg), C.create_simulation(cfg)]
        for sim in sims:
            self.assertTrue(sim.run()["success"])
            self.assertEqual(sim.main_distance_m, sims[0].main_distance_m)
            self.assertEqual(sim.samples, sims[0].samples)
            self.assertEqual(sim.events, sims[0].events)
            self.assertEqual(sim.monitor.frames, sims[0].monitor.frames)
        sim = sims[0]
        self.assertEqual(sim.report()["main_distance_m"], 46.)
        self.assertEqual(sim.report()["phase_time_s"]["FLY"], 46.)
        self.assertEqual(sim.report()["phase_time_s"]["DWELL"], 2.)
        self.assertAlmostEqual(sum(sim.report()["phase_time_s"].values()), sim.now)
        for i, sample in enumerate(sim.samples, 1):
            self.assertAlmostEqual(sample["time_s"], i * cfg.dt_s)
            self.assertAlmostEqual(sample["progress_m"], sample["progress_cells"] * cfg.cell_m)
            self.assertEqual(sample["segment_id"], route_position(sample["direction"], sample["progress_cells"])[1])
            for obs in sample["observations"]:
                if obs["valid"]:
                    self.assertAlmostEqual(sample["time_s"] - obs["sample_time_s"], .1)
        out = [(s["x"], s["y"]) for s in sim.samples if s["direction"] == "outbound" and s["phase"] == "FLY"]
        back = [(s["x"], s["y"]) for s in sim.samples if s["direction"] == "return" and s["phase"] == "FLY"]
        for a, b in zip(reversed([(0., 0.)] + out[:-1]), back):
            self.assertLess(math.dist(a, b), 1e-8)

    def test_zero_dwell_has_no_phantom_tick(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", terminal_dwell_s=0., home_dwell_s=0.))
        self.assertTrue(sim.run()["success"])
        self.assertNotIn("DWELL", sim.report()["phase_time_s"])
        self.assertEqual([e["dwell_s"] for e in sim.events if e["kind"] == "dwell_complete"], [0., 0.])

    def test_motion_readiness_and_settling_are_real(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="fixed"))
        states, first_arrival, first_ready = set(), None, None
        for _ in range(200):
            sim.step()
            status = sim.relay_status[0]
            states.add(status)
            if sim.relay_positions[0] == sim.targets[0] and first_arrival is None:
                first_arrival = sim.now
            if status == "READY":
                first_ready = sim.now
                break
            self.assertFalse(sim.feature_state()["relay_ready"][0])
            self.assertEqual(path_quality((1,), dict.fromkeys(LINKS, -40.),
                                          sim.feature_state()["relay_ready"]), -math.inf)
        self.assertTrue({"TAKEOFF", "MOVING", "SETTLING", "READY"} <= states)
        self.assertGreaterEqual(first_ready - first_arrival + 1e-8, sim.cfg.settle_s)
        self.assertGreaterEqual(first_arrival, sim.cfg.takeoff_s + 3 / sim.cfg.relay_speed_mps)

    def test_upstream_stays_until_replacement_ack(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none"))
        sim.relay_positions = [(1., 2.), (2., 4.)]
        sim.relay_status = ["READY", "READY"]
        sim.targets = [(0., 0.), (2., 4.)]
        sim.controller.active = (1, 2)
        good = dict.fromkeys(LINKS, -60.)
        sim.controller.request((2,), 0.)
        for now in (.1, .2, .4):
            sim.now = now
            sim._move_relays()
            sim.controller.advance(now, good, (True, True))
            self.assertEqual(sim.relay_positions[0], (1., 2.))
            self.assertFalse(sim.controller.acknowledge(999, now, good, (True, True)))
        self.assertTrue(sim.controller.acknowledge(1, .5, good, (True, True)))
        sim._move_relays()
        self.assertNotEqual(sim.relay_positions[0], (1., 2.))
        self.assertEqual(sim.relay_status[0], "MOVING")

    def test_reactive_does_not_predeploy_on_later_legs(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="reactive"))
        self.assertEqual(sim.phase, "FLY")
        self.assertEqual(sim.targets, [(0., 0.), (0., 0.)])
        self.assertEqual(sim.decisions[0]["state"]["planned_main_positions"], ((0., 0.),))
        sim.direction = "return"
        sim.position = outward_route()[-1]
        sim.relay_positions = [(1., 2.), (2., 4.)]
        sim.relay_status = ["READY", "READY"]
        sim._prepare_leg()
        self.assertEqual(sim.phase, "FLY")
        self.assertEqual(sim.targets, sim.relay_positions)
        self.assertEqual(sim.deployment_count, 0)

    def test_reactive_threshold_is_not_delayed_by_periodic_planner(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="reactive"))
        sim.now, sim.last_plan_at = 1., 0.
        sim.position, sim.progress = (1., 4.), 5.
        sim.monitor.measured = dict.fromkeys(LINKS, -95.)
        sim._rolling_plan()
        self.assertEqual(sim.phase, "DEPLOY")
        self.assertNotEqual(sim.targets, [(0., 0.), (0., 0.)])
        self.assertEqual(sim.decisions[-1]["state"]["planned_main_positions"], ((1., 4.),))


class MotionGateAcceptance(unittest.TestCase):
    def test_candidate_plans_exclude_relay_until_eta_and_never_fake_link(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="predeploy"))
        sim.now = 1.0
        sim.relay_positions = [(1., 0.), (0., 0.)]
        sim.targets = [(1., 2.), (0., 0.)]
        sim.relay_status = ["MOVING", "STANDBY"]
        state = sim.scheduling_state()
        eta = state.relay_eta_s[0]
        self.assertGreater(eta, 0.)
        self.assertEqual(state.relay_status, ("MOVING", "STANDBY"))
        scheduler = CandidateScheduler(sim.cfg, sim.world.nodes)
        plan = scheduler.plan(state)
        # Before ETA the moving relay is not a taskable station in any candidate.
        self.assertIsNone(plan.stations[0])
        self.assertTrue(scheduler.last_candidates)
        self.assertTrue(all(candidate.stations[0] is None
                            for candidate in scheduler.last_candidates))
        self.assertTrue(all("relay" not in candidate.rejection_reason.lower()
                            for candidate in scheduler.last_candidates))
        # A replan must not redirect the in-flight relay to home or another station.
        sim.plan = plan
        self.assertEqual(sim._plan_targets()[0], (1., 2.))
        features = sim.feature_state()
        self.assertFalse(features["relay_ready"][0])
        self.assertEqual(features["relay_eta_estimated_at_s"], 1.0)
        self.assertGreater(features["relay_eta_s"][0], 0.)
        # Even a very strong RSSI beside the main drone is not a valid path while
        # the relay is still MOVING; only READY enables a business route.
        measured = dict.fromkeys(LINKS, -30.)
        self.assertEqual(path_quality((1,), measured, (False, True)), -math.inf)
        self.assertEqual(path_quality((1, 2), measured, (False, True)), -math.inf)
        self.assertEqual(path_quality((1,), measured, (True, True)), -30.)

    def test_eta_counts_down_and_is_stamped_at_observation_time(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="fixed"))
        sim.now = 3.0
        sim.relay_positions = [(0., 0.), (0., 0.)]
        sim.targets = [(1., 2.), (2., 4.)]
        sim.relay_status = ["STANDBY", "STANDBY"]
        first = sim.feature_state()
        self.assertEqual(first["relay_eta_s"][0], 2.0 + 3.0 / 2.0 + 0.5)
        self.assertEqual(first["relay_eta_estimated_at_s"], 3.0)
        sim.now = 4.0
        sim.relay_positions = [(0.5, 0.), (0., 0.)]
        second = sim.feature_state()
        self.assertEqual(second["relay_eta_estimated_at_s"], 4.0)
        self.assertLess(second["relay_eta_s"][0], first["relay_eta_s"][0])
        # The earlier snapshot is immutable; no future arrival is back-filled.
        self.assertEqual(first["relay_eta_s"][0], 4.0)


class RuleAcceptance(unittest.TestCase):
    def test_passive_rule_needs_n_weak_slots_and_minus72_recovery(self):
        ctl = RouteController(service=-75., disconnect=-88., hysteresis=3., dwell=1., on_slots=3)
        ctl.active = ()
        measured = dict.fromkeys(LINKS, -60.)
        measured["G-M"] = -80.
        self.assertEqual(ctl.propose(.1, measured, (True, True)), ())
        self.assertEqual(ctl.propose(.2, measured, (True, True)), ())
        self.assertEqual(ctl.propose(.3, measured, (True, True)), (1,))
        # Recovery to direct needs three slots at or above -72 dBm.
        ctl.active, ctl.last_switch = (1,), -math.inf
        ctl.weak_run = ctl.strong_run = 0
        measured["G-M"] = -73.
        for now in (.4, .5, .6):
            self.assertEqual(ctl.propose(now, measured, (True, True)), (1,))
        measured["G-M"] = -70.
        self.assertEqual(ctl.propose(.7, measured, (True, True)), (1,))
        self.assertEqual(ctl.propose(.8, measured, (True, True)), (1,))
        self.assertEqual(ctl.propose(.9, measured, (True, True)), ())
        # A hard disconnect bypasses the N-slot counter for safety.
        ctl.active, ctl.last_switch = (), -math.inf
        hard = dict.fromkeys(LINKS, -60.)
        hard["G-M"] = -95.
        self.assertEqual(ctl.propose(1.0, hard, (True, True)), (1,))


class ObservationAcceptance(unittest.TestCase):
    def test_one_hop_outage_for_1_to_3s_is_never_carried_forward(self):
        cfg = ExperimentConfig()
        for duration in (1.0, 2.0, 3.0):
            with self.subTest(duration=duration):
                monitor = LinkMonitor(cfg)
                start = .1
                monitor.inject_outage("R1-M", start, start + duration)
                measured_during, measured_after = [], []
                for tick in range(1, 51):
                    now = round(tick * cfg.dt_s, 9)
                    monitor.update(now, [-60.] * 6, {}, 1)
                    if now < start + duration:
                        self.assertNotIn("R1-M", monitor.measured)
                        self.assertFalse(next(o for o in monitor.observations
                                              if o["link_id"] == "R1-M")["valid"])
                        measured_during.append(now)
                    elif now > start + duration:
                        self.assertIn("R1-M", monitor.measured)
                        measured_after.append(now)
                self.assertTrue(measured_during and measured_after)

    def test_scheduler_does_not_select_route_using_injected_hop(self):
        cfg = ExperimentConfig()
        monitor = LinkMonitor(cfg)
        monitor.inject_outage("R1-M", 0., 3.)
        for tick in range(1, 16):
            monitor.update(round(tick * cfg.dt_s, 9), [-60.] * 6, {}, 1)
        ready = (True, True)
        self.assertEqual(path_quality((1,), monitor.measured, ready), -math.inf)
        controller = RouteController()
        controller.active = (1,)
        desired = controller.propose(1.0, monitor.measured, ready)
        self.assertIsNotNone(desired)
        self.assertNotEqual(desired, (1,))
        self.assertNotIn("R1-M", ROUTE_LINKS[desired])
        self.assertGreaterEqual(path_quality(desired, monitor.measured, ready), cfg.disconnect_dbm)

    def test_all_paths_unavailable_returns_none_and_keeps_old_route(self):
        cfg = ExperimentConfig()
        monitor = LinkMonitor(cfg)
        for link in LINKS:
            monitor.inject_outage(link, 0., 3.)
        monitor.update(.1, [-60.] * 6, {}, 1)
        self.assertFalse(monitor.measured)
        controller = RouteController()
        controller.active = (1,)
        self.assertIsNone(controller.propose(.1, monitor.measured, (True, True)))
        self.assertEqual(controller.active, (1,))
        # The retained identifier is not evidence of a usable link.
        self.assertEqual(path_quality(controller.active, monitor.measured, (True, True)), -math.inf)

    def test_each_injected_observation_fault_records_degradation(self):
        for fault in ("missing", "stale", "nan"):
            with self.subTest(fault=fault):
                sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", strategy="reactive",
                                                       record_learning=True))
                sim.step()
                self.assertFalse(sim.degraded)
                if fault == "nan":
                    with patch("ideal_sim.simulation.hop_rssi", return_value=math.nan):
                        sim.step()
                else:
                    sim.monitor.cfg = replace(sim.cfg, measurement_delay_s=100. if fault == "missing" else .6)
                    if fault == "stale":
                        sim.now, sim.ticks = 1., 10
                    sim.step()
                self.assertTrue(sim.degraded)
                self.assertFalse(sim.monitor.measured)
                self.assertTrue(any(e["kind"] == "link_degraded" for e in sim.events))
                self.assertTrue(all(not obs["valid"] for obs in sim.samples[-1]["observations"]))
                json.dumps(sim.samples, allow_nan=False)
                json.dumps(sim.monitor.frames, allow_nan=False)

    def test_invalid_value_in_any_hop_cannot_pass_probe_or_ack(self):
        for route, links in ROUTE_LINKS.items():
            if not route:
                continue
            for link in links:
                for invalid in (None, math.nan, math.inf, -math.inf, True, "-50"):
                    with self.subTest(route=route, link=link, invalid=invalid):
                        bad = dict.fromkeys(LINKS, -60.)
                        bad[link] = invalid
                        self.assertEqual(path_quality(route, bad, (True, True)), -math.inf)
                        ctl = RouteController()
                        ctl.request(route, 0.)
                        ctl.advance(.2, dict.fromkeys(LINKS, -60.), (True, True))
                        self.assertFalse(ctl.acknowledge(1, .5, bad, (True, True)))
                        self.assertEqual(ctl.active, ())

    def test_sparse_stale_future_and_nan_frames_fail_closed(self):
        buffer = HistoryBuffer()
        for tick, bad in enumerate(({}, {"rssi_dbm": math.nan, "sample_time_s": 1.},
                                    {"rssi_dbm": -40., "sample_time_s": 0.},
                                    {"rssi_dbm": -40., "sample_time_s": 10.}), 10):
            buffer.push(tick / 10, [dict(link_id="G-M", valid=True, **bad)])
            self.assertFalse(any(buffer.latest_valid(link) for link in LINKS))
        ctl = RouteController()
        ctl.active = (1, 2)
        self.assertIsNone(ctl.propose(1., {}, (True, True)))
        ctl.request(None, 1.)
        self.assertEqual(ctl.active, (1, 2))

    def test_late_ack_without_advance_cannot_release_old_path(self):
        ctl = RouteController()
        ctl.active = (1,)
        good = dict.fromkeys(LINKS, -60.)
        ctl.request((2,), 0.)
        ctl.advance(.2, good, (True, True))
        self.assertFalse(ctl.acknowledge(1, 2.1, good, (True, True)))
        self.assertEqual(ctl.active, (1,))

    def test_no_observations_degrades_and_returns_instead_of_good_signal(self):
        sim = IdealSimulation(ExperimentConfig(rounds=1, mode="none", measurement_delay_s=100.))
        report = sim.run()
        self.assertFalse(report["success"])
        self.assertEqual(sim.position, (0., 0.))
        self.assertTrue(any(e["kind"] == "link_degraded" for e in sim.events))
        self.assertTrue(any(e["kind"] == "onboard_emergency_return" for e in sim.events))
        self.assertTrue(all(not s["measured_links"] for s in sim.samples))


if __name__ == "__main__":
    unittest.main()
