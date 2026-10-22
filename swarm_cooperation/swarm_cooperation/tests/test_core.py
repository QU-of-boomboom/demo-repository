"""不依赖 pymavlink 的行为验证；从交付目录运行 unittest discover。"""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from pathlib import Path

from config import CONFIG, NETWORK
from demo import SwarmSimulation
from geometry import link_rssi, shelf_columns, snake_path
from sitl_bridge import GeoPoint, MissionUpload, build_mission, gps_to_grid, grid_to_gps
from state_machine import EventLog, MainDrone, MainState, RelayState


class StateMachineTests(unittest.TestCase):
    def test_complete_normal_mission(self):
        sim = SwarmSimulation(output=None)
        outward = None
        while not sim.finished and sim.now < CONFIG.max_sim_s:
            sim.step()
            if sim.main.state == MainState.PICK and outward is None:
                outward = list(sim.main.history)
                self.assertEqual(sim.main.position, (3, 4))
        self.assertTrue(sim.finished)
        self.assertEqual(sim.main.state, MainState.LAND)
        self.assertTrue(sim.main.picked)
        self.assertFalse(sim.main.emergency_return)
        self.assertGreaterEqual(len(sim.log.events), 6)
        self.assertTrue(math.isfinite(sim.min_rssi))
        self.assertEqual(sim.main.return_trace, list(reversed(outward[:-1])))
        self.assertTrue(all(r.state == RelayState.RETURN and r.home_reached for r in sim.relays))
        self.assertTrue(any(route == (1, 2) for _, _, route in sim.route_changes))
        pickup = next(e for e in sim.log.events if e.drone == "主机" and e.new == "PICK")
        returned = next(e for e in sim.log.events if e.drone == "主机" and e.old == "PICK" and e.new == "RETURN")
        self.assertAlmostEqual(returned.time_s - pickup.time_s, CONFIG.pick_s)
        for a, b in zip(outward, outward[1:]):
            self.assertTrue(math.isclose(a[0], b[0]) or math.isclose(a[1], b[1]))

    def test_every_active_state_enters_failsafe(self):
        for state in MainState:
            if state == MainState.FAILSAFE:
                continue
            with self.subTest(state=state):
                main = MainDrone(EventLog(None))
                main.state = state
                main.tick(0.1, -90)
                self.assertEqual(main.state, MainState.FAILSAFE)

    def test_disconnect_threshold_strict_and_recovery_hysteresis(self):
        main = MainDrone(EventLog(None))
        main.tick(0.1, -88)
        self.assertNotEqual(main.state, MainState.FAILSAFE)
        main.tick(0.2, -88.01)
        main.tick(0.3, -85.1)
        self.assertEqual(main.state, MainState.FAILSAFE)
        main.tick(0.4, -85)
        self.assertEqual(main.state, MainState.TAKEOFF)

    def test_transient_fault_holds_position_and_preserves_pick_timer(self):
        main = MainDrone(EventLog(None))
        main.state, main.position, main.elapsed = MainState.PICK, (3, 4), 1.2
        main.tick(0.1, -95)
        for i in range(1, 20):
            main.tick(0.1 + i * 0.1, -95)
            self.assertEqual(main.position, (3, 4))
        main.tick(2.1, -70)
        self.assertEqual(main.state, MainState.PICK)
        self.assertAlmostEqual(main.elapsed, 1.2)

    def test_persistent_fault_holds_five_seconds_then_returns_once(self):
        sim = SwarmSimulation(output=None)
        while sim.now < 30:
            sim.step()
        history = list(sim.main.history)
        prior_failsafes = sim.main.failsafe_count
        sim.step(-95)
        frozen_position = sim.main.position
        for _ in range(49):
            sim.step(-95)
            self.assertEqual(sim.main.state, MainState.FAILSAFE)
            self.assertEqual(sim.main.position, frozen_position)
        sim.step(-95)
        self.assertEqual(sim.main.state, MainState.RETURN)
        while not sim.finished and sim.now < CONFIG.max_sim_s:
            sim.step(-95)
        self.assertTrue(sim.finished)
        self.assertEqual(sim.main.failsafe_count, prior_failsafes + 1)
        self.assertFalse(sim.main.picked)
        self.assertEqual(sim.main.return_trace, list(reversed(history[:-1])))

    def test_route_uses_weakest_hop(self):
        sim = SwarmSimulation(output=None)
        for relay in sim.relays:
            relay.position, relay.ready = relay.station, True
        sim.main.position = (3, 4)
        expected = min(link_rssi((0, 0), (1, 2)), link_rssi((1, 2), (2, 4)), link_rssi((2, 4), (3, 4)))
        self.assertAlmostEqual(sim.route_rssi((1, 2)), expected)
        sim.relays[0].ready = False
        self.assertEqual(sim.route_rssi((1, 2)), -math.inf)

    def test_geometry_and_model(self):
        path = snake_path()
        self.assertEqual(path, [(0, 0), (0, 4), (1, 4), (1, 0), (2, 0),
                                (2, 4), (3, 4), (3, 0), (3, 4)])
        self.assertEqual(sum(math.dist(a, b) for a, b in zip(path, path[1:])), 23)
        self.assertEqual(shelf_columns((0, 0), (3, 4)), 6)
        self.assertEqual(link_rssi((0, 0), (0, 0)), -40)
        self.assertAlmostEqual(link_rssi((0, 0), (3, 4)),
                               -40 - 22 * math.log10(5 * CONFIG.cell_m) - 12 * 6)
        origin = GeoPoint(31, 121)
        recovered = gps_to_grid(grid_to_gps((3, 4), origin), origin)
        self.assertAlmostEqual(recovered[0], 3)
        self.assertAlmostEqual(recovered[1], 4)

    def test_first_terminal_visit_does_not_end_survey(self):
        main = MainDrone(EventLog(None))
        visits = []
        for tick in range(2000):
            previous = main.position
            main.tick(tick * CONFIG.dt_s, -40)
            if main.position == (3, 4) and previous != main.position:
                visits.append(main.state)
            if main.state == MainState.PICK:
                break
        self.assertEqual(visits, [MainState.SNAKE_SURVEY, MainState.PICK])
        self.assertIn((3, 0), main.history)
        distance = sum(math.dist(a, b) for a, b in zip(main.history, main.history[1:]))
        self.assertAlmostEqual(distance, 23)


class FakeMav:
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_NAV_LOITER_TIME = 19
    MAV_CMD_NAV_LOITER_UNLIM = 17
    MAV_CMD_NAV_LAND = 21
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
    MAV_MISSION_TYPE_MISSION = 0
    MAV_MISSION_ACCEPTED = 0

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        return lambda *args: self.calls.append((name, args))


def message(kind, **fields):
    return SimpleNamespace(get_type=lambda: kind, **fields)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.mav = FakeMav()
        self.vehicle = SimpleNamespace(mav=self.mav, link=SimpleNamespace(mav=self.mav), system_id=4)
        self.items = build_mission("main", GeoPoint(31, 121), GeoPoint(31, 121), self.mav)

    def test_mission_contains_takeoff_pick_reverse_land(self):
        self.assertEqual(self.items[1].command, 22)
        self.assertEqual(self.items[-1].command, 21)
        pick = [item for item in self.items if item.command == 19]
        self.assertEqual(len(pick), 1)
        self.assertEqual(pick[0].hold_s, CONFIG.pick_s)
        origin = GeoPoint(31, 121)
        pick_index = self.items.index(pick[0])
        outward = self.items[2:pick_index]
        returning = self.items[pick_index + 1:-1]
        for items, expected in ((outward, snake_path()[1:]),
                                (returning, list(reversed(snake_path()[:-1])))):
            self.assertEqual(len(items), len(expected))
            for item, point in zip(items, expected):
                self.assertEqual(item.command, self.mav.MAV_CMD_NAV_WAYPOINT)
                actual = gps_to_grid(GeoPoint(item.lat, item.lon), origin)
                self.assertAlmostEqual(actual[0], point[0])
                self.assertAlmostEqual(actual[1], point[1])
        relay = build_mission("relay1", GeoPoint(31, 121), GeoPoint(31, 121), self.mav)
        self.assertEqual(relay[-1].command, 17)

    def test_request_order_duplicates_ack_and_int_units(self):
        upload = MissionUpload(self.vehicle, self.items)
        upload.on_message(message("MISSION_ACK", type=0))
        self.assertFalse(upload.done)
        for seq in list(reversed(range(len(self.items)))) + [1]:
            upload.on_message(message("MISSION_REQUEST_INT", seq=seq))
        name, args = self.mav.calls[-1]
        self.assertEqual(name, "mission_item_int_send")
        self.assertEqual(args[11:13], (310000000, 1210000000))
        upload.on_message(message("MISSION_ACK", type=0))
        self.assertTrue(upload.done)

    def test_old_request_gets_old_item(self):
        upload = MissionUpload(self.vehicle, self.items)
        upload.on_message(message("MISSION_REQUEST", seq=2))
        self.assertEqual(self.mav.calls[-1][0], "mission_item_send")

    def test_request_bounds_rejection_wrong_destination(self):
        upload = MissionUpload(self.vehicle, self.items)
        upload.on_message(message("MISSION_REQUEST_INT", seq=0, target_system=249))
        self.assertEqual(len(upload.sent), 0)
        upload.on_message(message("MISSION_REQUEST_INT", seq=len(self.items)))
        self.assertIsNotNone(upload.error)
        other = MissionUpload(self.vehicle, self.items)
        other.on_message(message("MISSION_ACK", type=1))
        self.assertIsNotNone(other.error)

    def test_upload_timeout_is_bounded(self):
        with patch("sitl_bridge.time.monotonic", return_value=0):
            upload = MissionUpload(self.vehicle, self.items)
        for step in range(NETWORK.max_retries + 1):
            now = (step + 1) * (NETWORK.mission_timeout_s + 0.1)
            with patch("sitl_bridge.time.monotonic", return_value=now):
                upload.tick(now)
        self.assertIsNotNone(upload.error)

    def test_xml_ids_and_names(self):
        root = ET.parse(Path(__file__).resolve().parents[1] / "protocol" / "swarm.xml")
        messages = root.findall("./messages/message")
        self.assertEqual([int(m.attrib["id"]) for m in messages], list(range(60000, 60007)))
        self.assertEqual(len({m.attrib["name"] for m in messages}), 7)


if __name__ == "__main__":
    unittest.main()
