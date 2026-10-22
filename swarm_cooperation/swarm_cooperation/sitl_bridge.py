"""第三部分：三机 SITL 接口骨架；默认只读遥测，--upload 上传任务，不自动解锁。"""

import argparse
from collections import deque
from dataclasses import dataclass
import math
import os
import time
from typing import Any, Dict, List, Optional

from config import CONFIG, NETWORK, Point
from geometry import snake_path


def load_mavutil():
    os.environ["MAVLINK20"] = "1"
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise RuntimeError("SITL 接口需要 pip install -r requirements.txt") from exc
    return mavutil


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


def grid_to_gps(point: Point, origin: GeoPoint) -> GeoPoint:
    """小范围 ENU 平面近似；全机共享主机 HOME 作为仓库原点。"""
    radius = 6378137.0
    east, north = point[0] * CONFIG.cell_m, point[1] * CONFIG.cell_m
    return GeoPoint(origin.lat + math.degrees(north / radius),
                    origin.lon + math.degrees(east / (radius * math.cos(math.radians(origin.lat)))))


def gps_to_grid(point: GeoPoint, origin: GeoPoint) -> Point:
    radius = 6378137.0
    return (math.radians(point.lon - origin.lon) * radius * math.cos(math.radians(origin.lat)) / CONFIG.cell_m,
            math.radians(point.lat - origin.lat) * radius / CONFIG.cell_m)


@dataclass(frozen=True)
class MissionItem:
    command: int
    lat: float
    lon: float
    altitude_m: float
    hold_s: float = 0.0


def build_mission(role: str, home: GeoPoint, origin: GeoPoint, mav: Any) -> List[MissionItem]:
    """ArduPilot 任务第0项为 HOME 占位；随后为起飞、格线航点及动作。"""
    altitude = NETWORK.flight_altitude_m

    def item(command, position, hold=0.0, alt=altitude):
        return MissionItem(command, position.lat, position.lon, alt, hold)

    items = [item(mav.MAV_CMD_NAV_WAYPOINT, home, alt=0), item(mav.MAV_CMD_NAV_TAKEOFF, home)]
    if role == "main":
        points = snake_path()
        items.extend(item(mav.MAV_CMD_NAV_WAYPOINT, grid_to_gps(p, origin)) for p in points[1:])
        items.append(item(mav.MAV_CMD_NAV_LOITER_TIME, grid_to_gps(points[-1], origin), CONFIG.pick_s))
        items.extend(item(mav.MAV_CMD_NAV_WAYPOINT, grid_to_gps(p, origin)) for p in reversed(points[:-1]))
        items.append(item(mav.MAV_CMD_NAV_LAND, home, alt=0))
    else:
        index = 0 if role == "relay1" else 1
        station = CONFIG.relay_positions[index]
        # 中继实际出生点可以不同；骨架先接入仓库原点，随后沿格线前出。
        points = [CONFIG.home, (CONFIG.home[0], station[1]), station]
        items.extend(item(mav.MAV_CMD_NAV_WAYPOINT, grid_to_gps(p, origin)) for p in points)
        items.append(item(mav.MAV_CMD_NAV_LOITER_UNLIM, grid_to_gps(station, origin)))
    return items


class MissionUpload:
    """由飞控请求驱动上传；正确处理重发、乱序请求、超时和最终 ACK。"""

    def __init__(self, vehicle, items: List[MissionItem]):
        if not items:
            raise ValueError("任务不能为空")
        self.vehicle, self.items = vehicle, items
        self.sent = set()
        self.last_seq = None
        self.legacy = False
        self.started = self.last_sent = time.monotonic()
        self.retries = 0
        self.done = False
        self.error: Optional[str] = None
        self.send_count()

    def send_count(self):
        v = self.vehicle
        v.link.mav.mission_count_send(v.system_id, NETWORK.autopilot_component, len(self.items), v.mav.MAV_MISSION_TYPE_MISSION)
        self.last_sent = time.monotonic()

    def send_item(self, seq: int, legacy: bool):
        v, item = self.vehicle, self.items[seq]
        common = (v.system_id, NETWORK.autopilot_component, seq)
        # MISSION_REQUEST 是旧版浮点请求，必须回 MISSION_ITEM；INT 请求回 INT。
        if legacy:
            v.link.mav.mission_item_send(*common, v.mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                        item.command, 0, 1, item.hold_s, 0, 0, float("nan"),
                                        item.lat, item.lon, item.altitude_m, v.mav.MAV_MISSION_TYPE_MISSION)
        else:
            v.link.mav.mission_item_int_send(*common, v.mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                            item.command, 0, 1, item.hold_s, 0, 0, float("nan"),
                                            round(item.lat * 1e7), round(item.lon * 1e7), item.altitude_m,
                                            v.mav.MAV_MISSION_TYPE_MISSION)
        self.sent.add(seq)
        self.last_seq, self.legacy, self.last_sent = seq, legacy, time.monotonic()

    def on_message(self, message):
        if self.done or self.error:
            return
        if getattr(message, "mission_type", 0) != self.vehicle.mav.MAV_MISSION_TYPE_MISSION:
            return
        if getattr(message, "target_system", NETWORK.ground_system) not in (0, NETWORK.ground_system):
            return
        if getattr(message, "target_component", NETWORK.ground_component) not in (0, NETWORK.ground_component):
            return
        kind = message.get_type()
        if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            if not 0 <= message.seq < len(self.items):
                self.error = f"飞控请求非法航点 {message.seq}"
                return
            self.retries = 0
            self.send_item(message.seq, legacy=kind == "MISSION_REQUEST")
        elif kind == "MISSION_ACK":
            if message.type != self.vehicle.mav.MAV_MISSION_ACCEPTED:
                self.error = f"飞控拒绝任务，MAV_MISSION_RESULT={message.type}"
            elif len(self.sent) == len(self.items):
                self.done = True
            # 未发齐前的 accepted 可能是旧事务 ACK，不能判为成功。

    def tick(self, now: float):
        if self.done or self.error:
            return
        if now - self.started >= NETWORK.mission_total_timeout_s:
            self.error = "任务上传总超时"
        elif now - self.last_sent >= NETWORK.mission_timeout_s:
            if self.retries >= NETWORK.max_retries:
                self.error = "任务上传重试耗尽"
                return
            self.retries += 1
            if self.last_seq is None:
                self.send_count()
            else:
                self.send_item(self.last_seq, self.legacy)


class VehicleLink:
    def __init__(self, role: str, system_id: int, endpoint: str, mavutil):
        self.role, self.system_id, self.mav = role, system_id, mavutil.mavlink
        self.link = mavutil.mavlink_connection(endpoint, source_system=NETWORK.ground_system,
                                              source_component=NETWORK.ground_component,
                                              dialect="ardupilotmega", autoreconnect=True)
        self.telemetry: Dict[str, Any] = {}
        self.received_at: Dict[str, float] = {}
        self.upload: Optional[MissionUpload] = None
        self.command_queue = deque()
        self.command_pending = None
        self.command_results = []
        self.last_heartbeat_sent = 0.0

    def fresh(self, kind: str, now: float, timeout_s: float) -> bool:
        return kind in self.received_at and now - self.received_at[kind] <= timeout_s

    def queue_command(self, command: int, *params: float) -> None:
        if len(params) > 7:
            raise ValueError("COMMAND_LONG 最多7个参数")
        self.command_queue.append((command, list(params) + [0.0] * (7 - len(params))))

    def set_mode(self, name: str) -> None:
        mapping = self.link.mode_mapping() or {}
        if name not in mapping:
            raise ValueError(f"当前飞控不支持模式 {name}；先等待飞控 HEARTBEAT")
        self.queue_command(self.mav.MAV_CMD_DO_SET_MODE, self.mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[name])

    def arm(self) -> None:
        self.queue_command(self.mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def takeoff(self, altitude_m: float = NETWORK.flight_altitude_m) -> None:
        self.queue_command(self.mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m)

    def land(self) -> None:
        self.set_mode("LAND")

    def goto(self, destination: GeoPoint, altitude_m: float = NETWORK.flight_altitude_m) -> None:
        # GUIDED 位置目标：只启用位置，忽略速度/加速度/偏航；到达必须用遥测确认。
        self.link.mav.set_position_target_global_int_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF, self.system_id, NETWORK.autopilot_component,
            self.mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000110111111000,
            round(destination.lat * 1e7), round(destination.lon * 1e7), altitude_m,
            0, 0, 0, 0, 0, 0, 0, 0)

    def hold(self) -> None:
        self.set_mode("BRAKE")

    def request_telemetry(self) -> None:
        for message_id, rate in ((self.mav.MAVLINK_MSG_ID_GPS_RAW_INT, NETWORK.gps_rate_hz),
                                 (self.mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, NETWORK.gps_rate_hz),
                                 (self.mav.MAVLINK_MSG_ID_ATTITUDE, NETWORK.attitude_rate_hz),
                                 (self.mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)):
            self.queue_command(self.mav.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 1e6 / rate)
        self.queue_command(self.mav.MAV_CMD_REQUEST_MESSAGE, self.mav.MAVLINK_MSG_ID_HOME_POSITION)

    def start_upload(self, items: List[MissionItem]) -> None:
        if self.upload and not (self.upload.done or self.upload.error):
            raise RuntimeError("每架飞控同一时刻只能有一个任务上传事务")
        self.upload = MissionUpload(self, items)

    def _send_command(self):
        pending = self.command_pending
        self.link.mav.command_long_send(self.system_id, NETWORK.autopilot_component, pending[0],
                                        pending[3], *pending[1])
        pending[2] = time.monotonic()

    def poll(self, now: float) -> None:
        if now - self.last_heartbeat_sent >= NETWORK.heartbeat_s:
            self.link.mav.heartbeat_send(self.mav.MAV_TYPE_GCS, self.mav.MAV_AUTOPILOT_INVALID,
                                         0, 0, self.mav.MAV_STATE_ACTIVE)
            self.last_heartbeat_sent = now
        for _ in range(100):
            message = self.link.recv_match(blocking=False)
            if message is None:
                break
            if message.get_srcSystem() != self.system_id or message.get_srcComponent() != NETWORK.autopilot_component:
                continue
            kind = message.get_type()
            if kind == "BAD_DATA":
                continue
            self.telemetry[kind], self.received_at[kind] = message, now
            if self.upload:
                self.upload.on_message(message)
            if kind == "COMMAND_ACK" and self.command_pending:
                if getattr(message, "target_system", 0) not in (0, NETWORK.ground_system):
                    continue
                if getattr(message, "target_component", 0) not in (0, NETWORK.ground_component):
                    continue
                if message.command == self.command_pending[0]:
                    if message.result == self.mav.MAV_RESULT_IN_PROGRESS:
                        self.command_pending[2] = now
                    else:
                        self.command_results.append((message.command, message.result))
                        self.command_pending = None
        if self.upload:
            self.upload.tick(now)
        if self.command_pending:
            if now - self.command_pending[4] > NETWORK.mission_total_timeout_s:
                self.command_results.append((self.command_pending[0], "命令总超时"))
                self.command_pending = None
            elif now - self.command_pending[2] > NETWORK.mission_timeout_s:
                if self.command_pending[3] >= NETWORK.max_retries:
                    self.command_results.append((self.command_pending[0], "命令 ACK 超时"))
                    self.command_pending = None
                else:
                    self.command_pending[3] += 1
                    self._send_command()
        if self.command_pending is None and self.command_queue:
            command, params = self.command_queue.popleft()
            self.command_pending = [command, params, now, 0, now]
            self._send_command()

    def snapshot(self, now: float) -> dict:
        gps = self.telemetry.get("GPS_RAW_INT")
        position = self.telemetry.get("GLOBAL_POSITION_INT")
        attitude = self.telemetry.get("ATTITUDE")
        return {
            "role": self.role,
            "heartbeat_ok": self.fresh("HEARTBEAT", now, NETWORK.heartbeat_timeout_s),
            "position_fresh": self.fresh("GLOBAL_POSITION_INT", now, NETWORK.telemetry_timeout_s),
            "gps": None if gps is None else {"lat": gps.lat / 1e7, "lon": gps.lon / 1e7,
                                             "alt_msl_m": gps.alt / 1000, "fix_type": gps.fix_type},
            "relative_alt_m": None if position is None else position.relative_alt / 1000,
            "attitude_rad": None if attitude is None else (attitude.roll, attitude.pitch, attitude.yaw),
        }

    def close(self) -> None:
        self.link.close()


class CollaborationHook:
    """SITL 挂接边界：实测输入替代仿真运动学，不能让两个控制器同时推进航线。"""

    def on_tick(self, now: float, vehicles: Dict[str, VehicleLink]) -> None:
        # 此处接入第一部分状态机：读取实测 GPS/LOCAL_POSITION_NED、姿态、飞行状态。
        # 使用 gps_to_grid 映射到统一仓库坐标；到达/起飞/降落用遥测确认，不能仅看计时器。
        # 第一部分 MainDrone.tick 内含运动学，实机需抽出 update_observation/decision，
        # 不应直接同时运行 SwarmSimulation.step 和飞控 AUTO 任务。
        # RSSI 从第二部分 SWARM_RSSI 接入；1s 无新测量或3s心跳超时视作断链。
        # TAKEOFF -> GUIDED -> arm(ACK与armed位) -> takeoff -> 高度达标。
        # SNAKE_SURVEY/RETURN -> goto 下一个格点 -> 位置到达 -> 下一个航点。
        # PICK -> hold + 夹爪确认；FAILSAFE -> hold -> 5s -> 反向历史航点。
        # LAND -> land + EXTENDED_SYS_STATE.on_ground + armed位清零后完成。
        # 中继准备 ACK != 到位：同时确认 ready、位置、新鲜上游RSSI才 COMMIT。
        # 当前骨架只提供接口，以下快照可供适配器消费，不自动生成飞行命令。
        self.observations = {role: vehicle.snapshot(now) for role, vehicle in vehicles.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--upload", action="store_true", help="上传三机航点任务，需SITL处于静止未解锁状态")
    args = parser.parse_args()
    mavutil = load_mavutil()
    roles = ("main", "relay1", "relay2")
    systems = (NETWORK.main_system, *NETWORK.relay_systems)
    vehicles: Dict[str, VehicleLink] = {}
    hook = CollaborationHook()
    try:
        for role, system_id, instance in zip(roles, systems, NETWORK.sitl_instances):
            port = NETWORK.sitl_tcp_base + NETWORK.sitl_port_stride * instance
            vehicles[role] = VehicleLink(role, system_id, f"tcp:{args.host}:{port}", mavutil)
        start, last_report = time.monotonic(), 0.0
        requested = uploaded = False
        while True:
            now = time.monotonic()
            for vehicle in vehicles.values():
                vehicle.poll(now)
            ready = all(v.fresh("HEARTBEAT", now, NETWORK.heartbeat_timeout_s) for v in vehicles.values())
            if not requested and ready:
                for vehicle in vehicles.values():
                    vehicle.request_telemetry()
                requested = True
            if not requested and now - start > NETWORK.connect_timeout_s:
                raise TimeoutError("未收到三架指定 sysid 的飞控心跳，请检查实例、端口及系统 ID")
            if args.upload and requested and not uploaded and all("HOME_POSITION" in v.telemetry for v in vehicles.values()):
                homes = {role: GeoPoint(v.telemetry["HOME_POSITION"].latitude / 1e7,
                                        v.telemetry["HOME_POSITION"].longitude / 1e7) for role, v in vehicles.items()}
                for role, vehicle in vehicles.items():
                    vehicle.start_upload(build_mission(role, homes[role], homes["main"], vehicle.mav))
                uploaded = True
            hook.on_tick(now, vehicles)
            if now - last_report >= 1:
                for vehicle in vehicles.values():
                    print(vehicle.snapshot(now))
                last_report = now
            for vehicle in vehicles.values():
                if vehicle.upload and vehicle.upload.error:
                    raise RuntimeError(f"{vehicle.role}: {vehicle.upload.error}")
            if now - start >= args.duration:
                if not requested:
                    raise TimeoutError("等待心跳未完成")
                if args.upload and not uploaded:
                    raise TimeoutError("缺少 HOME_POSITION，尚未上传")
                if not args.upload or all(v.upload and v.upload.done for v in vehicles.values()):
                    break
            if now - start > NETWORK.connect_timeout_s + NETWORK.mission_total_timeout_s:
                raise TimeoutError("SITL 接口总超时")
            time.sleep(CONFIG.dt_s)
        if args.upload:
            print("三机任务均收到 MISSION_ACK(ACCEPTED)，未解锁、未启动 AUTO。")
    finally:
        for vehicle in vehicles.values():
            vehicle.close()


if __name__ == "__main__":
    main()
