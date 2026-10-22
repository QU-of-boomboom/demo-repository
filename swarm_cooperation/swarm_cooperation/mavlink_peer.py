"""第二部分：自定义 dialect 的 UDP 收发与可靠消息示例，不驱动飞控。"""

import argparse
from dataclasses import dataclass
import secrets
import socket
import time
from typing import Any, Callable, Dict, Optional, Tuple

from config import CONFIG, NETWORK


@dataclass
class Pending:
    message: Any
    target: int
    sent_at: float
    retries: int = 0


class SwarmPeer:
    def __init__(self, system_id: int, endpoints: Dict[int, Tuple[str, int]],
                 on_message: Optional[Callable[[Any], int]] = None):
        try:
            from generated import swarm
        except ImportError as exc:
            raise RuntimeError("请先安装 requirements.txt 并运行 generate_dialect.py") from exc
        self.system_id = system_id
        self.component_id = NETWORK.ground_component if system_id == NETWORK.ground_system else NETWORK.companion_component
        self.endpoints = endpoints
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(endpoints[system_id])
        self.sock.setblocking(False)
        self.codec = swarm.MAVLink(None, srcSystem=system_id, srcComponent=self.component_id)
        self.codec.robust_parsing = True
        self.session_id = secrets.randbits(32)
        self.seq = 0
        self.started = time.monotonic()
        self.pending: Dict[int, Pending] = {}
        self.seen = {}
        self.last_heartbeat = {}
        self.failed = []
        self.broadcast_repeats = []
        self.on_message = on_message or (lambda message: 1)

    def make(self, name: str, **fields) -> Any:
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return getattr(self.codec, name.lower() + "_encode")(
            session_id=self.session_id, seq=self.seq,
            time_boot_ms=int((time.monotonic() - self.started) * 1000) & 0xFFFFFFFF, **fields)

    def _send(self, message: Any, target: int) -> None:
        self.sock.sendto(message.pack(self.codec), self.endpoints[target])
        self.codec.seq = (self.codec.seq + 1) & 0xFF

    def send(self, message: Any, target: int, reliable: bool = False) -> None:
        if reliable and message.get_type() not in ("RELAY_ASSIGN", "PICK_CONFIRM"):
            raise ValueError("只有 RELAY_ASSIGN 和 PICK_CONFIRM 使用单播 ACK")
        self._send(message, target)
        if reliable:
            self.pending[message.seq] = Pending(message, target, time.monotonic())

    def broadcast(self, message: Any) -> None:
        # 本机回环示例用逐地址 UDP 扇出；实际链式转发规则见 PROTOCOL.md。
        for target in self.endpoints:
            if target != self.system_id:
                self._send(message, target)

    def heartbeat(self, role: int, state: int, ready: bool = False) -> None:
        self.broadcast(self.make("SWARM_HEARTBEAT", role=role, state=state, ready=int(ready)))

    def report_position(self, x: float, y: float, z: float, vx=0.0, vy=0.0, vz=0.0) -> None:
        self.send(self.make("SWARM_POSITION", x=x, y=y, z=z, vx=vx, vy=vy, vz=vz), NETWORK.ground_system)

    def report_rssi(self, neighbor: int, dbm: float, sample_age_ms: int = 0) -> None:
        self.send(self.make("SWARM_RSSI", peer_system=neighbor, rssi_cdbm=round(dbm * 100),
                            sample_age_ms=sample_age_ms), NETWORK.ground_system)

    def assign(self, target: int, upstream: int, downstream: int, x: float, y: float,
               operation: int = 1, route_epoch: int = 1) -> None:
        self.send(self.make("RELAY_ASSIGN", target_system=target,
                            target_component=NETWORK.companion_component, route_epoch=route_epoch,
                            operation=operation, upstream_system=upstream, downstream_system=downstream,
                            x=x, y=y, z=NETWORK.flight_altitude_m), target, reliable=True)

    def confirm_pick(self, task_id: int, shelf_id: int, success: bool) -> None:
        self.send(self.make("PICK_CONFIRM", target_system=NETWORK.ground_system,
                            target_component=NETWORK.ground_component, task_id=task_id,
                            shelf_id=shelf_id, result=0 if success else 1), NETWORK.ground_system, reliable=True)

    def announce_failsafe(self, reason: int, action: int, rssi: float) -> None:
        message = self.make("SWARM_FAILSAFE", target_system=0, target_component=0,
                            reason=reason, action=action, rssi_cdbm=round(rssi * 100),
                            hold_ms=round(CONFIG.failsafe_hold_s * 1000))
        self.broadcast(message)
        self.broadcast_repeats.append([message, time.monotonic(), NETWORK.max_retries])

    def _ack(self, message: Any, result: int) -> None:
        source = message.get_srcSystem()
        reply = self.make("SWARM_ACK", target_system=source, target_component=message.get_srcComponent(),
                          ack_session_id=message.session_id, ack_seq=message.seq,
                          ack_message_id=message.get_msgId(), result=result)
        self._send(reply, source)

    def _receive(self, message: Any, now: float) -> None:
        source = message.get_srcSystem()
        if source not in self.endpoints or source == self.system_id:
            return
        expected_component = NETWORK.ground_component if source == NETWORK.ground_system else NETWORK.companion_component
        if message.get_srcComponent() != expected_component:
            return
        if getattr(message, "target_system", 0) not in (0, self.system_id):
            return
        if getattr(message, "target_component", 0) not in (0, self.component_id):
            return
        kind = message.get_type()
        if kind == "SWARM_ACK":
            pending = self.pending.get(message.ack_seq)
            if (pending and pending.target == source and message.ack_session_id == self.session_id
                    and message.ack_message_id == pending.message.get_msgId()):
                del self.pending[message.ack_seq]
                if message.result != 0:
                    self.failed.append((message.ack_seq, "接收方拒绝"))
            return
        if kind == "RELAY_ASSIGN" and source != NETWORK.ground_system:
            return
        if kind == "PICK_CONFIRM" and source != NETWORK.main_system:
            return
        reliable = kind in ("RELAY_ASSIGN", "PICK_CONFIRM")
        key = (source, message.get_srcComponent(), message.session_id, message.get_msgId(), message.seq)
        if key in self.seen:
            if reliable:
                self._ack(message, self.seen[key][1])
            return
        result = self.on_message(message)
        if kind == "SWARM_HEARTBEAT":
            self.last_heartbeat[source] = now
        if reliable or kind == "SWARM_FAILSAFE":
            self.seen[key] = (now, result)
        if reliable:
            self._ack(message, result)

    def poll(self) -> None:
        now = time.monotonic()
        # 先清理过期项；每次最多处理64个数据报，给状态机留出调度机会。
        self.seen = {key: value for key, value in self.seen.items() if now - value[0] < NETWORK.duplicate_cache_s}
        for _ in range(64):
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, ConnectionResetError):
                break
            for message in self.codec.parse_buffer(data) or []:
                if message.get_type() != "BAD_DATA":
                    self._receive(message, now)
        for seq, pending in list(self.pending.items()):
            if now - pending.sent_at >= NETWORK.ack_timeout_s:
                if pending.retries >= NETWORK.max_retries:
                    self.failed.append((seq, "ACK 重试耗尽"))
                    del self.pending[seq]
                else:
                    self._send(pending.message, pending.target)
                    pending.sent_at, pending.retries = now, pending.retries + 1
        for entry in list(self.broadcast_repeats):
            if now - entry[1] >= NETWORK.ack_timeout_s:
                self.broadcast(entry[0])
                entry[1], entry[2] = now, entry[2] - 1
                if entry[2] == 0:
                    self.broadcast_repeats.remove(entry)

    def stale_nodes(self) -> list:
        now = time.monotonic()
        return [node for node in self.endpoints if node != self.system_id
                and now - self.last_heartbeat.get(node, self.started) > NETWORK.heartbeat_timeout_s]

    def close(self) -> None:
        self.sock.close()


def local_endpoints() -> Dict[int, Tuple[str, int]]:
    nodes = (NETWORK.ground_system, NETWORK.main_system, *NETWORK.relay_systems)
    return {node: ("127.0.0.1", NETWORK.udp_base_port + (0 if node == NETWORK.ground_system else node)) for node in nodes}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sysid", type=int, required=True, choices=list(local_endpoints()))
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--assign-demo", action="store_true", help="地面站向中继1发送协议级准备消息")
    args = parser.parse_args()

    def handle(message):
        if message.get_type() in ("RELAY_ASSIGN", "PICK_CONFIRM", "SWARM_FAILSAFE"):
            print("收到：", message)
        # 此处接入第一部分状态机：校验路由版本并把命令入队。
        # 本示例只确认协议接收，未执行起飞，不将 ready 设为真。
        return 0

    peer = SwarmPeer(args.sysid, local_endpoints(), handle)
    started, last_heartbeat, last_report = time.monotonic(), 0.0, 0.0
    assigned = False
    role = 0 if args.sysid == NETWORK.ground_system else (1 if args.sysid == NETWORK.main_system else 2)
    try:
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            peer.poll()
            if now - last_heartbeat >= NETWORK.heartbeat_s:
                peer.heartbeat(role, state=1 if role else 0)
                last_heartbeat = now
            if role and now - last_report >= NETWORK.telemetry_s:
                peer.report_position(0.0, 0.0, 0.0)
                peer.report_rssi(NETWORK.ground_system, -40.0)
                last_report = now
            if args.assign_demo and role == 0 and not assigned and now - started > 2:
                x, y = CONFIG.relay_positions[0]
                peer.assign(NETWORK.relay_systems[0], NETWORK.ground_system, NETWORK.main_system,
                            x * CONFIG.cell_m, y * CONFIG.cell_m)
                assigned = True
            time.sleep(0.01)
        print("未确认事务：", list(peer.pending), "失败：", peer.failed, "心跳超时节点：", peer.stale_nodes())
    finally:
        peer.close()


if __name__ == "__main__":
    main()
