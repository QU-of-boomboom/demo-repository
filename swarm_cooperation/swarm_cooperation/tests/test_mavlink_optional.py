"""可选：生成 dialect 后验证真实 MAVLink 帧；传输使用内存队列，不需要飞控。"""

from collections import deque
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from config import NETWORK
from mavlink_peer import SwarmPeer, local_endpoints
from sitl_bridge import GeoPoint, MissionUpload, VehicleLink, build_mission

try:
    from generated import swarm
except ImportError:
    swarm = None

try:
    from pymavlink.dialects.v20 import ardupilotmega
except ImportError:
    ardupilotmega = None


class MemorySocket:
    network = {}

    def __init__(self, *args):
        self.queue = deque()

    def bind(self, address):
        self.address = address
        self.network[address] = self

    def setblocking(self, value):
        pass

    def sendto(self, data, address):
        if address in self.network:
            self.network[address].queue.append((data, self.address))

    def recvfrom(self, limit):
        if not self.queue:
            raise BlockingIOError()
        return self.queue.popleft()

    def close(self):
        self.network.pop(self.address, None)


@unittest.skipIf(swarm is None, "先运行 generate_dialect.py 才验证自定义帧")
class WireTests(unittest.TestCase):
    def setUp(self):
        MemorySocket.network = {}
        self.patcher = patch("mavlink_peer.socket.socket", MemorySocket)
        self.patcher.start()
        self.messages = []

        def receive(msg):
            self.messages.append(msg)
            return 0

        self.peers = {node: SwarmPeer(node, local_endpoints(), receive) for node in local_endpoints()}
        self.ground, self.relay, self.main = (self.peers[node] for node in (250, 5, 4))

    def tearDown(self):
        for peer in self.peers.values():
            peer.close()
        self.patcher.stop()

    def flush(self):
        for _ in range(3):
            for peer in self.peers.values():
                peer.poll()

    def test_seven_message_types_roundtrip(self):
        self.main.heartbeat(1, 3)
        self.main.report_position(6, 8, 3)
        self.main.report_rssi(6, -66.3)
        self.main.confirm_pick(42, 12, True)
        self.main.announce_failsafe(1, 1, -90)
        self.ground.assign(5, 250, 4, 2, 4)
        self.flush()
        kinds = {message.get_type() for message in self.messages}
        self.assertEqual(kinds, {"SWARM_HEARTBEAT", "SWARM_POSITION", "SWARM_RSSI", "PICK_CONFIRM", "SWARM_FAILSAFE", "RELAY_ASSIGN"})
        self.assertFalse(self.ground.pending)  # 第七类 SWARM_ACK 已由控制层消费
        self.assertFalse(self.main.pending)
        rssi = next(msg for msg in self.messages if msg.get_type() == "SWARM_RSSI")
        self.assertEqual(rssi.rssi_cdbm, -6630)

    def test_lost_ack_retransmission_is_idempotent(self):
        self.ground.assign(5, 250, 4, 2, 4)
        pending = next(iter(self.ground.pending.values()))
        self.relay.poll()
        self.ground.sock.queue.clear()  # 模拟ACK丢失
        self.ground._send(pending.message, 5)
        self.flush()
        received = [msg for msg in self.messages if msg.get_type() == "RELAY_ASSIGN"]
        self.assertEqual(len(received), 1)
        self.assertFalse(self.ground.pending)

    def test_wrong_peer_ack_does_not_commit(self):
        self.ground.assign(5, 250, 4, 2, 4)
        pending = next(iter(self.ground.pending.values()))
        false_ack = self.main.make("SWARM_ACK", target_system=250, target_component=190,
                                   ack_session_id=self.ground.session_id,
                                   ack_seq=pending.message.seq, ack_message_id=60003, result=0)
        self.main.send(false_ack, 250)
        self.ground.poll()
        self.assertEqual(len(self.ground.pending), 1)

    def test_retry_exhaustion(self):
        self.ground.assign(5, 250, 4, 2, 4)
        start = next(iter(self.ground.pending.values())).sent_at
        for i in range(NETWORK.max_retries + 1):
            with patch("mavlink_peer.time.monotonic", return_value=start + (i + 1) * (NETWORK.ack_timeout_s + 0.01)):
                self.ground.poll()
        self.assertFalse(self.ground.pending)
        self.assertEqual(len(self.ground.failed), 1)


@unittest.skipIf(ardupilotmega is None, "未安装 pymavlink，跳过标准飞控帧验证")
class ArduPilotWireTests(unittest.TestCase):
    def test_real_mission_count_and_item_int(self):
        output = io.BytesIO()
        codec = ardupilotmega.MAVLink(output, srcSystem=250, srcComponent=190)
        vehicle = SimpleNamespace(mav=ardupilotmega, link=SimpleNamespace(mav=codec), system_id=4)
        items = build_mission("main", GeoPoint(31, 121), GeoPoint(31, 121), ardupilotmega)
        upload = MissionUpload(vehicle, items)
        for seq in range(len(items)):
            upload.on_message(ardupilotmega.MAVLink_mission_request_int_message(250, 190, seq, 0))
        upload.on_message(ardupilotmega.MAVLink_mission_ack_message(250, 190, 0, 0))
        decoded = ardupilotmega.MAVLink(None).parse_buffer(output.getvalue())
        self.assertEqual(decoded[0].get_type(), "MISSION_COUNT")
        self.assertEqual(len(decoded), len(items) + 1)
        self.assertEqual(decoded[2].get_type(), "MISSION_ITEM_INT")
        self.assertEqual(decoded[2].x, 310000000)
        self.assertTrue(upload.done)

    def test_guided_position_frame_signature(self):
        output = io.BytesIO()
        vehicle = SimpleNamespace(mav=ardupilotmega, link=SimpleNamespace(mav=ardupilotmega.MAVLink(output)), system_id=4)
        VehicleLink.goto(vehicle, GeoPoint(31, 121))
        decoded = ardupilotmega.MAVLink(None).parse_buffer(output.getvalue())[0]
        self.assertEqual(decoded.type_mask, 3576)
        self.assertEqual(decoded.target_system, 4)
        self.assertEqual(decoded.lat_int, 310000000)


if __name__ == "__main__":
    unittest.main()
