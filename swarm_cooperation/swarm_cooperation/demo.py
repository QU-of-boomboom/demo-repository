"""默认五轮理想实验；--legacy 保留原单轮协议/状态机演示。"""

import math
import subprocess
import time
import sys
from pathlib import Path
from typing import Callable, Optional, Tuple

from config import CONFIG, Config
from geometry import link_rssi
from state_machine import EventLog, MainDrone, RelayDrone, RelayState
from ideal_sim.contract import LINKS, ENDPOINTS, finite_number
from relay_learning.routing import RouteController, ROUTE_LINKS

Route = Tuple[int, ...]


class SwarmSimulation:
    def __init__(self, cfg: Config = CONFIG, output=print):
        if cfg.dt_s <= 0 or cfg.cell_m <= 0 or cfg.main_speed_mps <= 0 or cfg.relay_speed_mps <= 0:
            raise ValueError("时间步长、格子尺寸与速度必须为正")
        self.cfg, self.log = cfg, EventLog(output)
        self.main = MainDrone(self.log, cfg)
        self.relays = [RelayDrone(i, self.log, cfg) for i in (1, 2)]
        self.route: Route = ()
        self.route_changes = []
        self.now = 0.0
        self.steps = 0
        self.min_rssi = math.inf
        self.link_wait = False
        self.controller = RouteController(service=cfg.relay_threshold_dbm, disconnect=cfg.disconnect_dbm,
                                          hysteresis=cfg.hysteresis_db)

    def route_rssi(self, route: Route) -> float:
        if any(not self.relays[i - 1].ready for i in route):
            return -math.inf
        points = [self.cfg.home] + [self.relays[i - 1].position for i in route] + [self.main.position]
        # 多跳质量取最弱一跳，不能只看最后一跳或累加 dBm。
        return min(link_rssi(a, b, self.cfg) for a, b in zip(points, points[1:]))

    def _set_route(self, route: Route) -> None:
        old = self.route
        if old == route:
            return
        for index in route:
            self.relays[index - 1].activate(self.now)
        if old == (1,) and route == (1, 2):
            self.relays[0].handover(self.now, False, "末跳业务交给中继2，保持地面站至中继2的上游转发")
        self.route = route
        self.route_changes.append((self.now, old, route))
        if self.log.output:
            self.log.output(f"[{self.now:6.1f}s] 路由：{self.route_name(old)} → {self.route_name(route)}，最弱跳 {self.route_rssi(route):.1f} dBm")

    @staticmethod
    def route_name(route: Route) -> str:
        return "地面站→" + "".join(f"中继{i}→" for i in route) + "主机"

    def coordinate(self) -> bool:
        if self.main.landed:
            # 先释放最远端；上游等下游返场，避免撤走仍被依赖的中继。
            self._set_route(())
            self.controller.active, self.controller.pending = (), None
            for relay in reversed(self.relays):
                if relay.state == RelayState.STANDBY or relay.home_reached:
                    continue
                relay.handover(self.now, True, "主机已降落，业务交回上游，按下游优先顺序返场")
                break
            return True
        nodes = (self.cfg.home, *(r.position for r in self.relays), self.main.position)
        measured = {link: link_rssi(nodes[a], nodes[b], self.cfg)
                    for link, (a, b) in zip(LINKS, ENDPOINTS)}
        ready = tuple(r.ready for r in self.relays)
        desired = self.controller.propose(self.now, measured, ready)
        self.controller.request(desired, self.now)
        self.controller.advance(self.now, measured, ready)
        if self.controller.pending:
            self.controller.acknowledge(self.controller.pending.epoch, self.now, measured, ready)
        self._set_route(self.controller.active)
        current = self.route_rssi(self.route)
        if current < self.cfg.relay_threshold_dbm:
            for relay in self.relays:
                if relay.state == RelayState.STANDBY:
                    relay.launch(self.now, f"当前链路 {current:.1f} dBm < -75 dBm，前出至 {relay.station}")
                    break
                if not relay.ready:
                    break
            self.link_wait = True
        elif current >= self.cfg.relay_threshold_dbm + self.cfg.hysteresis_db:
            self.link_wait = False
        # 处于回差带但已经完成路由切换时，可以继续，不必无限等待达到 -72。
        if self.link_wait and current >= self.cfg.disconnect_dbm and all(r.ready for r in self.relays):
            self.link_wait = False
        return not self.link_wait

    def step(self, forced_rssi: Optional[float] = None) -> None:
        self.steps += 1
        self.now = round(self.steps * self.cfg.dt_s, 10)
        for relay in self.relays:
            relay.tick(self.now)
        allow_motion = self.coordinate()
        rssi = self.route_rssi(self.route) if forced_rssi is None else forced_rssi
        if not finite_number(rssi):
            rssi = -math.inf
        self.min_rssi = min(self.min_rssi, rssi)
        self.main.tick(self.now, rssi, allow_motion)

    @property
    def finished(self) -> bool:
        return self.main.landed and all(r.home_reached or r.state == RelayState.STANDBY for r in self.relays)

    def run(self, realtime: bool = False, fault: Optional[Callable[[float], Optional[float]]] = None) -> None:
        while not self.finished and self.now < self.cfg.max_sim_s:
            start = time.monotonic()
            self.step(fault(self.now + self.cfg.dt_s) if fault else None)
            if realtime:
                time.sleep(max(0.0, self.cfg.dt_s - (time.monotonic() - start)))
        if not self.finished:
            raise RuntimeError(f"超过 {self.cfg.max_sim_s:g} s 尚未完成；状态={self.main.state.name}，路由={self.route}")


def main() -> None:
    if "--legacy" in sys.argv[1:]:
        snapshot = Path(__file__).resolve().with_name("legacy_v1") / "demo.py"
        print("B legacy_v1 snapshot: 2 m/cell, 3 s dwell; archive-only, not warehouse-abc-v3.")
        completed = subprocess.run([sys.executable, str(snapshot), *sys.argv[1:]],
                                   cwd=str(snapshot.parent))
        raise SystemExit(completed.returncode)
    from ideal_sim.cli import main as ideal_main
    raise SystemExit(ideal_main(sys.argv[1:]))


if __name__ == "__main__":
    main()
