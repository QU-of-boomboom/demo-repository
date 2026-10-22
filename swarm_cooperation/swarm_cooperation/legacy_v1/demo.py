"""默认五轮理想实验；--legacy 保留原单轮协议/状态机演示。"""

import argparse
import math
import time
import sys
from typing import Callable, Optional, Tuple

from config import CONFIG, Config
from geometry import link_rssi
from state_machine import EventLog, MainDrone, MainState, RelayDrone, RelayState

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
            for relay in reversed(self.relays):
                if relay.state == RelayState.STANDBY or relay.home_reached:
                    continue
                relay.handover(self.now, True, "主机已降落，业务交回上游，按下游优先顺序返场")
                break
            return True
        candidates = [(), (1,), (1, 2)]
        values = {route: self.route_rssi(route) for route in candidates}
        current = values[self.route]
        # 3 dB 回差：低于 -75 请求扩展；较短路径达到 -72 才允许回切。
        strong = [route for route in candidates if len(route) < len(self.route)
                  and values[route] >= self.cfg.relay_threshold_dbm + self.cfg.hysteresis_db]
        if strong:
            self._set_route(min(strong, key=len))
        elif current < self.cfg.relay_threshold_dbm:
            best = max(candidates, key=lambda route: values[route])
            if values[best] >= current + self.cfg.hysteresis_db:
                self._set_route(best)
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
        if self.link_wait and current >= self.cfg.relay_threshold_dbm and all(r.ready for r in self.relays):
            self.link_wait = False
        return not self.link_wait

    def step(self, forced_rssi: Optional[float] = None) -> None:
        self.steps += 1
        self.now = round(self.steps * self.cfg.dt_s, 10)
        for relay in self.relays:
            relay.tick(self.now)
        allow_motion = self.coordinate()
        rssi = self.route_rssi(self.route) if forced_rssi is None else forced_rssi
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
    if "--legacy" not in sys.argv[1:]:
        from ideal_sim.cli import main as ideal_main
        raise SystemExit(ideal_main(sys.argv[1:]))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy", action="store_true")
    parser.add_argument("--realtime", action="store_true", help="每步按真实时间 0.1 s 执行")
    args = parser.parse_args()
    sim = SwarmSimulation()
    sim.run(realtime=args.realtime)
    assert sim.main.state == MainState.LAND and sim.main.landed
    assert sim.main.picked and sim.main.failsafe_count == 0
    assert len(sim.log.events) >= 6
    assert all(relay.home_reached for relay in sim.relays)
    print(f"\n验收通过：主机 LAND，取货完成，两中继返场，状态迁移 {len(sim.log.events)} 次，"
          f"FAILSAFE {sim.main.failsafe_count} 次，最低链路 {sim.min_rssi:.1f} dBm，模拟耗时 {sim.now:.1f} s。")


if __name__ == "__main__":
    main()
