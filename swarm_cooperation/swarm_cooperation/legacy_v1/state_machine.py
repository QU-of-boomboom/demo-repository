"""纯 Python 协作状态机。所有计时由调用者的单调时间驱动，无线程和阻塞等待。"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, List, Optional

from config import CONFIG, Config, Point
from geometry import move_towards, snake_path


class MainState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    SNAKE_SURVEY = auto()
    PICK = auto()
    RETURN = auto()
    LAND = auto()
    FAILSAFE = auto()


class RelayState(Enum):
    STANDBY = auto()
    HOVER = auto()
    RELAYING = auto()
    HANDOVER = auto()
    RETURN = auto()


@dataclass(frozen=True)
class Transition:
    time_s: float
    drone: str
    old: str
    new: str
    reason: str


class EventLog:
    def __init__(self, output: Optional[Callable[[str], None]] = print):
        self.events: List[Transition] = []
        self.output = output

    def emit(self, now: float, drone: str, old: Enum, new: Enum, reason: str) -> None:
        event = Transition(now, drone, old.name, new.name, reason)
        self.events.append(event)
        if self.output:
            self.output(f"[{now:6.1f}s] {drone:8s} {old.name:12s} → {new.name:12s} | {reason}")


class MainDrone:
    def __init__(self, log: EventLog, cfg: Config = CONFIG):
        self.cfg, self.log = cfg, log
        self.state = MainState.IDLE
        self.position = cfg.home
        self.path = snake_path(cfg)
        self.next_waypoint = 1
        self.history: List[Point] = [self.position]
        self.return_trace: List[Point] = []
        self.elapsed = 0.0
        self.saved_state = MainState.IDLE
        self.saved_elapsed = 0.0
        self.failsafe_count = 0
        self.emergency_return = False
        self.picked = False
        self.landed = False

    def transition(self, state: MainState, now: float, reason: str) -> None:
        if state != self.state:
            self.log.emit(now, "主机", self.state, state, reason)
            self.state, self.elapsed = state, 0.0

    def begin_return(self, now: float, reason: str) -> None:
        # 记录的每个实际位置均在格线上；逆序弹栈，禁止用直线 RTL 穿过货架。
        self.transition(MainState.RETURN, now, reason)

    def tick(self, now: float, rssi: float, allow_motion: bool = True) -> None:
        if self.landed:
            return
        if rssi < self.cfg.disconnect_dbm and self.state != MainState.FAILSAFE and not self.emergency_return:
            self.saved_state, self.saved_elapsed = self.state, self.elapsed
            self.failsafe_count += 1
            self.transition(MainState.FAILSAFE, now, f"链路 {rssi:.1f} dBm < {self.cfg.disconnect_dbm:g} dBm")
            return
        self.elapsed += self.cfg.dt_s
        if self.state == MainState.FAILSAFE:
            if rssi >= self.cfg.disconnect_dbm + self.cfg.hysteresis_db:
                self.transition(self.saved_state, now, "链路恢复，满足 3 dB 回差，继续中断的阶段")
                self.elapsed = self.saved_elapsed
            elif self.elapsed + 1e-9 >= self.cfg.failsafe_hold_s:
                # 同一次持续断链只悬停一次，否则 RETURN 会反复进入 FAILSAFE，无法回家。
                self.emergency_return = True
                self.begin_return(now, "悬停 5 s 未恢复，锁定应急原路返航")
            return
        if self.state == MainState.IDLE:
            self.transition(MainState.TAKEOFF, now, "任务开始")
        elif self.state == MainState.TAKEOFF:
            if self.elapsed + 1e-9 >= self.cfg.takeoff_s:
                self.transition(MainState.SNAKE_SURVEY, now, "起飞完成，开始格线蛇形巡检")
        elif self.state == MainState.SNAKE_SURVEY and allow_motion:
            target = self.path[self.next_waypoint]
            self.position = move_towards(self.position, target, self.cfg.main_speed_mps * self.cfg.dt_s / self.cfg.cell_m)
            if self.position != self.history[-1]:
                self.history.append(self.position)
            if self.position == target:
                self.next_waypoint += 1
                if self.next_waypoint == len(self.path):
                    self.transition(MainState.PICK, now, "抵达 (3,4)，悬停取货")
        elif self.state == MainState.PICK:
            if self.elapsed + 1e-9 >= self.cfg.pick_s:
                self.picked = True
                self.begin_return(now, "悬停 3 s 完成，模拟夹爪确认取货")
        elif self.state == MainState.RETURN and (allow_motion or self.emergency_return):
            if len(self.history) > 1:
                self.history.pop()
                self.position = self.history[-1]
                self.return_trace.append(self.position)
            if len(self.history) == 1:
                self.transition(MainState.LAND, now, "沿已飞轨迹返回 (0,0)，开始降落")
        elif self.state == MainState.LAND:
            if self.elapsed + 1e-9 >= self.cfg.land_s:
                self.landed = True


class RelayDrone:
    def __init__(self, index: int, log: EventLog, cfg: Config = CONFIG):
        self.index, self.cfg, self.log = index, cfg, log
        self.state = RelayState.STANDBY
        self.position = cfg.home
        self.station = cfg.relay_positions[index - 1]
        self.elapsed = 0.0
        self.waypoints: List[Point] = []
        self.ready = False
        self.home_reached = False
        self.handover_returns = False

    def transition(self, state: RelayState, now: float, reason: str) -> None:
        if state != self.state:
            self.log.emit(now, f"中继{self.index}", self.state, state, reason)
            self.state, self.elapsed = state, 0.0

    def launch(self, now: float, reason: str) -> None:
        if self.state == RelayState.STANDBY:
            self.waypoints = [(self.cfg.home[0], self.station[1]), self.station]
            self.transition(RelayState.HOVER, now, reason)

    def activate(self, now: float) -> None:
        if self.ready and self.state == RelayState.HOVER:
            self.transition(RelayState.RELAYING, now, "已就位且上游可达，提交转发路径")

    def handover(self, now: float, returns: bool, reason: str) -> None:
        if self.state in (RelayState.RELAYING, RelayState.HOVER):
            self.handover_returns = returns
            self.transition(RelayState.HANDOVER, now, reason)

    def tick(self, now: float) -> None:
        self.elapsed += self.cfg.dt_s
        if self.state == RelayState.HOVER:
            if self.elapsed + 1e-9 >= self.cfg.takeoff_s:
                self._move()
                self.ready = not self.waypoints
        elif self.state == RelayState.HANDOVER and self.elapsed + 1e-9 >= self.cfg.handover_s:
            if self.handover_returns:
                self.ready = False
                self.waypoints = [(self.cfg.home[0], self.station[1]), self.cfg.home]
                self.transition(RelayState.RETURN, now, "交接确认完成，下游不再依赖本机")
            else:
                self.transition(RelayState.RELAYING, now, "交接完成，继续保留为上游骨干中继")
        elif self.state == RelayState.RETURN:
            self._move()
            if self.waypoints:
                self.elapsed = 0.0
            elif self.elapsed + 1e-9 >= self.cfg.land_s:
                self.home_reached = True

    def _move(self) -> None:
        if self.waypoints:
            self.position = move_towards(self.position, self.waypoints[0], self.cfg.relay_speed_mps * self.cfg.dt_s / self.cfg.cell_m)
            if self.position == self.waypoints[0]:
                self.waypoints.pop(0)
