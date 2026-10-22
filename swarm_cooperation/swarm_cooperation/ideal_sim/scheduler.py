"""可替换调度层：理想地图枚举基线 + AI 候选评分接入口。

模型只收到 SchedulingState 和 Candidate；两者不含真值地图或未来移箱事件。
后续可在模型内部接入 PyTorch / ONNX / NPU，归一化、窗口与输出版本由模型负责。
核心始终检查候选索引、分数有限性和断链/弱链路优先约束。
"""
from dataclasses import asdict, dataclass
import importlib
import itertools
import math
from typing import Optional, Protocol, Tuple

from .world import hop_rssi, sample_route
from relay_learning.routing import ROUTE_LINKS, path_quality

Point = Tuple[float, float]
Route = Tuple[int, ...]


@dataclass(frozen=True)
class SchedulingState:
    schema_version: str
    time_s: float
    round_index: int
    direction: str
    map_version: int
    known_occupied: Tuple[Tuple[int, int], ...]
    main_position: Point
    relay_positions: Tuple[Point, Point]
    planned_main_positions: Tuple[Point, ...]
    # [time_s, direct, ground-R1, ground-R2, R1-main, R2-main, R1-R2]
    link_history: Tuple[Tuple[float, ...], ...] = ()
    relay_battery: Tuple[float, float] = (1., 1.)
    relay_ready: Tuple[bool, bool] = (False, False)
    route_progress_cells: float = 0.
    active_route: Tuple[int, ...] = ()
    measured_links: Tuple[Tuple[str, float], ...] = ()
    # Motion is visible to planning: a relay in TAKEOFF/MOVING/SETTLING is not
    # available for tasking and must not appear in a candidate's station tuple.
    relay_status: Tuple[str, str] = ()
    relay_eta_s: Tuple[float, float] = (0., 0.)


@dataclass(frozen=True)
class Candidate:
    candidate_id: int
    stations: Tuple[Optional[Point], Optional[Point]]
    outage_samples: int
    weak_samples: int
    relay_hop_samples: int
    movement_m: float
    switches: int
    deployment_s: float = 0.
    feasible: bool = True
    rejection_reason: str = ""

    @property
    def cost(self):
        return (self.outage_samples, self.weak_samples, self.relay_hop_samples,
                self.movement_m, self.switches, self.candidate_id)


class SchedulingModel(Protocol):
    """一次规划返回每个候选的标量代价，越小越好；不能返回飞控命令。

可以组合“多跳预测网络 + 候选评分网络”，或只实现其中一层。
加载/预热在 factory 中完成；score 不负责运动、地图更新或随机环境。
应由后端实现推理时限；抛出 TimeoutError 会触发 CPU 基线回退。
"""
    def score(self, state: SchedulingState, candidates: Tuple[Candidate, ...]):
        ...


def load_model(spec):
    if not spec:
        return None
    module, separator, factory = spec.partition(":")
    if not separator or not module or not factory:
        raise ValueError("模型插件格式为 python.module:factory")
    model = getattr(importlib.import_module(module), factory)()
    if not callable(getattr(model, "score", None)):
        raise TypeError("模型必须实现 score(state, candidates)")
    return model


def routes_for(stations):
    active = tuple(i + 1 for i, station in enumerate(stations) if station is not None)
    return ((),) + tuple((i,) for i in active) + tuple(itertools.permutations(active, 2))


def select_route(qualities, cfg):
    # 理想、无迟滞的全候选当前探测；优先保业务，再减少承担转发的中继数量。
    return min(qualities, key=lambda r: (qualities[r] < cfg.disconnect_dbm,
                                        qualities[r] < cfg.service_dbm,
                                        len(r), -qualities[r], r))


def route_qualities(position, stations, occupied, cfg):
    result = {}
    for route in routes_for(stations):
        points = [(0., 0.)] + [stations[i - 1] for i in route] + [position]
        result[route] = min(hop_rssi(a, b, occupied, cfg) for a, b in zip(points, points[1:]))
    return result


def evaluate(stations, positions, occupied, cfg):
    metrics = dict(outage_samples=0, weak_samples=0, relay_hop_samples=0, switches=0)
    previous = ()
    for position in positions:
        values = route_qualities(position, stations, occupied, cfg)
        route = select_route(values, cfg)
        metrics["outage_samples"] += values[route] < cfg.disconnect_dbm
        metrics["weak_samples"] += values[route] < cfg.service_dbm
        metrics["relay_hop_samples"] += len(route)
        metrics["switches"] += route != previous
        previous = route
    return metrics


class CandidateScheduler:
    def __init__(self, cfg, nodes, model=None):
        self.cfg, self.nodes, self.model = cfg, nodes, model
        self.last_fallback = None
        self.last_candidates = ()
        self.last_scores = None

    def plan(self, state):
        cfg = self.cfg
        positions = state.planned_main_positions
        occupied = frozenset(state.known_occupied)
        # 物理跳只计算一次：之后枚举最多421个合法站位分配。
        cache = {}

        def hop(a, b):
            key = tuple(sorted((a, b)))
            if key not in cache:
                cache[key] = hop_rssi(a, b, occupied, cfg)
            return cache[key]

        assignments = [(None, None)]
        assignments += [(node, None) for node in self.nodes]
        assignments += [(None, node) for node in self.nodes]
        assignments += list(itertools.permutations(self.nodes, 2))
        candidates = []
        measured = dict(state.measured_links)
        statuses = state.relay_status or tuple("READY" if ready else "STANDBY"
                                               for ready in state.relay_ready)
        # A relay that has been commanded to move is not taskable until READY.
        # Candidates see None for it instead of its old or target position; this
        # is what prevents a moving relay from being used as a valid link.
        in_transit = tuple(status in ("TAKEOFF", "MOVING", "SETTLING") for status in statuses)
        protected = {
            relay for relay in state.active_route
            if not any(relay not in route and path_quality(route, measured, state.relay_ready) >= cfg.service_dbm
                       for route in ROUTE_LINKS)
        }
        for index, stations in enumerate(assignments):
            # The plan may still contain a future station for a standby relay,
            # but never for one that is already moving.
            stations = tuple(None if in_transit[i] else stations[i] for i in range(2))
            routes = routes_for(stations)
            backbones = {}
            for route in routes:
                points = [(0., 0.)] + [stations[i - 1] for i in route]
                backbones[route] = min((hop(a, b) for a, b in zip(points, points[1:])), default=math.inf)
            outage, weak, usage, switches, previous = 0, 0, 0, 0, ()
            for point in positions:
                values = {route: min(backbones[route], hop(stations[route[-1] - 1] if route else (0., 0.), point))
                          for route in routes}
                route = select_route(values, cfg)
                outage += values[route] < cfg.disconnect_dbm
                weak += values[route] < cfg.service_dbm
                usage += len(route)
                switches += previous != route
                previous = route
            movement = sum((abs(old[0] - (new or (0., 0.))[0])
                            + abs(old[1] - (new or (0., 0.))[1])) * cfg.cell_m
                           for old, new in zip(state.relay_positions, stations))
            distances = [sum(abs(a - b) for a, b in zip(old, new or (0., 0.))) * cfg.cell_m
                         for old, new in zip(state.relay_positions, stations)]
            deployment = sum(d / cfg.relay_speed_mps + (cfg.settle_s if d > 0 else 0.)
                             + (cfg.takeoff_s if old == (0., 0.) and d > 0 else 0.)
                             for d, old in zip(distances, state.relay_positions))
            battery_ok = all(
                battery - (d + sum(new or (0., 0.)) * cfg.cell_m) * cfg.battery_per_m
                - deployment * cfg.battery_per_s >= cfg.battery_reserve
                for d, new, battery in zip(distances, stations, state.relay_battery))
            dependency_ok = not any(
                stations[i - 1] != state.relay_positions[i - 1] for i in protected)
            feasible = battery_ok and dependency_ok
            reason = "battery reserve" if not battery_ok else "active upstream dependency" if not dependency_ok else ""
            candidates.append(Candidate(index, stations, outage, weak, usage, movement, switches,
                                        deployment, feasible, reason))
        self.last_candidates = tuple(candidates)
        self.last_fallback, self.last_scores = None, None
        feasible = [c for c in candidates if c.feasible]
        if not feasible:
            raise RuntimeError("No plan satisfies battery and active-route dependencies")
        baseline = min(feasible, key=lambda c: c.cost)
        if self.model is None:
            return baseline
        try:
            scores = tuple(float(v) for v in self.model.score(state, self.last_candidates))
            if len(scores) != len(candidates) or not all(math.isfinite(v) for v in scores):
                raise ValueError("模型分数数量不匹配或包含NaN/Inf")
            self.last_scores = scores
            eligible = [c for c in feasible if c.cost[:2] == baseline.cost[:2]]
            return min(eligible, key=lambda c: (scores[c.candidate_id], c.cost))
        except Exception as exc:
            self.last_fallback = f"{type(exc).__name__}: {exc}"
            return baseline

    @staticmethod
    def record(state, chosen, fallback):
        return {"state": asdict(state), "chosen": asdict(chosen), "model_fallback": fallback}
