"""真值地图与在线地图分开保存，调度器只接收已观测地图。格点0基，货箱1基。"""
import math
from .contract import OUTWARD, REFERENCE_DISTANCE_M, finite_number


def outward_route():
    return OUTWARD


def length(route):
    return sum(math.dist(a, b) for a, b in zip(route, route[1:]))


def at_distance(route, distance):
    for a, b in zip(route, route[1:]):
        size = math.dist(a, b)
        if distance <= size:
            f = max(0., distance) / size
            return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
        distance -= size
    return route[-1]


def sample_route(route, step):
    total = length(route)
    return tuple(at_distance(route, min(i * step, total))
                 for i in range(math.ceil(total / step) + 1))


def intersects_cell(a, b, cell):
    """仅射线进入货箱格开集内部才算遮挡；沿格线或触角不计。"""
    enter, leave = 0., 1.
    for axis in (0, 1):
        low, high = cell[axis] - 1, cell[axis]
        delta = b[axis] - a[axis]
        if abs(delta) < 1e-10:
            if not low + 1e-9 < a[axis] < high - 1e-9:
                return False
        else:
            t0, t1 = (low - a[axis]) / delta, (high - a[axis]) / delta
            enter, leave = max(enter, min(t0, t1)), min(leave, max(t0, t1))
    return leave - enter > 1e-9


def hop_rssi(a, b, occupied, cfg):
    if not all(finite_number(v) for point in (a, b) for v in point):
        return -math.inf
    distance = max(math.dist(a, b) * cfg.cell_m, REFERENCE_DISTANCE_M)
    return (cfg.p0_dbm - 10 * cfg.path_loss_n * math.log10(distance)
            - cfg.shelf_loss_db * sum(intersects_cell(a, b, cell) for cell in occupied))


class WarehouseMap:
    def __init__(self, cfg):
        self.cfg = cfg
        self.nodes = tuple((float(x), float(y)) for x in range(4) for y in range(5))
        self.cells = tuple((x, y) for x in range(1, 4) for y in range(1, 5))
        self.edges = tuple((a, b) for i, a in enumerate(self.nodes) for b in self.nodes[i + 1:]
                           if math.dist(a, b) == 1)
        self.truth = set(self.cells)
        # 理想初始建图：首次巡线前所有节点、航段与初始货箱都已知。
        self.known = set(self.cells)
        self.initial = frozenset(self.cells)
        self.version = 0
        self.last_observed = {}
        self.node_samples = {}

    def visible(self, position, cell):
        center = (cell[0] - .5, cell[1] - .5)
        return math.dist(position, center) <= self.cfg.sensor_range_cells + 1e-9

    def observable_ahead(self, route, progress):
        remaining = length(route) - progress
        step = self.cfg.main_speed_mps * self.cfg.dt_s / self.cfg.cell_m
        # 与下一次实际运动/观测采样对齐；仅当前可见但下一步已错过的不算。
        samples = (at_distance(route, progress + min(i * step, remaining))
                   for i in range(1, math.ceil(remaining / step) + 1))
        result = set()
        for point in samples:
            result.update(cell for cell in self.truth if self.visible(point, cell))
        return result

    def observe(self, position, now):
        changed = []
        for cell in self.cells:
            if not self.visible(position, cell):
                continue
            self.last_observed[cell] = now
            if (cell in self.known) != (cell in self.truth):
                if cell in self.truth:
                    self.known.add(cell)
                else:
                    self.known.remove(cell)
                changed.append(cell)
        if changed:
            self.version += 1
        nearest = min(self.nodes, key=lambda n: math.dist(n, position))
        if math.dist(nearest, position) < 1e-8:
            self.node_samples[nearest] = {
                "sample_time_s": now, "map_version": self.version,
                "direct_rssi_dbm": hop_rssi((0., 0.), position, self.truth, self.cfg)}
        return changed

    def snapshot(self):
        return {"version": self.version, "nodes": self.nodes, "edges": self.edges,
                "occupied": sorted(self.known), "observed_cells": len(self.last_observed),
                "node_samples": [{"node": n, **v} for n, v in sorted(self.node_samples.items())]}
