"""格线航路和可替换的简化 RSSI 模型。"""

import math
from typing import List

from config import CONFIG, Config, Point
from ideal_sim.contract import OUTWARD, OCCUPIED
from ideal_sim.world import hop_rssi, intersects_cell


def snake_path(cfg: Config = CONFIG) -> List[Point]:
    """沿竖格线蛇形巡检，完整覆盖后补飞至右上终点；坐标单位为格。"""
    if (cfg.columns, cfg.rows) != (3, 4):
        raise ValueError("The shared route requires a 3 by 4 map")
    return list(OUTWARD)


def move_towards(a: Point, b: Point, step_cells: float) -> Point:
    distance = math.dist(a, b)
    if distance <= step_cells + 1e-9:
        return b
    scale = step_cells / distance
    return (a[0] + (b[0] - a[0]) * scale, a[1] + (b[1] - a[1]) * scale)


def shelf_columns(a: Point, b: Point, cfg: Config = CONFIG) -> int:
    """Compatibility name: now counts occupied CELL interiors, like A/B/C."""
    return sum(intersects_cell(a, b, cell) for cell in OCCUPIED)


def link_rssi(a: Point, b: Point, cfg: Config = CONFIG) -> float:
    return hop_rssi(a, b, OCCUPIED, cfg)
