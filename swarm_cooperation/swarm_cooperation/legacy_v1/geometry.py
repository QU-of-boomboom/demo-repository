"""格线航路和可替换的简化 RSSI 模型。"""

import math
from typing import List

from config import CONFIG, Config, Point


def snake_path(cfg: Config = CONFIG) -> List[Point]:
    """沿竖格线蛇形巡检，完整覆盖后补飞至右上终点；坐标单位为格。"""
    points = [(0.0, 0.0)]
    for col in range(cfg.columns + 1):
        points.append((float(col), float(cfg.rows) if col % 2 == 0 else 0.0))
        if col < cfg.columns:
            points.append((float(col + 1), points[-1][1]))
    terminal = (float(cfg.columns), float(cfg.rows))
    if points[-1] != terminal:
        points.append(terminal)
    return points


def move_towards(a: Point, b: Point, step_cells: float) -> Point:
    distance = math.dist(a, b)
    if distance <= step_cells + 1e-9:
        return b
    scale = step_cells / distance
    return (a[0] + (b[0] - a[0]) * scale, a[1] + (b[1] - a[1]) * scale)


def shelf_columns(a: Point, b: Point, cfg: Config = CONFIG) -> int:
    """穿过 x=0.5、1.5、2.5 的列屏障计数；这是演示模型，不是射线追踪。"""
    low, high = sorted((a[0], b[0]))
    return sum(low < x + 0.5 <= high for x in range(cfg.columns))


def link_rssi(a: Point, b: Point, cfg: Config = CONFIG) -> float:
    distance = max(cfg.reference_distance_m, math.dist(a, b) * cfg.cell_m)
    return (cfg.p0_dbm - 10 * cfg.path_loss_n * math.log10(distance / cfg.reference_distance_m)
            - cfg.shelf_loss_db * shelf_columns(a, b, cfg))
