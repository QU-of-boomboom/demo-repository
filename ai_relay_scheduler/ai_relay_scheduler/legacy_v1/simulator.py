"""统一链路环境。两策略只读取同一轨迹的当前/历史观测，不能修改环境。"""
from __future__ import annotations

import runtime_compat
import numpy as np


def build_route(c):
    """沿竖格线蛇形巡检，补齐至右上终点；回程严格按同一路线逆序。"""
    s, cols, rows = c["cell_size_m"], c["columns"], c["rows"]
    points = [(0.0, 0.0)]
    for col in range(cols + 1):
        points.append((col * s, (rows if col % 2 == 0 else 0) * s))
        if col < cols:
            points.append(((col + 1) * s, points[-1][1]))
    terminal = (cols * s, rows * s)
    if points[-1] != terminal:
        points.append(terminal)
    return np.array(points, dtype=np.float64)


def crossed_shelf_columns(a, b, c):
    """线段进入货架开集内部才计遮挡，同列只计一次；沿格线不计。

    向量化 Liang–Barsky 区间求交，与现有 warehouse_relay_sim 的几何定义一致。
    """
    a, b = np.broadcast_arrays(np.asarray(a, dtype=float), np.asarray(b, dtype=float))
    a, b = np.atleast_2d(a), np.atleast_2d(b)
    delta = b - a
    counts = np.zeros(len(a), dtype=np.int16)
    s, eps = c["cell_size_m"], 1e-9
    for col in range(c["columns"]):
        column_hit = np.zeros(len(a), dtype=bool)
        for row in range(c["rows"]):
            enter, leave = np.zeros(len(a)), np.ones(len(a))
            valid = np.ones(len(a), dtype=bool)
            for axis, lower in ((0, col * s), (1, row * s)):
                upper = lower + s
                stationary = np.abs(delta[:, axis]) < eps
                valid &= ~stationary | ((a[:, axis] > lower + eps) & (a[:, axis] < upper - eps))
                denom = np.where(stationary, 1.0, delta[:, axis])
                t0, t1 = (lower - a[:, axis]) / denom, (upper - a[:, axis]) / denom
                enter = np.maximum(enter, np.where(stationary, -np.inf, np.minimum(t0, t1)))
                leave = np.minimum(leave, np.where(stationary, np.inf, np.maximum(t0, t1)))
            column_hit |= valid & (leave - enter > eps)
        counts += column_hit
    return counts


def hop_rssi(a, b, shelf_loss, c):
    a, b = np.asarray(a), np.asarray(b)
    distance = np.maximum(np.linalg.norm(b - a, axis=-1), c["reference_distance_m"])
    return (c["p0_dbm"] - 10 * c["path_loss_exponent"] * np.log10(distance / c["reference_distance_m"])
            - shelf_loss * crossed_shelf_columns(a, b, c))


def trajectory(rng, c, randomized=True):
    route = build_route(c)
    lengths = np.linalg.norm(np.diff(route, axis=0), axis=1)
    knots = np.r_[0.0, np.cumsum(lengths)]
    length = knots[-1]
    dt, v = c["dt_s"], c["nominal_speed_m_s"]
    refresh = round(c["speed_refresh_s"] / dt)
    # 每航程独立速度序列，每秒更新一次，所有完整采样间隔的设定速度均在 ±30% 内。
    cap = int(np.ceil(2 * length / (v * (1 - c["speed_variation"]) * dt))) + 10
    commands = v * (rng.uniform(1 - c["speed_variation"], 1 + c["speed_variation"], cap // refresh + 2)
                    if randomized else np.ones(cap // refresh + 2))
    commands = np.repeat(commands, refresh)
    progress, speeds = [0.0], [commands[0]]
    for target in (length, 2 * length):
        while progress[-1] < target - 1e-9:
            speed = commands[len(progress) - 1]
            nxt = min(progress[-1] + speed * dt, target)
            speeds.append((nxt - progress[-1]) / dt)
            progress.append(nxt)
    # 到达终点/回家时最后一个网格采样允许减速，确保坐标精确、时间步长固定。
    progress = np.asarray(progress)
    one_way = np.minimum(progress, 2 * length - progress)
    index = np.minimum(np.searchsorted(knots, one_way, side="right") - 1, len(lengths) - 1)
    ratio = (one_way - knots[index]) / lengths[index]
    xy = route[index] + ratio[:, None] * (route[index + 1] - route[index])
    return xy, np.asarray(speeds), progress


def features(direct, xy, speed, c):
    """历史含当前，共20点；起飞不足20点用首个已知RSSI左填充，不使用未来。"""
    h = c["history_steps"]
    history = np.lib.stride_tricks.sliding_window_view(np.pad(direct, (h - 1, 0), mode="edge"), h)
    distances = np.linalg.norm(xy[:, None, :] - np.asarray(c["relay_xy"])[None, :, :], axis=-1)
    return np.column_stack((history, xy, speed, distances)).astype(np.float32)


def future_labels(direct, c):
    """y[t] = min(RSSI[t+1:t+16])<-75；只保留拥有完整15步未来的样本。"""
    horizon = round(c["forecast_s"] / c["dt_s"])
    future = np.lib.stride_tricks.sliding_window_view(direct[1:], horizon)
    return (future.min(axis=1) < c["relay_threshold_dbm"]).astype(np.uint8)


def simulate_voyage(voyage_id, c, stream="data", randomized=True):
    # 数据、200航程评估、统计bootstrap使用独立随机流；同一id可单独复现。
    seed = c["data_seed"] if stream == "data" else c["evaluation_seed"]
    rng = np.random.default_rng(np.random.SeedSequence([seed, int(voyage_id)]))
    shelf_loss = rng.uniform(*c["shelf_loss_range_db"]) if randomized else c["nominal_shelf_loss_db"]
    sigma = rng.uniform(*c["noise_sigma_range_db"]) if randomized else 0.0
    xy, speed, progress = trajectory(rng, c, randomized)
    n = len(xy)
    base = np.asarray(c["base_xy"])
    direct = hop_rssi(base, xy, shelf_loss, c) + rng.normal(0, sigma, n)
    relayed = []
    for relay in c["relay_xy"]:
        access = hop_rssi(relay, xy, shelf_loss, c) + rng.normal(0, sigma, n)
        backhaul = hop_rssi(base, relay, shelf_loss, c) + rng.normal(0, sigma, n)
        # 端到端双跳取较弱跳，6dB增益只计一次。
        relayed.append(np.minimum(access, backhaul) + c["relay_gain_db"])
    return dict(voyage_id=int(voyage_id), xy=xy, speed=speed, progress=progress,
                time=np.arange(n) * c["dt_s"], links=np.column_stack((direct, *relayed)),
                x=features(direct, xy, speed, c), y=future_labels(direct, c),
                shelf_loss_db=float(shelf_loss), sigma_db=float(sigma))
