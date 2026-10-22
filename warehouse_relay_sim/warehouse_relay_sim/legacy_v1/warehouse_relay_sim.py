#!/usr/bin/env python3
"""无人机中继巡检仿真：米、秒、dBm；仅绘图依赖 matplotlib。

python warehouse_relay_sim.py
python warehouse_relay_sim.py --endpoint lower-right --output-dir demo_39s

严格蛇形遍历的落点是 (3,0)。默认额外上行 4m 到指定终点 (3,4)，
往返飞行 46s + 取货停留 1s = 47s；lower-right 模式为 39s。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

Point = tuple[float, float]
BASE: Point = (0.0, 0.0)
RELAYS: dict[str, Point] = {"relay_1": (1.0, 2.0), "relay_2": (2.0, 4.0)}
MODE_NAMES = {"direct": "直连", "relay_1": "中继1", "relay_2": "中继2"}
EPS = 1e-9


@dataclass(frozen=True)
class Config:
    dt_s: float = 0.1
    speed_m_s: float = 1.0
    pickup_dwell_s: float = 1.0
    p0_dbm: float = -40.0
    path_loss_exponent: float = 2.2
    shelf_loss_db: float = 12.0
    relay_gain_db: float = 6.0
    relay_threshold_dbm: float = -75.0
    disconnect_threshold_dbm: float = -88.0
    hysteresis_db: float = 3.0
    seed: int = 7
    endpoint: str = "upper-right"


@dataclass
class Sample:
    time_s: float
    x_m: float
    y_m: float
    rssi_dbm: float
    mode: str
    direct_dbm: float
    relay_1_dbm: float
    relay_2_dbm: float
    phase: str
    distance_m: float


@dataclass
class Event:
    time_s: float
    kind: str
    message: str
    details: dict


def segment_enters_cell(a: Point, b: Point, col: int, row: int) -> bool:
    """判断线段是否进入货架格的开集内部；沿格线或仅触碰角点不算。

    格坐标为 1-based：(col,row) 对应 (col-1,col) × (row-1,row)。
    用线段参数区间求交，避免按 x 坐标差估算导致地面边界也算穿架。
    """
    enter, leave = 0.0, 1.0
    for start, finish, lo, hi in (
        (a[0], b[0], col - 1, col), (a[1], b[1], row - 1, row)
    ):
        delta = finish - start
        if abs(delta) < EPS:
            if not lo + EPS < start < hi - EPS:
                return False
        else:
            t0, t1 = sorted(((lo - start) / delta, (hi - start) / delta))
            enter, leave = max(enter, t0), min(leave, t1)
            if leave - enter <= EPS:
                return False
    return leave - enter > EPS


def crossed_shelf_columns(a: Point, b: Point) -> int:
    """同一列即使穿过多个货架格，也只累计一次 12dB。"""
    return sum(
        any(segment_enters_cell(a, b, col, row) for row in range(1, 5))
        for col in range(1, 4)
    )


def hop_rssi(a: Point, b: Point, cfg: Config) -> float:
    # P0 是 1m 参考值；近场 d<1m 统一按 1m，避免 log10(0) 和虚高 RSSI。
    distance = max(math.dist(a, b), 1.0)
    return (
        cfg.p0_dbm - 10 * cfg.path_loss_exponent * math.log10(distance)
        - cfg.shelf_loss_db * crossed_shelf_columns(a, b)
    )


def link_candidates(position: Point, cfg: Config) -> dict[str, float]:
    values = {"direct": hop_rssi(BASE, position, cfg)}
    for name, relay in RELAYS.items():
        values[name] = min(hop_rssi(BASE, relay, cfg), hop_rssi(relay, position, cfg)) + cfg.relay_gain_db
    return values


def select_link(current: str, values: dict[str, float], cfg: Config) -> tuple[str, str]:
    """阈值回差优先直连；中继之间使用候选相对当前的 3dB 回差。

    直连 < -75dBm 才接管；直连 >= -72dBm 才释放中继。
    接管和中继间切换要求至少改善 3dB。紧急掉线时选最强可用链路。
    """
    best_relay = max(RELAYS, key=lambda name: values[name])
    if values[current] < cfg.disconnect_threshold_dbm:
        best = max(values, key=values.get)
        if values[best] > values[current]:
            return best, "当前链路低于断链门限，切换到最强候选"
    if current == "direct":
        if (
            values["direct"] < cfg.relay_threshold_dbm
            and values[best_relay] >= values["direct"] + cfg.hysteresis_db
        ):
            return best_relay, "直连低于 -75dBm，最优中继改善至少 3dB"
    elif values["direct"] >= cfg.relay_threshold_dbm + cfg.hysteresis_db:
        return "direct", "直连恢复至 -72dBm，满足 3dB 阈值回差"
    elif best_relay != current and values[best_relay] >= values[current] + cfg.hysteresis_db:
        return best_relay, "另一中继较当前中继改善至少 3dB"
    return current, "保持当前链路"


def build_route(endpoint: str) -> list[Point]:
    route: list[Point] = [BASE]
    for col in range(4):
        route.append((float(col), 4.0 if col % 2 == 0 else 0.0))
        if col < 3:
            route.append((float(col + 1), route[-1][1]))
    if endpoint == "upper-right":
        route.append((3.0, 4.0))
    elif endpoint != "lower-right":
        raise ValueError(f"不支持的终点模式：{endpoint}")
    return route


def make_pickups(seed: int) -> list[tuple[int, int]]:
    rng = random.Random(seed)
    cells = [(col, row) for col in range(1, 4) for row in range(1, 5)]
    return rng.sample(cells, rng.randint(2, 3))


def simulate(cfg: Config) -> dict:
    route = build_route(cfg.endpoint)
    targets = make_pickups(cfg.seed)
    samples: list[Sample] = []
    events: list[Event] = []
    picked: list[dict] = []
    picked_cells: set[tuple[int, int]] = set()
    mode = "direct"
    tick = 0
    distance_m = 0.0

    def emit(kind: str, message: str, **details) -> None:
        events.append(Event(round(tick * cfg.dt_s, 10), kind, message, details))

    def record(position: Point, phase: str) -> None:
        nonlocal mode
        values = link_candidates(position, cfg)
        previous = mode
        mode, reason = select_link(mode, values, cfg)
        if previous != mode:
            emit(
                "switch",
                f"链路切换 {MODE_NAMES[previous]} -> {MODE_NAMES[mode]}；"
                f"位置 ({position[0]:.1f},{position[1]:.1f})；"
                f"切换前 {values[previous]:.2f} -> 切换后 {values[mode]:.2f} dBm；{reason}",
                old_mode=previous, new_mode=mode, before_dbm=values[previous],
                after_dbm=values[mode], position=list(position), reason=reason,
            )
        samples.append(Sample(
            round(tick * cfg.dt_s, 10), *position, values[mode], mode,
            values["direct"], values["relay_1"], values["relay_2"], phase, round(distance_m, 10),
        ))
        if values[mode] < cfg.disconnect_threshold_dbm:
            emit("disconnect", f"断链：{values[mode]:.2f} dBm", mode=mode, rssi_dbm=values[mode])

    def walk(points: list[Point], phase: str) -> None:
        nonlocal tick, distance_m
        for start, finish in zip(points, points[1:]):
            length = math.dist(start, finish)
            count = round(length / cfg.speed_m_s / cfg.dt_s)
            if count < 1 or not math.isclose(count * cfg.dt_s * cfg.speed_m_s, length, abs_tol=EPS):
                raise ValueError("航段时长必须是步长的整数倍")
            for index in range(1, count + 1):
                position = tuple(round(a + (b - a) * index / count, 10) for a, b in zip(start, finish))
                tick += 1
                distance_m += length / count
                record(position, phase)
                # 每格左边界中点设服务点；主机不进入货架格，不增加沿途停留时间。
                if phase == "outbound":
                    for cell in targets:
                        service_point = (cell[0] - 1.0, cell[1] - 0.5)
                        if cell not in picked_cells and math.dist(position, service_point) < EPS:
                            picked_cells.add(cell)
                            item = {"cell": list(cell), "time_s": samples[-1].time_s, "position": list(position)}
                            picked.append(item)
                            emit("pickup", f"取货：货架格 {cell}，服务点 ({position[0]:.1f},{position[1]:.1f})", **item)

    emit("start", f"从地面站 (0,0) 起飞；随机种子 {cfg.seed}；货架取货目标 {targets}")
    record(BASE, "outbound")
    walk(route, "outbound")
    arrival_time_s = samples[-1].time_s
    outbound_positions = [(s.x_m, s.y_m) for s in samples]
    emit("terminal_pickup_start", f"完成蛇形遍历，到达终点 {route[-1]}；模拟终点取货，停留 {cfg.pickup_dwell_s:.1f}s")
    hold_count = round(cfg.pickup_dwell_s / cfg.dt_s)
    if not math.isclose(hold_count * cfg.dt_s, cfg.pickup_dwell_s, abs_tol=EPS):
        raise ValueError("取货停留时长必须是步长的整数倍")
    for _ in range(hold_count):
        tick += 1
        record(route[-1], "terminal_pickup")
    turnaround_time_s = samples[-1].time_s
    emit("terminal_pickup_done", "终点取货完成")
    emit("turnaround", f"折返：在 {route[-1]} 沿去程路径严格反向返回")
    return_start_index = len(samples) - 1
    walk(list(reversed(route)), "return")
    emit("home", "主机返回起点 (0,0)，任务完成")

    result = {
        "contract": {"version": "legacy-a-v1", "incompatible_with": "warehouse-abc-v3"},
        "config": asdict(cfg), "route": route, "targets": targets, "picked": picked,
        "samples": samples, "events": events, "arrival_time_s": arrival_time_s,
        "turnaround_time_s": turnaround_time_s, "total_time_s": samples[-1].time_s,
        "flight_time_s": round(distance_m / cfg.speed_m_s, 10), "distance_m": round(distance_m, 10),
        "return_is_exact_reverse": [(s.x_m, s.y_m) for s in samples[return_start_index:]] == list(reversed(outbound_positions)),
    }
    result["validation"] = validate(result)
    return result


def validate(result: dict) -> dict:
    cfg = Config(**result["config"])
    samples = result["samples"]
    switches = [e for e in result["events"] if e.kind == "switch"]
    takeovers = [e for e in switches if e.details["new_mode"] != "direct"]
    min_rssi = min(s.rssi_dbm for s in samples)
    final = samples[-1]
    expected_s = (47.0 if cfg.endpoint == "upper-right" else 39.0)
    expected_endpoint = (3.0, 4.0) if cfg.endpoint == "upper-right" else (3.0, 0.0)
    checks = {
        "no_disconnect": min_rssi >= cfg.disconnect_threshold_dbm,
        "at_least_four_switches": len(switches) >= 4,
        "relay_takeover_above_minus75": bool(takeovers) and all(e.details["after_dbm"] > cfg.relay_threshold_dbm for e in takeovers),
        "returned_to_origin": (final.x_m, final.y_m) == BASE,
        "all_selected_goods_collected_once": sorted(tuple(p["cell"]) for p in result["picked"]) == sorted(result["targets"]),
        "return_is_exact_reverse": result["return_is_exact_reverse"],
        "constant_timestep": all(math.isclose(b.time_s - a.time_s, cfg.dt_s, abs_tol=EPS) for a, b in zip(samples, samples[1:])),
        "speed_and_distance": all(math.isclose(math.dist((a.x_m, a.y_m), (b.x_m, b.y_m)), cfg.speed_m_s * cfg.dt_s if b.phase != "terminal_pickup" else 0, abs_tol=EPS) for a, b in zip(samples, samples[1:])),
        "route_time_matches_geometry": math.isclose(result["total_time_s"], expected_s, abs_tol=EPS),
        "selected_endpoint_reached": result["route"][-1] == expected_endpoint,
    }
    return {
        "checks": checks, "model_checks_pass": all(checks.values()),
        "accepted_total_time_s": expected_s,
        "accepted_endpoint": expected_endpoint,
        "all_acceptance_checks_pass": all(checks.values()),
        "min_rssi_dbm": min_rssi, "switch_count": len(switches),
        "relay_takeover_count": len(takeovers), "sample_count": len(samples),
    }


def choose_chinese_font(explicit_path: str | None) -> str:
    from matplotlib import font_manager, rcParams
    if explicit_path:
        font_manager.fontManager.addfont(explicit_path)
        name = font_manager.FontProperties(fname=explicit_path).get_name()
    else:
        available = {f.name for f in font_manager.fontManager.ttflist}
        candidates = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Micro Hei", "PingFang SC", "Heiti SC", "SimSun"]
        name = next((name for name in candidates if name in available), None)
        if not name:
            raise RuntimeError("未找到中文字体。请安装 Noto Sans CJK SC，或使用 --font 中文字体文件路径。数据与日志已保存。")
    rcParams["font.family"] = [name, "DejaVu Sans"]
    rcParams["axes.unicode_minus"] = False
    rcParams["font.size"] = 10
    return name


def plot_demo(result: dict, output: Path, font_path: str | None = None) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Rectangle

    font_name = choose_chinese_font(font_path)
    blue, orange, green = "#2673c5", "#ed8a22", "#bbe0b2"
    samples = result["samples"]
    cfg = Config(**result["config"])
    validation = result["validation"]
    switches = [e for e in result["events"] if e.kind == "switch"]
    fig, (map_ax, rssi_ax) = plt.subplots(1, 2, figsize=(15.5, 7.8), gridspec_kw={"width_ratios": [1, 1.55]})
    fig.patch.set_facecolor("#f7f9fc")
    fig.suptitle("无人机中继巡检仿真｜3 × 4 仓库", fontsize=21, fontweight="bold", y=0.965, color="#172b46")
    fig.text(0.5, 0.907, f"1 架主机 + 2 架中继  ·  速度 1 m/s  ·  步长 0.1 s  ·  往返 {result['distance_m']:.0f} m  ·  总时长 {result['total_time_s']:.1f} s", ha="center", fontsize=12, color="#46576e")
    picked_cells = {tuple(item["cell"]) for item in result["picked"]}
    for col in range(1, 4):
        for row in range(1, 5):
            cell = (col, row)
            map_ax.add_patch(Rectangle((col - 0.94, row - 0.94), 0.88, 0.88, facecolor=green if cell in picked_cells else "#e6eaf0", edgecolor="none", zorder=1))
            map_ax.text(col - 0.5, row - 0.5, f"{col},{row}" + ("\n已取货" if cell in picked_cells else ""), ha="center", va="center", fontsize=10, color="#25552f" if cell in picked_cells else "#8590a0")
    for x in range(4):
        map_ax.plot([x, x], [0, 4], color="#222b38", linewidth=1.1, zorder=2)
    for y in range(5):
        map_ax.plot([0, 3], [y, y], color="#222b38", linewidth=1.1, zorder=2)

    # 去返程同一几何路线必然重叠：先画去程宽线，再画返程细虚线。
    # x=3 额外往返也保留在原坐标，不把图示偏移伪装成实际飞行轨迹。
    for phase, width, style in [("outbound", 5.5, "solid"), ("return", 2.0, (0, (3, 3)))]:
        segments, colors = [], []
        for a, b in zip(samples, samples[1:]):
            if b.phase == phase and (a.x_m, a.y_m) != (b.x_m, b.y_m):
                segments.append([(a.x_m, a.y_m), (b.x_m, b.y_m)])
                colors.append(blue if b.mode == "direct" else orange)
        map_ax.add_collection(LineCollection(segments, colors=colors, linewidths=width, linestyles=style, zorder=3 if phase == "outbound" else 4))
    for x, dy, y in [(0, 0.5, 1.3), (1, -0.5, 3.0), (2, 0.5, 1.1), (3, -0.5, 2.8)]:
        map_ax.annotate("", (x, y + dy), (x, y), arrowprops={"arrowstyle": "-|>", "color": "#172b46", "lw": 1.4}, zorder=6)
    map_ax.scatter(*BASE, marker="s", s=130, color="#172b46", edgecolor="white", linewidth=1, zorder=7)
    end = result["route"][-1]
    map_ax.scatter(*end, marker="*", s=290, color="#b34080", edgecolor="white", linewidth=1, zorder=8)
    map_ax.annotate("地面站 / 起点", BASE, xytext=(8, -25), textcoords="offset points", fontsize=10)
    map_ax.annotate("终点", end, xytext=(-5, 17), textcoords="offset points", ha="right", fontsize=10, color="#9f326e")
    for index, (name, point) in enumerate(RELAYS.items(), 1):
        map_ax.scatter(*point, marker="^", s=180, color="#794ca6", edgecolor="white", linewidth=1.2, zorder=8)
        map_ax.annotate(f"中继{index}", point, xytext=(9, 10), textcoords="offset points", fontsize=10, color="#643990")
    map_ax.set(xlim=(-0.25, 3.35), ylim=(-0.55, 4.55), xlabel="x / m", ylabel="y / m", xticks=range(4), yticks=range(5))
    map_ax.set_aspect("equal")
    map_ax.set_title("航线与取货位置", fontsize=14, pad=12, loc="left")
    map_ax.legend(handles=[
        Line2D([0], [0], color=blue, lw=4, label="直连"),
        Line2D([0], [0], color=orange, lw=4, label="中继转发"),
        Patch(facecolor=green, label="已取货格"),
        Line2D([0], [0], color="#667080", lw=2, ls="--", label="返程（虚线）"),
    ], ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.11), frameon=False, fontsize=10)

    times = [s.time_s for s in samples]
    rssi_ax.plot(times, [s.direct_dbm for s in samples], color="#a9b3c1", lw=1, alpha=0.7, label="直连候选")
    rssi_ax.plot(times, [s.rssi_dbm for s in samples], color="#687a8a", lw=0.9, alpha=0.5, zorder=3)
    for name in ("direct", "relay_1", "relay_2"):
        values = [s.rssi_dbm if s.mode == name else float("nan") for s in samples]
        if any(s.mode == name for s in samples):
            rssi_ax.plot(times, values, color=blue if name == "direct" else orange, lw=2.5, label="实际直连" if name == "direct" else f"实际{MODE_NAMES[name]}", zorder=4)
    # 切换前后使用同一时刻的候选 RSSI；竖箭头展示调度改善或释放。
    for event in switches:
        d = event.details
        rssi_ax.annotate("", (event.time_s, d["after_dbm"]), (event.time_s, d["before_dbm"]), arrowprops={"arrowstyle": "->", "color": "#8660a4", "lw": 0.9}, zorder=5)
        rssi_ax.scatter(event.time_s, d["after_dbm"], s=23, color="#794ca6", zorder=6)
    rssi_ax.axhline(-75, color="#d9444c", ls="--", lw=1.5, label="中继门限 -75 dBm")
    rssi_ax.axhline(-88, color="#7e8794", ls=":", lw=1.8, label="断链门限 -88 dBm")
    rssi_ax.axvline(result["turnaround_time_s"], color="#343b55", ls="-.", lw=1.3, label=f"折返 {result['turnaround_time_s']:.1f} s")
    rssi_ax.axvspan(result["arrival_time_s"], result["turnaround_time_s"], color="#ead4e3", alpha=0.6, zorder=0)
    rssi_ax.text(result["turnaround_time_s"] + 0.6, -37, "终点取货后折返", fontsize=9, color="#505b70")
    rssi_ax.set(xlim=(0, result["total_time_s"]), ylim=(-96, -32), xlabel="时间 / s", ylabel="RSSI / dBm")
    rssi_ax.set_title("全程链路质量与自动切换", fontsize=14, pad=12, loc="left")
    rssi_ax.grid(alpha=0.18)
    rssi_ax.legend(loc="lower left", ncol=2, fontsize=9, framealpha=0.96)
    link_status = "无断链" if validation["checks"]["no_disconnect"] else "存在断链"
    rssi_ax.text(0.03, 0.97, f"最低实际 RSSI  {validation['min_rssi_dbm']:.2f} dBm\n链路切换  {validation['switch_count']} 次   |   {link_status}", transform=rssi_ax.transAxes, va="top", fontsize=11, color="#254861", bbox={"boxstyle": "round,pad=0.5", "fc": "#edf4fa", "ec": "none"})
    for ax in (map_ax, rssi_ax):
        ax.spines[["top", "right"]].set_visible(False)
        ax.spines[["bottom", "left"]].set_color("#c3cbd5")
        ax.tick_params(colors="#516176")
    note = ("完整蛇形落点为 (3,0)，额外上行至 (3,4)：往返飞行 46 s + 终点取货 1 s = 47 s。"
            if cfg.endpoint == "upper-right" else "39 s 对照模式：终点采用完整蛇形的自然落点 (3,0)，往返飞行 38 s + 终点取货 1 s。")
    fig.text(0.5, 0.045, note, ha="center", fontsize=11, color="#4b5c73")
    fig.text(0.5, 0.014, "穿架按不同货架列计数；格线为零宽航线；货架填色留白仅用于排版。中继2保留为候选备用。", ha="center", fontsize=9, color="#748195")
    fig.subplots_adjust(left=0.055, right=0.975, top=0.845, bottom=0.21, wspace=0.24)
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    return font_name


def format_log(result: dict) -> str:
    v = result["validation"]
    lines = [
        "无人机中继巡检仿真 | FPGA 地面站算法原型",
        f"终点模式：{result['config']['endpoint']}；随机种子：{result['config']['seed']}",
        "路径说明：严格蛇形自然落在 (3,0)，单程19m；指定(3,4)需补4m。",
        "模型说明：开集货架格遮挡，同一列只计一次；沿格线不穿架；距离至少1m。",
        "调度说明：直连<-75dBm接管，直连>=-72dBm释放；中继间改善>=3dB切换。",
        "切换前后 RSSI 都在切换时刻、同一位置计算；切换次数含直连与中继互切。",
        "沿途货架取货按瞬时事件；终点另模拟1s取货停留。",
        "-" * 78,
    ]
    lines.extend(f"[{event.time_s:05.1f}s] {event.message}" for event in result["events"])
    lines.extend([
        "-" * 78,
        f"总航程：{result['distance_m']:.1f}m；飞行时间：{result['flight_time_s']:.1f}s；终点停留：{result['config']['pickup_dwell_s']:.1f}s；总任务时间：{result['total_time_s']:.1f}s",
        f"取货清单：{[tuple(item['cell']) for item in result['picked']]}；终点取货：完成",
        f"主机最终位置：({result['samples'][-1].x_m:.1f},{result['samples'][-1].y_m:.1f})",
        f"数据点：{v['sample_count']}；链路切换：{v['switch_count']}次；中继接管：{v['relay_takeover_count']}次",
        f"最低实际RSSI：{v['min_rssi_dbm']:.2f}dBm；距断链门限余量：{v['min_rssi_dbm'] + 88:.2f}dB",
        "模型及链路检查：" + ("全部通过" if v["model_checks_pass"] else "存在失败"),
        f"验收时间基准：{v['accepted_total_time_s']:.1f}s；验收终点：{tuple(v['accepted_endpoint'])}；最终返航点：(0,0)",
        "确认后的要求：保留终点(3,4)与完整遍历时，接受47s总时长。",
    ])
    for name, passed in v["checks"].items():
        lines.append(f"  [{'PASS' if passed else 'FAIL'}] {name}")
    return "\n".join(lines) + "\n"


def save_data(result: dict, output_dir: Path) -> None:
    with (output_dir / "flight_samples.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        rows = [asdict(sample) for sample in result["samples"]]
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "events.json").write_text(json.dumps([asdict(e) for e in result["events"]], ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {key: value for key, value in result.items() if key not in ("samples", "events")}
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "event_log.txt").write_text(format_log(result), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, default=7, help="取货随机种子，默认7，可复现")
    parser.add_argument("--endpoint", choices=["upper-right", "lower-right"], default="upper-right", help="upper-right 保留(3,4)，47s；lower-right 以(3,0)为终点，39s")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent, help="默认输出到脚本同目录")
    parser.add_argument("--font", help="中文字体文件路径（ttf/otf/ttc）")
    parser.add_argument("--no-plot", action="store_true", help="仅仿真与导出数据，无需matplotlib")
    args = parser.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = simulate(Config(seed=args.seed, endpoint=args.endpoint))
    save_data(result, output_dir)
    print(format_log(result), end="")
    if not args.no_plot:
        font = plot_demo(result, output_dir / "warehouse_relay_demo.png", args.font)
        print(f"绘图字体：{font}；图像：{output_dir / 'warehouse_relay_demo.png'}（150 dpi）")
    print(f"输出目录：{output_dir}")
    return 0 if result["validation"]["model_checks_pass"] else 1


if __name__ == "__main__":
    if "--legacy" in sys.argv:
        sys.argv.remove("--legacy")
        raise SystemExit(main())
    # One maintained simulation core for A display, B coordination and C datasets.
    swarm_root = Path(__file__).resolve().parents[2] / "swarm_cooperation" / "swarm_cooperation"
    sys.path.insert(0, str(swarm_root))
    from ideal_sim.cli import main as unified_main
    raise SystemExit(unified_main())
