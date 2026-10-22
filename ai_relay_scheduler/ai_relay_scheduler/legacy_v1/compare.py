"""第四部分：200个独立且配对的随机航程，规则与部署INT8模型公平对比。"""
import csv
import json
import math

import runtime_compat
import numpy as np

from common import load_config, normalize, output_path, parser, sha256, write_json
from model import load_model
from quantize import make_session, ort_predict
from simulator import simulate_voyage


def schedule(links, probabilities, threshold, c, policy, delay_s=None):
    """因果调度状态机；延迟期间保持原链路，不人为注入切换丢包。

    AI入场由模型概率触发；退出需直连达到-72且模型认为未来安全。
    两组均使用同一个候选选择器、中继间3dB回差和切换建立延迟。
    """
    delay = round((c["handover_delay_s"] if delay_s is None else delay_s) / c["dt_s"])
    current, pending, ready_at = 0, None, -1
    modes = np.zeros(len(links), dtype=np.int8)
    effective = np.empty(len(links))
    events, requests = [], []
    for t, values in enumerate(links):
        if pending is not None and t >= ready_at:
            events.append((t, current, pending))
            current, pending = pending, None
        if pending is None:
            best_relay = 1 + int(np.argmax(values[1:]))
            need_relay = values[0] < c["relay_threshold_dbm"] if policy == "rule" else probabilities[t] >= threshold
            safe_to_exit = values[0] >= c["relay_threshold_dbm"] + c["hysteresis_db"]
            if policy == "ai":
                safe_to_exit &= not need_relay
            target = current
            if current == 0:
                if need_relay and values[best_relay] >= c["disconnect_threshold_dbm"]:
                    target = best_relay
            elif safe_to_exit:
                target = 0
            elif best_relay != current and values[best_relay] >= values[current] + c["hysteresis_db"]:
                target = best_relay
            if target != current:
                requests.append((t, current, target))
                if delay == 0:
                    events.append((t, current, target))
                    current = target
                else:
                    pending, ready_at = target, t + delay
        modes[t], effective[t] = current, values[current]
    outage = effective < c["disconnect_threshold_dbm"]
    starts = outage & ~np.r_[False, outage[:-1]]
    # 紧邻的两次已完成切换A→B、B→A，间隔不超过2s，计一个乒乓。
    pingpong = sum(a[1] == b[2] and a[2] == b[1] and (b[0] - a[0]) * c["dt_s"] <= c["pingpong_window_s"] + 1e-9
                   for a, b in zip(events, events[1:]))
    metrics = dict(outage_events=int(starts.sum()), outage_samples=int(outage.sum()),
                   outage_seconds=float(outage.sum() * c["dt_s"]), switches=len(events),
                   switch_requests=len(requests), mean_margin_db=float(np.mean(effective - c["disconnect_threshold_dbm"])),
                   margin_sum_db=float(np.sum(effective - c["disconnect_threshold_dbm"])), pingpong=int(pingpong),
                   relay_samples=int(np.sum(modes != 0)), samples=len(links),
                   relay_fraction=float(np.mean(modes != 0)))
    return metrics, dict(mode=modes, effective=effective, events=np.asarray(events, dtype=np.int32).reshape(-1, 3),
                         requests=np.asarray(requests, dtype=np.int32).reshape(-1, 3))


def aggregate(records):
    keys = ("outage_events", "outage_samples", "outage_seconds", "switches", "switch_requests", "pingpong", "samples", "relay_samples")
    result = {key: sum(r[key] for r in records) for key in keys}
    result["mean_margin_db"] = sum(r["margin_sum_db"] for r in records) / result["samples"]
    result["relay_fraction"] = result["relay_samples"] / result["samples"]
    return result


def paired_statistics(rule, ai, c):
    delta = np.array([b["outage_events"] - a["outage_events"] for a, b in zip(rule, ai)], dtype=float)
    rng = np.random.default_rng(np.random.SeedSequence([c["seed"], 5678]))
    boot = delta[rng.integers(0, len(delta), (c["bootstrap_repeats"], len(delta)))].mean(axis=1)
    wins, losses = int(np.sum(delta < 0)), int(np.sum(delta > 0))
    n = wins + losses
    pvalue = sum(math.comb(n, k) for k in range(wins, n + 1)) / 2 ** n if n else 1.0
    return dict(mean_outage_event_difference=float(delta.mean()),
                paired_bootstrap_95_ci=np.percentile(boot, [2.5, 97.5]).tolist(),
                one_sided_sign_test_p=pvalue, ai_better_voyages=wins, ai_worse_voyages=losses,
                tied_voyages=int(np.sum(delta == 0)), bootstrap_repeats=c["bootstrap_repeats"])


def configure_plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    candidates = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Zen Hei", "Arial Unicode MS"]
    font = next((f for f in candidates if f in available), None)
    if font is None:
        raise RuntimeError("请安装中文字体 Noto Sans CJK SC / 微软雅黑后重新绘图，以保证中文标注可读。")
    plt.rcParams.update({"font.sans-serif": [font], "axes.unicode_minus": False, "font.size": 11})
    return plt


def plot_results(report, out, c):
    plt = configure_plot()
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fields = [("outage_events", "断链事件次数", "次"), ("switches", "中继切换总次数", "次"),
              ("mean_margin_db", "全程平均链路余量", "dB，相对 −88 dBm"), ("pingpong", "乒乓切换次数", "次，2秒内 A→B→A")]
    for ax, (key, title, unit) in zip(axes.flat, fields):
        values = [report["rule"][key], report["ai"][key]]
        bars = ax.bar([-0.21, 0.21], values, width=0.34, color=["#64748b", "#059669"])
        ax.set_xticks([-0.21, 0.21], ["规则调度", "AI 调度（INT8）"])
        ax.set_title(title, fontweight="bold")
        ax.set_ylabel(unit)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_axisbelow(True)
        ax.grid(axis="y", alpha=0.2)
        ax.bar_label(bars, labels=[f"{v:.2f}" if key == "mean_margin_db" else str(int(v)) for v in values], padding=4)
        ax.set_ylim(min(0, min(values) * 1.2), max(max(values) * 1.22, 1))
    fig.suptitle(f"仓库巡检：规则与 AI 调度配对实验\n{c['evaluation_voyages']} 个随机往返航程 · 切换建立 {c['handover_delay_s']:.1f} 秒", fontsize=17)
    fig.savefig(out / "ai_vs_rule.png", dpi=180)
    plt.close(fig)
    with np.load(out / "example_trace.npz") as v:
        fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
        axes[0].plot(v["time"], v["links"][:, 0], color="#94a3b8", alpha=.6, label="直连探测 RSSI")
        for key, label, color in (("rule", "规则有效链路", "#64748b"), ("ai", "AI有效链路", "#059669")):
            axes[0].plot(v["time"], v[f"{key}_effective"], lw=1, label=label, color=color)
            axes[2].step(v["time"], v[f"{key}_mode"], where="post", label=label, color=color)
        axes[0].axhline(c["disconnect_threshold_dbm"], color="#dc2626", ls="--", label="断链门限 −88 dBm")
        axes[0].axhline(c["relay_threshold_dbm"], color="#d97706", ls=":", label="切中继门限 −75 dBm")
        axes[0].set_ylabel("RSSI / dBm")
        axes[0].legend(ncol=3, fontsize=9)
        axes[1].plot(v["time"], v["probability"], color="#059669", label="未来1.5秒弱链路概率")
        axes[1].axhline(report["threshold"], ls="--", color="#d97706", label="验证集选定概率阈值")
        axes[1].set_ylabel("预测概率")
        axes[1].legend()
        axes[2].set_yticks([0, 1, 2], ["直连", "中继1", "中继2"])
        axes[2].set_xlabel("时间 / 秒")
        axes[2].legend()
        fig.suptitle("预先指定评估航程 0：链路、预测与切换时序")
        fig.savefig(out / "example_trace.png", dpi=180)
        plt.close(fig)


def compare(c):
    out = output_path(c)
    _, checkpoint = load_model(out / "model.pt")
    if checkpoint["config"] != c:
        raise ValueError("配置与训练权重不一致，请用同一配置复跑。")
    threshold = checkpoint["threshold"]
    quant = json.loads((out / "quantization_report.json").read_text(encoding="utf-8"))
    if quant["model_weights_sha256"] != sha256(out / "model.pt") or quant["rows"]["INT8"]["sha256"] != sha256(out / "model_int8.onnx"):
        raise ValueError("训练权重/INT8模型与量化报告不匹配，请重新运行 quantize.py。")
    session = make_session(out / "model_int8.onnx")
    records = {"rule": [], "ai": [], "rule_zero_delay": [], "ai_zero_delay": []}
    csv_rows = []
    for vid in range(c["evaluation_voyages"]):
        v = simulate_voyage(vid, c, stream="evaluation")
        probability = ort_predict(session, normalize(v["x"], c))
        example = {}
        for name in records:
            policy = name.split("_")[0]
            delay = 0.0 if name.endswith("zero_delay") else c["handover_delay_s"]
            metrics, trace = schedule(v["links"], probability, threshold, c, policy, delay)
            records[name].append(metrics)
            csv_rows.append(dict(voyage_id=vid, policy=name, shelf_loss_db=v["shelf_loss_db"], sigma_db=v["sigma_db"], **metrics))
            if vid == 0 and not name.endswith("zero_delay"):
                example.update({f"{name}_{key}": value for key, value in trace.items()})
        if vid == 0:
            np.savez_compressed(out / "example_trace.npz", time=v["time"], xy=v["xy"], links=v["links"],
                                probability=probability, **example)
        if (vid + 1) % 50 == 0:
            print(f"配对仿真 {vid + 1}/{c['evaluation_voyages']} 航程", flush=True)
    report = {name: aggregate(rows) for name, rows in records.items()}
    report.update(threshold=threshold, evaluation_seed=c["evaluation_seed"], voyages=c["evaluation_voyages"],
                  handover_delay_s=c["handover_delay_s"], deployed_model="model_int8.onnx",
                  statistics=paired_statistics(records["rule"], records["ai"], c))
    rule, ai = report["rule"], report["ai"]
    growth = ai["switches"] / rule["switches"] - 1 if rule["switches"] else (0.0 if ai["switches"] == 0 else None)
    report["switch_growth"] = growth
    report["outage_reduction_fraction"] = 1 - ai["outage_events"] / rule["outage_events"] if rule["outage_events"] else None
    report["acceptance"] = dict(significantly_fewer_outages=(report["statistics"]["paired_bootstrap_95_ci"][1] < 0
                                                              and report["statistics"]["one_sided_sign_test_p"] < .05),
                                switch_growth_at_most_50_percent=growth is not None and growth <= c["switch_growth_limit"],
                                ideal_zero_outages=ai["outage_events"] == 0)
    with (out / "voyage_metrics.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
    write_json(out / "comparison_report.json", report)
    lines = [f"规则 vs AI（INT8），同一批{c['evaluation_voyages']}个独立随机航程。", "",
             "| 策略 | 断链事件 | 断链采样点 | 切换次数 | 平均余量(dB) | 乒乓次数 | 中继占用率 |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for key, name in (("rule", "规则"), ("ai", "AI"), ("rule_zero_delay", "规则/零延迟"), ("ai_zero_delay", "AI/零延迟")):
        r = report[key]
        lines.append(f"| {name} | {r['outage_events']} | {r['outage_samples']} | {r['switches']} | {r['mean_margin_db']:.3f} | {r['pingpong']} | {r['relay_fraction']:.2%} |")
    lines += ["", f"切换增幅：{growth:.2%}" if growth is not None else "规则组切换为0，AI组非0，增幅无法定义。",
              f"配对统计：{report['statistics']}", f"验收：{report['acceptance']}"]
    (out / "comparison_table.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines), flush=True)
    plot_results(report, out, c)
    return report


if __name__ == "__main__":
    compare(load_config(parser(__doc__).parse_args().config))
