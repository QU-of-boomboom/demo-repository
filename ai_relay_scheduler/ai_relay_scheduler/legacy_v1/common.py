"""参数、度量与文件工具；所有工程可调参数集中在 config.json。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import runtime_compat  # 必须早于第三方数值库导入
import numpy as np

ROOT = Path(__file__).resolve().parent


def parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--config", type=Path, default=ROOT / "config.json")
    return p


def load_config(path=ROOT / "config.json"):
    c = json.loads(Path(path).read_text(encoding="utf-8"))
    assert c["columns"] > 0 and c["rows"] > 0
    assert c["dt_s"] > 0 and c["nominal_speed_m_s"] > 0
    assert 0 <= c["speed_variation"] < 1
    assert c["reference_distance_m"] > 0 and c["cell_size_m"] > 0
    assert c["history_steps"] >= 1 and len(c["relay_xy"]) == 2
    assert c["forecast_s"] > 0 and c["handover_delay_s"] >= 0
    assert c["voyages"] >= 10 and c["evaluation_voyages"] >= 2
    assert 0 < c["train_fraction"] < 1 and 0 < c["validation_fraction"] < 1
    assert c["train_fraction"] + c["validation_fraction"] < 1
    assert c["evaluation_seed"] != c["data_seed"]
    for name in ("forecast_s", "speed_refresh_s", "handover_delay_s"):
        assert np.isclose(c[name] / c["dt_s"], round(c[name] / c["dt_s"])), name
    for name in ("shelf_loss_range_db", "noise_sigma_range_db"):
        assert 0 <= c[name][0] <= c[name][1]
    return c


def output_path(c):
    p = Path(c["output_dir"])
    p = p if p.is_absolute() else ROOT / p
    p.mkdir(parents=True, exist_ok=True)
    return p


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def normalize(x, c):
    """固定物理量缩放，不从验证/测试数据拟合；顺序与 simulator.features 一致。"""
    h = c["history_steps"]
    shift = np.array([-65.0] * h + [0.0] * 5, dtype=np.float32)
    diag = np.hypot(c["columns"], c["rows"]) * c["cell_size_m"]
    scale = np.array([25.0] * h + [c["columns"] * c["cell_size_m"],
                     c["rows"] * c["cell_size_m"], c["nominal_speed_m_s"], diag, diag], dtype=np.float32)
    return np.ascontiguousarray((np.asarray(x, dtype=np.float32) - shift) / scale)


def sigmoid(logits):
    return 1.0 / (1.0 + np.exp(-np.clip(np.asarray(logits), -60, 60)))


def classification_metrics(y, probability, threshold):
    y = np.asarray(y, dtype=bool)
    pred = np.asarray(probability).ravel() >= threshold
    tp, tn = int(np.sum(pred & y)), int(np.sum(~pred & ~y))
    fp, fn = int(np.sum(pred & ~y)), int(np.sum(~pred & y))
    recall = tp / max(tp + fn, 1)
    precision = tp / max(tp + fp, 1)
    return dict(accuracy=(tp + tn) / len(y), recall=recall, precision=precision,
                f2=5 * precision * recall / max(4 * precision + recall, 1e-12),
                tp=tp, tn=tn, fp=fp, fn=fn, threshold=float(threshold), samples=len(y))


def select_threshold(y, probability, c):
    """仅在验证集选阈值：满足召回率目标后优先精确率，避免盲目常开中继。"""
    grid = np.linspace(c["threshold_grid_min"], c["threshold_grid_max"], c["threshold_grid_count"])
    # 排序一次，通过累计计数计算整条阈值曲线，避免反复扫描百万样本。
    order = np.argsort(probability)
    p = np.asarray(probability)[order]
    labels = np.asarray(y)[order]
    positive_prefix = np.r_[0, np.cumsum(labels, dtype=np.int64)]
    total_positive = int(positive_prefix[-1])
    candidates = []
    for threshold in grid:
        index = int(np.searchsorted(p, threshold, side="left"))
        tp = total_positive - int(positive_prefix[index])
        fp = len(p) - index - tp
        fn, tn = total_positive - tp, index - int(positive_prefix[index])
        recall, precision = tp / max(total_positive, 1), tp / max(tp + fp, 1)
        candidates.append(dict(accuracy=(tp + tn) / len(p), recall=recall, precision=precision,
                               f2=5 * precision * recall / max(4 * precision + recall, 1e-12),
                               tp=tp, tn=tn, fp=fp, fn=fn, threshold=float(threshold), samples=len(p)))
    eligible = [m for m in candidates if m["recall"] >= c["target_validation_recall"]]
    return max(eligible, key=lambda m: (m["precision"], m["threshold"])) if eligible else max(candidates, key=lambda m: (m["recall"], m["precision"]))
