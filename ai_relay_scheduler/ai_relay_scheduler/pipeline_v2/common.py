import argparse
import hashlib
import json
from pathlib import Path

from relay_learning.features import FeatureSpec
from . import SWARM_ROOT

ROOT = Path(__file__).resolve().parents[1]


def parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--config", type=Path, default=ROOT / "config.json")
    return p


def load_config(path=ROOT / "config.json"):
    c = json.loads(Path(path).read_text(encoding="utf-8"))
    spec = FeatureSpec(**c["feature_spec"])
    if not 0 < c["train_fraction"] < 1 or not 0 < c["validation_fraction"] < 1:
        raise ValueError("Invalid split fractions")
    if c["train_fraction"] + c["validation_fraction"] >= 1:
        raise ValueError("Test split must be nonempty")
    if c["voyages"] < 6 or min(c["epochs"], c["batch_size"], c["sample_stride"]) < 1:
        raise ValueError("Need at least six simulation voyages and positive training sizes")
    if c["data_seed"] == c["evaluation_seed"]:
        raise ValueError("Evaluation seed must be independent")
    if c["data_source"] not in ("simulation", "real"):
        raise ValueError("data_source must be simulation or real")
    if c["data_source"] == "real" and not c.get("telemetry_jsonl"):
        raise ValueError("Set telemetry_jsonl for real data")
    if spec.input_size != 96:
        raise ValueError("Unexpected v2 input dimension")
    return c


def output_path(c):
    path = Path(c["output_dir"])
    path = path if path.is_absolute() else ROOT / path
    if path.resolve() == (ROOT / "outputs").resolve():
        raise ValueError("Use outputs_v2/...; do not overwrite historical v1 artifacts")
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pipeline_source_hashes():
    paths = [ROOT / "pipeline_v2" / name for name in ("dataset.py", "model.py")]
    paths += [SWARM_ROOT / "ideal_sim" / name for name in ("contract.py", "config.py", "world.py", "scheduler.py", "simulation.py")]
    paths += [SWARM_ROOT / "relay_learning" / name for name in ("features.py", "monitor.py", "routing.py")]
    return {f"{p.parent.name}/{p.name}": sha256(p) for p in paths}


def sigmoid(values):
    import numpy as np
    return 1 / (1 + np.exp(-np.clip(values, -60, 60)))


def metrics(y, probability, threshold):
    import numpy as np
    y, p = np.asarray(y, dtype=bool), np.asarray(probability)
    if not len(y):
        return {"samples": 0, "recall": None, "precision": None, "accuracy": None, "brier": None}
    pred = p >= threshold
    tp, fp, fn = int(sum(pred & y)), int(sum(pred & ~y)), int(sum(~pred & y))
    return dict(samples=len(y), positives=int(sum(y)), tp=tp, fp=fp, fn=fn,
                recall=tp / max(tp + fn, 1), precision=tp / max(tp + fp, 1),
                accuracy=float(np.mean(pred == y)), brier=float(np.mean((p - y.astype(float)) ** 2)),
                threshold=float(threshold))


def select_threshold(y, probability, target):
    import numpy as np
    if len(set(np.asarray(y).tolist())) < 2:
        raise ValueError("Validation labels need both classes for each forecast horizon")
    choices = [metrics(y, probability, t) for t in np.linspace(.005, .995, 199)]
    return max(choices, key=lambda m: (m["recall"] >= target,
                                       m["precision"] if m["recall"] >= target else m["recall"],
                                       m["threshold"]))


def verify(c):
    import numpy as np
    out = output_path(c)
    manifest = json.loads((out / "dataset_manifest.json").read_text(encoding="utf-8"))
    if manifest["config"] != c:
        raise ValueError("Dataset configuration differs; regenerate with this config")
    if manifest.get("pipeline_source_hashes") != pipeline_source_hashes():
        raise ValueError("Feature/simulation/data code changed; regenerate data before training")
    groups, episodes = {}, {}
    for split in ("train", "val", "test"):
        path = out / f"{split}.npz"
        if sha256(path) != manifest["splits"][split]["sha256"]:
            raise ValueError(f"{split} dataset hash mismatch")
        with np.load(path, allow_pickle=False) as data:
            if data["x"].shape[1] != FeatureSpec(**c["feature_spec"]).input_size:
                raise ValueError("Wrong feature count")
            if not np.isfinite(data["x"]).all() or not np.isfinite(data["y"]).all():
                raise ValueError("Nonfinite training data")
            if not ((data["mask"] == 0) | (data["mask"] == 1)).all():
                raise ValueError("Invalid label masks")
            groups[split], episodes[split] = set(data["group_id"]), set(data["episode_id"])
    for a, b in (("train", "val"), ("val", "test"), ("train", "test")):
        if groups[a] & groups[b] or episodes[a] & episodes[b]:
            raise ValueError("Data leakage across collection groups/episodes")
    return manifest


def verify_main():
    c = load_config(parser("Verify v2 feature schema, hashes and whole-group splits").parse_args().config)
    manifest = verify(c)
    print(f"Verified {manifest['data_source']} dataset; groups/episodes are disjoint.")
