"""Generate synthetic smoke data or import real telemetry without changing the encoder."""
from collections import defaultdict
from dataclasses import asdict
import json
import math
from pathlib import Path
import random

import numpy as np

from ideal_sim.config import ExperimentConfig
from ideal_sim.contract import valid_observation, finite_number, manifest as contract_manifest
from ideal_sim.simulation import IdealSimulation
from relay_learning.features import FeatureSpec, HistoryBuffer, LINKS, SCHEMA
from .common import ROOT, load_config, output_path, parser, sha256, write_json, pipeline_source_hashes


def samples_from_episode(frames, spec, stride):
    """Future observations are used ONLY for labels; unknown future is masked."""
    if not frames:
        return [], [], [], [], []
    buffer = HistoryBuffer(spec)
    x, y, masks, times, links = [], [], [], [], []
    for index, frame in enumerate(frames):
        if frame["schema_version"] != SCHEMA:
            raise ValueError("Telemetry schema mismatch")
        now = float(frame["time_s"])
        buffer.push(now, frame["links"])
        if index % stride:
            continue
        for link_index, link in enumerate(LINKS):
            labels, valid_labels = [], []
            for horizon in spec.horizons_s:
                steps = round(horizon / spec.dt_s)
                future = frames[index + 1:index + steps + 1]
                valid = (len(future) == steps and all(
                    abs(f["time_s"] - (now + (i + 1) * spec.dt_s)) < 1e-5
                    and f["plan_epoch"] == frame["plan_epoch"] for i, f in enumerate(future)))
                values = []
                if valid:
                    for f in future:
                        if f.get("source") == "simulation":
                            value = f["label_rssi_dbm"][link]
                            valid = finite_number(value)
                        else:
                            # Real labels must be observed in the future slot, not stale
                            # carried-forward values. Resampling/clock alignment is upstream.
                            measurement = next((m for m in f["links"] if m["link_id"] == link), {})
                            value = measurement.get("rssi_dbm")
                            valid = valid_observation(measurement, f["time_s"], spec.dt_s)
                        if not valid:
                            break
                        values.append(value)
                labels.append(float(min(values) < spec.weak_dbm) if valid and values else 0.)
                valid_labels.append(float(valid and bool(values)))
            if not any(valid_labels):
                continue
            x.append(buffer.encode(frame["state"], link))
            y.append(labels)
            masks.append(valid_labels)
            times.append(now)
            links.append(link_index)
    return x, y, masks, times, links


def simulation_config(c, episode, *, evaluation=False, strategy="predeploy"):
    spec = FeatureSpec(**c["feature_spec"])
    seed = (c["evaluation_seed"] if evaluation else c["data_seed"]) + episode
    rng = random.Random(seed)
    # Vary radio conditions, speed, telemetry and initial station assignments.
    nodes = [(float(x), float(y)) for x in range(4) for y in range(5)]
    stations = tuple(rng.sample(nodes[1:], 2)) if c.get("randomize_stations", True) else ((1., 2.), (2., 4.))
    return ExperimentConfig(
        rounds=c["simulation_rounds"], seed=seed, mode="random", dt_s=spec.dt_s,
        main_speed_mps=rng.uniform(.8, 1.2), relay_speed_mps=rng.uniform(1.5, 2.5),
        initial_stations=stations, shelf_loss_db=rng.uniform(8., 16.),
        noise_sigma_db=rng.uniform(.3, 2.), measurement_drop_rate=rng.uniform(0., .05),
        measurement_delay_s=rng.choice((0., spec.dt_s, 2 * spec.dt_s)),
        short_horizon_s=spec.horizons_s[0], deploy_horizon_s=spec.horizons_s[1],
        service_dbm=spec.weak_dbm, measurement_max_age_s=spec.max_age_s,
        record_learning=not evaluation, strategy=strategy,
        map_observation=c.get("map_observation", "static"))


def real_episodes(c):
    path = Path(c["telemetry_jsonl"])
    path = path if path.is_absolute() else ROOT / path
    episodes = defaultdict(list)
    group_of = {}
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            frame = json.loads(line)
            if frame.get("source") != "real" or "label_rssi_dbm" in frame:
                raise ValueError(f"line {line_number}: real input cannot contain simulation truth")
            episode, group = str(frame["episode_id"]), str(frame["split_group"])
            if not episode or not group or (episode in group_of and group_of[episode] != group):
                raise ValueError("Each episode must have one nonempty collection split_group")
            group_of[episode] = group
            episodes[episode].append(frame)
    return episodes, group_of, path


def generate(c):
    out, spec = output_path(c), FeatureSpec(**c["feature_spec"])
    if c["data_source"] == "real":
        episodes, group_of, source_path = real_episodes(c)
        keys = sorted(episodes)
    else:
        episodes = None
        keys = [f"sim-{i:04d}" for i in range(c["voyages"])]
        group_of = {key: key for key in keys}
        source_path = None
    groups = sorted(set(group_of.values()))
    if len(groups) < 6:
        raise ValueError("At least six independent collection groups are required")
    random.Random(c["seed"]).shuffle(groups)
    train_end = max(1, int(len(groups) * c["train_fraction"]))
    val_end = train_end + max(1, int(len(groups) * c["validation_fraction"]))
    if val_end >= len(groups):
        raise ValueError("No test groups left")
    split_of = {g: "train" if i < train_end else "val" if i < val_end else "test"
                for i, g in enumerate(groups)}
    chunks = {s: defaultdict(list) for s in ("train", "val", "test")}
    scenarios = []
    for index, key in enumerate(keys):
        if episodes is None:
            # Fixed station policies cover diverse configurations; each full mission
            # stays in one split. Planning changes censor affected future labels.
            cfg = simulation_config(c, index, strategy="fixed" if index % 2 else "predeploy")
            sim = IdealSimulation(cfg)
            report = sim.run()
            frames = sim.monitor.frames
            scenarios.append(dict(episode_id=key, config=asdict(cfg), success=report["success"],
                                  failure_reason=report["failure_reason"]))
        else:
            frames = episodes[key]
        x, y, mask, times, links = samples_from_episode(frames, spec, c["sample_stride"])
        if not x:
            raise ValueError(f"{key} contains no valid future labels")
        split = split_of[group_of[key]]
        for name, values in (("x", x), ("y", y), ("mask", mask), ("time_s", times),
                             ("link_index", links), ("episode_id", [key] * len(x)),
                             ("group_id", [group_of[key]] * len(x))):
            chunks[split][name].extend(values)
        print(f"{index + 1}/{len(keys)} {key}: {len(x)} link samples -> {split}", flush=True)
        if index == 0:
            # Small concrete schema example, not the entire training dataset.
            with (out / "telemetry_example.jsonl").open("w", encoding="utf-8") as f:
                for frame in frames[:3]:
                    f.write(json.dumps(dict(frame, episode_id=key, split_group=group_of[key])) + "\n")
    manifest = dict(schema_version=SCHEMA, data_source=c["data_source"], config=c,
                    simulation_contract=contract_manifest(simulation_config(c, 0)),
                    pipeline_source_hashes=pipeline_source_hashes(),
                    split_unit="whole_collection_group; all rounds and links of an episode stay together",
                    source_sha256=sha256(source_path) if source_path else None,
                    feature_contract=spec.manifest(), scenarios=scenarios, splits={})
    for split, chunk in chunks.items():
        arrays = {k: np.asarray(v, dtype=np.float32 if k in ("x", "y", "mask")
                               else np.float64 if k == "time_s"
                               else np.int16 if k == "link_index" else str)
                  for k, v in chunk.items()}
        path = out / f"{split}.npz"
        np.savez_compressed(path, **arrays)
        manifest["splits"][split] = dict(samples=len(arrays["x"]), sha256=sha256(path),
                                          groups=sorted(set(chunk["group_id"])),
                                          valid_labels=arrays["mask"].sum(axis=0).tolist())
    write_json(out / "dataset_manifest.json", manifest)
    write_json(out / "feature_contract.json", spec.manifest())
    return manifest


def main():
    generate(load_config(parser(__doc__).parse_args().config))
