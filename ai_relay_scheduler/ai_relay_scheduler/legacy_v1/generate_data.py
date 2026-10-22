"""第一部分：生成按航程隔离的训练/验证/测试 .npz。"""
import time

import runtime_compat
import numpy as np

from common import load_config, output_path, parser, write_json
from simulator import simulate_voyage


def generate(c):
    started = time.perf_counter()
    out = output_path(c)
    ids = np.random.default_rng(c["seed"]).permutation(c["voyages"])
    ntrain = int(len(ids) * c["train_fraction"])
    nval = int(len(ids) * c["validation_fraction"])
    groups = {"train": ids[:ntrain], "val": ids[ntrain:ntrain + nval], "test": ids[ntrain + nval:]}
    summary = {"config": c, "split_unit": "whole_voyage", "splits": {}}
    for split, group in groups.items():
        xs, ys, voyage_ids, ticks, parameters = [], [], [], [], []
        for index, vid in enumerate(group):
            v = simulate_voyage(int(vid), c)
            size = len(v["y"])
            xs.append(v["x"][:size])
            ys.append(v["y"])
            voyage_ids.append(np.full(size, vid, dtype=np.int32))
            ticks.append(np.arange(size, dtype=np.int16))
            parameters.append([int(vid), v["shelf_loss_db"], v["sigma_db"], len(v["time"])])
            if (index + 1) % 500 == 0:
                print(f"[{split}] {index + 1}/{len(group)} 航程", flush=True)
        x, y = np.concatenate(xs), np.concatenate(ys)
        np.savez_compressed(out / f"{split}.npz", x=x, y=y, voyage_id=np.concatenate(voyage_ids),
                            tick=np.concatenate(ticks), voyage_parameters=np.asarray(parameters),
                            feature_names=np.array([f"rssi_t_minus_{i}" for i in range(c["history_steps"] - 1, -1, -1)]
                                                   + ["x", "y", "speed", "distance_relay_1", "distance_relay_2"]))
        summary["splits"][split] = dict(voyages=len(group), samples=len(y), positives=int(y.sum()),
                                        positive_fraction=float(y.mean()), voyage_ids=group.tolist())
        print(f"{split}: {len(group)}航程, {len(y):,}样本, 正例{y.mean():.2%}", flush=True)
        del x, y, xs, ys, voyage_ids, ticks
    summary["generation_seconds"] = time.perf_counter() - started
    write_json(out / "dataset_manifest.json", summary)
    return summary


if __name__ == "__main__":
    generate(load_config(parser(__doc__).parse_args().config))
