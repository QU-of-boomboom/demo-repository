"""核对真实数据划分与产物，防止重叠航程、陈旧配置或错配模型污染结果。"""
import csv
import json

import runtime_compat
import numpy as np

from common import load_config, output_path, parser, sha256, write_json
from simulator import simulate_voyage


def verify(c):
    out = output_path(c)
    manifest = json.loads((out / "dataset_manifest.json").read_text(encoding="utf-8"))
    assert manifest["config"] == c, "数据配置不一致"
    seen, count = set(), 0
    for split in ("train", "val", "test"):
        with np.load(out / f"{split}.npz", allow_pickle=False) as data:
            x, y, ids, tick = [data[key] for key in ("x", "y", "voyage_id", "tick")]
        unique = set(map(int, np.unique(ids)))
        assert not seen.intersection(unique), "航程跨集合泄漏"
        assert unique == set(manifest["splits"][split]["voyage_ids"])
        assert len(y) == manifest["splits"][split]["samples"]
        assert x.shape == (len(y), c["history_steps"] + 5) and np.isfinite(x).all()
        assert np.isin(y, [0, 1]).all()
        # 每组抽取固定的首、末航程；重建原始轨迹，核对特征和完整未来标签。
        for vid in sorted(unique)[::max(len(unique) - 1, 1)]:
            v = simulate_voyage(vid, c)
            mask = ids == vid
            np.testing.assert_array_equal(tick[mask], np.arange(len(v["y"])))
            np.testing.assert_array_equal(x[mask], v["x"][:len(v["y"])])
            np.testing.assert_array_equal(y[mask], v["y"])
        seen.update(unique)
        count += len(y)
    assert seen == set(range(c["voyages"]))
    quant = json.loads((out / "quantization_report.json").read_text(encoding="utf-8"))
    assert quant["model_weights_sha256"] == sha256(out / "model.pt")
    for key in ("FP32", "INT8"):
        assert quant["rows"][key]["sha256"] == sha256(out / f"model_{key.lower()}.onnx")
        assert quant["rows"][key]["latency"]["repetitions"] == c["latency_repeats"]
    with (out / "voyage_metrics.csv").open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 4 * c["evaluation_voyages"]
    keys = {(row["policy"], int(row["voyage_id"])) for row in rows}
    assert len(keys) == len(rows)
    for vid in range(c["evaluation_voyages"]):
        group = [r for r in rows if int(r["voyage_id"]) == vid]
        assert len({(r["shelf_loss_db"], r["sigma_db"], r["samples"]) for r in group}) == 1
    result = dict(pass_all=True, voyages=len(seen), samples=count, evaluation_rows=len(rows),
                  split_disjoint=True, reconstructed_voyages_match=True, model_hashes_match=True)
    write_json(out / "artifact_verification.json", result)
    print(f"产物核对通过：{len(seen)}航程、{count:,}样本、{len(rows)}条评估记录。", flush=True)
    return result


if __name__ == "__main__":
    verify(load_config(parser(__doc__).parse_args().config))
