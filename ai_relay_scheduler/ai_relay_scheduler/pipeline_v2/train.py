"""Train a masked two-horizon classifier; validate before touching test labels."""
import time
import numpy as np
import torch

from .common import load_config, metrics, output_path, parser, select_threshold, sha256, verify, write_json
from .model import create_model, load_model, predict


def train(c):
    manifest = verify(c)
    out = output_path(c)
    torch.set_num_threads(c["cpu_threads"])
    torch.manual_seed(c["seed"])
    torch.use_deterministic_algorithms(True)
    with np.load(out / "train.npz", allow_pickle=False) as data:
        x = torch.from_numpy(data["x"])
        y, mask = torch.from_numpy(data["y"]), torch.from_numpy(data["mask"])
    with np.load(out / "val.npz", allow_pickle=False) as data:
        xv, yv, mv = data["x"], data["y"], data["mask"].astype(bool)
    model = create_model(c)
    optimizer = torch.optim.AdamW(model.parameters(), lr=c["learning_rate"], weight_decay=c["weight_decay"])
    loss_fn = torch.nn.BCEWithLogitsLoss(reduction="none")
    history, best, start = [], None, time.perf_counter()
    for epoch in range(c["epochs"]):
        model.train()
        order, loss_sum, count = torch.randperm(len(x)), 0., 0.
        for offset in range(0, len(x), c["batch_size"]):
            indices = order[offset:offset + c["batch_size"]]
            optimizer.zero_grad(set_to_none=True)
            raw = loss_fn(model(x[indices]), y[indices]) * mask[indices]
            denominator = mask[indices].sum().clamp_min(1)
            loss = raw.sum() / denominator
            loss.backward()
            optimizer.step()
            loss_sum += raw.sum().item()
            count += denominator.item()
        model.eval()
        probability = predict(model, xv)
        validation = [select_threshold(yv[mv[:, h], h], probability[mv[:, h], h],
                                       c["target_validation_recall"]) for h in range(2)]
        score = (sum(m["recall"] >= c["target_validation_recall"] for m in validation),
                 sum(m["precision"] for m in validation), -sum(m["brier"] for m in validation))
        record = dict(epoch=epoch + 1, loss=loss_sum / count, validation=validation)
        history.append(record)
        if best is None or score > best:
            best = score
            torch.save(dict(schema_version="relay-link-v2", state_dict=model.state_dict(), config=c,
                            thresholds=[m["threshold"] for m in validation], epoch=epoch + 1,
                            validation=validation, data_source=c["data_source"],
                            dataset_manifest_sha256=sha256(out / "dataset_manifest.json")), out / "model.pt")
        print(f"epoch {epoch + 1}/{c['epochs']} loss={record['loss']:.4f} "
              f"validation recall={[round(m['recall'], 4) for m in validation]}", flush=True)
    model, checkpoint = load_model(out / "model.pt")
    with np.load(out / "test.npz", allow_pickle=False) as data:
        p = predict(model, data["x"])
        test = [metrics(data["y"][data["mask"][:, h] > 0, h], p[data["mask"][:, h] > 0, h],
                        checkpoint["thresholds"][h]) for h in range(2)]
    report = dict(schema_version="relay-link-v2", data_source=c["data_source"],
                  status="synthetic_pipeline_test" if c["data_source"] == "simulation" else "real_data_offline_evaluation",
                  npu_validated=False, parameter_count=sum(p.numel() for p in model.parameters()),
                  selected_epoch=checkpoint["epoch"], thresholds=checkpoint["thresholds"],
                  validation=checkpoint["validation"], test=test, history=history,
                  training_seconds=time.perf_counter() - start, torch_version=str(torch.__version__),
                  model_sha256=sha256(out / "model.pt"))
    write_json(out / "training_report.json", report)
    return report


def main():
    train(load_config(parser(__doc__).parse_args().config))
