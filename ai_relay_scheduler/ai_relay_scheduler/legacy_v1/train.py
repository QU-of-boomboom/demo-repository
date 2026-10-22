"""第二部分：加权BCE训练，验证集选工作阈值/模型，最后才评估独立测试集。"""
import time

import runtime_compat
import numpy as np
import torch

from common import classification_metrics, load_config, normalize, output_path, parser, select_threshold, sha256, write_json
from model import create_model, predict, load_model


def train(c):
    out = output_path(c)
    torch.set_num_threads(c["cpu_threads"])
    torch.manual_seed(c["seed"])
    np.random.seed(c["seed"])
    torch.use_deterministic_algorithms(True)
    with np.load(out / "train.npz", allow_pickle=False) as data:
        x = torch.from_numpy(normalize(data["x"], c))
        y = torch.from_numpy(data["y"].astype(np.float32)).reshape(-1, 1)
    with np.load(out / "val.npz", allow_pickle=False) as data:
        xv, yv = normalize(data["x"], c), data["y"]
    model = create_model(c)
    count = sum(p.numel() for p in model.parameters())
    optimizer = torch.optim.AdamW(model.parameters(), lr=c["learning_rate"], weight_decay=c["weight_decay"])
    loss_fn = torch.nn.BCEWithLogitsLoss(pos_weight=torch.tensor([c["positive_loss_weight"]]))
    best_score, log = (-1.0, -1.0, -1.0), []
    start = time.perf_counter()
    for epoch in range(c["epochs"]):
        model.train()
        order = torch.randperm(len(x))
        total = 0.0
        for begin in range(0, len(x), c["batch_size"]):
            ix = order[begin:begin + c["batch_size"]]
            optimizer.zero_grad(set_to_none=True)
            loss = loss_fn(model(x[ix]), y[ix])
            loss.backward()
            optimizer.step()
            total += loss.item() * len(ix)
        model.eval()
        metrics = select_threshold(yv, predict(model, xv), c)
        # 达到召回目标后，模型选择优先精确率；没有达到则优先召回。
        achieved = metrics["recall"] >= c["target_validation_recall"]
        score = (float(achieved), metrics["precision"] if achieved else metrics["recall"], metrics["f2"])
        record = dict(epoch=epoch + 1, loss=total / len(x), validation=metrics)
        log.append(record)
        if score > best_score:
            best_score = score
            torch.save(dict(state_dict=model.state_dict(), config=c, threshold=metrics["threshold"],
                            epoch=epoch + 1, validation=metrics, parameter_count=count), out / "model.pt")
        print(f"epoch {epoch + 1:02d}/{c['epochs']}: loss={record['loss']:.4f}, val acc={metrics['accuracy']:.4%}, "
              f"recall={metrics['recall']:.4%}, precision={metrics['precision']:.4%}, threshold={metrics['threshold']:.3f}", flush=True)
    model, ckpt = load_model(out / "model.pt")
    with np.load(out / "test.npz", allow_pickle=False) as data:
        metrics = classification_metrics(data["y"], predict(model, normalize(data["x"], c)), ckpt["threshold"])
    report = dict(parameter_count=count, selected_epoch=ckpt["epoch"], threshold=ckpt["threshold"],
                  validation=ckpt["validation"], test=metrics, history=log,
                  training_seconds=time.perf_counter() - start, torch_version=str(torch.__version__),
                  model_sha256=sha256(out / "model.pt"))
    write_json(out / "training_report.json", report)
    print(f"测试集: 准确率={metrics['accuracy']:.4%}, 召回率={metrics['recall']:.4%}, 参数={count:,}", flush=True)
    return report


if __name__ == "__main__":
    train(load_config(parser(__doc__).parse_args().config))
