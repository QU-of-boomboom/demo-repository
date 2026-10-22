"""第三部分：ONNX 导出、ORT 动态 INT8 PTQ 与 PC CPU 验证。"""
import platform
import time

import runtime_compat
import numpy as np
import onnx
import onnxruntime as ort
import torch

from common import classification_metrics, load_config, normalize, output_path, parser, sha256, sigmoid, write_json
from model import load_model


def make_session(path):
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    return ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])


def ort_predict(session, x, single_sample=True):
    """动态量化比例受批次影响，正式评估使用真实部署的 batch=1。"""
    if not single_sample:
        return sigmoid(session.run(None, {"features": x})[0].ravel())
    logits = np.empty(len(x), dtype=np.float32)
    for i in range(len(x)):
        logits[i] = session.run(None, {"features": x[i:i + 1]})[0][0, 0]
    return sigmoid(logits)


def latency_ms(session, x, c):
    for i in range(c["latency_warmup"]):
        session.run(None, {"features": x[i % len(x):i % len(x) + 1]})
    inputs = [np.ascontiguousarray(x[i:i + 1]) for i in np.linspace(0, len(x) - 1, c["latency_repeats"], dtype=int)]
    elapsed = []
    for sample in inputs:
        t = time.perf_counter_ns()
        session.run(None, {"features": sample})
        elapsed.append((time.perf_counter_ns() - t) / 1e6)
    return dict(mean_ms=float(np.mean(elapsed)), p50_ms=float(np.median(elapsed)),
                p95_ms=float(np.percentile(elapsed, 95)), repetitions=len(elapsed))


def quantize(c):
    from onnxruntime.quantization import QuantType, quantize_dynamic
    from onnxruntime.quantization.shape_inference import quant_pre_process
    out = output_path(c)
    torch.set_num_threads(c["cpu_threads"])
    model, checkpoint = load_model(out / "model.pt")
    if checkpoint["config"] != c:
        raise ValueError("配置与训练权重不一致，请用同一配置复跑。")
    example = torch.zeros(1, c["history_steps"] + 5)
    fp_path, int_path = out / "model_fp32.onnx", out / "model_int8.onnx"
    # 兼容已安装 PyTorch 2.8 的传统导出器；固定 opset 13，避免要求 onnxscript。
    torch.onnx.export(model, example, str(fp_path), input_names=["features"], output_names=["logit"],
                      dynamic_axes={"features": {0: "batch"}, "logit": {0: "batch"}},
                      opset_version=13, dynamo=False)
    # 额外提供固定 batch=1 的 FP32 源图，供目标 Icraft 版本自行解析/校准/编译。
    torch.onnx.export(model, example, str(out / "model_fp32_batch1.onnx"), input_names=["features"],
                      output_names=["logit"], opset_version=13, dynamo=False)
    quant_pre_process(str(fp_path), str(out / "model_preprocessed.onnx"), skip_optimization=False)
    # 传ModelProto让ORT在系统临时目录做原生shape inference，兼容Windows中文工程路径。
    quantize_dynamic(onnx.load(str(out / "model_preprocessed.onnx")), str(int_path),
                     weight_type=QuantType.QInt8, per_channel=True,
                     op_types_to_quantize=["MatMul", "Gemm"])
    for path in (fp_path, int_path, out / "model_fp32_batch1.onnx"):
        onnx.checker.check_model(onnx.load(str(path)))
    graph = onnx.load(str(int_path))
    quantized_ops = sum(node.op_type == "MatMulInteger" for node in graph.graph.node)
    if quantized_ops != len(c["hidden_sizes"]) + 1:
        raise RuntimeError(f"仅发现{quantized_ops}个INT8线性层，量化不完整。")
    with np.load(out / "test.npz", allow_pickle=False) as data:
        x, y = normalize(data["x"], c), data["y"]
    # Icraft 真正静态量化的代表样本只取训练集，绝不使用测试集校准。
    with np.load(out / "train.npz", allow_pickle=False) as data:
        train_x = data["x"]
        ix = np.random.default_rng(c["seed"]).choice(len(train_x), size=min(1024, len(train_x)), replace=False)
        np.save(out / "icraft_calibration_inputs.npy", normalize(train_x[ix], c))
    fp_session, int_session = make_session(fp_path), make_session(int_path)
    # FP32 ONNX 与 PyTorch 对齐，防止导出错误被后续指标掩盖。
    with torch.inference_mode():
        expected = model(torch.from_numpy(x[:1024])).numpy()
    exported = fp_session.run(None, {"features": x[:1024]})[0]
    np.testing.assert_allclose(exported, expected, atol=2e-5, rtol=2e-5)
    report = dict(stage="Icraft上板前的PC端仿真验证，非NPU实测", threshold=checkpoint["threshold"],
                  model_weights_sha256=sha256(out / "model.pt"),
                  environment=dict(python=platform.python_version(), platform=platform.platform(),
                                   processor=platform.processor(), onnx=onnx.__version__, onnxruntime=ort.__version__,
                                   torch=str(torch.__version__), threads=1, provider="CPUExecutionProvider"),
                  quantized_linear_layers=quantized_ops, onnx_export_max_abs_error=float(np.max(np.abs(expected - exported))),
                  rows={})
    probabilities = {}
    for name, path, session in (("FP32", fp_path, fp_session), ("INT8", int_path, int_session)):
        prob = ort_predict(session, x)
        probabilities[name] = prob
        metrics = classification_metrics(y, prob, checkpoint["threshold"])
        report["rows"][name] = dict(**metrics, size_bytes=path.stat().st_size,
                                   latency=latency_ms(session, x, c), sha256=sha256(path))
        print(f"{name}: accuracy={metrics['accuracy']:.4%}, recall={metrics['recall']:.4%}, "
              f"size={path.stat().st_size / 1024:.2f} KiB, latency={report['rows'][name]['latency']['mean_ms']:.5f} ms", flush=True)
    report["recall_drop_pp"] = 100 * (report["rows"]["FP32"]["recall"] - report["rows"]["INT8"]["recall"])
    report["quantization_pass"] = report["recall_drop_pp"] <= c["recall_drop_limit_pp"]
    report["prediction_disagreement_fraction"] = float(np.mean((probabilities["FP32"] >= checkpoint["threshold"])
                                                              != (probabilities["INT8"] >= checkpoint["threshold"])))
    np.savez_compressed(out / "quantization_test_predictions.npz", y=y, fp32=probabilities["FP32"], int8=probabilities["INT8"])
    write_json(out / "quantization_report.json", report)
    lines = ["Icraft 上板前的 PC 端仿真验证（CPU，batch=1，单线程，预热后1000次均值）", "",
             "| 模型 | 准确率 | 召回率 | ONNX大小(KiB) | 单次均值(ms) |", "|---|---:|---:|---:|---:|"]
    for name, r in report["rows"].items():
        lines.append(f"| {name} | {r['accuracy']:.4%} | {r['recall']:.4%} | {r['size_bytes']/1024:.2f} | {r['latency']['mean_ms']:.5f} |")
    lines += ["", f"INT8召回率下降：{report['recall_drop_pp']:.4f} 个百分点；通过：{report['quantization_pass']}。",
              "动态量化包含量化比例计算开销，小模型未必更快；此数值不能推算FMQL30TAI板端延迟。"]
    (out / "quantization_table.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    quantize(load_config(parser(__doc__).parse_args().config))
