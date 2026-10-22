"""Export fixed [1,96] FP32 and a PC-only INT8 comparison; no Icraft claims."""
from dataclasses import asdict
import platform
import time
import numpy as np
import onnx
import onnxruntime as ort
import torch
from onnxruntime.quantization import QuantType, quantize_dynamic
from onnxruntime.quantization.shape_inference import quant_pre_process

from relay_learning.features import FeatureSpec
from .common import load_config, metrics, output_path, parser, sha256, sigmoid, verify, write_json
from .model import load_model


def session(path):
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    return ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])


def infer(session_, x):
    return np.concatenate([session_.run(None, {"features": row[None, :]})[0] for row in x])


def export(c):
    verify(c)
    out, spec = output_path(c), FeatureSpec(**c["feature_spec"])
    model, checkpoint = load_model(out / "model.pt")
    if checkpoint["config"] != c or checkpoint["dataset_manifest_sha256"] != sha256(out / "dataset_manifest.json"):
        raise ValueError("Model, configuration and dataset must match")
    torch.set_num_threads(c["cpu_threads"])
    fp_path, int_path = out / "model_fp32_batch1.onnx", out / "model_int8_pc.onnx"
    torch.onnx.export(model, torch.zeros(1, spec.input_size), str(fp_path),
                      input_names=["features"], output_names=["logits"], opset_version=13, dynamo=False)
    processed = out / "model_preprocessed.onnx"
    # The native infer_shapes_path API does not reliably handle Chinese Windows
    # paths. Infer this tiny fixed graph in memory, then use Python serialization.
    shaped = onnx.shape_inference.infer_shapes(onnx.load(str(fp_path)))
    quant_pre_process(shaped, str(processed), skip_optimization=True, skip_onnx_shape=True)
    quantize_dynamic(onnx.load(str(processed)), str(int_path), weight_type=QuantType.QInt8,
                     per_channel=True, op_types_to_quantize=["Gemm", "MatMul"])
    for path in (fp_path, int_path):
        onnx.checker.check_model(onnx.load(str(path)))
    quantized_layers = sum(n.op_type == "MatMulInteger" for n in onnx.load(str(int_path)).graph.node)
    if quantized_layers != len(c["hidden_sizes"]) + 1:
        raise RuntimeError("Not all linear layers were quantized")
    with np.load(out / "train.npz", allow_pickle=False) as data:
        indices = np.random.default_rng(c["seed"]).choice(len(data["x"]), min(512, len(data["x"])), replace=False)
        calibration = data["x"][indices]
    np.save(out / "icraft_calibration_inputs.npy", calibration)
    golden = calibration[:32]
    with torch.inference_mode():
        golden_logits = model(torch.from_numpy(golden)).numpy()
    np.save(out / "golden_inputs.npy", golden)
    np.save(out / "golden_logits.npy", golden_logits)
    fp_session, int_session = session(fp_path), session(int_path)
    fp_golden = infer(fp_session, golden)
    np.testing.assert_allclose(fp_golden, golden_logits, rtol=2e-5, atol=2e-5)
    with np.load(out / "test.npz", allow_pickle=False) as data:
        x, y, mask = data["x"], data["y"], data["mask"]
    report = dict(data_source=c["data_source"], stage="PC CPU only; not NPU validation",
                  quantized_layers=quantized_layers, npu_validated=False,
                  export_max_abs_error=float(abs(fp_golden - golden_logits).max()),
                  environment=dict(python=platform.python_version(), torch=str(torch.__version__),
                                   numpy=np.__version__, onnx=onnx.__version__, onnxruntime=ort.__version__),
                  models={})
    for name, path, runner in (("FP32", fp_path, fp_session), ("INT8_PC", int_path, int_session)):
        p = sigmoid(infer(runner, x))
        for _ in range(20):
            runner.run(None, {"features": golden[:1]})
        timings = []
        for i in range(c["latency_repeats"]):
            start = time.perf_counter_ns()
            runner.run(None, {"features": golden[i % len(golden):i % len(golden) + 1]})
            timings.append((time.perf_counter_ns() - start) / 1e6)
        report["models"][name] = dict(
            horizons=[metrics(y[mask[:, h] > 0, h], p[mask[:, h] > 0, h], checkpoint["thresholds"][h])
                      for h in range(2)], bytes=path.stat().st_size, sha256=sha256(path),
            latency_ms=dict(mean=float(np.mean(timings)), p95=float(np.percentile(timings, 95)),
                            p99=float(np.percentile(timings, 99)), repetitions=len(timings)))
    drops = [100 * (report["models"]["FP32"]["horizons"][h]["recall"]
                    - report["models"]["INT8_PC"]["horizons"][h]["recall"]) for h in range(2)]
    report["recall_drop_pp"] = drops
    report["quantization_pass"] = max(drops) <= c["recall_drop_limit_pp"]
    write_json(out / "quantization_report.json", report)
    from ideal_sim.contract import VERSION
    write_json(out / "deployment.json", dict(
        simulation_contract_version=VERSION,
        schema_version=spec.schema_version, feature_spec=asdict(spec), input_size=spec.input_size,
        input_name="features", input_shape=[1, spec.input_size], input_dtype="float32",
        output_name="logits", output_shape=[1, 2], output_transform="CPU sigmoid",
        thresholds=checkpoint["thresholds"], data_source=c["data_source"], npu_validated=False,
        model_hashes={path.name: sha256(path) for path in (fp_path, int_path)},
        calibration_source="train split only", dataset_manifest_sha256=sha256(out / "dataset_manifest.json"),
        icraft_input="model_fp32_batch1.onnx", pc_comparison_only="model_int8_pc.onnx",
        note="Icraft import, board quantization, compilation and runtime must be tested with the installed SDK."))
    print(f"Exported fixed [1,{spec.input_size}] -> [1,2]; FP32 max error={report['export_max_abs_error']:.3g}")
    return report


def main():
    export(load_config(parser(__doc__).parse_args().config))
