"""Bounded inference process. Replace the worker's session call with Icraft later.

The simulator does not load an old 25-input model or untrained random weights.
"""
import json
import multiprocessing as mp
from pathlib import Path
import queue

from .features import FeatureSpec
from ideal_sim.contract import VERSION


def _worker(model_path, requests, results):
    try:
        import numpy as np
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(model_path, options, providers=["CPUExecutionProvider"])
        results.put(("ready", None))
        while True:
            batch = requests.get()
            if batch is None:
                return
            # Fixed batch=1 is the deployment contract.
            logits = [session.run(None, {"features": np.asarray([row], dtype=np.float32)})[0][0]
                      for row in batch]
            probability = 1. / (1. + np.exp(-np.clip(np.asarray(logits), -60, 60)))
            results.put(("ok", probability.tolist()))
    except Exception as exc:
        results.put(("error", f"{type(exc).__name__}: {exc}"))


class OnnxPredictor:
    def __init__(self, path, timeout_s=.2):
        path = Path(path).resolve()
        self.metadata = json.loads(path.with_name("deployment.json").read_text(encoding="utf-8"))
        if self.metadata.get("simulation_contract_version") != VERSION:
            raise ValueError("Model simulation contract differs; regenerate data/train/export under " + VERSION)
        self.spec = FeatureSpec(**self.metadata["feature_spec"])
        self.thresholds = self.metadata["thresholds"]
        self.timeout_s = timeout_s
        self.failed = False
        if self.metadata.get("input_size") != self.spec.input_size:
            raise ValueError("Model input schema mismatch; legacy weights are not compatible")
        if not path.is_file():
            raise FileNotFoundError(path)
        import hashlib
        expected = self.metadata.get("model_hashes", {}).get(path.name)
        if expected is None or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError("Model hash is absent or does not match deployment.json")
        context = mp.get_context("spawn")
        self.requests, self.results = context.Queue(), context.Queue()
        self.process = context.Process(target=_worker, args=(str(path), self.requests, self.results), daemon=True)
        self.process.start()
        try:
            kind, value = self.results.get(timeout=30.)
            if kind != "ready":
                raise RuntimeError(value)
            self.predict([[0.] * self.spec.input_size] * 6)  # explicit preflight warmup
        except Exception:
            self.close()
            raise

    def predict(self, rows):
        if self.failed:
            raise RuntimeError("Inference backend disabled after failure")
        self.requests.put(rows)
        try:
            kind, value = self.results.get(timeout=self.timeout_s)
        except queue.Empty:
            self.failed = True
            self.close()
            raise TimeoutError("Inference deadline exceeded; CPU fallback required") from None
        if kind != "ok":
            self.failed = True
            self.close()
            raise RuntimeError(value)
        return value

    def close(self):
        if hasattr(self, "process") and self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.)
        for name in ("requests", "results"):
            channel = getattr(self, name, None)
            if channel is not None:
                channel.close()
                channel.cancel_join_thread()
