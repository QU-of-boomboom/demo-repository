"""Run the v2 training template, optionally with a small synthetic smoke configuration."""
import subprocess
import sys

from .common import ROOT, load_config, output_path, parser, verify, write_json


def main():
    p = parser(__doc__)
    p.add_argument("--smoke", action="store_true", help="6 synthetic missions, 3 epochs; not a trained product")
    p.add_argument("--skip-data", action="store_true")
    p.add_argument("--skip-compare", action="store_true")
    args = p.parse_args()
    c = load_config(args.config)
    if args.smoke:
        c.update(data_source="simulation", voyages=6, epochs=3, simulation_rounds=2,
                 sample_stride=10, evaluation_voyages=2, output_dir="outputs_v2/smoke",
                 latency_repeats=100)
    out = output_path(c)
    used = out / "run_config.json"
    write_json(used, c)
    if args.skip_data:
        verify(c)
    steps = ([] if args.skip_data else ["generate_data.py"]) + ["train.py", "quantize.py", "verify_artifacts.py"]
    if not args.skip_compare:
        steps.append("compare.py")
    steps.append("make_report.py")
    for step in steps:
        print(f"\nV2: {step}", flush=True)
        with (out / f"{step[:-3]}.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen([sys.executable, "-u", str(ROOT / step), "--config", str(used)],
                                       cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding="utf-8", errors="replace",
                                       env=__import__("os").environ | {"PYTHONIOENCODING": "utf-8"})
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
            if process.wait():
                raise RuntimeError(f"{step} failed; see {log.name}")
    print(f"V2 outputs: {out.resolve()}\nStatus: {c['data_source']}; NPU has not been validated.")
