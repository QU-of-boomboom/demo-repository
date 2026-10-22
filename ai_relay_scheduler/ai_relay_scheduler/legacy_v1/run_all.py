"""完整复跑入口；每个阶段独立Python进程，避免训练/量化内存叠加。"""
import json
import os
import subprocess
import sys

from common import ROOT, load_config, output_path, parser


def main():
    p = parser(__doc__)
    p.add_argument("--skip-data", action="store_true", help="只在配置完全匹配已有数据时复用数据")
    p.add_argument("--strict", action="store_true", help="任一必需验收项失败时退出码为2")
    args = p.parse_args()
    c = load_config(args.config)
    out = output_path(c)
    if args.skip_data:
        manifest = json.loads((out / "dataset_manifest.json").read_text(encoding="utf-8"))
        if manifest["config"] != c or not all((out / f"{s}.npz").exists() for s in ("train", "val", "test")):
            raise ValueError("已有数据与配置不一致，不能跳过生成。")
    steps = ([] if args.skip_data else ["generate_data.py"]) + ["train.py", "quantize.py", "compare.py", "verify_artifacts.py", "make_report.py"]
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
    subprocess.run([sys.executable, "-m", "unittest", "-v", "test_core"], cwd=ROOT, env=env, check=True)
    for step in steps:
        print(f"\n运行 {step}", flush=True)
        with (out / f"{step[:-3]}.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen([sys.executable, "-u", str(ROOT / step), "--config", str(args.config.resolve())],
                                       cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding="utf-8", errors="replace")
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
            if process.wait():
                raise RuntimeError(f"{step}运行失败，检查 {log.name}")
    acceptance = json.loads((out / "acceptance.json").read_text(encoding="utf-8"))
    if args.strict and not acceptance["all_pass"]:
        sys.exit(2)


if __name__ == "__main__":
    main()
