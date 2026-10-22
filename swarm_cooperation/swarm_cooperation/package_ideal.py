"""打包B交付目录，排除环境、缓存及历史日志；保留两份新版实验产物。"""
import argparse
import hashlib
from pathlib import Path
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    target = args.output.resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    excluded = {"__pycache__", ".venv", "venv", ".git", "logs", "build", "install", "log", ".pytest_cache"}
    files = []
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        if not path.is_file() or path.is_symlink() or any(p in excluded or p.endswith(".egg-info") for p in relative.parts):
            continue
        if path.suffix in (".pyc", ".zip", ".sha256") or path.resolve() == target:
            continue
        if relative.parts[0] == "outputs" and relative.parts[1] not in ("ideal_random_seed42", "ideal_window_seed42"):
            continue
        files.append((path, relative))
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path, relative in files:
            archive.write(path, str(Path("warehouse_relay_ideal") / relative))
    with zipfile.ZipFile(target) as archive:
        bad = archive.testzip()
        if bad:
            raise RuntimeError(f"ZIP校验失败：{bad}")
    digest = hashlib.sha256(target.read_bytes()).hexdigest()
    target.with_suffix(".zip.sha256").write_text(f"{digest}  {target.name}\n", encoding="utf-8")
    print(f"{target}\n{len(files)} files, {target.stat().st_size:,} bytes\nSHA256 {digest}")


if __name__ == "__main__":
    main()
