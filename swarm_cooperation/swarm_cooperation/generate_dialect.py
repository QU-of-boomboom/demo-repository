"""生成本地 dialect，无需修改 pymavlink 安装目录。"""

from pathlib import Path


def main() -> None:
    try:
        from pymavlink.generator import mavgen
    except ImportError as exc:
        raise SystemExit("请先执行 python3 -m pip install -r requirements.txt") from exc
    root = Path(__file__).resolve().parent
    output = root / "generated"
    output.mkdir(exist_ok=True)
    (output / "__init__.py").touch()
    options = mavgen.Opts(str(output / "swarm"), wire_protocol="2.0", language="Python3")
    if not mavgen.mavgen(options, [str(root / "protocol" / "swarm.xml")]):
        raise SystemExit("dialect 生成失败")
    print("已生成 generated/swarm.py（MAVLink 2）")


if __name__ == "__main__":
    main()
