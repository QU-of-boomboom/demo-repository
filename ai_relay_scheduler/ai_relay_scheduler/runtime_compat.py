"""仅对Windows Conda解释器预载系统VC运行库，不修改任何系统/Conda文件。

本机Conda自带14.27运行库会抢先于系统14.51被加载，导致ONNX DLL失败。
在第三方数值库之前加载Windows自带的同名运行库，可避免这一旧版本冲突。
"""
import ctypes
import os
import sys
from pathlib import Path

_HANDLES = []
if sys.platform == "win32" and (Path(sys.base_prefix) / "conda-meta").is_dir():
    system = Path(os.environ["SystemRoot"]) / "System32"
    for name in ("msvcp140.dll", "msvcp140_1.dll", "msvcp140_2.dll", "vcruntime140.dll", "vcruntime140_1.dll"):
        path = system / name
        if path.is_file():
            _HANDLES.append(ctypes.WinDLL(str(path)))
