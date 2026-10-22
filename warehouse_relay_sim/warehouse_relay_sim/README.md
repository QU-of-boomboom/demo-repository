# A：统一巡检仿真入口

当前 A 与 B/C 共用 `warehouse-abc-v3` 的几何、路线进度、观测入口和执行器。默认 1m/格、主机 1m/s、0.1s/步，竖列蛇形单程 23m，端点各停留 1s。总时间包含中继部署、链路等待和回收。

在本目录运行：

```powershell
python warehouse_relay_sim.py --mode none --rounds 1 --quiet --output outputs_v2/p0/a_cli
python warehouse_relay_sim.py --mode random --strategy predeploy
python warehouse_relay_sim.py --mode random --strategy reactive
python -m unittest -v
```

默认入口只需 Python 标准库。输出 `report.json`、`trace.csv`、`events.jsonl`、`decisions.jsonl` 和交互 `replay.html`。Python API 的 `create_simulation(Config(...))` 返回共享执行器；`simulate(Config(...))` 返回报告、轨迹和事件。

同轨迹跨模块验收在工作区根运行 `python verify_abc.py`；该验收同时加载 C 数据接口，需要工作区 NumPy/PyTorch 环境。详见 [统一契约](../../docs/ABC统一契约与验收.md)。

原先按列遮挡、+6dB、固定悬停中继的图文原型及测试保存在 `legacy_v1`。历史演示须显式运行 `python warehouse_relay_sim.py --legacy --no-plot`；历史 API、47s 数值和根目录旧图表不作为当前验收。旧版绘图才需要 Matplotlib。
