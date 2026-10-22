# B legacy_v1 快照（2m/格、停留 3s）

这是从 `deliverables/B_ideal_ros2_ubuntu22.04.zip` 原样取出的历史单轮协同/协议状态机，用于 P0 要求的“冻结 legacy、只做存档对照”。

- 参数：`cell_m=2.0`、`pick_s=3.0`、主机 1m/s、中继 2m/s；单程 46m，往返 92m。
- 本目录不读取根目录 `mission_profile.json`，不参与 `warehouse-abc-v3` 验收，也不得用它的结果与当前 1m/格基线混算。
- 入口（在 B 模块目录运行）：

```powershell
python legacy_v1\demo.py --legacy
```

或从工作区根目录：

```powershell
python swarm_cooperation\swarm_cooperation\legacy_v1\demo.py --legacy
```

当前 `demo.py --legacy` 也转发到这里；当前五轮理想仿真仍走 `ideal_sim` 的 1m/格契约。

历史记录：[B验证记录_更新前](../../../docs/history_20261020/B验证记录_更新前.md)。
