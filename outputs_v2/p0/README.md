# P0 出口验收单

日期：2026-10-21。契约：`warehouse-abc-v3`。唯一参数文件：[mission_profile.json](../../mission_profile.json)，SHA256 `1687015d91b7cb17a545312178ef4eea63fe5d44b806ba588ec11eb58a7ab40b`。

`python check_dod.py` 退出码 **0**；**101 项测试通过，0 失败、0 跳过**。机器可读结果见 [dod_report.json](dod_report.json)。

## 出口检查单

- [x] **三模块同轨迹 diff 通过**：A/B/C 各 607 个时刻、3642 条物理跳记录，位置、距离、穿列/穿格、六跳 RSSI 零差异（`mismatch_count={"B":0,"C":0}`）；完整任务 46m，含移动中继与移箱。另有标准去程 23 格、1m/s 的 230 个 0.1s 槽 pytest 用例。证据：[diff_report.json](diff/diff_report.json)、[A](diff/A_six_hops.csv)/[B](diff/B_six_hops.csv)/[C](diff/C_six_hops.csv)、[test_diff_contract.py](../../tests/test_diff_contract.py)。
- [x] **移动中继无假就位，ETA 是“当时估计”**：`STANDBY → TAKEOFF(2s) → MOVING → SETTLING(0.5s) → READY`；TAKEOFF/MOVING/SETTLING 中继不会出现在任何候选可用站位，重规划也不会改派在飞中继。`relay_eta_s` 用当时位置/目标/速度/剩余计时计算，带 `relay_eta_estimated_at_s`，不回填未来到达。移动中继即使经过主机旁，`path_quality` 仍返回不可用。测试见 `tests/test_abc_alignment.py` 的 `MotionGateAcceptance` 与 `ExecutionAcceptance`。
- [x] **缺测/过期/NaN 注入通过，降级入口有记录**：新增逐跳 1–3s 观测中断注入；缺失/过期/NaN/Inf/非数值/未来时间戳均按无效处理，不沿用上次良好值；无可行路径时 `propose` 返回 `None`、保留旧路径标识但不宣称可用，仿真记录 `link_degraded`，持续缺测进入 `LINK_HOLD` 并最终机载返航。测试见 `ObservationAcceptance`。
- [x] **两条规则基线跑出指标表并存档**：[RESULTS.md](baselines/RESULTS.md)。`reactive` 与 `predeploy` 各 5 个种子、每场景 5 个完整往返，10 次任务全部完成；表中含断链次数/时长、最长断链、切换、中继占用率、到位成功率、总时长。原始 JSON 与回放见 [baselines/](baselines/)。
- [x] **B 测试全绿（含 pymavlink 两项），A/C 不回归**：B **46/46**；A **24/24**、C **8/8**；历史 A **12/12**、历史 C **10/10**、冻结 B legacy 快照 **1/1**。合计 **101 项，0 失败、0 跳过**。见 [tests_B.json](tests_B.json) 等。
- [x] **README 更新**：根 README 标明 `warehouse-abc-v3`、`mission_profile.json`、**固定 +6dB 假设作废**，历史结果与 V3 隔离。

## P0 附加要求

- **B `ideal_sim` 为统一基准**：A/C 的当前 CLI、仿真 API、数据生成均调用同一 `IdealSimulation`；`diff_abc.py` 按 0.1s 槽比对三方。
- **唯一任务配置**：格长、速度、停留、航线、12 个占用格、六条物理跳、五条路径、门限、站点、基线种子全部在 `mission_profile.json`；`ExperimentConfig`、`contract.py`、`compare_baselines.py` 均从它读取，`check_dod` 校验 profile 与 config/contract 一致。
- **legacy 隔离**：`demo.py --legacy` 现直接转发到冻结快照 `swarm_cooperation/swarm_cooperation/legacy_v1/`（2m/格、3s 停留、125.8s 历史流程），仅作存档；当前五轮契约和单轮 `SwarmSimulation`/SITL 库仍为 1m/格、1s。`check_dod` 的 B_legacy 套件断言 2m/3s 与单轮完成，历史不参与当前基线。
- **调度侧观测纪律**：`RouteController` 对每条候选路径逐跳调用 `path_quality`；缺失/过期/NaN/Inf 或中继未 READY 一律不可用；无可行路径返回 `None`，旧路径保留但标记不可用。
- **失联入口**：所有可行路径不可用 → `link_degraded`；持续缺测 → `LINK_HOLD` → 5s 后 `onboard_emergency_return`；事件均落盘。这是将来真实心跳逻辑的仿真替身。
- **旧链路保护**：HANDOVER 的 prepare/probe/commit/ACK 未确认前，旧路径保持；错误/迟到 ACK、超时、探测失败都不释放旧路径。
- **96 维冻结**：`scope_lock_after.json` 核对 `features.py`、特征清单与既有 ONNX/PT 权重哈希未变；本次未重训、未改 Icraft/FPGA。

## 复现

```powershell
.venv-rssi\Scripts\python.exe check_dod.py
.venv-rssi\Scripts\python.exe diff_abc.py
.venv-rssi\Scripts\python.exe tests\test_diff_contract.py -v
.venv-rssi\Scripts\python.exe swarm_cooperation\swarm_cooperation\demo.py --legacy
```

规则基线（B 目录）：

```powershell
python -m ideal_sim.compare_baselines
```

## 范围限制

这是 PC 仿真与协议单元测试验收，不是实飞、ROS/Gazebo 实启或 NPU 板端结果。基线表中 `predeploy` 的断链次数并非每个种子都低于 `reactive`（如场景 0、4），但断链时长与最长连续中断更低；与 AI 比较时必须同时报告两条基线，不能只截取有利指标。规则基线不含预测模型，不代表 AI 已胜出。

## Git 上传选录说明

本目录保留测试摘要与日志、三份六跳 CSV、规则基线指标和首场景 report。完整 trace/events/decisions/replay 未上传；运行根目录 `python scripts/validate.py` 后在 `.cache/validation/p0/` 查看新生成的完整证据。
