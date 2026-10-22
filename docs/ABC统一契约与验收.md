# A/B/C 统一契约与验收

更新：2026-10-21。当前仿真契约为 `warehouse-abc-v3`，权威实现为 `swarm_cooperation/swarm_cooperation/ideal_sim/contract.py`；唯一任务参数为根目录 [mission_profile.json](../mission_profile.json)。

新方向“1 架固定航线主机 + 2 台地面小车”使用独立 variant [mission_profile_1drone_2cars.json](../mission_profile_1drone_2cars.json)：`vehicle_kind=ground_car`，跳过 TAKEOFF/SETTLING，ETA 只算地面路程/速度；当前 P0 relay-drone 默认配置不因此改变。详见 [单主机两小车架构](单主机两小车架构_20261021.md)。

## 统一范围

A 的 CLI 与 Python API、B 默认理想仿真、C 的仿真/数据生成/策略评估，共用 `IdealSimulation`，并从 `mission_profile.json` 读取格长、速度、停留、航线、物理跳和门限。A 原来的固定中继绘图脚本及测试移入 `legacy_v1`，必须显式 `--legacy` 才会运行。历史 A/C 的 +6dB 结果与现有 `outputs`、`outputs_v2/smoke`、旧交付 ZIP 保留为历史记录，不能并入 V3 验收。

B 的 `--legacy` 现在直接运行冻结快照 `swarm_cooperation/swarm_cooperation/legacy_v1/`（2m/格、停留3s），仅作存档对照，不参与当前参数或验收。B 的单轮 `SwarmSimulation`/SITL 库仍为 README 统一的 1m/格、1s，二者不能混算。它仍有独立的主机起降/协议流程，完整任务时间不作为共享执行器的等价结果。SITL 尚未做真实飞控闭环验证。

| 项目 | V3 定义 |
|---|---|
| 唯一配置 | 根目录 `mission_profile.json`；A/B/C 默认值由 `ideal_sim.profile` 读取，不在模块内重复常数 |
| 格长、采样 | 默认 1m/格、0.1s/步；可改参数，但比较双方必须一致 |
| 航线 | 竖列蛇形，末尾补飞至 (3,4)；单程 23 格；返程严格逆序 |
| 速度 | 主机 1m/s，中继 2m/s；配置可覆盖 |
| 停留 | 每次去程终点 1s，每次返程到家 1s，包括最后一轮；0s 不额外耗一个时间步 |
| 时间预算 | 一轮纯飞行 46s + 任务停留 2s；总时间还包含部署、链路等待及中继回收，不能再写死 47s |
| 进度 | `progress_cells` 从当前单程起点累计；`progress_m = progress_cells × cell_m`；只在换单程时清零，等待时不增长 |
| 航段 | `segment_id` 为当前方向路线的 0 基编号；顶点归入到达该点的航段 |
| 总航程 | `mission_distance_m` 累计实际运动；不把部署/停留时间换算成主机航程 |
| 时间戳 | 从本次运行开始计的单调秒；每步后记录，第一条轨迹为 0.1s；`sample_time_s` 为采集时间，`time_s` 为处理/决策时间 |
| 遮挡 | 射线穿过每个已占用货架格的开集内部各扣 12dB；格线、仅触角点不算；移除格不再遮挡 |
| 物理跳 | G-M、G-R1、G-R2、R1-M、R2-M、R1-R2；仿真显式假设对称 |
| 路径 | 直连、经 R1、经 R2、经 R1→R2、经 R2→R1；路径质量取各跳最小 RSSI；无 +6dB |

例如从 (0,0) 到 (3,4) 穿过 6 个货架格，距离 5m，RSSI 为 `-40 - 22 log10(5) - 12×6 ≈ -127.37734dBm`。不能再用“3 列”代替 6 个格。

## 中继状态和路径提交

共享执行器按 `STANDBY → TAKEOFF → MOVING → SETTLING → READY` 推进。默认起飞 2s、稳定 0.5s；移动按格线和有限速度计算。只有 `READY` 中继才可组成业务路径；目标站位不是实际站位。主机部署等待纳入总时间。正在 TAKEOFF/MOVING/SETTLING 的中继不会出现在任何候选计划的可用站位中，重规划也不会把它改派到别处；即使它经过主机旁边，`path_quality` 仍因未 READY 返回不可用。

`relay_eta_s` 是在 `relay_eta_estimated_at_s` 时刻，用当时位置、目标、配置速度及剩余起飞/稳定计时计算的名义剩余时间。它不是事后实际到达时间，不回填、不使用未来到位事件；不包含串行移动排队、未完成的路径交接/ACK 等未知等待。`relay_eta_blocked` 标识当前路由依赖阻挡；ETA=0 也不代表就位，只有状态反馈 `READY` 才能就位。这些解释字段只加在原始状态/日志中，96 维编码、顺序和归一化完全不变。

切换采用 prepare → probe → commit → ACK。只有 epoch 匹配、未超时、等待满足、所有跳有效且中继就位时才提交新路径。缺失/错误/迟到 ACK 和探测失败均保留旧路径。旧路径的上游中继不能移动，直到已确认的新路径不再依赖它。主机任务完成到家后才解除最后的业务依赖。

## 观测入口与降级

`valid_observation` 是在线窗口与真实数据未来标签的共同有效性入口。缺少链路项、`valid != true`、缺时间戳、过期、来自未来、NaN/Inf、非数值均无效；不沿用上一次良好值冒充当前观测。历史帧缺口插入无效槽，禁止压缩时间。重复链路 ID 或未知 ID 拒绝，时间帧必须严格递增。

`path_quality` 再检查每个物理跳的有限性，避免 NaN 位于不同跳位时被 `min()` 忽略。无可行路径返回 `None` 并记录 `link_degraded`，保留旧路由标识不代表链路仍然可用。持续缺测先本地悬停，超时后原路应急返航；任务记录为失败。仿真中的真实物理断链与观测缺失分开统计，不能将有物理信号但缺少观测称为已确认通信。

## 两条规则基线

- `reactive`：每条新观测检查当前路径门限；直连低于 -75dBm 连续 N 个槽（配置 `reactive_on_slots`，默认 3）才切中继，恢复直连需连续高于 -72dBm（3dB 回差）；只按当前主机位置选择站位。去返程及后续轮次均不偷偷预部署，首次门限反应不受 5s 周期规划器延迟。
- `predeploy`：从第一程开始使用已知路线与已观测地图，在飞行前、单程交界和周期规划时选择站位；使用当前 ETA/部署时间判断能否在需求前到位，不使用未来真实 RSSI 或尚未观测的移箱结果。

两者共用速度、起飞/稳定、握手延迟、有效性门槛、电量及失败处置。按相同场景种子配对，噪声和丢包以时间/链路编号索引；策略改变的位置和时间会重新计算链路。报告同时列出完成状态、航程、断链次数/时长、切换、中继占用率和到位成功率，不能拿提前失败导致的少量累计断链宣称收益。默认按 `mission_profile.json` 的 5 个种子各跑 5 个完整往返，存档在 [outputs_v2/p0/baselines](../outputs_v2/p0/baselines/RESULTS.md)；AI 评估同时输出相对两条基线的差值；本次规则实验不证明 AI 胜出。

## 复现

在工作区根目录执行（可将 `python` 换为 `.venv-rssi/Scripts/python.exe`）：

```powershell
python verify_abc.py
python diff_abc.py
python check_dod.py
python warehouse_relay_sim/warehouse_relay_sim/warehouse_relay_sim.py --mode none --rounds 1 --quiet --output outputs_v2/p0/a_cli
python swarm_cooperation/swarm_cooperation/demo.py --mode none --rounds 1 --quiet --output outputs_v2/p0/b_cli
python ai_relay_scheduler/ai_relay_scheduler/simulator.py --mode none --rounds 1 --quiet --output outputs_v2/p0/c_cli
```

在 B 目录运行两条规则对照（默认读取 `mission_profile.json` 的 5 轮、5 个种子）：

```powershell
python -m ideal_sim.compare_baselines
python -m unittest discover -s tests -v
```

在 C 目录运行：`python -m unittest pipeline_v2.tests -v`。

跨模块验收覆盖独立手算遮挡/RSSI、1m/2m 缩放、正返程全部 462 个位置、移箱前后五种拓扑、完整轨迹/事件/训练帧、停留和时间戳、中继到位和 ACK、无效观测和降级。结果见 [P0 验收单](../outputs_v2/p0/README.md)，规则实测见 [RESULTS.md](../outputs_v2/p0/baselines/RESULTS.md)。

## 数据和权重兼容

特征张量仍为 `relay-link-v2` 的 96 维、两时域输出，但仿真契约另行版本化。数据源哈希新增 `contract.py`；数据与代码不一致时，训练/导出拒绝复用。新的 `deployment.json` 必须声明 `simulation_contract_version: warehouse-abc-v3`，在线加载拒绝旧契约权重。现有旧权重未在本次重训，不能用旧报告替代 V3 的 AI 对照。

本次验收限于 PC 仿真与协议单元测试，不包含实飞、ROS/Gazebo 实启或 NPU 板端结果。

## 出口检查

`diff_abc.py` 对真实逐步位置（含移动中继）输出 A/B/C 各自的六跳表，逐字段比较位置、距离、穿列/穿格、六跳 RSSI；在移箱事件时间戳应用地图变化。穿列是诊断统计，损耗仍按穿格计算。差异或任务失败时退出码非零；测试另注入只改变 R1-R2 非瓶颈跳的错误，验证不会被路径最小值掩盖。标准去程 23 格、1m/s 的 230 个 0.1s 槽由 `tests/test_diff_contract.py` 固化，pytest/unittest 可直接收集。

`check_dod.py` 拒绝任何跳过的测试，同时运行 B 全套、A/C 当前测试与 A/C 历史兼容测试，按 `mission_profile.json` 重跑 5 轮×5 种子的两条规则基线并归档。`scope_lock_before/after.json` 验证冻结的特征源文件、完整特征清单与既有权重哈希未改变。运行 B 标准飞控帧测试需先在当前环境安装 `swarm_cooperation/swarm_cooperation/requirements.txt`。
