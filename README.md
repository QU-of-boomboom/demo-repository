# 无人机与地面小车中继调度

固定航线仓库巡检：**1 架主机 + 2 台地面中继小车**。主机由飞控执行既定航线，地面站根据链路观测和路线进度调度小车到位、接力、切换与返场。

当前提供 PC 仿真、CPU 调度、通信协议骨架和 RSSI 预测训练流程。实机小车通信、真实无线测量、ROS/Gazebo 实启、FPGA 与 NPU 上板闭环仍待完成。

## 从这里开始

两小车演示只需 Python 3.10+ 标准库，在仓库根目录运行：

```powershell
# Windows PowerShell
.\run_cars.ps1
```

```bash
# Linux / macOS
bash run_cars.sh
```

运行后打开 `.cache/cars/demo/replay.html` 查看交互回放。报告与轨迹也在同一目录。演示运行一个完整往返，主机航程 46 m；小车状态为 `STANDBY / MOVING / READY`。

已保存的示例：[两小车运行报告](outputs_v2/p0_cars/demo/report.json)、[回放文件](outputs_v2/p0_cars/demo/replay.html)。HTML 需下载到本地后用浏览器打开。

## 代码导航

| 部分 | 入口 | 职责 |
|---|---|---|
| A：仿真展示 | [warehouse_relay_sim](warehouse_relay_sim/warehouse_relay_sim/README.md) | 巡检仿真 CLI 与 Python API |
| B：调度与通信 | [swarm_cooperation](swarm_cooperation/swarm_cooperation/README.md) | 共享仿真核心、候选调度、路由事务、MAVLink 与 ROS/SITL 接口 |
| C：预测训练 | [ai_relay_scheduler](ai_relay_scheduler/ai_relay_scheduler/README.md) | 数据生成、96 维链路预测、模型导出与策略比较 |
| 配置 | [两小车配置](mission_profile_1drone_2cars.json) / [P0 配置](mission_profile.json) | 独立选择车辆类型与任务参数 |
| 文档 | [文档索引](docs/README.md) | 架构、数据接口、部署与研发记录 |
| 测试 | [tests](tests/) / [完整验证入口](scripts/validate.py) | 三模块一致性、故障注入、协议与小车配置验证 |
| 结果 | [结果索引](outputs_v2/README.md) | P0 验收与两小车样例分开记录 |

A/B/C 共用 B 中的 `ideal_sim` 和 `relay_learning`，请克隆完整仓库。保留既有双层目录，避免改变导入与 ROS 包路径。

## 配置与结果范围

| 配置 / 记录 | 含义 | 使用边界 |
|---|---|---|
| `mission_profile_1drone_2cars.json` | 当前实机方向：主机 + 两台地面小车 | 通过入口脚本或 `MISSION_PROFILE` 显式启用；小车没有起飞与悬停稳定阶段 |
| `mission_profile.json` | `warehouse-abc-v3` 的 P0 relay-drone 回归基准 | 保留经过验收的默认语义与 5 轮 × 5 种子规则基线 |
| `legacy_v1` 与 C 历史输出 | 早期算法、模型和实验记录 | 不作为两小车架构或 V3 AI 的性能结论 |

路径质量按实际物理跳的最弱 RSSI 计算，**固定 +6dB 假设作废**。现有模型权重是历史回归样本；新架构的预测效果需要重新生成数据、训练并验证。模型输出链路风险，CPU 负责可行性检查和控制决策。

## 安装与完整验证

完整测试和 C 训练流程需要额外依赖；推荐使用独立虚拟环境：

```powershell
python -m venv .venv
# Windows
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe scripts/validate.py
```

```bash
# Linux / macOS
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/validate.py
```

`scripts/validate.py` 会生成自定义 MAVLink 方言，执行 P0 出口检查，再单独验证两小车配置。跳过测试同样视为失败。新结果写入 `.cache/validation/`，保存的验收证据保留在 `outputs_v2/`。P0 完整基线运行时间长于单轮演示。

训练入口与参数见 [C 模块说明](ai_relay_scheduler/ai_relay_scheduler/README.md)。新的训练输出使用 `outputs_v2/template/`，与历史样本分开。

## 当前进度

- 已提供：共享仿真契约、CPU 调度、ETA/READY 门控、观测有效性检查、路由 prepare/probe/commit/ACK、两小车配置及回归测试。
- 待完成：小车通信适配与定位反馈、真实链路测量、新架构模型训练、Icraft/NPU 与 FPGA 集成、实机闭环。
- 详细设计与历史实验见 [项目技术说明](README-无人机中继巡检.md)、[两小车架构](docs/单主机两小车架构_20261021.md) 和 [AI 辅助工作记录](llm_usage.md)。

仓库上传范围与结果取舍见 [仓库维护说明](docs/GitHub组织与仓库上传.md)。
