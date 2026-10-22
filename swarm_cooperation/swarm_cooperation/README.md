# 主机 + 两地面小车协同层（历史三机兼容）

## 当前默认：五轮理想调度实验

`demo.py` 默认执行5个完整往返（10个单程）。首次巡线前建立全图节点；第一轮采用固定中继站位巡检，返程支持随机移除2–4块货箱或在时间窗口内手动移除；在线地图随观测更新，第二轮起枚举优化中继站位与通信路径。

```bash
python3 demo.py --mode random --seed 42
python3 demo.py --mode window --interactive
```

手动窗口开放后输入 `remove 1,1 2,2 3,4`。默认窗口为首轮返程开始运动后第2–12秒，货箱编号从1开始。默认1m/格，五轮主机总航程230m。结果含JSON/CSV与可在浏览器播放的 `replay.html`。

- [实验说明、窗口规则与AI接口](IDEAL_SIM.md)
- [Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 仿真途径](ROS2_GAZEBO.md)
- 当前两小车样例见 [报告与回放](../../outputs_v2/p0_cars/README.md)；随机/窗口回放可运行上方命令生成。

理想核心位于 `ideal_sim/`，候选规划与AI模型接口位于 `ideal_sim/scheduler.py`。ROS包名 `warehouse_relay_ideal`；已提供 `package.xml`、`setup.py`、启动文件与Gazebo世界。默认使用CPU枚举，未训练或搭载真实AI模型。

以下为冻结的**单轮协同/飞控协议快照**，通过 `--legacy` 运行 `legacy_v1/`（2m/格、停留3s，仅存档；当前 SITL 库仍为1m/1s）。默认理想仿真与 A/C 共用 `warehouse-abc-v3`，详见 [统一契约](../../docs/ABC统一契约与验收.md)。协议骨架的主机起降和任务流程仍独立计时。B 当前测试 **46/46**（含 pymavlink 两项），规则基线见 [outputs_v2/p0/baselines/RESULTS.md](../../outputs_v2/p0/baselines/RESULTS.md)。

这是独立的 Python 交付目录。当前实机方向是地面站协调 1 架主机和 2 台地面中继小车；历史三机 relay-drone 模型保留用于 P0 验收。程序面向后续 FMQL30TAI CPU 核上的 Linux 用户态程序。第一部分仅用 Python 标准库；第二、三部分使用 pymavlink。建议 Python 3.8 或更新版本。

## 1. 快速运行与文件索引

在此目录打开终端：

```bash
python3 demo.py --legacy
python3 -m unittest discover -s tests -v
```

Windows 可将 `python3` 换成 `python`。旧版每步推进 **0.1 s 模拟时间**，无需等待约两分钟；`python3 demo.py --legacy --realtime` 按真实时间演示。中文终端若乱码，可启用 Python UTF-8 模式，例如 `python -X utf8 demo.py --legacy`。新版理想实验要求Python 3.10及以上。

| 交付 | 文件 | 内容 |
|---|---|---|
| 第一部分 | `state_machine.py`、`geometry.py`、`demo.py` | 角色状态机、格线路径、RSSI、协同调度和完整演示 |
| 集中参数 | `config.py` | 地图、速度、计时、门限、地址、SITL 端口及超时 |
| 第二部分 | `PROTOCOL.md`、`protocol/swarm.xml`、`mavlink_peer.py` | 消息表、MAVLink 2 dialect、UDP 收发和可靠事务 |
| dialect 生成 | `generate_dialect.py`、`requirements.txt` | 用 pymavlink 生成本地 `generated/swarm.py` |
| 第三部分 | `SITL.md`、`sitl_bridge.py` | 多节点启动、任务上传、遥测与状态机挂接接口（历史三机 SITL 骨架） |
| 验证 | `tests/test_core.py`、`VALIDATION.md` | 正常流程、故障分支、协议与上传状态机验证 |

## 2. 第一部分设计

主机：`IDLE → TAKEOFF → SNAKE_SURVEY → PICK → RETURN → LAND`。

中继：`STANDBY → TAKEOFF → MOVING → SETTLING → HOVER → RELAYING → HANDOVER → RETURN`。稳定完成后才置 `ready`；`RETURN` 包含返场和降落，其 `home_reached` 标志表示返场完成。

链式接力时，中继1还承担中继2的上游转发，所以它在交接后先从 `HANDOVER` 回到 `RELAYING`，继续保留骨干链路。主机降落后，中继2先交回上游并返场，中继1等待中继2落地再返场。所有状态迁移输出触发原因。

主循环用一个线程同时推进各节点状态，每步每机各执行一次 `tick`；任务停留1秒、失联悬停5秒均以累计时间实现，不阻塞其他节点。当前新车模式没有起飞/稳定阶段，历史 relay-drone 模型保留。

### 地图、轨迹与 RSSI 假设

- 3×4 指货架格子数，因此有 x=0…3、y=0…4 共20个格点。主机去程与总 README 一致：`(0,0) → (0,4) → (1,4) → (1,0) → (2,0) → (2,4) → (3,4) → (3,0) → (3,4)`。沿四条竖格线巡检，首次经过 (3,4) 不结束巡检，完成最后一列并补飞回 (3,4) 后才进入任务停留。返航逆序回放实际走过的位置，遇到中途故障也从当前点原路返回。SITL 主机任务共用此航线。
- 去程长23格，默认1m/格时单程23m、往返46m；总任务时间还包含起降、终点停留和等待中继就位。
- 默认 **1 m/格**，通过 `Config.cell_m` 修改。默认主机1 m/s，中继2 m/s。模型只模拟平面运动与起降耗时，没有完整避碰、旋翼气流或 D435 感知。
- RSSI 为 `P0 - 10*n*log10(max(d,1m)/1m) - 12*穿越货架格数`，P0=-40 dBm、n=2.2。d为物理距离；格线和仅触角点不计遮挡，`geometry.link_rssi` 调用共享实现。
- 每段链路独立计算，五种路径取各自最弱一跳，无固定增益。默认终点直连约 -127.38dBm；门限反应及 ACK 延迟可能造成暂时断链，应按实报告。
- RSSI低于 -75 dBm 才启动下一架中继。主机在等待就位时保留当前安全链路并悬停；只有新中继 `ready` 且完整候选链路可用才提交路由。
- 新路径至少改善3 dB才切换；切回较短链路要求达到 -72 dBm。蛇形往返会多次回切，属于实际位置变化，避免了门限附近的小幅抖动。

### FAILSAFE 行为

主机从任何未完成的状态（包括起飞、取货、返航和降落过程）检测到 RSSI **严格低于 -88 dBm**，进入 FAILSAFE，原地保持5秒。恢复到 -85 dBm 或更好时，回到原状态并保留取货/起飞等阶段的已用时间；否则终止原任务并沿历史轨迹返航。

持续断链会锁定应急返航，防止同一次故障在 `RETURN ↔ FAILSAFE` 之间无限循环。已经落地、解除运行的终态不再重新起飞或反复触发故障。恢复链路不会取消已决定的应急返航。

在真机上，断链后的5秒计时、原路航点和失联执行器必须位于**主机机载伴随计算机/飞控侧**。仅在地面站运行这段逻辑无法在链路已断后控制无人机。ArduPilot 原生 GCS failsafe 与这里的应用状态机也需要统一配置和验证。

## 3. 第二、三部分运行入口

```bash
python3 -m pip install -r requirements.txt
python3 generate_dialect.py
```

分别开终端运行第二部分（先启接收端）：

```bash
python3 mavlink_peer.py --sysid 5
python3 mavlink_peer.py --sysid 6
python3 mavlink_peer.py --sysid 4
python3 mavlink_peer.py --sysid 250 --assign-demo
```

默认20秒结束。示例通过本机UDP直连验证消息收发，协议级 ACK 表示接收命令；没有执行起飞，也不伪造已就位状态。完整链式路由是设计接口，详见 [协议说明](PROTOCOL.md)。

按照 [SITL说明](SITL.md) 启动仿真后：

```bash
python3 sitl_bridge.py --duration 30
python3 sitl_bridge.py --upload --duration 30
```

第一条只读取各节点遥测，第二条上传任务并等待各自最终 ACK；均不自动解锁。`CollaborationHook` 明确标注“此处接入第一部分状态机”。SITL接口是历史三机骨架，不宣称已经实现三机闭环联飞；单主机两小车适配见 [架构文档](../../docs/单主机两小车架构_20261021.md)。

## 4. 迁移到 FMQL30TAI

先把此目录复制到板端CPU的Linux文件系统，运行纯Python演示。后续把 `SwarmPeer` 的本机UDP地址换成实际无线接口，把 `VehicleLink` 的连接串替换成串口或网络端点；MAVLink系统号独立于IP地址和UDP端口。

保留状态机、消息编解码、飞控适配三层边界。无线驱动输出每条相邻链路RSSI与采样时刻，经 CPU 协调器形成路径决策；已有PL链路监测模块可实现同一个测量接口。D435识别/取货结果接入 `PICK_CONFIRM`，实际硬件中不能只凭3秒计时宣告抓取成功。

真机适配必须用实际位置、飞行高度、armed和landed遥测完成状态转换；纯仿真的位置推进函数不作为真机位置来源。电量、避碰、定位失效、取货传感器以及中继路由提交握手应在各自适配器完善。
