# ArduPilot 三机 SITL 对接

## 1. 正确解释 -I3

`sim_vehicle.py -I3` 只启动**实例编号3的一架**，不是三架。要从编号3起三架，使用 `-I3 --count 3 --auto-sysid`。当前官方启动脚本按 `instance+1` 分配系统号，因此实例3、4、5对应sysid 4、5、6。[sim_vehicle.py 源码](https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/sim_vehicle.py)

在已准备好ArduPilot构建环境的Linux/WSL终端执行，路径按本地克隆位置调整：

```bash
cd /path/to/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad -I3 --count 3 --auto-sysid --location CMAC --auto-offset-line 90,2 --no-mavproxy
```

`--auto-offset-line 90,2` 给三机不同的出生点，沿向东每架间隔2 m。`--no-mavproxy` 让本交付的Python程序直接连接各飞控TCP端口。三机地面位置因此不完全重合；Python纯仿真把起点抽象为同一个(0,0)，真实/SITL适配需保留各机出生点与共同仓库原点的区别。[官方多机SITL说明](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#swarming-with-sitl)

| 角色 | instance | sysid | SERIAL0 TCP：5760+10×I | MAVProxy默认UDP输出：14550+10×I | SITL RC输入：5501+10×I |
|---|---:|---:|---:|---:|---:|
| 主机 | 3 | 4 | 5790 | 14580 | 5531 |
| 中继1 | 4 | 5 | 5800 | 14590 | 5541 |
| 中继2 | 5 | 6 | 5810 | 14600 | 5551 |

后三列中，UDP输出只有启用MAVProxy才存在，RC输入不是MAVLink遥测端口。共同MAVProxy进程可能将多架飞机报文转发到同一个输出，接收端仍须按sysid区分。本交付选直接TCP，每个端口期望固定系统号，并校验源compid=1。若选 `-I0 --count 3`，对应系统号1/2/3及端口5760/5770/5780，需要同步修改 `config.py`。

各版本SITL的系统参数名称可能使用 `SYSID_THISMAV` 或 `MAV_SYSID`；此处通过 `--auto-sysid` 赋值，并以收到的心跳头为准。[启动脚本端口与实例实现](https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/sim_vehicle.py)

## 2. 连接与任务上传

在交付目录执行：

```bash
python3 -m pip install -r requirements.txt
python3 sitl_bridge.py --host 127.0.0.1 --duration 30
python3 sitl_bridge.py --host 127.0.0.1 --upload --duration 30
```

建议SITL与Python均在同一WSL/Linux环境运行；跨Windows/WSL时，`--host` 使用实际可达的SITL地址，不能假定回环地址自动跨越两个网络命名空间。

`VehicleLink` 封装连接、标准GCS心跳、消息速率请求、GPS/姿态读取、单播命令、GUIDED目标和任务上传。三机均由非阻塞 `poll()` 推进；单机上传不阻塞其他两架的心跳与遥测处理。

上传协议：`MISSION_COUNT → 飞控MISSION_REQUEST_INT(seq) → GCS MISSION_ITEM_INT(seq)`，逐条请求直到最终 `MISSION_ACK(MAV_MISSION_ACCEPTED)`。代码支持重复和非顺序请求、旧版 `MISSION_REQUEST/MISSION_ITEM` 回退、非法seq拒绝、目的地址检查、2 s重试和120 s总期限。只有航点全部发送且最终ACK接受，才报告成功。[官方任务协议](https://mavlink.io/en/services/mission.html)

MAVLink任务上传没有本项目自定义的事务号，因此同一飞控同时只允许一个上传者；请避免另一个地面站同时上传任务。[ArduPilot任务接口](https://ardupilot.org/dev/docs/mavlink-mission-upload-download.html)

本例航点采用 `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`，x/y分别为纬度/经度乘10^7，z为相对HOME高度米。任务第0项保留HOME占位；随后是起飞、航点和动作。ENU转换以主机的 `HOME_POSITION` 为共同仓库原点；相对高度仍相对于各自HOME，真实不同地面高程需补偿。

主机任务内容：起飞 → 蛇形到(3,4) → LOITER_TIME 3 s → 逆序蛇形 → LAND。中继任务：起飞 → 进入共同原点并沿格线到指定位置 → LOITER_UNLIM。中继释放后的返航由协作层下发后续GUIDED格线目标；这里不把无限悬停之后的任务当作自动可执行返航。

**上传任务不等于执行任务。** 脚本不解锁、不设置AUTO；`arm()`、`set_mode()`、`takeoff()`、`goto()`、`hold()`、`land()` 均已提供接口。命令队列按ACK串行推进并保存结果，上层必须检查ACK结果与实际遥测，拒绝或超时后不得假定动作成功。AUTO任务本身不包含完整的RSSI驱动协同，这一部分是任务协议骨架。

## 3. 遥测与状态机挂接

| 数据 | 标准消息 | 解释与用途 |
|---|---|---|
| 连接/飞行模式/解锁 | HEARTBEAT | 匹配sysid与compid，3 s陈旧则不可达；base_mode中的armed位决定是否已解锁 |
| 原始GPS | GPS_RAW_INT | lat/lon除1e7、alt除1000；fix_type用于定位质量判定 |
| 融合位置 | GLOBAL_POSITION_INT | 与HOME统一坐标后供航点到达判断；relative_alt除1000为米 |
| 姿态 | ATTITUDE | roll/pitch/yaw为弧度；需要显示角度时再转换 |
| 起点 | HOME_POSITION | 三机各自HOME，主机HOME兼作仓库原点 |
| 着陆状态 | EXTENDED_SYS_STATE | landed_state与armed位联合确认落地 |

`CollaborationHook.on_tick()` 包含明确的“**此处接入第一部分状态机**”注释。具体挂接流程：

1. 将实测位置转换到统一格点坐标，将第二部分的各跳RSSI及年龄转换成有效链路质量。缺失/过期数据不得继续使用旧强信号。
2. 把第一部分的模拟运动学从决策逻辑中抽离；实机位置由遥测更新，决策输出离散动作，不调用模拟器替飞控推进位置。
3. 起飞依次等待GUIDED模式、arm ACK、armed位、起飞高度；巡检/原路返航逐个发送GUIDED目标，确认实际到达再移动到下一航点。
4. PICK保持位置3秒并接入夹爪实际确认；FAILSAFE立即保持、等待恢复，持续失联则执行机载保存的历史航点。ArduPilot原生RTL可能直飞HOME，**不等价于仓库原路返航**。
5. LAND根据着陆状态和解除armed确认完成；中继到达目标后仍须检查上游链路再发布ready，路由交接ACK不能替代实际到达。

`build_mission()` 提供整条预装任务，`goto()` 提供逐点GUIDED控制，两者供不同验证阶段选择。正式闭环应明确由哪一种执行方式控制航线，不能同时让AUTO和协同层独立推进位置。

仓库实际飞行不应假设GPS可用，需要接入视觉/里程计定位并统一坐标；D435数据处理及外部定位消息不在本次骨架范围。应用FAILSAFE需要在机载端保存和执行；地面站链路断开后无法补发返航航点。

## 4. 验证边界

本交付没有启动或飞行SITL。已进行的离线测试、实际pymavlink版本和日志见 [VALIDATION.md](VALIDATION.md)。多机实飞、无线链式转发、夹爪动作、定位闭环及ArduPilot原生failsafe参数联调仍需在后续环境验证。
