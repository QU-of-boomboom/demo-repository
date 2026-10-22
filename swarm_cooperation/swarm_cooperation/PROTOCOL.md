# 集群通信协议 v0.1

> 2026-10-21 说明：本文是历史“三节点集群”自定义协议，不是赛题 PDF 的官方协议。当前硬件方向为 1 架主机 + 2 台地面小车，消息角色、`MOVE/ARRIVED/READY` 流程和小车适配见 [单主机两小车架构](../../docs/单主机两小车架构_20261021.md)。

## 1. 选型与作用域

选用 **MAVLink 2 私有 dialect**，消息号为本项目内部约定的 **60000–60006**。它们不是官方分配的专属区间；发布公共 dialect 前需申请范围并检查与其他 dialect 的冲突。不要把 42000–42999 当作通用自定义消息区间，该消息范围已用于 `icarous.xml`。[官方消息ID规则](https://mavlink.io/en/guide/define_xml_element.html#message-id-ranges)

选 dialect 的原因：保留有符号RSSI、整数事务号、路由版本和明确字段语义，不受 `COMMAND_LONG` 七个float参数的限制。独立 `swarm.xml` 只包含集群应用消息，由伴随计算机与地面站解析；飞控命令仍使用标准 `ardupilotmega` dialect。库存 ArduPilot 不会自动执行这里新增的 `RELAY_ASSIGN`。

所有消息的帧头 `sysid/compid` 是**原始发送者**，转发不得改写。下表中的 C 为公共前缀：`session_id:uint32`（进程启动随机会话号），`seq:uint32`（应用序号），`time_boot_ms:uint32`（源节点启动后的单调毫秒，模2^32）。单位写在字段名或说明中。XML是字段定义的唯一依据，线上的字段排列由mavgen生成，不手工拼包。

## 2. 消息表

| 名称 / 自定义ID | 字段定义（除C外） | 发送方 → 接收方 | 频率 | 超时与重传 |
|---|---|---|---|---|
| `SWARM_HEARTBEAT` / **60000** | `role:u8`，`state:u8`，`ready:u8` | 地面站、三机伴随计算机 → 全体 | 1 Hz | 连续3 s未收到则不可达；周期覆盖，不重传旧心跳 |
| `SWARM_POSITION` / **60001** | `x,y,z:float` 米，`vx,vy,vz:float` m/s，统一仓库ENU | 三机 → 地面站 | 10 Hz | 1 s无新值则失效；不重传过期位置 |
| `SWARM_RSSI` / **60002** | `peer_system:u8`，`rssi_cdbm:i16`，`sample_age_ms:u16` | 每个无线接收节点 → 地面站；必要时共享给邻机 | 每条邻接链路10 Hz | 1 s失效；不以最后一个强信号无限续命；-32768为无效 |
| `RELAY_ASSIGN` / **60003** | `target_system,target_component:u8`，`route_epoch:u32`，`operation:u8`（1准备/2提交/3释放），`upstream_system,downstream_system:u8`，`x,y,z:float` 米 | 地面站 → 指定中继的伴随计算机 | 事件触发 | 0.5 s无ACK重发，最多额外3次；失败保留旧路径；每阶段独立seq、同一epoch |
| `PICK_CONFIRM` / **60004** | `target_system,target_component:u8`，`task_id:u32`，`shelf_id:u16`，`result:u8`（0成功/1失败） | 主机伴随计算机 → 地面站 | 抓取结果事件 | 0.5 s无ACK重发，额外3次；事件相同seq，重复确认不重复计数 |
| `SWARM_FAILSAFE` / **60005** | `target_system=0,target_component=0:u8`，`reason:u8`（1弱链路/2心跳超时/3遥测陈旧），`action:u8`（1悬停/2原路返航/3恢复），`rssi_cdbm:i16`，`hold_ms:u16` | 检测故障的节点 → 全体可达节点 | 事件首发；0.5 s间隔再发3次 | 不等待广播ACK，避免ACK风暴；相同事件序号去重；失联执行不依赖广播成功 |
| `SWARM_ACK` / **60006** | `target_system,target_component:u8`，`ack_session_id:u32`，`ack_seq:u32`，`ack_message_id:u32`，`result:u8`（0已接收/1拒绝） | 可靠消息接收者 → 原发送者 | 收到可靠消息时立即发送 | ACK不再被ACK；重复事务重复回ACK、不重复执行 |

`role`：0地面站、1主机、2中继。地面站 `state=0`。主机状态数字依次为 IDLE=1、TAKEOFF=2、SNAKE_SURVEY=3、PICK=4、RETURN=5、LAND=6、FAILSAFE=7；中继为 STANDBY=1、HOVER=2、RELAYING=3、HANDOVER=4、RETURN=5，对应 Python Enum。

`rssi_cdbm=-7500` 表示 -75.00 dBm。帧头源节点是**测量接收方**，`peer_system` 是被测发送方，不能混同方向。双向通信质量需取两方向较差者，再对所有跳取最小值。第一部分为了可重复演示假定链路对称。

心跳和采样超时以本机接收单调时间计算，不直接相减两台机器未经同步的 `time_boot_ms`。真实接入层还需验证同一会话的应用seq递增、抛弃迟到遥测，并计入转发/传输延迟。地面站可结合 TIMESYNC 校准采样年龄；超过有效期的测量视作断链。

自定义心跳服务于集群业务，飞控与GCS还须独立发送标准 **HEARTBEAT(ID 0)**。GPS_RAW_INT(24)、ATTITUDE(30)、GLOBAL_POSITION_INT(33)和任务协议均沿用标准消息。标准 `RADIO_STATUS.rssi` 不保证是直接可用的负数 dBm，必须按照具体无线驱动标定转换。[标准消息定义](https://mavlink.io/en/messages/common.html)

## 3. 系统号、组件号与地址

| 节点 | sysid | 飞控compid | 协同应用compid | 本地协议演示UDP端口 | SITL直接TCP端口 |
|---|---:|---:|---:|---:|---:|
| FMQL30TAI地面站 | 250 | — | 190 | 16000 | TCP客户端 |
| 主机（D435） | 4 | 1 | 191 | 16004 | 5790 |
| 中继1 | 5 | 1 | 191 | 16005 | 5800 |
| 中继2 | 6 | 1 | 191 | 16006 | 5810 |

TCP端口对应 `-I3 --count 3 --auto-sysid`。无线网络的示例IP可按地面站10.42.0.1、中继1 .11、中继2 .12、主机 .20规划，配置不等同于已存在实际链路。实机可更改系统号，但 `config.py`、飞控系统ID、路由表必须一致。

逻辑路径按距离延伸：

```text
近端：地面站 ↔ 主机
中段：地面站 ↔ 中继1 ↔ 主机
远端：地面站 ↔ 中继1 ↔ 中继2 ↔ 主机
```

此拓扑表示独立的点对点无线边，不能因四个节点都在同一IP子网就认为链式转发已实现。

## 4. 转发与交接规则

1. 每个中继有上游、下游接口和本地飞控/应用端点。稳定远端拓扑中，地面站去4或6走中继1；中继1去4或6走中继2，去250走地面站；中继2去4走主机，去250或5走中继1；主机去其他节点走中继2。发给中继自身sysid的消息按compid交本机飞控或协同应用。
2. 单播由 `target_system/target_component` 决定目的地。广播与无目标遥测转发到其他允许接口，不回送入接口。正常链路维护树结构，交接时原路径保留到新路径提交成功，避免断链窗口。
3. 保持原始 MAVLink 帧字节、源sysid/compid、帧序号及签名，不能将转发报文重新编码成“中继发出”。MAVLink没有通用TTL字段，底层IP设置有限TTL；若使用自建链路封装，可在外层加hop_limit=4。不要修改内层已签名帧。
4. 原始帧转发去重可用源sysid、compid、帧seq和完整帧摘要，缓存约2 s；单独用8位帧seq会在回绕时误丢合法消息。应用事务去重用源sysid、compid、session、消息ID、应用seq，示例缓存30 s，覆盖本例2 s的完整重传窗口。
5. 只有地面站250/190能发送中继分配，只有主机4/191能报告取货。地址白名单不是加密认证，实际无线网络接入时应加 MAVLink 2 签名或受认证的传输通道。

这些转发原则依据 [MAVLink路由规范](https://mavlink.io/en/guide/routing.html)。自定义消息的路由器需理解本项目dialect，或在独立链式IP通道中透传整个帧；不要假定库存飞控能解析未知私有消息的目标字段。

应用交接：地面站发 PREPARE，接收方验证epoch和位置参数后回ACK；新中继完成起飞、位置到达、上游双向探测，周期上报 `ready=1`。地面站只有在位置/RSSI/心跳均新鲜时才发 COMMIT；收到提交确认后更新主机路径。旧中继仍是新链路上游时不得 RELEASE，只有无依赖时才释放返航。阶段超时保留旧路并令主机悬停。

## 5. 代码与完成边界

`mavlink_peer.py` 实现七类消息的编解码调用、单播ACK、0.5 s重传、最多3次重试、广播重复发送和应用去重。`SwarmPeer.poll()` 非阻塞，适合接入0.1 s协同主循环。`on_message` 返回0仅表示命令已接收入队，不表示到位或抓取成功。

本机示例用UDP直接扇出消息；链式路由、epoch提交状态、飞行执行和GPS/RSSI新鲜度到状态机的绑定是后续适配点，文档中的完整交接规则不能视为已经接上真实电台。第一部分以 `ready` 和一次主循环内的路由提交模拟这些确认条件。

运行命令见README。重新生成dialect后，所有端点必须使用同一份XML生成的代码，否则CRC_EXTRA或字段布局可能不一致。
