# 验证记录

## 2026-10-12 五轮理想调度改包

Windows、Python 3.13.13；核心代码另按Python 3.10语法解析，以适配Ubuntu 22.04系统Python。

- 测试：32项，30项通过；2项旧标准飞控帧测试因缺少pymavlink跳过。新增覆盖五轮10段、230m、预置全图、逐轮逆序、首次归位前变更观测、窗口边界、非法输入、幂等、未提交任务、AI无效输出回退、风险优先约束及产物导出。
- 随机示例：`python -X utf8 demo.py --mode random --seed 42 --quiet`，移除 `(3,4)`、`(2,1)`，五轮完成、主机230m、全阶段断链0s。
- 窗口接口脚本示例：`python -X utf8 demo.py --mode window --scripted-command examples/window_command.json --quiet`，返程第3秒提交3块移除，五轮完成、主机230m、全阶段断链0s。此为自动注入手动接口的可复现测试，未宣称真人交互或ROS话题联调已完成。
- 两个示例的第二轮同场景对照中：更新地图避免了旧地图枚举的一次2m调位，断链/弱链路与中继转发占用采样数持平；相对首轮固定站位方案持平。完整数值保存在各自 `outputs/ideal_*/report.json`，不把持平写成改善。
- 提供 `replay.html`、`trace.csv`、`events.jsonl`、`decisions.jsonl`。ROS节点、launch、package.xml和SDF已静态检查，未实际运行Ubuntu、ROS 2或Gazebo。

## 2026-10-12 去程路线修改验证

主机去程已改为总 README 指定的竖列蛇形及末尾补飞，仿真与 SITL 任务生成共用 `snake_path()`。默认仍为2m/格，单程46m、往返92m。

- `python -X utf8 -m unittest discover -s tests -v`：20项测试，18项通过，2项因缺少 pymavlink 跳过。覆盖完整去程、首次经过 (3,4) 不提前结束、实际轨迹逆序返航，以及 SITL 去返程航点顺序。
- `python -X utf8 demo.py`：主机最终 LAND，两中继均返场；15次状态迁移、0次 FAILSAFE、最低有效 RSSI -79.5dBm、总模拟时长125.8s（包含起降、任务停留及等待中继）。
- 未运行三机 SITL；以下记录及 `logs/demo.log`、`logs/tests.log` 为旧横行路线的历史结果。

## 2026-10-06 历史验证

验证日期：2026-10-06。环境：Windows、Python 3.11.7；可选协议验证使用隔离安装的 pymavlink 2.4.50。

## 第一部分实际运行

`python -X utf8 demo.py` 已完成整个正常流程：

| 项目 | 结果 |
|---|---|
| 主机最终状态 | LAND，landed=True |
| 取货 | 在(3,4)悬停3秒，模拟取货成功 |
| 两架中继 | 均已返场落地 |
| 状态迁移事件 | 23次 |
| FAILSAFE次数 | 0次 |
| 最低有效链路RSSI | -79.5 dBm，高于-88 dBm断链门限 |
| 模拟时长 | 109.8秒，步长0.1秒 |

完整控制台输出保存在 `logs/demo.log`。`--realtime` 仅改变墙钟等待，不改变状态机的模拟时间步长。

## 自动验证

`python -m unittest discover -s tests -v`：**19项测试全部通过**（含可选依赖），输出保存在 `logs/tests.log`。

- 13项基础测试覆盖正常闭环、格线路径、完整逆序返航、3秒取货、各活动状态断链、门限与恢复回差、短暂失联恢复、5秒悬停后持续断链返航、多跳最弱链路、坐标换算、上传请求/重传/拒绝/超时和XML消息号。
- 4项自定义协议测试使用生成的真实MAVLink 2帧，经内存数据报队列验证七类消息、丢ACK重传去重、错误来源ACK拒绝和重试耗尽。未将内存队列测试表述为无线链路或操作系统UDP实测。
- 2项标准协议测试使用真实pymavlink编解码器，验证MISSION_COUNT/MISSION_ITEM_INT上传序列、整数经纬度、GUIDED位置目标编码和type_mask。

自定义 `protocol/swarm.xml` 已通过mavgen校验并生成 `generated/swarm.py`。其生成产物使用自动生成风格，项目手写代码采用中文注释。

缺少pymavlink或未生成dialect时，相应可选测试会显示 `skipped`，基础13项不受影响。复现全部19项请先安装 `requirements.txt` 并运行 `generate_dialect.py`。

## 未运行的部分

没有启动ArduPilot SITL、没有连接实体飞控、没有执行飞行或硬件取货。第三部分已校验语法、消息签名、编解码和任务上传状态机；飞控实际握手、任务接受条件、飞行动力学及多机链式转发仍需后续SITL联调。
