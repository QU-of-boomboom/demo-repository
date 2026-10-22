# 验证记录

## 2026-10-21 P0 出口（当前）

Windows、Python 3.13.13，工作区 `.venv-rssi`（pymavlink 2.4.50、lxml 6.1.3、fastcrc 0.5.0）。当前契约 `warehouse-abc-v3`，任务参数读根目录 `mission_profile.json`。

- A **24/24**（含跨模块 0.1s 六跳 diff、标准去程 230 槽、ETA/READY 门控、逐跳 1–3s 缺测注入、被动 N 槽规则）。
- B **46/46**，含两项实际 pymavlink 标准飞控帧测试；无跳过。
- C **8/8**；历史 A 12/12、历史 C 10/10、冻结 B legacy 快照 1/1 作为兼容回归。
- 两条规则基线：`reactive` 与 `predeploy` 各 5 个种子、每场景 5 个完整往返，10 次任务全部完成；指标表与原始 JSON 在 `outputs_v2/p0/baselines/`。
- 全量出口：根目录 `python check_dod.py`，通过 101 项、0 失败、0 跳过；证据与命令见 `outputs_v2/p0/README.md`。

本次未重训模型、未修改 96 维特征定义、未修改 Icraft/FPGA。历史 2m/格、3s 停留的 B 协议版本通过 `demo.py --legacy` 运行 `legacy_v1/`，仅作存档，不参与当前验收。旧记录见 [B验证记录_更新前](../../docs/history_20261020/B验证记录_更新前.md)、[历史 B 理想仿真](../../docs/history_20261020/B理想仿真_更新前.md)。

ROS 2/Gazebo 包和世界只做静态检查，未在 Ubuntu 实启；SITL 未完成三机闭环。
