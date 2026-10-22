# 验收与演示结果

| 目录 | 配置与用途 |
|---|---|
| [p0](p0/README.md) | `mission_profile.json`，`warehouse-abc-v3` 的 relay-drone 回归、六跳 diff 与 5 轮 × 5 种子规则基线 |
| [p0_cars](p0_cars/README.md) | `mission_profile_1drone_2cars.json`，当前两小车配置的独立验证和单轮回放 |
| [upload_validation](upload_validation/README.md) | 整理后工程的上传前验证记录 |

P0 的 101 项回归与两小车的独立测试分别报告。PC 仿真与协议测试不代表实机、ROS/Gazebo 或 NPU 上板验收。

Git 版本保留关键报告、六跳 CSV 和一个两小车回放；P0 完整逐步回放运行 `python scripts/validate.py` 后在 `.cache/validation/p0/baselines/` 查看。
