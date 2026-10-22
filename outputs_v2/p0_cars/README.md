# 1 架主机 + 2 台地面中继小车

配置：[mission_profile_1drone_2cars.json](../../mission_profile_1drone_2cars.json)。本样例显式启用 `ground_car`，完成一个往返，主机航程 46 m；它不是 5 轮多种子的实机性能结论。

- [运行报告](demo/report.json)
- [交互回放](demo/replay.html)：下载后在浏览器打开。
- [配置回归测试](../../tests/test_car_profile.py)：检查任务完成、46 m 航程、小车移动与到位，以及没有 `TAKEOFF/SETTLING` 状态。
- [本轮测试日志](../upload_validation/ground_car.log)

在仓库根执行 `run_cars.ps1` 或 `bash run_cars.sh`，输出到 `.cache/cars/demo/`。

本次示例报告全阶段断链 2.1 s，保留实际指标，不宣称零断链。
