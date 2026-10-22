# P0 规则基线存档

- 契约：`warehouse-abc-v3`
- mission profile SHA256：`1687015d91b7cb17a545312178ef4eea63fe5d44b806ba588ec11eb58a7ab40b`
- 策略：`reactive`（被动门限，连续 N 槽低于 -75dBm，恢复需连续高于 -72dBm）与 `predeploy`（已知航线 + 当前地图/ETA 预部署）
- 场景：5 个种子（42–46），每场景每策略 5 个完整往返；共 10 次任务
- 指标：断链次数/时长、最长连续断链、切换次数、中继占用率、到位成功率、总时长
- 文件：`comparison_report.json`（全量配置、逐行指标、配对差值）、`RESULTS.md`（人读表）、`reactive/` 与 `predeploy/`（首场景 report；完整 trace/events/replay 可重建）

复现（在 `swarm_cooperation/swarm_cooperation` 目录）：

```powershell
python -m ideal_sim.compare_baselines
```

该表是 P1 AI 实验的对照基准；失败任务按失败计，不得用提前中止后的较小累计断链宣称收益。
