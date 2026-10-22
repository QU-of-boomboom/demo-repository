# C：RSSI 预测与共享中继调度

当前入口使用 `pipeline_v2`，与 A/B 共用 `warehouse-abc-v3` 的仿真、特征和 CPU 执行器。特征为 96 维，输出未来 1.5s/6s 的逐物理跳弱链路风险；模型不直接下发电机或航点命令。

统一格长、停留、遮挡、路径与时间戳见 [ABC统一契约与验收](../../docs/ABC统一契约与验收.md)。`config.json` 默认 24 个训练/验证/测试航程、6 个额外评估场景，是流程模板；任务格长、速度、停留和门限来自根目录 [mission_profile.json](../../mission_profile.json)，不在 `config.json` 重复。`config.json` 不能当成真实无线或 NPU 实测。

在本目录运行（推荐工作区 `.venv-rssi` 环境）：

```powershell
python generate_data.py
python train.py
python quantize.py
python compare.py
python verify_artifacts.py
python make_report.py
python -m unittest pipeline_v2.tests -v
```

各脚本支持 `--config 路径`。相对 `output_dir` 相对于本目录。新产物必须使用 `outputs_v2/...` 等独立目录，禁止覆盖历史 `outputs`。数据按完整采集组/航程分割，源代码哈希变化后须重新生成；真实 JSONL 接口见 [V2数据与部署接口](../../docs/V2数据与部署接口.md)。

也可独立运行共享仿真：

```powershell
python simulator.py --mode none --rounds 1 --quiet --output outputs_v3/demo
```

策略比较包含 `fixed`、`reactive`（被动门限）、`predeploy`（已知航线预部署）、`predictive`。AI 同时对照两条规则；所有策略共用移动/稳定/ACK 与观测约束，失败任务单独标记，不能靠提前中止获得较少累计断链。

无权重时可在 B 目录运行 `python -m ideal_sim.compare_baselines`。本轮配对结果见 [规则基线实测](../../outputs_v2/p0/baselines/RESULTS.md)，不宣称 AI 胜出。

`deployment.json` 必须声明 `simulation_contract_version: warehouse-abc-v3`；旧版权重将被拒绝加载。现存 `outputs_v2/smoke` 与 `outputs` 是旧契约记录，本次未重训，须重新生成数据、训练及导出后才可评估。

25维输入、固定中继 +6dB、5000航程/200评估的旧原型及说明保存在 `legacy_v1`，不属于当前验收。ONNX/INT8 为 PC 测试产物，尚无 Icraft/NPU 或三机实飞结果。
