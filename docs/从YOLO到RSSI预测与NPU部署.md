# 你负责的部分：RSSI 模型、NPU 部署与三机调度

更新：2026-10-20。对应 V2 代码。先完成一个模型：**多跳链路弱信号预测 MLP**。暂时不需要学习强化学习、视觉识别或 FPGA RTL。

## 1. 先把三件事分清楚

| 部分 | 输入 | 输出 | 由谁执行 |
|---|---|---|---|
| 预测 | RSSI 历史、路线进度、位置、中继状态 | 各跳在未来两个窗口内变弱的风险分数 | 你的 MLP，最终放到 NPU |
| 调度 | 风险分数、候选站位、到位时间、电量、路由依赖 | 保持、移动、接入、交接、返场 | CPU 上的调度器 |
| 飞行执行 | 目标点与任务版本 | 实际位置、到位、任务完成或失败 | 机载程序和飞控 |

NPU 是运行神经网络的硬件；MLP 是你要训练的网络。你可以先在 PC 完成全部模型工作，再接 NPU。预测器不直接控制电机，也不直接生成坐标。

```mermaid
flowchart LR
    A["网卡测量 + 飞行状态"] --> B["相同的特征处理"]
    B --> C["MLP：预测各跳弱链路风险"]
    C --> D["CPU：筛选站位与路由"]
    D --> E["准备、到位、探测、提交、确认"]
    E --> F["机载执行与反馈"]
    F --> A
```

## 2. 用你熟悉的 YOLO 流程理解

| 使用 YOLO 时 | 现在做 RSSI 预测 |
|---|---|
| 一张图片 | 某一物理跳的历史和当前状态，共 96 个数 |
| 人工标注类别和框 | 从后续实测记录生成“未来窗口内是否低于弱链路门限”标签 |
| 检测网络 | `96 → 64 → 32 → 2` 的小型 MLP |
| 检测损失 | 两个独立二分类的 `BCEWithLogitsLoss` |
| mAP、召回率 | 每个预测窗口的召回率、精确率、Brier 分数；还要看闭环断链和任务完成情况 |
| `.pt` 权重和 ONNX | 同样生成 `.pt` 和 ONNX |
| 检测结果接业务逻辑 | 风险预测接中继站位规划、路由交接 |

输出的两个数分别对应 **1.5 秒短时窗口、6 秒部署窗口**，不是“直连/中继”两个类别。CPU 对每个 logit 分别做 sigmoid 和阈值比较；不要用 softmax 或 argmax。

例如：对 `G-M` 这一跳，模型输出 `[0.82, 0.94]`，表示两个时域的弱链路风险分数。CPU 再检查哪条完整中继路径能用、飞机是否到位，决定是否提前切换。这些分数未经真实数据校准，不能直接当成可靠的真实概率。

## 3. 已修改的训练模板

当前入口仍是 `ai_relay_scheduler/ai_relay_scheduler/`，实现集中在 `pipeline_v2/`。旧 Python 脚本备份到 `legacy_v1/`，旧结果保留在 `outputs/`；新结果进入 `outputs_v2/`。

| 文件 | 你通常修改的内容 |
|---|---|
| `config.json` | 数据来源、训练轮数、预测时域、模型宽度、输出路径 |
| `pipeline_v2/model.py` | 网络结构 |
| `pipeline_v2/dataset.py` | 仿真数据生成、真实遥测导入、标签构造 |
| `pipeline_v2/train.py` | 训练、验证集选模型和阈值 |
| `pipeline_v2/export.py` | FP32 ONNX、PC INT8 对照、校准数据、数值对齐 |
| `pipeline_v2/evaluate.py` | 固定站位、被动规则、预部署、预测辅助四组仿真对照 |

训练和在线推理共同调用 B 中的 `relay_learning/features.py`，避免训练时一种输入、部署时另一种输入。

模型目前共 **8,354 个参数**。六条物理跳共用同一个模型，每条跳输入一次；不会把主机直连的 RSSI 历史复制给其他链路。

## 4. 今天先运行什么

以下命令在项目根目录的 PowerShell 执行。本轮已建立独立的 `.venv-rssi` 环境。常规 Python 3.10+ 环境也可安装 C 的 `requirements.txt`。

```powershell
# 不加载模型，先看五轮集群仿真
python swarm_cooperation\swarm_cooperation\demo.py --mode random --map-observation static --quiet

# 用少量仿真数据检查：生成 → 训练 → 导出 → 数值核对 → 四组策略对照
.\.venv-rssi\Scripts\python.exe ai_relay_scheduler\ai_relay_scheduler\run_all.py --smoke

# 单独运行刚导出的预测器
.\.venv-rssi\Scripts\python.exe swarm_cooperation\swarm_cooperation\demo.py --strategy predictive --predictor ai_relay_scheduler\ai_relay_scheduler\outputs_v2\smoke\model_fp32_batch1.onnx --map-observation static --quiet
```

`--smoke` 只用 6 个仿真任务、3 个训练 epoch；用途是查流程和接口。它不是正式训练结果，不应以它的准确率或仿真收益作为答辩结论。完整模板默认 24 个任务、16 个 epoch，也仍然只是仿真模板。

Windows 下，ORT 动态量化内部使用临时文件，临时目录需要是可写的 ASCII 路径；正常用户的 `%TEMP%` 通常符合要求。若手动将 `TEMP/TMP` 改到了含中文的项目目录，请恢复默认或指定可写的英文目录。模型输出目录本身可以包含中文。

结果入口：

- `outputs_v2/smoke/RESULTS.md`：本次实际生成的模型、量化和策略对照报告。
- `outputs_v2/smoke/replays/predictive/replay.html`：预测辅助调度的交互回放。
- `outputs_v2/smoke/deployment.json`：输入输出、特征版本、阈值和模型哈希。

## 5. 换成真实数据时，你要做什么

先记录原始遥测，再训练。每个时间槽需要三机位置、主机路线进度、每条物理跳的 RSSI 与测量时间，以及中继目标、到位状态、估计剩余到位时间和电量。格式见 [V2 数据与部署接口](V2数据与部署接口.md)。

1. 将遥测整理为 0.1 秒时间槽的 JSONL；缺测写无效标志，不能填“上次良好信号”并假装是新测量。
2. 同一次采集批次或相同场景族使用同一个 `split_group`，每个完整任务使用一个 `episode_id`。
3. 复制配置，把 `data_source` 改成 `real`，填写 `telemetry_jsonl`，选择新的 `output_dir`。
4. 用这个配置运行 `run_all.py --config ...`。脚本按完整采集组划分训练、验证、测试。
5. 首先检查真实测试集与仿真有多大差距，再检查漏报、提前量、误触发和实际调度结果。

训练标签可以读取“后来实际发生的 RSSI”；这是离线制作答案。模型输入只能读取当时和过去的信息。未来缺测、记录不足、预测窗口内改变中继任务时，对应标签会被屏蔽。

`compare.py` 始终是仿真对照，即便加载的权重来自真实数据，也不能替代实机测试。

## 6. 交给 NPU 的是什么

先拿这几个文件对接板端：

| 文件 | 用途 |
|---|---|
| `model_fp32_batch1.onnx` | 固定 `[1,96] → [1,2]` 的模型源图，供 Icraft 导入 |
| `icraft_calibration_inputs.npy` | 仅从训练集抽取的代表输入，供目标工具链校准 |
| `feature_contract.json`、`deployment.json` | 特征顺序、归一化、输出含义、阈值、版本 |
| `golden_inputs.npy`、`golden_logits.npy` | 在 PC 上已核对的输入与 FP32 输出，供板端逐项对比 |

`model_int8_pc.onnx` 是 ONNX Runtime 的 PC 动态量化对照，**不等于 Icraft 编译产物**。

板端顺序：核对 Icraft 版本与支持的算子 → 导入 FP32 ONNX → 按该版本要求校准和编译 → 用 golden inputs 核对 → 测真实推理耗时 → 接入调度器。当前没有 SDK/板卡连接，因此没有编造 Icraft 命令或 NPU 性能数据。

现在的 ONNX 推理适配器是 `relay_learning/backend.py`。它放在独立进程中，超时会停用并退回 CPU 规则。将来保留 `predict(rows) → 六组两维风险分数` 的接口，把内部 ONNX 调用替换成已验证的 Icraft 运行时。

当前站位搜索仍由 CPU 完成。部署时域预测用于触发重新规划；超过可靠预测时域的移动，采用已知路线预部署或让主机等待。不能把 6 秒模型无限外推。

## 7. 你和 FPGA 同学的接口

你提供：原始记录字段、20 点窗口规则、有效位与时间语义、归一化定义、测试输入和正确输出。FPGA 同学可以先实现窗口缓存与特征打包，再与 Python 参考实现逐项对齐。

当前 Python 已有完整软件参考，尚未实现 PL IP 或 DMA。你现在可以先使用 CPU 生成相同的 `[1,96]` 输入，完成 NPU 模型和集群控制工作；以后接入 PL 时，再检查数值与时序是否一致。

## 8. 当前仿真的边界

- `--map-observation static`：使用起飞前已知地图，不观察货箱变化；适合当前不新增视觉识别任务的主线。它也不是“完全不知道地图，只看 RSSI”。
- `--map-observation ideal`：沿用文档中的理想局部占用观测，用于地图更新实验；这项能力不是 RSSI 模型提供的。
- 已加入有限速度、起飞与稳定时间、路由版本和确认、缺测/时延/噪声、上游依赖限制、失联等待与本地返场。
- 失联判断使用链路观测可用性作心跳代理，返场假设定位有效；真实系统还需接业务心跳和独立定位失效处理。
- 电量采用简化代理模型，尚无完整避碰、飞行动力学、真实组网和三机飞控闭环。ROS 2 入口已同步，尚未在 Ubuntu 实启验收。

技术参考：[PyTorch 二分类损失](https://docs.pytorch.org/docs/stable/generated/torch.nn.BCEWithLogitsLoss.html)、[PyTorch ONNX 导出](https://docs.pytorch.org/docs/stable/onnx.html)、[ONNX Runtime 量化](https://onnxruntime.ai/docs/performance/model-optimizations/quantization.html)。
