# 仓库无人机 AI 中继调度（FMQL30TAI 上板前 PC 验证）

本目录独立实现数据生成、PyTorch 训练、ONNX/INT8 量化和规则/AI 配对实验。全部逻辑为 Python 3，默认配置实际生成 **5000 个往返航程**，两策略使用同一批额外的 **200 个随机航程**。

实测结果见 [outputs/RESULTS.md](../outputs/RESULTS.md)，原始数值保存在 JSON/CSV 中。验收脚本按真实结果判定，不硬编码“AI一定通过”。板卡的8TOPS INT8能力与Icraft工具链来自项目背景；本工程未在实板执行，CPU延迟不能当作NPU延迟。

## 快速复跑

建议 Python 3.11（支持有对应依赖wheel的 Python 3 版本），需要 NumPy、PyTorch CPU、ONNX、ONNX Runtime、matplotlib。默认完整运行占用约数百 MB 数据空间；建议至少 4GB 可用内存。无需 CUDA。绘图需中文字体：Windows 自带微软雅黑；Linux 可安装 Noto Sans CJK SC。

Windows PowerShell，从本目录运行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_all.py --strict
```

当前交付目录已包含在此电脑验证过的 `.venv`，可直接运行最后一条命令。虚拟环境不可直接复制到另一电脑，搬迁时按上面步骤重建。此机器预装PyTorch/NumPy，实际环境复用了系统包；全新环境安装requirements即可。

Linux/macOS：

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 run_all.py --strict
```

`--strict` 在任一必需验收项不通过时返回退出码2，仍保存全部结果。外部依赖或运行错误返回非零退出码。已生成数据且配置完全一致时，可以 `python3 run_all.py --skip-data --strict`；模型仍重新训练，不复用旧测量。

四个部分也可分别运行：

```bash
python3 generate_data.py
python3 train.py
python3 quantize.py
python3 compare.py
python3 verify_artifacts.py
python3 make_report.py
python3 -m unittest -v test_core
```

每个脚本支持 `--config /path/to/config.json`。全部可调仿真/训练/调度参数集中在 [config.json](config.json)。相对 `output_dir` 总是相对本工程目录，不受当前终端目录影响。改参数后应完整重跑，避免混用数据、权重与量化文件。正式验收必须保留 `voyages >= 5000`、`evaluation_voyages = 200`、`latency_repeats = 1000`。若仅做冒烟验证，可以复制一份配置，减少航程与epochs并指定另一个输出目录；不要将冒烟结果作为正式验收。

## 环境定义与明确假设

1. **地图与路线**：每格1m，地面站(0,0)，棋盘范围[0,3]×[0,4]。沿竖格线蛇形路线为 `(0,0)→(0,4)→(1,4)→(1,0)→(2,0)→(2,4)→(3,4)→(3,0)→(3,4)`；随后沿该路线逆序返回。严格竖向蛇形本来落在(3,0)，最后补4m到要求的(3,4)，与已有仓库仿真约定一致。单程23m、往返46m，标称46秒；不额外加入取货停留。
2. **运动采样**：dt=0.1秒；随机航程每秒更新一次速度，均匀取0.7～1.3m/s，随机数流按航程独立。航程在终点/回家前最后一个采样间隔会减速以精确落点，该不足一步的速度可以小于0.7m/s。返程路径严格逆序，随机采样坐标不要求与去程逐点相等。标称无扰动时，去/回程采样坐标逐点逆序一致。
3. **货架几何**：每个单元格的开集内部为货架；无线射线进入同一列中任一货架才计该列，单列多格仅计一次。沿网格边界或只触角点不计遮挡。这一几何定义复现已有 `warehouse_relay_sim`。边界处RSSI可能突变，属于此模型的性质。
4. **RSSI**：`P0 - 10*n*log10(max(d,1m)/1m) - shelf_loss * crossed_columns`，P0=-40dBm，n=2.2。每航程统一抽取遮挡损耗8～16dB、噪声sigma=1～3dB；每条物理跳在每个采样点独立叠加零均值高斯噪声。标称模式遮挡12dB、无噪声、1m/s。d<1m按1m截断，避免log10(0)。
5. **中继**：悬停(1,2)和(2,4)。`relay_RSSI = min(base→relay, relay→drone) + 6dB`，增益仅加一次。中继位置固定，建立链路延迟不代表无人机飞行时间。
6. **切换建立时间**：用户原模型未指定，因此明确新增可配置假设，默认0.5秒；从请求到完成一直使用原链路，完成时才接入目标链路。两组共享此状态机，不人为添加切换丢包。额外的零延迟对照始终输出；不应把0.5秒条件下的收益解释为“瞬时切换条件也有相同收益”。
7. **可观测性**：默认地面站持续得到直连与两条中继候选RSSI探测值，即使当前已接入中继仍有直连历史。模型本身只读取直连历史、位置、速度和两中继距离，不读取未来RSSI、隐含遮挡/噪声参数、航程编号或时间标签。仿真不建模探测报文开销、电量、吞吐或中继容量限制。

## 第一部分：数据

按**整个航程**固定随机拆分：3500训练/750验证/750测试。各航程的遮挡参数、噪声强度与速度随机；评估200航程使用不同的随机种子流。不能将高度重叠的时间窗随机按样本拆分，否则会泄漏。

每个样本25维，顺序固定：

```text
[RSSI(t-19), ..., RSSI(t), x, y, speed, distance_relay1, distance_relay2]
```

20点表示20个0.1秒采样槽，最早至最新点的时间戳跨度为1.9秒（通常称2秒历史窗）。起飞不足20点，用第一个已知RSSI向左填充。标签为 `min(RSSI(t+1),...,RSSI(t+15)) < -75`，不含当前点；末尾没有完整15步未来的点丢弃，不标成负例。调度评估则保留完整往返航程。

`train.npz / val.npz / test.npz` 内有：`x`原始float32特征、`y`二分类标签、`voyage_id`、`tick`、`voyage_parameters`（航程id/遮挡/sigma/完整点数）、`feature_names`。可用 `np.load(path, allow_pickle=False)` 读取。

## 第二部分：模型训练

MLP结构 `25 → Linear(64) → ReLU → Linear(32) → ReLU → Linear(1)`，**3777参数**。宿主端对logit做sigmoid，以概率阈值生成二分类结果。算子简单且规模小，便于后续NPU适配，但兼容性须以所用Icraft版本的解析/编译结果确认。

固定缩放在 [common.py](common.py) 的 `normalize()` 中：RSSI为`(rssi+65)/25`，x/3、y/4、speed/1、两距离/5。此缩放按配置中的地图尺寸计算，未使用验证或测试数据拟合统计量。导出的ONNX输入已归一化，板端必须使用同样变换。

训练使用加权BCE（正例权重3），AdamW，16轮。每轮仅在验证集选择阈值：首先满足99%目标召回，再最大化精确率；检查点也只根据验证集选择。目标若无法达到，会保留真实指标，不能伪造达标。最终只评估一次测试集，测试集不用于调阈值。`model.pt`保存权重、配置、阈值、最佳epoch与验证指标；可用 `weights_only=True`安全加载。

## 第三部分：PC端INT8量化验证

导出opset13 FP32 ONNX，额外提供固定batch=1源图；使用ONNX Runtime `quantize_dynamic` 对所有3个线性层做每通道有符号INT8权重量化。脚本检查3个 `MatMulInteger` 节点确实存在，并验证FP32 ONNX与PyTorch数值一致。该PC格式含动态激活量化算子、浮点偏置/输出，不是所有张量全INT8，也不是Icraft二进制。

量化前后使用**同一固定阈值、同一测试集、同一CPUExecutionProvider、同一单线程设置**。动态量化比例可能随batch变化，所以测试集和调度均逐样本batch=1推理。延迟预热100次，取1000个不同测试样本分别计时并报告均值（另存p50/p95）；只包含 `session.run` 调用及ORT内部开销，不包含磁盘加载、特征拼接和归一化。文件大小为完整ONNX磁盘字节数，含图结构/量化参数，不用参数理论位宽估算。小网络INT8可能因量化开销而比FP32更慢，如实报告。

**这些表格仅对应 Icraft 上板前的 PC 端仿真验证环节。** 上板时应使用 `model_fp32_batch1.onnx` 和仅来自训练集的 `icraft_calibration_inputs.npy`，依照实际安装Icraft版本的手册完成导入、代表数据静态校准、编译和运行时接入。不能假设ORT的 `DynamicQuantizeLinear/MatMulInteger` 模型可直接加载到Icraft。此工程不编造未知Icraft版本的命令；板端需重新测召回率、延迟、算子支持和真实无线切换时间。

参考官方说明：[ONNX Runtime量化](https://onnxruntime.ai/docs/performance/model-optimizations/quantization.html)、[PyTorch ONNX导出](https://docs.pytorch.org/docs/stable/onnx.html)。为匹配本机已安装的PyTorch 2.8，脚本显式使用传统导出器`dynamo=False`。复跑依赖固定为ONNX1.17.0、ORT1.20.1、protobuf4.25.8。本机Conda自带旧VC运行库14.27，会导致ONNX的DLL初始化失败或schema初始化阻塞；`runtime_compat.py`仅在Windows Conda环境、第三方库导入前预载已有系统VC运行库（本机14.51）。不覆盖Conda DLL，也不修改系统。该类问题可参考[ONNX Runtime官方仓库诊断](https://github.com/microsoft/onnxruntime/issues/21024)。

## 第四部分：规则与AI调度

**规则组**：直连 `< -75dBm` 触发中继；直连 `>= -72dBm` 返回直连。**AI组**：预测概率达到验证集阈值便提前请求中继；退出需直连 `>= -72dBm` 且预测未来安全。AI没有用未来标签兜底，也没有暗中增加规则触发。给AI保留“预测不安全时继续使用中继”的条件，可避免刚提前进入便因当前直连仍强而退出。

两组均选当前RSSI最强的中继，且候选必须达到-88dBm才允许从直连接入；中继之间切换需改善至少3dB；所有切换使用同一建立延迟。AI可以在当前直连仍较强时选择已可用但当前RSSI较弱的中继，因为其目标是预防未来弱链路。无额外驻留时间或人工轨迹位置规则。建立期间不撤销请求，完成后根据最新观测重新决策。切换计已完成的实际状态改变，未完成请求数另存。

主实验使用**最终INT8模型**。评估种子固定，200个环境轨迹只生成一遍，四种策略（规则/AI、默认延迟/零延迟）逐轨迹复用完全相同的速度、遮挡和噪声；策略不会影响环境。图中示例固定为评估航程0，不挑选最好看的航程。

统计口径：

| 指标 | 定义 |
|---|---|
| 断链事件数 | 有效当前链路RSSI首次进入 `< -88dBm` 的次数，连续低于门限算一次 |
| 断链采样点数 | 所有低于门限的点数；近似断链时长=点数×0.1s，作为补充输出 |
| 切换总次数 | 直连↔中继及中继1↔中继2的已完成切换总数 |
| 平均链路余量 | 所有航程所有采样点的 `effective_RSSI - (-88dBm)` 均值（按样本加权） |
| 乒乓切换 | 相邻已完成切换A→B、B→A，间隔≤2秒；一个相邻反转对计一次 |
| 中继占用率 | 使用中继的采样点/总采样点；用于揭示提前进入、延后退出的资源代价 |

“显著减少”同时要求按航程配对bootstrap的AI减规则断链事件差值95%置信区间上界<0，以及单侧配对符号检验p<0.05；平局不参与符号检验。切换增幅为 `AI切换/规则切换 - 1`，要求≤50%。量化召回下降为 `100×(FP32召回-INT8召回)`，单位**百分点**，要求≤2。理想0断链单列为可选目标，不混同必需验收。

固定几何模型上的高召回并不能保证真实仓库泛化。采用真实飞行数据后，应仍按航程/场景隔离评估，并依据实际切换延迟复核预测提前量与断链收益。仿真的不连续货架边界、独立高斯噪声、固定航线及悬停点均需随真实场景校准。

## 文件清单

| 文件 | 用途 |
|---|---|
| config.json | 集中参数 |
| simulator.py / common.py | 公用仿真、特征与度量 |
| generate_data.py | 第一部分：数据生成 |
| model.py / train.py | 第二部分：模型定义、训练与测试 |
| quantize.py | 第三部分：导出、动态INT8量化、CPU测试 |
| compare.py | 第四部分：因果调度、配对比较与中文图 |
| run_all.py | 一键复跑并保存阶段日志 |
| test_core.py | 几何、时序、无未来泄漏、回差等关键测试 |
| verify_artifacts.py | 核对实际NPZ航程隔离、重建抽样与模型哈希 |
| runtime_compat.py | Windows Conda旧VC运行库的本进程兼容处理 |
| make_report.py | 从实测JSON生成验收报告 |
| outputs/train.npz / val.npz / test.npz | 5000航程数据集 |
| outputs/dataset_manifest.json | 每个划分的航程编号及生成配置 |
| outputs/model.pt | PyTorch权重及工作阈值 |
| outputs/model_fp32.onnx / model_int8.onnx | PC FP32与动态INT8 ONNX |
| outputs/model_fp32_batch1.onnx | Icraft导入候选源图，固定输入[1,25] |
| outputs/icraft_calibration_inputs.npy | 1024个训练集归一化代表样本 |
| outputs/training_report.json | 每轮训练/验证与独立测试结果 |
| outputs/quantization_report.json | 量化度量、环境版本、延迟与模型SHA256 |
| outputs/quantization_test_predictions.npz | 同阈值量化比较的逐样本预测 |
| outputs/voyage_metrics.csv | 所有200航程×4策略的指标 |
| outputs/comparison_report.json | 汇总指标和配对统计 |
| outputs/ai_vs_rule.png | 中文分组柱状图 |
| outputs/example_trace.npz / .png | 固定航程0的RSSI、预测及切换时序 |
| outputs/acceptance.json / RESULTS.md | 机器可读验收状态与中文报告 |
