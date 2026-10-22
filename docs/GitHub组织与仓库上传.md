# 仓库维护与上传范围

目标仓库：[QU-of-boomboom/demo-repository](https://github.com/QU-of-boomboom/demo-repository)。A/B/C 共用任务配置和 B 的仿真执行器，采用单仓库维护。

## 纳入版本管理

- A/B/C 当前源码、配置、测试、历史兼容源码；自定义 MAVLink XML 保留，生成代码由验证入口重新生成。
- 总体设计、当前两小车架构、协议、训练接口、部署步骤和研发记录。
- P0 测试结果、逐跳 diff、两条规则基线完整指标；两小车单轮报告与一个交互回放。
- C 历史报告、图表和九个小型 ONNX/PT 冻结样本。`check_dod.py` 使用它们核对哈希；它们不是当前两小车模型。
- 精选历史报告与旧文档快照。

## 本地保留、排除上传

- 用户放在工程旁的 VIO/Kalibr 标定文件夹、ZIP、脚本、操作文档和续接记录。
- 虚拟环境、缓存、凭据、训练数据 `.npz/.npy`、新训练输出和重复压缩包。
- 重复 HTML 回放、大型逐步调度日志与可重新生成的历史产物。

完整 P0 回放可在克隆后运行 `python scripts/validate.py` 重建，位于 `.cache/validation/p0/baselines/`。两小车演示由 `run_cars.ps1` / `run_cars.sh` 重建到 `.cache/cars/demo/`。历史 C 报告引用的数据集未上传，复跑训练须重新生成数据。

## 开发方式

GitHub 插件可以在已授权仓库中提交文件；Git 命令行或 GitHub Desktop 也能上传，无需额外安装插件。

```bash
git clone https://github.com/QU-of-boomboom/demo-repository.git
cd demo-repository
python -m venv .venv
# 激活环境后安装完整开发依赖
python -m pip install -r requirements.txt
python scripts/validate.py
```

使用与虚拟环境对应的 Python。克隆私有仓库需要自己的 GitHub 访问权限。提交前检查 `git status` 与 `git diff`，确认实际变更和待上传文件。

`.gitattributes` 保留文件字节，避免不同平台的 Git 换行转换影响冻结哈希。修改冻结特征或替换权重属于新的模型版本工作，必须配套更新训练与验证记录。

## 原始演示仓库

GitHub 创建组织时生成的示例文件保存在 `docs/history_github_demo/`；原始提交继续保留。归档的 `package.json` 和工作流只属于该演示模板，不是本 Python 工程的依赖或许可声明。
