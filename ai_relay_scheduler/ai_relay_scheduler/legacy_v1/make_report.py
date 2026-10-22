"""从实际运行结果生成中文验收报告，不把预期指标写成实测结果。"""
import json
from datetime import datetime, timezone, timedelta

from common import load_config, output_path, parser, write_json


def make_report(c):
    out = output_path(c)
    def read(name):
        return json.loads((out / name).read_text(encoding="utf-8"))
    data, train, quant, comp = [read(name) for name in ("dataset_manifest.json", "training_report.json", "quantization_report.json", "comparison_report.json")]
    checks = dict(dataset_voyages_at_least_5000=sum(r["voyages"] for r in data["splits"].values()) >= 5000,
                  evaluation_voyages_at_least_200=comp["voyages"] >= 200,
                  cpu_latency_repetitions_at_least_1000=all(r["latency"]["repetitions"] >= 1000 for r in quant["rows"].values()),
                  parameters_under_100000=train["parameter_count"] < 100000,
                  quantization_recall_drop_at_most_2pp=quant["quantization_pass"],
                  significantly_fewer_outages=comp["acceptance"]["significantly_fewer_outages"],
                  switch_growth_at_most_50_percent=comp["acceptance"]["switch_growth_at_most_50_percent"])
    all_pass = all(checks.values())
    stamp = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")
    write_json(out / "acceptance.json", dict(generated_at=stamp, checks=checks, all_pass=all_pass,
                                           ideal_zero_outages=comp["acceptance"]["ideal_zero_outages"]))
    lines = ["# AI 中继调度实测报告", "", f"生成时间（北京时间）：{stamp}", "",
             f"必需验收项：{'全部通过' if all_pass else '存在未通过项，见下表'}。理想零断链：{comp['acceptance']['ideal_zero_outages']}。", "",
             "## 数据与模型", "", "| 划分 | 航程数 | 样本数 | 正例比例 |", "|---|---:|---:|---:|"]
    for key, name in (("train", "训练"), ("val", "验证"), ("test", "测试")):
        r = data["splits"][key]
        lines.append(f"| {name} | {r['voyages']} | {r['samples']:,} | {r['positive_fraction']:.2%} |")
    lines += ["", f"MLP参数量：{train['parameter_count']:,}；验证集选定第{train['selected_epoch']}轮，概率阈值{train['threshold']:.3f}。",
              f"PyTorch测试集准确率{train['test']['accuracy']:.4%}，召回率{train['test']['recall']:.4%}，精确率{train['test']['precision']:.4%}。",
              "", "## 量化验证", "", (out / "quantization_table.md").read_text(encoding="utf-8"),
              "## 配对调度实验", "", (out / "comparison_table.md").read_text(encoding="utf-8"),
              "", "![分组柱状图](ai_vs_rule.png)", "", "![航程0时序](example_trace.png)", "",
              "## 验收明细", "", "| 检查 | 通过 |", "|---|---|"]
    lines += [f"| {key} | {'是' if ok else '否'} |" for key, ok in checks.items()]
    lines += ["", "## 适用范围", "",
              "以上结果来自给定几何模型、独立高斯噪声与预先固定的随机种子；不代表真实仓库或NPU上板结果。",
              "默认切换建立0.5秒，期间使用原链路；两组条件相同。零延迟行用于显示该假设对结果的影响。",
              "报告额外给出中继占用率。AI提前进入、延后退出中继会增加空口/中继工作时间，应结合实际电量与吞吐评估。",
              "任何失败项均保留真实结果；不筛选评估航程、不按测试集修改阈值、不将负面实验隐藏。", ""]
    (out / "RESULTS.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"验收报告: {out / 'RESULTS.md'}；必需项通过={all_pass}", flush=True)
    return all_pass


if __name__ == "__main__":
    make_report(load_config(parser(__doc__).parse_args().config))
