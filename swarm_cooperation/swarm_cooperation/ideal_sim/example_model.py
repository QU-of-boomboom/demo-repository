"""接口演示，仅为手写评分；不是已训练AI模型。

运行：python demo.py --model-plugin ideal_sim.example_model:build_model
将 score 的内部替换为实际模型推理即可。不要直接加载 C 的25维旧模型充当调度模型。
"""


class ExampleCostModel:
    def score(self, state, candidates):
        return [c.relay_hop_samples + .1 * c.movement_m + .05 * c.switches for c in candidates]


def build_model():
    return ExampleCostModel()
