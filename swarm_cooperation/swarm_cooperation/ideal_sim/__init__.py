"""五轮往返的理想调度实验；纯 Python 核心不依赖 ROS 或 AI 运行时。"""

from .config import ExperimentConfig

__all__ = ["ExperimentConfig", "IdealSimulation"]


def __getattr__(name):
    # The lightweight contract is also imported by the feature encoder. Do not
    # eagerly import the executor back into a partially initialized encoder.
    if name == "IdealSimulation":
        from .simulation import IdealSimulation
        return IdealSimulation
    raise AttributeError(name)
