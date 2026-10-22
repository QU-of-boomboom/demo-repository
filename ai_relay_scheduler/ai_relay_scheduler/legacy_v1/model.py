"""25→64→32→1 MLP：只使用 Linear/ReLU，概率在宿主CPU用sigmoid计算。"""
import runtime_compat
import numpy as np
import torch
from torch import nn

from common import sigmoid


class LinkPredictor(nn.Module):
    def __init__(self, input_size=25, hidden_sizes=(64, 32)):
        super().__init__()
        sizes = [input_size, *hidden_sizes, 1]
        layers = []
        for index, (a, b) in enumerate(zip(sizes, sizes[1:])):
            layers.append(nn.Linear(a, b))
            if index < len(sizes) - 2:
                layers.append(nn.ReLU())
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def create_model(c):
    m = LinkPredictor(c["history_steps"] + 5, c["hidden_sizes"])
    assert sum(p.numel() for p in m.parameters()) < 100_000
    return m


def load_model(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    model = create_model(checkpoint["config"])
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    return model, checkpoint


@torch.inference_mode()
def predict(model, normalized, batch_size=8192):
    result = []
    for start in range(0, len(normalized), batch_size):
        x = torch.from_numpy(normalized[start:start + batch_size])
        result.append(model(x).numpy().ravel())
    return sigmoid(np.concatenate(result))
