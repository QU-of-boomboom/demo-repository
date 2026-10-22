"""96 -> 64 -> 32 -> 2; the two logits represent short/deployment weak-link risk."""
import numpy as np
import torch
from torch import nn

from relay_learning.features import FeatureSpec
from .common import sigmoid


class LinkPredictor(nn.Module):
    def __init__(self, input_size=96, hidden_sizes=(64, 32)):
        super().__init__()
        sizes = [input_size, *hidden_sizes, 2]
        layers = []
        for index, (a, b) in enumerate(zip(sizes, sizes[1:])):
            layers.append(nn.Linear(a, b))
            if index < len(sizes) - 2:
                layers.append(nn.ReLU())
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def create_model(c):
    return LinkPredictor(FeatureSpec(**c["feature_spec"]).input_size, c["hidden_sizes"])


def load_model(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if checkpoint.get("schema_version") != "relay-link-v2":
        raise ValueError("Legacy 25-input weights are not compatible with v2")
    model = create_model(checkpoint["config"])
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    return model, checkpoint


@torch.inference_mode()
def predict(model, x, batch_size=4096):
    return sigmoid(np.concatenate([model(torch.from_numpy(x[i:i + batch_size])).numpy()
                                   for i in range(0, len(x), batch_size)]))
