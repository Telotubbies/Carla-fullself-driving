#!/usr/bin/env python3
"""LSTM Temporal Predictor for Lane Polynomial Coefficients."""
import torch
import torch.nn as nn
import numpy as np
from collections import deque
from dataclasses import dataclass

@dataclass
class LSTMConfig:
    input_dim: int = 3
    hidden_dim: int = 32
    num_layers: int = 2
    output_dim: int = 3
    seq_len: int = 10
    dropout: float = 0.1

class LaneLSTM(nn.Module):
    def __init__(self, cfg: LSTMConfig = None):
        super().__init__()
        self.cfg = cfg or LSTMConfig()
        c = self.cfg
        self.lstm = nn.LSTM(input_size=c.input_dim, hidden_size=c.hidden_dim, num_layers=c.num_layers, batch_first=True, dropout=c.dropout if c.num_layers > 1 else 0.0)
        self.fc = nn.Sequential(nn.Linear(c.hidden_dim, 16), nn.ReLU(inplace=True), nn.Linear(16, c.output_dim))
    def forward(self, x):
        lstm_out, _ = self.lstm(x)
        return self.fc(lstm_out[:, -1, :])

class LaneTemporalSmoother:
    def __init__(self, model_path: str = None, device: torch.device = None, seq_len: int = 10, ema_alpha: float = 0.3):
        self.device = device or torch.device("cpu")
        self.seq_len = seq_len
        self.ema_alpha = ema_alpha
        self._history = deque(maxlen=seq_len)
        self._prev_output = np.zeros(3)
        self.model = None
        if model_path:
            self.model = LaneLSTM(LSTMConfig()).to(self.device)
            ckpt = torch.load(model_path, map_location=self.device, weights_only=True)
            self.model.load_state_dict(ckpt.get("model_state_dict", ckpt))
            self.model.eval()
    def update(self, cte: float, heading: float, curvature: float):
        state = np.array([cte, heading, curvature], dtype=np.float32)
        self._history.append(state)
        if self.model is not None and len(self._history) >= self.seq_len:
            seq = np.array(list(self._history), dtype=np.float32)
            out = self.model(torch.from_numpy(seq).unsqueeze(0).to(self.device)).squeeze(0).cpu().numpy()
            self._prev_output = out
            return out
        self._prev_output = self.ema_alpha * self._prev_output + (1 - self.ema_alpha) * state
        return self._prev_output.copy()
    def reset(self):
        self._history.clear()
        self._prev_output = np.zeros(3)
