"""Load trained LSTM model for inference. Supports LSTMPredictor and AttentionLSTM."""
import torch
import numpy as np
import logging
from pathlib import Path
from temporal import LSTMPredictor
logger = logging.getLogger(__name__)

def load_trained_lstm(model_path: str):
    model_path = Path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")
    checkpoint = torch.load(model_path, map_location='cpu')
    if isinstance(checkpoint, dict):
        state_dict = checkpoint.get('model_state_dict', checkpoint.get('state_dict', checkpoint))
        config = checkpoint.get('config', {})
    else:
        state_dict = checkpoint
        config = {}
    has_attention = any('attention' in k for k in state_dict.keys())
    has_fc1_fc2 = any('fc1' in k or 'fc2' in k for k in state_dict.keys())
    has_fc = any('fc.weight' in k for k in state_dict.keys())
    if has_attention or (has_fc1_fc2 and not has_fc):
        logger.info("Detected AttentionLSTM")
        from training.advanced_training import AttentionLSTM
        model = AttentionLSTM(
            input_size=config.get('input_size', 512), hidden_size=config.get('hidden_size', 256),
            num_layers=config.get('num_layers', 2), output_size=config.get('output_size', 4),
            sequence_length=config.get('sequence_length', 10))
    else:
        logger.info("Detected LSTMPredictor")
        model = LSTMPredictor(
            input_size=config.get('input_size', 512), hidden_size=config.get('hidden_size', 256),
            num_layers=config.get('num_layers', 2), sequence_length=config.get('sequence_length', 10),
            output_size=config.get('output_size', 4))
    try:
        model.load_state_dict(state_dict, strict=True)
    except Exception:
        model.load_state_dict(state_dict, strict=False)
    from utils.device_utils import get_device
    model = model.to(get_device())
    model.eval()
    state_mean = checkpoint.get('state_mean', np.zeros(4)) if isinstance(checkpoint, dict) else np.zeros(4)
    state_std = checkpoint.get('state_std', np.ones(4)) if isinstance(checkpoint, dict) else np.ones(4)
    return model, state_mean, state_std
