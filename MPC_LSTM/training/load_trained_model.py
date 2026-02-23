"""
Load trained LSTM model for inference.

Supports both regular LSTMPredictor and AttentionLSTM models.
"""

import torch
import numpy as np
import logging
from pathlib import Path
from temporal import LSTMPredictor

logger = logging.getLogger(__name__)


def load_trained_lstm(model_path: str):
    """
    Load trained LSTM model.
    
    Automatically detects if model is AttentionLSTM or regular LSTMPredictor
    based on state_dict keys.
    
    Args:
        model_path: Path to best_model.pth
    
    Returns:
        model: Trained LSTM model (AttentionLSTM or LSTMPredictor)
        state_mean: State normalization mean
        state_std: State normalization std
    """
    model_path = Path(model_path)
    
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")
    
    checkpoint = torch.load(model_path, map_location='cpu')
    
    # Handle different checkpoint formats
    if isinstance(checkpoint, dict):
        if 'model_state_dict' in checkpoint:
            state_dict = checkpoint['model_state_dict']
            config = checkpoint.get('config', {})
        elif 'state_dict' in checkpoint:
            state_dict = checkpoint['state_dict']
            config = checkpoint.get('config', {})
        else:
            # Assume the dict itself is the state_dict
            state_dict = checkpoint
            config = {}
    else:
        state_dict = checkpoint
        config = {}
    
    # Detect model type based on state_dict keys
    has_attention = any('attention' in key for key in state_dict.keys())
    has_fc1_fc2 = any('fc1' in key or 'fc2' in key for key in state_dict.keys())
    has_fc = any('fc.weight' in key or 'fc.bias' in key for key in state_dict.keys())
    
    if has_attention or (has_fc1_fc2 and not has_fc):
        # AttentionLSTM model
        logger.info("Detected AttentionLSTM model")
        from training.advanced_training import AttentionLSTM
        
        # Get config or use defaults
        input_size = config.get('input_size', 512)
        hidden_size = config.get('hidden_size', 256)
        num_layers = config.get('num_layers', 2)
        output_size = config.get('output_size', 4)
        sequence_length = config.get('sequence_length', 10)
        
        model = AttentionLSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            output_size=output_size,
            sequence_length=sequence_length
        )
    else:
        # Regular LSTMPredictor
        logger.info("Detected regular LSTMPredictor model")
        
        # Get config or use defaults
        input_size = config.get('input_size', 512)
        hidden_size = config.get('hidden_size', 256)
        num_layers = config.get('num_layers', 2)
        sequence_length = config.get('sequence_length', 10)
        output_size = config.get('output_size', 4)
        
        model = LSTMPredictor(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            sequence_length=sequence_length,
            output_size=output_size
        )
    
    # Load weights
    try:
        model.load_state_dict(state_dict, strict=True)
        logger.info("✅ Loaded model weights (strict=True)")
    except Exception as e:
        logger.warning(f"Strict load failed: {e}, trying strict=False")
        model.load_state_dict(state_dict, strict=False)
        logger.info("✅ Loaded model weights (strict=False)")
    
    # Move to proper device
    from utils.device_utils import get_device
    device = get_device()
    model = model.to(device)
    model.device = device
    model.eval()
    
    # Get normalization parameters
    state_mean = checkpoint.get('state_mean', np.zeros(4)) if isinstance(checkpoint, dict) else np.zeros(4)
    state_std = checkpoint.get('state_std', np.ones(4)) if isinstance(checkpoint, dict) else np.ones(4)
    
    return model, state_mean, state_std

