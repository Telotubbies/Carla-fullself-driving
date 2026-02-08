"""
Load trained LSTM model for inference.
"""

import torch
import numpy as np
from pathlib import Path
from temporal import LSTMPredictor


def load_trained_lstm(model_path: str):
    """
    Load trained LSTM model.
    
    Args:
        model_path: Path to best_model.pth
        
    Returns:
        model: Trained LSTM model
        state_mean: State normalization mean
        state_std: State normalization std
    """
    model_path = Path(model_path)
    
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")
    
    checkpoint = torch.load(model_path, map_location='cpu')
    config = checkpoint['config']
    
    # Create model
    model = LSTMPredictor(
        input_size=config['input_size'],
        hidden_size=config['hidden_size'],
        num_layers=config['num_layers'],
        sequence_length=config['sequence_length'],
        output_size=config['output_size']
    )
    
    # Load weights
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    
    state_mean = checkpoint.get('state_mean', np.zeros(4))
    state_std = checkpoint.get('state_std', np.ones(4))
    
    return model, state_mean, state_std

