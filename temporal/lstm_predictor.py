"""
LSTM-based temporal predictor for autonomous driving.

Predicts future vehicle states from sequence of feature vectors.
"""

import torch
import torch.nn as nn
import numpy as np
import logging
from typing import List, Optional, Tuple
from collections import deque
import sys
from pathlib import Path

# Add utils to path
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.device_utils import get_device

logger = logging.getLogger(__name__)


class LSTMPredictor(nn.Module):
    """LSTM network for predicting future vehicle states."""
    
    def __init__(
        self,
        input_size: int = 512,
        hidden_size: int = 256,
        num_layers: int = 2,
        sequence_length: int = 10,
        dropout: float = 0.1,
        output_size: int = 4,  # x, y, yaw, velocity
        use_lane_features: bool = False,
        lane_feature_dim: int = 128
    ):
        """
        Initialize LSTM predictor.
        
        Args:
            input_size: Size of input feature vector (ResNet features)
            hidden_size: LSTM hidden state size
            num_layers: Number of LSTM layers
            sequence_length: Length of input sequence
            dropout: Dropout probability
            output_size: Size of output state vector
            use_lane_features: Whether to use lane features
            lane_feature_dim: Dimension of lane features
        """
        super(LSTMPredictor, self).__init__()
        
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.sequence_length = sequence_length
        self.output_size = output_size
        self.use_lane_features = use_lane_features
        self.lane_feature_dim = lane_feature_dim
        
        # Adjust input size if using lane features
        lstm_input_size = input_size
        if use_lane_features:
            lstm_input_size = input_size + lane_feature_dim
            self.lane_projection = nn.Linear(lane_feature_dim, lane_feature_dim)
        
        # LSTM layers
        self.lstm = nn.LSTM(
            input_size=lstm_input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            dropout=dropout if num_layers > 1 else 0,
            batch_first=True
        )
        
        # Output projection layer
        self.fc = nn.Linear(hidden_size, output_size)
        
        self.device = get_device()  # Supports ROCm/CUDA/CPU
        self.to(self.device)
        self.eval()
        
        logger.info(f"✅ LSTMPredictor initialized (input={input_size}, hidden={hidden_size}, layers={num_layers})")
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.
        
        Args:
            x: Input sequence tensor (B, sequence_length, input_size)
            
        Returns:
            Predicted state tensor (B, output_size)
        """
        # LSTM forward
        lstm_out, (h_n, c_n) = self.lstm(x)
        
        # Use last hidden state
        last_hidden = lstm_out[:, -1, :]
        
        # Project to output
        output = self.fc(last_hidden)
        
        return output
    
    def predict(self, feature_sequence: np.ndarray, lane_features: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Predict future state from feature sequence.
        
        Args:
            feature_sequence: Sequence of feature vectors (sequence_length, input_size)
            lane_features: Optional lane features (sequence_length, lane_feature_dim)
            
        Returns:
            Predicted state vector (output_size,): [x, y, yaw, velocity]
        """
        with torch.no_grad():
            # Convert to tensor
            if isinstance(feature_sequence, np.ndarray):
                feature_tensor = torch.FloatTensor(feature_sequence).unsqueeze(0).to(self.device)
            else:
                feature_tensor = feature_sequence.to(self.device)
            
            # Combine with lane features if provided
            if self.use_lane_features and lane_features is not None:
                if isinstance(lane_features, np.ndarray):
                    lane_tensor = torch.FloatTensor(lane_features).unsqueeze(0).to(self.device)
                else:
                    lane_tensor = lane_features.to(self.device)
                
                # Project lane features
                lane_tensor = self.lane_projection(lane_tensor)
                
                # Concatenate
                feature_tensor = torch.cat([feature_tensor, lane_tensor], dim=-1)
            
            # Ensure correct shape
            if feature_tensor.dim() == 2:
                feature_tensor = feature_tensor.unsqueeze(0)  # Add batch dimension
            
            # Forward pass
            output = self.forward(feature_tensor)
            
            # Convert to numpy
            if isinstance(output, torch.Tensor):
                output = output.cpu().numpy().squeeze()
            
            return output


class SequenceBuffer:
    """Buffer for maintaining feature sequence."""
    
    def __init__(self, sequence_length: int = 10, feature_dim: int = 512):
        """
        Initialize sequence buffer.
        
        Args:
            sequence_length: Length of sequence to maintain
            feature_dim: Dimension of feature vectors
        """
        self.sequence_length = sequence_length
        self.feature_dim = feature_dim
        self.buffer = deque(maxlen=sequence_length)
    
    def add(self, feature: np.ndarray) -> None:
        """
        Add feature to buffer.
        
        Args:
            feature: Feature vector (feature_dim,)
        """
        if feature.shape[0] != self.feature_dim:
            raise ValueError(f"Feature dimension mismatch: expected {self.feature_dim}, got {feature.shape[0]}")
        
        self.buffer.append(feature.copy())
    
    def get_sequence(self) -> Optional[np.ndarray]:
        """
        Get current sequence.
        
        Returns:
            Sequence array (sequence_length, feature_dim) or None if not enough data
        """
        if len(self.buffer) < self.sequence_length:
            return None
        
        return np.array(list(self.buffer))
    
    def is_ready(self) -> bool:
        """Check if buffer has enough data."""
        return len(self.buffer) >= self.sequence_length
    
    def reset(self) -> None:
        """Reset buffer."""
        self.buffer.clear()

