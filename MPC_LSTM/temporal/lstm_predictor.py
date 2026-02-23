"""
LSTM-based temporal predictor for autonomous driving.

Predicts future vehicle states from sequence of feature vectors.
Implements ITemporalModule interface.
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
from core.interfaces import ITemporalModule
from core.exceptions import DataValidationError, ModelLoadError
from core.validators import FeatureValidator, PredictionValidator

logger = logging.getLogger(__name__)


class LSTMPredictor(nn.Module, ITemporalModule):
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
    
    def predict(self, sequence: np.ndarray) -> np.ndarray:
        """
        Predict future state from sequence.
        
        Implements ITemporalModule.predict().
        
        Args:
            sequence: Feature sequence (sequence_length, feature_dim)
        
        Returns:
            Predicted state (4,) [x, y, yaw, velocity]
        
        Raises:
            DataValidationError: If sequence is invalid
            ModelLoadError: If prediction fails
        """
        # Validate input sequence
        if sequence is None:
            raise DataValidationError("Sequence is None")
        
        if not isinstance(sequence, np.ndarray):
            raise DataValidationError(f"Sequence must be numpy array, got {type(sequence)}")
        
        if sequence.ndim != 2:
            raise DataValidationError(f"Sequence must be 2D (sequence_length, feature_dim), got shape {sequence.shape}")
        
        if sequence.shape[0] != self.sequence_length:
            raise DataValidationError(f"Sequence length mismatch: expected {self.sequence_length}, got {sequence.shape[0]}")
        
        if sequence.shape[1] != self.input_size:
            raise DataValidationError(f"Feature dimension mismatch: expected {self.input_size}, got {sequence.shape[1]}")
        
        if np.any(np.isnan(sequence)) or np.any(np.isinf(sequence)):
            raise DataValidationError("Sequence contains NaN or Inf values")
        
        with torch.no_grad():
            try:
                # Convert to tensor
                feature_tensor = torch.FloatTensor(sequence).unsqueeze(0).to(self.device)
                
                # Forward pass
                output = self.forward(feature_tensor)
                
                # Convert to numpy
                if isinstance(output, torch.Tensor):
                    output = output.cpu().numpy().squeeze()
                
                # Validate output
                try:
                    PredictionValidator.validate(output, expected_size=self.output_size)
                except DataValidationError as e:
                    logger.error(f"Prediction validation failed: {e}")
                    raise ModelLoadError(f"Invalid prediction output: {e}") from e
                
                return output
            except Exception as e:
                logger.error(f"Prediction failed: {e}", exc_info=True)
                raise ModelLoadError(f"Failed to predict state: {e}") from e
    
    def get_sequence_length(self) -> int:
        """
        Get required sequence length.
        
        Implements ITemporalModule.get_sequence_length().
        
        Returns:
            Sequence length
        """
        return self.sequence_length
    
    def load_model(self, model_path: str) -> None:
        """
        Load trained model weights from checkpoint.
        
        Args:
            model_path: Path to model checkpoint (.pth file)
            
        Raises:
            ModelLoadError: If loading fails
        """
        try:
            model_path = Path(model_path)
            if not model_path.exists():
                raise FileNotFoundError(f"Model file not found: {model_path}")
            
            checkpoint = torch.load(model_path, map_location=self.device)
            
            # Handle different checkpoint formats
            if isinstance(checkpoint, dict):
                # Check if it's a checkpoint with 'model_state_dict'
                if 'model_state_dict' in checkpoint:
                    state_dict = checkpoint['model_state_dict']
                elif 'state_dict' in checkpoint:
                    state_dict = checkpoint['state_dict']
                else:
                    # Assume the dict itself is the state_dict
                    state_dict = checkpoint
            else:
                state_dict = checkpoint
            
            # Load state dict (strict=False to handle partial matches)
            try:
                self.load_state_dict(state_dict, strict=False)
                logger.info(f"✅ Loaded LSTM model from {model_path}")
            except Exception as e:
                logger.warning(f"Partial load: {e}, trying compatible keys...")
                # Try to match compatible keys
                model_dict = self.state_dict()
                compatible_dict = {k: v for k, v in state_dict.items() 
                                 if k in model_dict and model_dict[k].shape == v.shape}
                if len(compatible_dict) > 0:
                    model_dict.update(compatible_dict)
                    self.load_state_dict(model_dict)
                    logger.info(f"✅ Loaded {len(compatible_dict)}/{len(state_dict)} compatible weights")
                else:
                    raise ModelLoadError(f"No compatible weights found: {e}")
            
            self.eval()
            
        except Exception as e:
            logger.error(f"Failed to load LSTM model: {e}")
            raise ModelLoadError(f"Failed to load model from {model_path}: {e}") from e
    
    def get_feature_dim(self) -> int:
        """
        Get input feature dimension.
        
        Returns:
            Feature dimension
        """
        return self.input_size


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

