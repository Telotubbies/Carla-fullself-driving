"""
Data Loading for LSTM Training.

Loads and preprocesses data for LSTM training.
"""

import logging
import numpy as np
import pandas as pd
from pathlib import Path
from typing import Tuple, Optional

logger = logging.getLogger(__name__)


class LSTMDataLoader:
    """Loads data for LSTM training."""
    
    @staticmethod
    def load_training_data(
        data_dir: str,
        use_processed: bool = True
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Load features and states from data directory.
        
        Args:
            data_dir: Directory containing features.npy and data
            use_processed: Use processed data if available
        
        Returns:
            features: (N, feature_dim)
            states: (N, 4) [x, y, yaw, velocity] (normalized)
            state_mean: Normalization mean
            state_std: Normalization std
        """
        data_dir = Path(data_dir)
        
        # Load features
        features_path = data_dir / "features.npy"
        if not features_path.exists():
            raise FileNotFoundError(f"features.npy not found in {data_dir}")
        
        features = np.load(features_path)
        logger.info(f"Loaded features: {features.shape}")
        
        # Load states
        if use_processed:
            csv_path = data_dir / "processed" / "data_processed.csv"
            if not csv_path.exists():
                csv_path = data_dir / "data_valid.csv"
        else:
            csv_path = data_dir / "data.csv"
        
        if not csv_path.exists():
            raise FileNotFoundError(f"Data CSV not found: {csv_path}")
        
        df = pd.read_csv(csv_path)
        logger.info(f"Loaded {len(df)} data samples")
        
        # Extract states
        states = df[['x', 'y', 'yaw', 'velocity']].values.astype(np.float32)
        
        # Normalize states
        state_mean = np.mean(states, axis=0)
        state_std = np.std(states, axis=0) + 1e-6  # Avoid division by zero
        
        states_normalized = (states - state_mean) / state_std
        
        logger.info(f"State normalization:")
        logger.info(f"  Mean: {state_mean}")
        logger.info(f"  Std: {state_std}")
        
        # Validate
        if len(features) != len(states):
            min_len = min(len(features), len(states))
            logger.warning(f"Feature and state length mismatch, using {min_len} samples")
            features = features[:min_len]
            states_normalized = states_normalized[:min_len]
        
        return features, states_normalized, state_mean, state_std
    
    @staticmethod
    def validate_data(
        features: np.ndarray,
        states: np.ndarray,
        sequence_length: int
    ) -> bool:
        """
        Validate training data.
        
        Args:
            features: Feature array
            states: State array
            sequence_length: Required sequence length
        
        Returns:
            True if valid
        """
        if len(features) < sequence_length + 1:
            raise ValueError(
                f"Insufficient data: need at least {sequence_length + 1} samples, "
                f"got {len(features)}"
            )
        
        # Check for NaN/Inf
        if np.any(np.isnan(features)) or np.any(np.isinf(features)):
            raise ValueError("Features contain NaN or Inf values")
        if np.any(np.isnan(states)) or np.any(np.isinf(states)):
            raise ValueError("States contain NaN or Inf values")
        
        logger.info("✅ Data validation passed")
        return True


