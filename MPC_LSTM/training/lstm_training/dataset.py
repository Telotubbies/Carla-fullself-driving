"""
LSTM Training Dataset.

Dataset classes for LSTM sequence training.
"""

import logging
import numpy as np
import torch
from torch.utils.data import Dataset

logger = logging.getLogger(__name__)


class LSTMSequenceDataset(Dataset):
    """Dataset for LSTM training with sequences."""
    
    def __init__(self, features: np.ndarray, states: np.ndarray, sequence_length: int = 10):
        """
        Initialize dataset.
        
        Args:
            features: Feature array (N, feature_dim)
            states: State array (N, 4) [x, y, yaw, velocity]
            sequence_length: Length of input sequence
        """
        self.sequence_length = sequence_length
        self.features = features
        self.states = states
        
        # Create sequences
        self.sequences = []
        self.targets = []
        
        for i in range(len(features) - sequence_length):
            seq = features[i:i+sequence_length]
            target = states[i+sequence_length]  # Predict next state
            self.sequences.append(seq)
            self.targets.append(target)
        
        logger.info(f"Created {len(self.sequences)} sequences")
    
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        return (
            torch.FloatTensor(self.sequences[idx]),
            torch.FloatTensor(self.targets[idx])
        )


