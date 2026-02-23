"""
Abstract interfaces for all major components.

This module defines the contracts that all implementations must follow.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Tuple, List
import numpy as np


class IPerceptionModule(ABC):
    """Interface for perception modules (feature extraction)."""
    
    @abstractmethod
    def encode(self, image: np.ndarray) -> np.ndarray:
        """
        Encode image to feature vector.
        
        Args:
            image: Input image (H, W, 3) RGB
        
        Returns:
            Feature vector (feature_dim,)
        
        Raises:
            DataValidationError: If image is invalid
        """
        pass
    
    @abstractmethod
    def get_feature_dim(self) -> int:
        """Get feature dimension."""
        pass


class ITemporalModule(ABC):
    """Interface for temporal prediction modules."""
    
    @abstractmethod
    def predict(self, sequence: np.ndarray) -> np.ndarray:
        """
        Predict future state from sequence.
        
        Args:
            sequence: Feature sequence (sequence_length, feature_dim)
        
        Returns:
            Predicted state (4,) [x, y, yaw, velocity]
        
        Raises:
            DataValidationError: If sequence is invalid
        """
        pass
    
    @abstractmethod
    def get_sequence_length(self) -> int:
        """Get required sequence length."""
        pass


class IControlModule(ABC):
    """Interface for control modules."""
    
    @abstractmethod
    def compute_control(
        self,
        current_state: Dict[str, Any],
        reference_trajectory: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float]:
        """
        Compute control output.
        
        Args:
            current_state: Current vehicle state
            reference_trajectory: Optional reference trajectory (N+1, 4)
        
        Returns:
            (steering, throttle, brake)
        
        Raises:
            ControlError: If control computation fails
        """
        pass
    
    @abstractmethod
    def set_reference_trajectory(self, trajectory: np.ndarray) -> None:
        """Set reference trajectory for control."""
        pass


class IVisualizationModule(ABC):
    """Interface for visualization modules."""
    
    @abstractmethod
    def update(
        self,
        image: np.ndarray,
        vehicle_state: Dict[str, Any],
        predicted_trajectory: Optional[np.ndarray] = None,
        mpc_horizon: Optional[np.ndarray] = None,
        lane_info: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Update visualization.
        
        Args:
            image: Camera image
            vehicle_state: Current vehicle state
            predicted_trajectory: LSTM predicted trajectory
            mpc_horizon: MPC predicted horizon
            lane_info: Lane detection information
        
        Returns:
            True if should continue, False to stop
        """
        pass
    
    @abstractmethod
    def close(self) -> None:
        """Close visualization."""
        pass


class IConfigManager(ABC):
    """Interface for configuration management."""
    
    @abstractmethod
    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value."""
        pass
    
    @abstractmethod
    def validate(self) -> bool:
        """Validate configuration."""
        pass
    
    @abstractmethod
    def get_section(self, section: str) -> Dict[str, Any]:
        """Get configuration section."""
        pass


class ILogger(ABC):
    """Interface for logging."""
    
    @abstractmethod
    def info(self, message: str, **kwargs) -> None:
        """Log info message."""
        pass
    
    @abstractmethod
    def warning(self, message: str, **kwargs) -> None:
        """Log warning message."""
        pass
    
    @abstractmethod
    def error(self, message: str, **kwargs) -> None:
        """Log error message."""
        pass
    
    @abstractmethod
    def debug(self, message: str, **kwargs) -> None:
        """Log debug message."""
        pass

