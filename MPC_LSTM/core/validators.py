"""
Data validation utilities.
"""

import numpy as np
from typing import Dict, Any, Optional, Tuple
from .exceptions import DataValidationError


class ImageValidator:
    """Validator for camera images."""
    
    @staticmethod
    def validate(image: np.ndarray, expected_shape: Optional[Tuple[int, ...]] = None) -> bool:
        """
        Validate camera image.
        
        Args:
            image: Image array
            expected_shape: Optional expected shape (H, W, C)
        
        Returns:
            True if valid
        
        Raises:
            DataValidationError: If image is invalid
        """
        if image is None:
            raise DataValidationError("Image is None")
        
        if not isinstance(image, np.ndarray):
            raise DataValidationError(f"Image must be numpy array, got {type(image)}")
        
        if len(image.shape) != 3:
            raise DataValidationError(f"Image must be 3D (H, W, C), got shape {image.shape}")
        
        h, w, c = image.shape
        if h == 0 or w == 0:
            raise DataValidationError(f"Image dimensions must be > 0, got {image.shape}")
        
        if c != 3:
            raise DataValidationError(f"Image must have 3 channels (RGB), got {c}")
        
        if expected_shape is not None:
            if image.shape != expected_shape:
                raise DataValidationError(f"Image shape mismatch: expected {expected_shape}, got {image.shape}")
        
        if np.any(np.isnan(image)) or np.any(np.isinf(image)):
            raise DataValidationError("Image contains NaN or Inf values")
        
        return True


class StateValidator:
    """Validator for vehicle states."""
    
    @staticmethod
    def validate(state: Dict[str, Any]) -> bool:
        """
        Validate vehicle state.
        
        Args:
            state: Vehicle state dictionary
        
        Returns:
            True if valid
        
        Raises:
            DataValidationError: If state is invalid
        """
        required_keys = ['x', 'y', 'yaw', 'velocity']
        missing_keys = [key for key in required_keys if key not in state]
        if missing_keys:
            raise DataValidationError(f"Missing required state keys: {missing_keys}")
        
        # Check for NaN or Inf
        for key in required_keys:
            value = state[key]
            if not isinstance(value, (int, float)):
                raise DataValidationError(f"State {key} must be numeric, got {type(value)}")
            if np.isnan(value) or np.isinf(value):
                raise DataValidationError(f"State {key} contains NaN or Inf: {value}")
        
        # Check reasonable ranges
        if abs(state['x']) > 10000 or abs(state['y']) > 10000:
            raise DataValidationError(f"State position out of range: ({state['x']}, {state['y']})")
        
        if abs(state['yaw']) > 360:
            raise DataValidationError(f"State yaw out of range: {state['yaw']}")
        
        if state['velocity'] < 0 or state['velocity'] > 100:  # m/s
            raise DataValidationError(f"State velocity out of range: {state['velocity']} m/s")
        
        return True


class FeatureValidator:
    """Validator for feature vectors."""
    
    @staticmethod
    def validate(features: np.ndarray, expected_dim: int) -> bool:
        """
        Validate feature vector.
        
        Args:
            features: Feature vector
            expected_dim: Expected feature dimension
        
        Returns:
            True if valid
        
        Raises:
            DataValidationError: If features are invalid
        """
        if features is None:
            raise DataValidationError("Features are None")
        
        if not isinstance(features, np.ndarray):
            raise DataValidationError(f"Features must be numpy array, got {type(features)}")
        
        if features.ndim != 1:
            raise DataValidationError(f"Features must be 1D, got shape {features.shape}")
        
        if features.shape[0] != expected_dim:
            raise DataValidationError(f"Feature dimension mismatch: expected {expected_dim}, got {features.shape[0]}")
        
        if np.any(np.isnan(features)) or np.any(np.isinf(features)):
            raise DataValidationError("Features contain NaN or Inf values")
        
        return True


class PredictionValidator:
    """Validator for LSTM predictions."""
    
    @staticmethod
    def validate(prediction: np.ndarray, expected_size: int = 4) -> bool:
        """
        Validate LSTM prediction.
        
        Args:
            prediction: Predicted state
            expected_size: Expected prediction size
        
        Returns:
            True if valid
        
        Raises:
            DataValidationError: If prediction is invalid
        """
        if prediction is None:
            raise DataValidationError("Prediction is None")
        
        if not isinstance(prediction, np.ndarray):
            raise DataValidationError(f"Prediction must be numpy array, got {type(prediction)}")
        
        if len(prediction) != expected_size:
            raise DataValidationError(f"Prediction size mismatch: expected {expected_size}, got {len(prediction)}")
        
        if np.any(np.isnan(prediction)) or np.any(np.isinf(prediction)):
            raise DataValidationError("Prediction contains NaN or Inf values")
        
        return True


class ControlValidator:
    """Validator for control values."""
    
    @staticmethod
    def validate(steering: float, throttle: float, brake: float) -> bool:
        """
        Validate control values.
        
        Args:
            steering: Steering angle
            throttle: Throttle value
            brake: Brake value
        
        Returns:
            True if valid
        
        Raises:
            DataValidationError: If control values are invalid
        """
        if not all(isinstance(x, (int, float)) for x in [steering, throttle, brake]):
            raise DataValidationError("Control values must be numeric")
        
        if np.isnan(steering) or np.isnan(throttle) or np.isnan(brake):
            raise DataValidationError("Control values contain NaN")
        
        if not (-1.1 <= steering <= 1.1):
            raise DataValidationError(f"Steering out of range: {steering}")
        
        if not (-0.1 <= throttle <= 1.1):
            raise DataValidationError(f"Throttle out of range: {throttle}")
        
        if not (-0.1 <= brake <= 1.1):
            raise DataValidationError(f"Brake out of range: {brake}")
        
        return True

