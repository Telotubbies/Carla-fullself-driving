"""
MPC Validation Module.

Implements STEP 5.3 requirements:
- Constraint satisfaction validation
- Steering limits validation
- Tracking error calculation
"""

import logging
import numpy as np
from typing import Dict, Any, List, Tuple, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class MPCValidator:
    """Validates MPC controller performance."""
    
    @staticmethod
    def validate_constraints(
        steering: float,
        throttle: float,
        brake: float,
        max_steer: float = 0.5,
        max_throttle: float = 1.0,
        max_brake: float = 1.0
    ) -> Dict[str, bool]:
        """
        Validate constraint satisfaction.
        
        Args:
            steering: Steering angle
            throttle: Throttle value
            brake: Brake value
            max_steer: Maximum steering angle
            max_throttle: Maximum throttle
            max_brake: Maximum brake
        
        Returns:
            Dictionary with constraint validation results
        """
        return {
            'steering_ok': abs(steering) <= max_steer,
            'throttle_ok': 0.0 <= throttle <= max_throttle,
            'brake_ok': 0.0 <= brake <= max_brake,
            'all_ok': (
                abs(steering) <= max_steer and
                0.0 <= throttle <= max_throttle and
                0.0 <= brake <= max_brake
            )
        }
    
    @staticmethod
    def calculate_tracking_error(
        reference_trajectory: np.ndarray,
        actual_trajectory: np.ndarray
    ) -> Dict[str, float]:
        """
        Calculate tracking error.
        
        Args:
            reference_trajectory: Reference trajectory (N, 4) [x, y, yaw, velocity]
            actual_trajectory: Actual trajectory (N, 4) [x, y, yaw, velocity]
        
        Returns:
            Dictionary with tracking error metrics
        """
        if reference_trajectory.shape != actual_trajectory.shape:
            raise ValueError("Trajectory shape mismatch")
        
        # Position error
        pos_error = np.sqrt(
            (reference_trajectory[:, 0] - actual_trajectory[:, 0]) ** 2 +
            (reference_trajectory[:, 1] - actual_trajectory[:, 1]) ** 2
        )
        
        # Heading error
        heading_error = np.abs(reference_trajectory[:, 2] - actual_trajectory[:, 2])
        # Normalize to [-pi, pi]
        heading_error = np.minimum(heading_error, 2 * np.pi - heading_error)
        
        # Velocity error
        velocity_error = np.abs(reference_trajectory[:, 3] - actual_trajectory[:, 3])
        
        return {
            'position_error': {
                'mean': float(np.mean(pos_error)),
                'std': float(np.std(pos_error)),
                'max': float(np.max(pos_error))
            },
            'heading_error': {
                'mean': float(np.mean(heading_error)),
                'std': float(np.std(heading_error)),
                'max': float(np.max(heading_error))
            },
            'velocity_error': {
                'mean': float(np.mean(velocity_error)),
                'std': float(np.std(velocity_error)),
                'max': float(np.max(velocity_error))
            },
            'overall_rmse': float(np.sqrt(np.mean(pos_error ** 2)))
        }
    
    @staticmethod
    def validate_steering_limits(
        steering_history: List[float],
        max_steer: float = 0.5
    ) -> Dict[str, Any]:
        """
        Validate steering limits over time.
        
        Args:
            steering_history: List of steering values
            max_steer: Maximum allowed steering
        
        Returns:
            Dictionary with validation results
        """
        steering_array = np.array(steering_history)
        
        violations = np.abs(steering_array) > max_steer
        violation_count = np.sum(violations)
        violation_rate = violation_count / len(steering_history) if len(steering_history) > 0 else 0.0
        
        return {
            'total_samples': len(steering_history),
            'violations': int(violation_count),
            'violation_rate': float(violation_rate),
            'max_steering': float(np.max(np.abs(steering_array))),
            'mean_steering': float(np.mean(np.abs(steering_array))),
            'within_limits': violation_count == 0
        }

