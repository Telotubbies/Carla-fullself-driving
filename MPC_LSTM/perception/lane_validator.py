"""
Lane Detection Fail-Safe Validation Logic

Implements validation and fail-safe mechanisms for robust lane detection.
"""

import numpy as np
from typing import Optional, Tuple, Dict
import logging

logger = logging.getLogger(__name__)


class LaneValidator:
    """
    Validates lane detection results and applies fail-safe logic.
    """
    
    def __init__(
        self,
        max_curvature: float = 0.1,  # 1/meters (R_min = 10m)
        max_lateral_shift: float = 0.5,  # meters per frame
        min_lane_width: float = 2.5,  # meters
        max_lane_width: float = 4.5,  # meters
        expected_lane_width: float = 3.5,  # meters
        far_zone_confidence_threshold: float = 0.3
    ):
        """
        Initialize lane validator.
        
        Args:
            max_curvature: Maximum acceptable curvature (1/meters)
            max_lateral_shift: Maximum acceptable lateral shift per frame (meters)
            min_lane_width: Minimum lane width (meters)
            max_lane_width: Maximum lane width (meters)
            expected_lane_width: Expected lane width (meters)
            far_zone_confidence_threshold: Minimum confidence for far zone
        """
        self.max_curvature = max_curvature
        self.max_lateral_shift = max_lateral_shift
        self.min_lane_width = min_lane_width
        self.max_lane_width = max_lane_width
        self.expected_lane_width = expected_lane_width
        self.far_zone_confidence_threshold = far_zone_confidence_threshold
        
        # Previous state for temporal validation
        self.previous_coefficients = None
        
        logger.info("LaneValidator initialized")
    
    def compute_curvature(
        self,
        coefficients: np.ndarray,
        y: float = 50.0
    ) -> float:
        """
        Compute curvature from polynomial coefficients.
        
        κ = |2a| / (1 + (2a·y + b)²)^(3/2)
        
        Args:
            coefficients: Polynomial coefficients [a, b, c]
            y: Y coordinate to evaluate (meters)
            
        Returns:
            Curvature in [1/meters]
        """
        a, b, c = coefficients
        x_prime = 2 * a * y + b
        x_double_prime = 2 * a
        curvature = abs(x_double_prime) / (1 + x_prime**2)**(3/2)
        return curvature
    
    def validate_curvature(
        self,
        coefficients: np.ndarray
    ) -> Tuple[bool, str]:
        """
        Validate that curvature is within acceptable range.
        
        Args:
            coefficients: Polynomial coefficients [a, b, c]
            
        Returns:
            Tuple of (is_valid, reason)
        """
        curvature = self.compute_curvature(coefficients)
        
        if curvature > self.max_curvature:
            return False, f"Curvature too high: {curvature:.4f} > {self.max_curvature}"
        
        return True, "OK"
    
    def validate_lateral_shift(
        self,
        current_coefficients: np.ndarray,
        previous_coefficients: Optional[np.ndarray] = None
    ) -> Tuple[bool, str]:
        """
        Validate that lateral shift is not too sudden.
        
        Args:
            current_coefficients: Current polynomial coefficients
            previous_coefficients: Previous frame coefficients (optional)
            
        Returns:
            Tuple of (is_valid, reason)
        """
        if previous_coefficients is None:
            if self.previous_coefficients is None:
                return True, "OK"  # First frame
            previous_coefficients = self.previous_coefficients
        
        # Lateral shift = change in constant term (c)
        lateral_shift = abs(current_coefficients[2] - previous_coefficients[2])
        
        if lateral_shift > self.max_lateral_shift:
            return False, f"Sudden lateral shift: {lateral_shift:.3f}m > {self.max_lateral_shift}m"
        
        return True, "OK"
    
    def validate_lane_width(
        self,
        left_coefficients: np.ndarray,
        right_coefficients: np.ndarray,
        y_range: Tuple[float, float] = (0.0, 50.0),
        num_samples: int = 50
    ) -> Tuple[bool, str, float]:
        """
        Validate that lane width is within acceptable range.
        
        Args:
            left_coefficients: Left lane polynomial coefficients
            right_coefficients: Right lane polynomial coefficients
            y_range: Y range to evaluate (meters)
            num_samples: Number of samples
            
        Returns:
            Tuple of (is_valid, reason, average_width)
        """
        y_samples = np.linspace(y_range[0], y_range[1], num_samples)
        
        # Evaluate polynomials
        x_left = np.polyval(left_coefficients, y_samples)
        x_right = np.polyval(right_coefficients, y_samples)
        
        # Compute lane widths
        widths = np.abs(x_right - x_left)
        avg_width = np.mean(widths)
        
        if avg_width < self.min_lane_width:
            return False, f"Lane too narrow: {avg_width:.2f}m < {self.min_lane_width}m", avg_width
        
        if avg_width > self.max_lane_width:
            return False, f"Lane too wide: {avg_width:.2f}m > {self.max_lane_width}m", avg_width
        
        return True, "OK", avg_width
    
    def validate_far_zone_confidence(
        self,
        far_zone_confidence: float
    ) -> Tuple[bool, str]:
        """
        Validate far zone confidence.
        
        Args:
            far_zone_confidence: Confidence score for far zone
            
        Returns:
            Tuple of (is_valid, reason)
        """
        if far_zone_confidence < self.far_zone_confidence_threshold:
            return False, f"Far zone confidence too low: {far_zone_confidence:.3f} < {self.far_zone_confidence_threshold}"
        
        return True, "OK"
    
    def estimate_missing_boundary(
        self,
        detected_coefficients: np.ndarray,
        lane_width: Optional[float] = None,
        is_left_detected: bool = True
    ) -> np.ndarray:
        """
        Estimate missing lane boundary from detected one.
        
        Args:
            detected_coefficients: Coefficients of detected lane
            lane_width: Lane width (uses expected if None)
            is_left_detected: True if left lane detected, False if right
            
        Returns:
            Estimated coefficients for missing boundary
        """
        if lane_width is None:
            lane_width = self.expected_lane_width
        
        # Copy coefficients
        estimated = detected_coefficients.copy()
        
        # Shift constant term (c) by lane width
        if is_left_detected:
            # Left detected, estimate right (shift right)
            estimated[2] += lane_width
        else:
            # Right detected, estimate left (shift left)
            estimated[2] -= lane_width
        
        return estimated
    
    def apply_fail_safe(
        self,
        current_coefficients: Optional[np.ndarray],
        previous_coefficients: Optional[np.ndarray] = None,
        confidence: float = 0.0,
        left_coefficients: Optional[np.ndarray] = None,
        right_coefficients: Optional[np.ndarray] = None
    ) -> Dict[str, any]:
        """
        Apply fail-safe logic based on validation results.
        
        Args:
            current_coefficients: Current lane coefficients
            previous_coefficients: Previous frame coefficients
            confidence: Detection confidence
            left_coefficients: Left lane coefficients (optional)
            right_coefficients: Right lane coefficients (optional)
            
        Returns:
            Dict with validation results and fail-safe actions
        """
        result = {
            'valid': True,
            'warnings': [],
            'actions': [],
            'final_coefficients': current_coefficients,
            'confidence_adjusted': confidence
        }
        
        if current_coefficients is None:
            result['valid'] = False
            result['warnings'].append("No coefficients provided")
            result['actions'].append("use_previous_frame")
            if previous_coefficients is not None:
                result['final_coefficients'] = previous_coefficients
            return result
        
        # Validate curvature
        curvature_valid, curvature_reason = self.validate_curvature(current_coefficients)
        if not curvature_valid:
            result['valid'] = False
            result['warnings'].append(curvature_reason)
            result['actions'].append("reject_current_use_previous")
            if previous_coefficients is not None:
                result['final_coefficients'] = previous_coefficients
                result['confidence_adjusted'] *= 0.5  # Reduce confidence
        
        # Validate lateral shift
        lateral_valid, lateral_reason = self.validate_lateral_shift(
            current_coefficients,
            previous_coefficients
        )
        if not lateral_valid:
            result['warnings'].append(lateral_reason)
            result['actions'].append("gradual_transition")
            # Use weighted average for gradual transition
            if previous_coefficients is not None:
                alpha = 0.3  # Weight for current
                result['final_coefficients'] = (
                    alpha * current_coefficients +
                    (1 - alpha) * previous_coefficients
                )
                result['confidence_adjusted'] *= 0.7
        
        # Validate lane width (if both boundaries available)
        if left_coefficients is not None and right_coefficients is not None:
            width_valid, width_reason, avg_width = self.validate_lane_width(
                left_coefficients, right_coefficients
            )
            if not width_valid:
                result['warnings'].append(width_reason)
                result['actions'].append("adjust_lane_width")
                # Could adjust one boundary to match expected width
                result['confidence_adjusted'] *= 0.8
        
        # Update previous state
        self.previous_coefficients = result['final_coefficients'].copy()
        
        return result
    
    def reset(self):
        """Reset validator state."""
        self.previous_coefficients = None

