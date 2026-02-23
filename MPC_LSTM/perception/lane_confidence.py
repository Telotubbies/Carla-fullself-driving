"""
Lane Detection Confidence Scoring System

Implements comprehensive confidence metrics:
- Coverage ratio
- Stability score
- Symmetry score
- Temporal smoothness
- Distance-weighted confidence
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
import logging
from collections import deque

logger = logging.getLogger(__name__)


class LaneConfidenceScorer:
    """
    Computes comprehensive confidence scores for lane detection.
    """
    
    def __init__(
        self,
        coverage_weight: float = 0.25,
        stability_weight: float = 0.25,
        symmetry_weight: float = 0.20,
        temporal_weight: float = 0.15,
        distance_weight: float = 0.15,
        stability_window: int = 10,
        min_points_required: int = 5
    ):
        """
        Initialize confidence scorer.
        
        Args:
            coverage_weight: Weight for coverage score
            stability_weight: Weight for stability score
            symmetry_weight: Weight for symmetry score
            temporal_weight: Weight for temporal smoothness
            distance_weight: Weight for distance-weighted confidence
            stability_window: Number of frames for stability computation
            min_points_required: Minimum points required for valid detection
        """
        self.coverage_weight = coverage_weight
        self.stability_weight = stability_weight
        self.symmetry_weight = symmetry_weight
        self.temporal_weight = temporal_weight
        self.distance_weight = distance_weight
        self.stability_window = stability_window
        self.min_points_required = min_points_required
        
        # History buffers for temporal metrics
        self.coefficient_history = deque(maxlen=stability_window)
        
        # Normalize weights
        total_weight = (coverage_weight + stability_weight + symmetry_weight +
                      temporal_weight + distance_weight)
        if abs(total_weight - 1.0) > 0.01:
            logger.warning(f"Weights don't sum to 1.0, normalizing...")
            self.coverage_weight /= total_weight
            self.stability_weight /= total_weight
            self.symmetry_weight /= total_weight
            self.temporal_weight /= total_weight
            self.distance_weight /= total_weight
        
        logger.info("LaneConfidenceScorer initialized")
    
    def compute_coverage_score(
        self,
        num_valid_points: int,
        num_expected_points: Optional[int] = None
    ) -> float:
        """
        Compute coverage ratio score.
        
        C_cov = min(1.0, N_valid / N_min_required)
        
        Args:
            num_valid_points: Number of valid detected points
            num_expected_points: Expected number of points (optional)
            
        Returns:
            Coverage score in [0, 1]
        """
        if num_expected_points is None:
            num_expected_points = self.min_points_required
        
        if num_expected_points == 0:
            return 0.0
        
        coverage = min(1.0, num_valid_points / num_expected_points)
        return coverage
    
    def compute_stability_score(
        self,
        current_coefficients: np.ndarray,
        threshold_a: float = 0.001,
        threshold_b: float = 0.01,
        threshold_c: float = 0.1
    ) -> float:
        """
        Compute stability score based on coefficient variation.
        
        C_stab = 1 - min(1.0, σ(Δcoeffs) / threshold)
        
        Args:
            current_coefficients: Current polynomial coefficients [a, b, c]
            threshold_a: Maximum acceptable variation for a
            threshold_b: Maximum acceptable variation for b
            threshold_c: Maximum acceptable variation for c
            
        Returns:
            Stability score in [0, 1]
        """
        if len(self.coefficient_history) < 2:
            # Not enough history, return neutral score
            self.coefficient_history.append(current_coefficients)
            return 0.5
        
        # Compute coefficient changes
        history_array = np.array(self.coefficient_history)
        coefficient_changes = np.diff(history_array, axis=0)
        
        # Compute standard deviation of changes
        std_changes = np.std(coefficient_changes, axis=0)
        
        # Normalize by thresholds
        normalized_std = np.array([
            min(1.0, std_changes[0] / threshold_a),
            min(1.0, std_changes[1] / threshold_b),
            min(1.0, std_changes[2] / threshold_c)
        ])
        
        # Stability score (higher = more stable)
        stability = 1.0 - np.mean(normalized_std)
        
        # Update history
        self.coefficient_history.append(current_coefficients)
        
        return max(0.0, min(1.0, stability))
    
    def compute_symmetry_score(
        self,
        left_coefficients: Optional[np.ndarray],
        right_coefficients: Optional[np.ndarray],
        y_eval: float = 50.0,
        kappa_max_diff: float = 0.05
    ) -> float:
        """
        Compute symmetry score between left and right lanes.
        
        C_sym = 1 - min(1.0, |κ_left - κ_right| / κ_max)
        
        Args:
            left_coefficients: Left lane polynomial coefficients
            right_coefficients: Right lane polynomial coefficients
            y_eval: Y coordinate to evaluate curvature (meters)
            kappa_max_diff: Maximum acceptable curvature difference
            
        Returns:
            Symmetry score in [0, 1]
        """
        if left_coefficients is None or right_coefficients is None:
            return 0.0
        
        # Compute curvatures
        kappa_left = self._compute_curvature(left_coefficients, y_eval)
        kappa_right = self._compute_curvature(right_coefficients, y_eval)
        
        # Curvature difference
        kappa_diff = abs(kappa_left - kappa_right)
        
        # Symmetry score
        symmetry = 1.0 - min(1.0, kappa_diff / kappa_max_diff)
        
        return max(0.0, min(1.0, symmetry))
    
    def _compute_curvature(
        self,
        coefficients: np.ndarray,
        y: float
    ) -> float:
        """
        Compute curvature at given y coordinate.
        
        κ = |2a| / (1 + (2a·y + b)²)^(3/2)
        
        Args:
            coefficients: Polynomial coefficients [a, b, c]
            y: Y coordinate (forward distance in meters)
            
        Returns:
            Curvature in [1/meters]
        """
        a, b, c = coefficients
        
        # First derivative: x'(y) = 2a·y + b
        x_prime = 2 * a * y + b
        
        # Second derivative: x''(y) = 2a
        x_double_prime = 2 * a
        
        # Curvature: κ = |x''| / (1 + x'²)^(3/2)
        curvature = abs(x_double_prime) / (1 + x_prime**2)**(3/2)
        
        return curvature
    
    def compute_temporal_smoothness(
        self,
        current_coefficients: np.ndarray,
        threshold: float = 0.5
    ) -> float:
        """
        Compute temporal smoothness score.
        
        C_temp = 1 - min(1.0, ||Δx_k|| / threshold)
        
        Args:
            current_coefficients: Current coefficients [a, b, c]
            threshold: Maximum acceptable change magnitude
            
        Returns:
            Temporal smoothness score in [0, 1]
        """
        if len(self.coefficient_history) == 0:
            return 0.5  # Neutral score
        
        # Get previous coefficients
        previous_coefficients = self.coefficient_history[-1]
        
        # Compute change magnitude
        delta = current_coefficients - previous_coefficients
        change_magnitude = np.linalg.norm(delta)
        
        # Temporal smoothness
        smoothness = 1.0 - min(1.0, change_magnitude / threshold)
        
        return max(0.0, min(1.0, smoothness))
    
    def compute_distance_weighted_confidence(
        self,
        points: np.ndarray,
        point_confidences: Optional[np.ndarray] = None,
        sigma: float = 25.0
    ) -> float:
        """
        Compute distance-weighted confidence.
        
        C_dist = Σ w_i · confidence_i / Σ w_i
        Where w_i = exp(-d_i² / σ²)
        
        Args:
            points: Array of shape (N, 2) with BEV coordinates
            point_confidences: Optional per-point confidences (default: 1.0)
            sigma: Distance decay parameter (meters)
            
        Returns:
            Distance-weighted confidence in [0, 1]
        """
        if len(points) == 0:
            return 0.0
        
        # Default confidences
        if point_confidences is None:
            point_confidences = np.ones(len(points))
        
        # Distance weights: w = exp(-d² / σ²)
        distances = points[:, 1]  # Forward distance
        weights = np.exp(-distances**2 / (2 * sigma**2))
        
        # Weighted average
        weighted_sum = np.sum(weights * point_confidences)
        weight_sum = np.sum(weights)
        
        if weight_sum > 0:
            return weighted_sum / weight_sum
        
        return 0.0
    
    def compute_comprehensive_confidence(
        self,
        num_valid_points: int,
        coefficients: np.ndarray,
        points: Optional[np.ndarray] = None,
        left_coefficients: Optional[np.ndarray] = None,
        right_coefficients: Optional[np.ndarray] = None,
        point_confidences: Optional[np.ndarray] = None
    ) -> Tuple[float, Dict[str, float]]:
        """
        Compute comprehensive confidence score.
        
        C_final = w1·C_cov + w2·C_stab + w3·C_sym + w4·C_temp + w5·C_dist
        
        Args:
            num_valid_points: Number of valid detected points
            coefficients: Current polynomial coefficients
            points: Optional BEV points for distance weighting
            left_coefficients: Optional left lane coefficients (for symmetry)
            right_coefficients: Optional right lane coefficients (for symmetry)
            point_confidences: Optional per-point confidences
            
        Returns:
            Tuple of (final_confidence, component_scores)
        """
        # Coverage score
        C_cov = self.compute_coverage_score(num_valid_points)
        
        # Stability score
        C_stab = self.compute_stability_score(coefficients)
        
        # Symmetry score
        C_sym = 0.0
        if left_coefficients is not None and right_coefficients is not None:
            C_sym = self.compute_symmetry_score(left_coefficients, right_coefficients)
        
        # Temporal smoothness
        C_temp = self.compute_temporal_smoothness(coefficients)
        
        # Distance-weighted confidence
        C_dist = 0.0
        if points is not None:
            C_dist = self.compute_distance_weighted_confidence(points, point_confidences)
        
        # Weighted combination
        C_final = (
            self.coverage_weight * C_cov +
            self.stability_weight * C_stab +
            self.symmetry_weight * C_sym +
            self.temporal_weight * C_temp +
            self.distance_weight * C_dist
        )
        
        # Component scores for debugging
        component_scores = {
            'coverage': C_cov,
            'stability': C_stab,
            'symmetry': C_sym,
            'temporal': C_temp,
            'distance': C_dist
        }
        
        return max(0.0, min(1.0, C_final)), component_scores
    
    def reset_history(self):
        """Reset coefficient history."""
        self.coefficient_history.clear()

