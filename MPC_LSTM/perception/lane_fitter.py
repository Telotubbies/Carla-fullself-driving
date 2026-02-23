"""
Robust Polynomial Fitting for Lane Detection

Implements:
- RANSAC outlier rejection
- Weighted Least Squares (WLS)
- Distance-aware weighting
- Multi-zone fitting and fusion
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
import logging
from scipy.optimize import least_squares

logger = logging.getLogger(__name__)


class RobustLaneFitter:
    """
    Robust polynomial fitting with RANSAC and weighted least squares.
    
    Fits polynomial: x = a·y² + b·y + c
    """
    
    def __init__(
        self,
        polynomial_order: int = 2,
        ransac_threshold: float = 0.5,
        ransac_max_iterations: int = 100,
        ransac_min_samples: int = 3,
        distance_decay_sigma: float = 25.0,
        use_ransac: bool = True
    ):
        """
        Initialize robust lane fitter.
        
        Args:
            polynomial_order: Order of polynomial (2 for quadratic)
            ransac_threshold: Inlier threshold for RANSAC (meters)
            ransac_max_iterations: Maximum RANSAC iterations
            ransac_min_samples: Minimum samples for RANSAC (must be >= order+1)
            distance_decay_sigma: Distance decay parameter for weighting (meters)
            use_ransac: Enable RANSAC outlier rejection
        """
        self.polynomial_order = polynomial_order
        self.ransac_threshold = ransac_threshold
        self.ransac_max_iterations = ransac_max_iterations
        self.ransac_min_samples = max(ransac_min_samples, polynomial_order + 1)
        self.distance_decay_sigma = distance_decay_sigma
        self.use_ransac = use_ransac
        
        logger.info(f"RobustLaneFitter initialized: order={polynomial_order}, RANSAC={use_ransac}")
    
    def compute_distance_weights(
        self,
        points: np.ndarray
    ) -> np.ndarray:
        """
        Compute distance-aware weights for points.
        
        Weight formula: w_i = exp(-d_i² / σ²)
        Where d_i is forward distance (y-coordinate in BEV).
        
        Args:
            points: Array of shape (N, 2) with BEV coordinates [x, y]
            
        Returns:
            Array of shape (N,) with weights
        """
        if len(points) == 0:
            return np.array([])
        
        # Distance from vehicle (y-coordinate in BEV)
        distances = points[:, 1]  # Forward distance
        
        # Exponential decay: w = exp(-d² / σ²)
        weights = np.exp(-distances**2 / (2 * self.distance_decay_sigma**2))
        
        # Normalize to [0, 1] range
        if weights.max() > 0:
            weights = weights / weights.max()
        
        return weights
    
    def fit_polynomial_wls(
        self,
        points: np.ndarray,
        weights: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, float]:
        """
        Fit polynomial using Weighted Least Squares.
        
        Polynomial: x = a·y² + b·y + c (for order=2)
        
        Matrix form: X·θ = Y
        Solution: θ = (Xᵀ·W·X)⁻¹ · Xᵀ·W·Y
        
        Args:
            points: Array of shape (N, 2) with BEV coordinates [x, y]
            weights: Optional array of shape (N,) with point weights
            
        Returns:
            Tuple of (coefficients, residual_error)
            - coefficients: [a, b, c] for order=2
            - residual_error: Mean squared error
        """
        if len(points) < self.polynomial_order + 1:
            logger.warning(f"Insufficient points for fitting: {len(points)} < {self.polynomial_order + 1}")
            return None, float('inf')
        
        y_coords = points[:, 1]  # Forward distance
        x_coords = points[:, 0]  # Lateral position
        
        # Build design matrix X
        # For order=2: X = [y², y, 1]
        X = np.column_stack([y_coords**i for i in range(self.polynomial_order, -1, -1)])
        
        # Default weights (uniform)
        if weights is None:
            weights = np.ones(len(points))
        
        # Weight matrix (diagonal)
        W = np.diag(weights)
        
        # Weighted Least Squares: θ = (Xᵀ·W·X)⁻¹ · Xᵀ·W·Y
        try:
            XTWX = X.T @ W @ X
            XTWY = X.T @ W @ x_coords
            
            # Solve: (Xᵀ·W·X)·θ = Xᵀ·W·Y
            coefficients = np.linalg.solve(XTWX, XTWY)
            
            # Compute residual error
            x_predicted = np.polyval(coefficients, y_coords)
            residuals = x_coords - x_predicted
            mse = np.mean(weights * residuals**2)
            
            return coefficients, mse
            
        except np.linalg.LinAlgError as e:
            logger.warning(f"WLS fitting failed: {e}")
            return None, float('inf')
    
    def compute_residual(
        self,
        point: np.ndarray,
        coefficients: np.ndarray
    ) -> float:
        """
        Compute residual error for a point given polynomial coefficients.
        
        Args:
            point: Point [x, y] in BEV
            coefficients: Polynomial coefficients [a, b, c]
            
        Returns:
            Absolute residual error
        """
        x, y = point
        x_predicted = np.polyval(coefficients, y)
        return abs(x - x_predicted)
    
    def fit_polynomial_ransac(
        self,
        points: np.ndarray,
        weights: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], np.ndarray, float]:
        """
        Fit polynomial using RANSAC for outlier rejection.
        
        Algorithm:
        1. Sample minimum points
        2. Fit polynomial
        3. Count inliers
        4. Repeat N times
        5. Refit on best inliers
        
        Args:
            points: Array of shape (N, 2) with BEV coordinates
            weights: Optional array of shape (N,) with point weights
            
        Returns:
            Tuple of (coefficients, inlier_mask, residual_error)
        """
        if len(points) < self.ransac_min_samples:
            logger.warning(f"Insufficient points for RANSAC: {len(points)} < {self.ransac_min_samples}")
            return None, np.array([]), float('inf')
        
        best_model = None
        best_inliers = np.zeros(len(points), dtype=bool)
        best_score = 0
        
        # Default weights
        if weights is None:
            weights = np.ones(len(points))
        
        # RANSAC iterations
        for iteration in range(self.ransac_max_iterations):
            # Sample random points
            sample_indices = np.random.choice(
                len(points),
                size=self.ransac_min_samples,
                replace=False
            )
            sample_points = points[sample_indices]
            
            # Fit polynomial to sample
            coeffs, _ = self.fit_polynomial_wls(sample_points)
            
            if coeffs is None:
                continue
            
            # Count inliers
            inlier_mask = np.zeros(len(points), dtype=bool)
            for i, point in enumerate(points):
                residual = self.compute_residual(point, coeffs)
                if residual < self.ransac_threshold:
                    inlier_mask[i] = True
            
            # Score = weighted sum of inliers
            score = np.sum(weights[inlier_mask])
            
            # Update best model
            if score > best_score:
                best_score = score
                best_inliers = inlier_mask
                best_model = coeffs
        
        # Refit on all inliers using WLS
        if best_model is not None and np.sum(best_inliers) >= self.ransac_min_samples:
            inlier_points = points[best_inliers]
            inlier_weights = weights[best_inliers] if weights is not None else None
            
            best_model, mse = self.fit_polynomial_wls(inlier_points, inlier_weights)
            
            if best_model is not None:
                return best_model, best_inliers, mse
        
        return None, np.array([]), float('inf')
    
    def fit_polynomial(
        self,
        points: np.ndarray,
        weights: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], np.ndarray, float]:
        """
        Fit polynomial with optional RANSAC.
        
        Args:
            points: Array of shape (N, 2) with BEV coordinates
            weights: Optional array of shape (N,) with point weights
            
        Returns:
            Tuple of (coefficients, inlier_mask, residual_error)
        """
        if len(points) == 0:
            return None, np.array([]), float('inf')
        
        # Compute distance weights if not provided
        if weights is None:
            weights = self.compute_distance_weights(points)
        
        # Use RANSAC if enabled
        if self.use_ransac and len(points) >= self.ransac_min_samples:
            return self.fit_polynomial_ransac(points, weights)
        else:
            # Direct WLS fitting
            coeffs, mse = self.fit_polynomial_wls(points, weights)
            inlier_mask = np.ones(len(points), dtype=bool)
            return coeffs, inlier_mask, mse
    
    def fit_multi_zone(
        self,
        points: np.ndarray,
        zones: Dict[str, Tuple[float, float]],
        weights: Optional[np.ndarray] = None
    ) -> Dict[str, Tuple[Optional[np.ndarray], np.ndarray, float]]:
        """
        Fit separate polynomials for different distance zones.
        
        Args:
            points: Array of shape (N, 2) with BEV coordinates
            zones: Dict mapping zone name to (y_min, y_max) range
            weights: Optional array of shape (N,) with point weights
            
        Returns:
            Dict mapping zone name to (coefficients, inlier_mask, residual_error)
        """
        results = {}
        
        for zone_name, (y_min, y_max) in zones.items():
            # Filter points in zone
            zone_mask = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
            zone_points = points[zone_mask]
            
            if len(zone_points) < self.polynomial_order + 1:
                results[zone_name] = (None, np.array([]), float('inf'))
                continue
            
            # Filter weights for zone
            zone_weights = weights[zone_mask] if weights is not None else None
            
            # Fit polynomial
            coeffs, inliers, mse = self.fit_polynomial(zone_points, zone_weights)
            
            # Map inliers back to original point indices
            full_inlier_mask = np.zeros(len(points), dtype=bool)
            zone_indices = np.where(zone_mask)[0]
            full_inlier_mask[zone_indices[inliers]] = True
            
            results[zone_name] = (coeffs, full_inlier_mask, mse)
        
        return results
    
    def fuse_polynomials(
        self,
        polynomials: Dict[str, Optional[np.ndarray]],
        zone_weights: Optional[Dict[str, float]] = None
    ) -> Optional[np.ndarray]:
        """
        Fuse multiple zone polynomials into single polynomial.
        
        Uses weighted average at zone boundaries.
        
        Args:
            polynomials: Dict mapping zone name to polynomial coefficients
            zone_weights: Optional dict mapping zone name to weight
            
        Returns:
            Fused polynomial coefficients
        """
        valid_polynomials = {k: v for k, v in polynomials.items() if v is not None}
        
        if len(valid_polynomials) == 0:
            return None
        
        # Default weights (near zone has higher weight)
        if zone_weights is None:
            zone_weights = {
                'near': 0.5,
                'mid': 0.3,
                'far': 0.2
            }
        
        # Weighted average of coefficients
        total_weight = 0.0
        fused_coeffs = np.zeros(self.polynomial_order + 1)
        
        for zone_name, coeffs in valid_polynomials.items():
            weight = zone_weights.get(zone_name, 0.1)
            fused_coeffs += weight * coeffs
            total_weight += weight
        
        if total_weight > 0:
            fused_coeffs /= total_weight
            return fused_coeffs
        
        return None

