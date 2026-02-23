"""
Real-World Lane Metrics Computation

Computes real-world metrics from lane polynomials:
- Curvature in meters
- Lateral offset
- Lane width
- Heading angle
- MPC-compatible waypoints
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
import logging

logger = logging.getLogger(__name__)


class LaneMetricsCalculator:
    """
    Computes real-world metrics from lane polynomials.
    """
    
    def __init__(
        self,
        waypoint_spacing: float = 1.0,  # meters
        waypoint_range: Tuple[float, float] = (0.0, 100.0)  # meters
    ):
        """
        Initialize metrics calculator.
        
        Args:
            waypoint_spacing: Spacing between waypoints (meters)
            waypoint_range: Range for waypoint generation (y_min, y_max) in meters
        """
        self.waypoint_spacing = waypoint_spacing
        self.waypoint_range = waypoint_range
        
        logger.info("LaneMetricsCalculator initialized")
    
    def compute_curvature(
        self,
        coefficients: np.ndarray,
        y: float
    ) -> float:
        """
        Compute curvature at given y coordinate.
        
        κ = |2a| / (1 + (2a·y + b)²)^(3/2)  [1/meters]
        
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
    
    def compute_radius_of_curvature(
        self,
        coefficients: np.ndarray,
        y: float
    ) -> float:
        """
        Compute radius of curvature.
        
        R = 1/κ  [meters]
        
        Args:
            coefficients: Polynomial coefficients
            y: Y coordinate
            
        Returns:
            Radius of curvature in meters
        """
        curvature = self.compute_curvature(coefficients, y)
        if curvature < 1e-6:  # Avoid division by zero
            return float('inf')
        return 1.0 / curvature
    
    def compute_heading_angle(
        self,
        coefficients: np.ndarray,
        y: float
    ) -> float:
        """
        Compute heading angle (yaw) at given y coordinate.
        
        ψ = arctan(x'(y)) = arctan(2a·y + b)  [radians]
        
        Args:
            coefficients: Polynomial coefficients [a, b, c]
            y: Y coordinate (forward distance in meters)
            
        Returns:
            Heading angle in radians
        """
        a, b, c = coefficients
        x_prime = 2 * a * y + b
        heading = np.arctan(x_prime)
        return heading
    
    def compute_lateral_offset(
        self,
        centerline_coefficients: np.ndarray,
        vehicle_y: float = 0.0
    ) -> float:
        """
        Compute vehicle lateral offset from lane centerline.
        
        Δy = x_vehicle - f_center(y_vehicle)
        
        Args:
            centerline_coefficients: Centerline polynomial coefficients
            vehicle_y: Vehicle forward position (typically 0)
            
        Returns:
            Lateral offset in meters (positive = right, negative = left)
        """
        x_centerline = np.polyval(centerline_coefficients, vehicle_y)
        # Assuming vehicle is at x=0 in BEV
        lateral_offset = -x_centerline  # Negative because centerline x is lateral position
        return lateral_offset
    
    def compute_lane_width(
        self,
        left_coefficients: np.ndarray,
        right_coefficients: np.ndarray,
        y: float
    ) -> float:
        """
        Compute lane width at given y coordinate.
        
        W(y) = |f_right(y) - f_left(y)|  [meters]
        
        Args:
            left_coefficients: Left lane polynomial coefficients
            right_coefficients: Right lane polynomial coefficients
            y: Y coordinate (forward distance)
            
        Returns:
            Lane width in meters
        """
        x_left = np.polyval(left_coefficients, y)
        x_right = np.polyval(right_coefficients, y)
        width = abs(x_right - x_left)
        return width
    
    def compute_centerline(
        self,
        left_coefficients: np.ndarray,
        right_coefficients: np.ndarray
    ) -> np.ndarray:
        """
        Compute lane centerline from left and right boundaries.
        
        f_center(y) = (f_left(y) + f_right(y)) / 2
        
        Args:
            left_coefficients: Left lane polynomial coefficients
            right_coefficients: Right lane polynomial coefficients
            
        Returns:
            Centerline polynomial coefficients [a, b, c]
        """
        # Average coefficients
        centerline = (left_coefficients + right_coefficients) / 2.0
        return centerline
    
    def generate_waypoints(
        self,
        centerline_coefficients: np.ndarray,
        num_waypoints: Optional[int] = None
    ) -> np.ndarray:
        """
        Generate waypoints along centerline for MPC.
        
        Args:
            centerline_coefficients: Centerline polynomial coefficients
            num_waypoints: Number of waypoints (uses spacing if None)
            
        Returns:
            Array of shape (N, 2) with waypoints [x, y] in BEV (meters)
        """
        y_min, y_max = self.waypoint_range
        
        if num_waypoints is None:
            num_waypoints = int((y_max - y_min) / self.waypoint_spacing) + 1
        
        # Generate y coordinates
        y_coords = np.linspace(y_min, y_max, num_waypoints)
        
        # Compute x coordinates from polynomial
        x_coords = np.polyval(centerline_coefficients, y_coords)
        
        # Stack into waypoints
        waypoints = np.column_stack([x_coords, y_coords])
        
        return waypoints
    
    def generate_mpc_reference(
        self,
        centerline_coefficients: np.ndarray,
        num_waypoints: int = 50
    ) -> Dict[str, np.ndarray]:
        """
        Generate MPC-compatible reference trajectory.
        
        Args:
            centerline_coefficients: Centerline polynomial coefficients
            num_waypoints: Number of waypoints
            
        Returns:
            Dict with:
            - x_ref: Lateral positions [meters]
            - y_ref: Forward positions [meters]
            - psi_ref: Heading angles [radians]
            - kappa_ref: Curvatures [1/meters]
        """
        waypoints = self.generate_waypoints(centerline_coefficients, num_waypoints)
        
        y_coords = waypoints[:, 1]
        x_coords = waypoints[:, 0]
        
        # Compute heading angles
        psi_ref = np.array([
            self.compute_heading_angle(centerline_coefficients, y)
            for y in y_coords
        ])
        
        # Compute curvatures
        kappa_ref = np.array([
            self.compute_curvature(centerline_coefficients, y)
            for y in y_coords
        ])
        
        return {
            'x_ref': x_coords,
            'y_ref': y_coords,
            'psi_ref': psi_ref,
            'kappa_ref': kappa_ref
        }
    
    def compute_lane_statistics(
        self,
        left_coefficients: Optional[np.ndarray],
        right_coefficients: Optional[np.ndarray],
        centerline_coefficients: Optional[np.ndarray] = None,
        y_eval: float = 50.0
    ) -> Dict[str, float]:
        """
        Compute comprehensive lane statistics.
        
        Args:
            left_coefficients: Left lane coefficients
            right_coefficients: Right lane coefficients
            centerline_coefficients: Centerline coefficients (computed if None)
            y_eval: Y coordinate for evaluation
            
        Returns:
            Dict with lane statistics
        """
        stats = {}
        
        # Compute centerline if not provided
        if centerline_coefficients is None:
            if left_coefficients is not None and right_coefficients is not None:
                centerline_coefficients = self.compute_centerline(
                    left_coefficients, right_coefficients
                )
        
        # Centerline metrics
        if centerline_coefficients is not None:
            stats['curvature'] = self.compute_curvature(centerline_coefficients, y_eval)
            stats['radius'] = self.compute_radius_of_curvature(centerline_coefficients, y_eval)
            stats['heading'] = self.compute_heading_angle(centerline_coefficients, y_eval)
            stats['lateral_offset'] = self.compute_lateral_offset(centerline_coefficients)
        
        # Lane width
        if left_coefficients is not None and right_coefficients is not None:
            stats['lane_width'] = self.compute_lane_width(
                left_coefficients, right_coefficients, y_eval
            )
        
        return stats

