"""
Real-World Lane Detection Module
ใช้ Computer Vision แทน CARLA API เพื่อให้ใกล้เคียง real-world
"""

import numpy as np
import cv2
from typing import Tuple, Optional, List
import logging


class LaneDetector:
    """
    Lane Detection using Computer Vision (Real-World Approach)
    
    Process:
    1. Edge Detection (Canny)
    2. Region of Interest (ROI)
    3. Hough Line Transform
    4. Polynomial Fitting
    5. Calculate lane center, curvature, waypoint
    """
    
    def __init__(self, image_width: int = 160, image_height: int = 90, 
                 edge_detection_method: str = 'multiscale_canny'):
        self.image_width = image_width
        self.image_height = image_height
        self.edge_detection_method = edge_detection_method  # 'canny', 'multiscale_canny', 'enhanced_canny'
        
        # ROI (Region of Interest) - focus on lower half (road area)
        self.roi_top = int(image_height * 0.5)  # Lower half
        self.roi_bottom = image_height
        self.roi_left = 0
        self.roi_right = image_width
        
        # Canny Edge Detection parameters
        self.canny_low = 50
        self.canny_high = 150
        
        # Multi-scale Canny parameters (fine-tuned: best performance)
        self.multiscale_scales = [1.0, 0.5, 1.25, 2.0]  # Fine-tuned scales for better lane detection
        
        # Enhanced Canny parameters
        self.enhanced_clahe_clip_limit = 2.0
        self.enhanced_clahe_tile_size = (8, 8)
        self.enhanced_bilateral_d = 9
        self.enhanced_bilateral_sigma_color = 75
        self.enhanced_bilateral_sigma_space = 75
        
        # Hough Line Transform parameters
        self.hough_rho = 1  # Distance resolution in pixels
        self.hough_theta = np.pi / 180  # Angular resolution in radians
        self.hough_threshold = 30  # Minimum votes
        self.hough_min_line_len = 20  # Minimum line length
        self.hough_max_line_gap = 10  # Maximum gap between line segments
        
        # Polynomial fitting
        self.poly_order = 2  # 2nd order polynomial
        
        # Waypoint projection distance (in pixels, then convert to meters)
        self.waypoint_ahead_pixels = int(image_height * 0.3)  # 30% ahead
        
    def detect_lanes(self, image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect left and right lane lines from image
        
        Args:
            image: RGB image (H, W, 3) normalized [0, 1] or uint8 [0, 255]
            
        Returns:
            left_lane_points: (N, 2) array of (x, y) points for left lane
            right_lane_points: (N, 2) array of (x, y) points for right lane
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            if image.max() <= 1.0:
                # Normalized [0, 1] -> uint8 [0, 255]
                gray = (image.mean(axis=2) * 255).astype(np.uint8)
            else:
                gray = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_RGB2GRAY)
        else:
            gray = image.astype(np.uint8) if image.max() <= 1.0 else image
        
        # Apply edge detection based on method
        if self.edge_detection_method == 'multiscale_canny':
            edges = self._multiscale_canny(gray)
        elif self.edge_detection_method == 'enhanced_canny':
            edges = self._enhanced_canny(gray)
        else:  # Default: standard Canny
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # Region of Interest (ROI) - focus on lower half
        roi_mask = np.zeros_like(edges)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        # Hough Line Transform
        lines = cv2.HoughLinesP(
            masked_edges,
            self.hough_rho,
            self.hough_theta,
            self.hough_threshold,
            np.array([]),
            minLineLength=self.hough_min_line_len,
            maxLineGap=self.hough_max_line_gap
        )
        
        if lines is None or len(lines) == 0:
            return None, None
        
        # Separate left and right lanes
        left_points = []
        right_points = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate slope
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter by slope (lanes should be roughly vertical)
            if abs(slope) < 0.3:  # Too horizontal
                continue
            
            # Left lane: negative slope, right lane: positive slope
            if slope < 0:  # Left lane
                left_points.extend([(x1, y1), (x2, y2)])
            else:  # Right lane
                right_points.extend([(x1, y1), (x2, y2)])
        
        # Convert to numpy arrays
        left_lane = np.array(left_points) if left_points else None
        right_lane = np.array(right_points) if right_points else None
        
        return left_lane, right_lane
    
    def _multiscale_canny(self, gray: np.ndarray) -> np.ndarray:
        """
        Multi-scale Canny Edge Detection
        Combines edges detected at different scales for better lane detection
        """
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        all_edges = []
        
        for scale in self.multiscale_scales:
            if scale != 1.0:
                h, w = blurred.shape
                scaled = cv2.resize(blurred, (int(w*scale), int(h*scale)))
                edges = cv2.Canny(scaled, self.canny_low, self.canny_high)
                edges = cv2.resize(edges, (w, h))
            else:
                edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
            all_edges.append(edges)
        
        # Combine all scales (take maximum)
        combined = np.maximum.reduce(all_edges)
        return combined
    
    def _enhanced_canny(self, gray: np.ndarray) -> np.ndarray:
        """
        Enhanced Canny with better preprocessing
        Uses CLAHE (Contrast Limited Adaptive Histogram Equalization) and Bilateral Filter
        """
        # 1. Histogram equalization for better contrast
        clahe = cv2.createCLAHE(
            clipLimit=self.enhanced_clahe_clip_limit,
            tileGridSize=self.enhanced_clahe_tile_size
        )
        equalized = clahe.apply(gray)
        
        # 2. Bilateral filter (preserves edges while reducing noise)
        filtered = cv2.bilateralFilter(
            equalized,
            self.enhanced_bilateral_d,
            self.enhanced_bilateral_sigma_color,
            self.enhanced_bilateral_sigma_space
        )
        
        # 3. Adaptive Canny (uses median for thresholds)
        median = np.median(filtered)
        low = int(max(0, 0.5 * median))
        high = int(min(255, 1.5 * median))
        
        edges = cv2.Canny(filtered, low, high)
        return edges
    
    def fit_polynomial(self, points: np.ndarray) -> Optional[np.ndarray]:
        """
        Fit polynomial to lane points
        
        Args:
            points: (N, 2) array of (x, y) points
            
        Returns:
            Polynomial coefficients [a, b, c] for y = ax² + bx + c
            or None if not enough points
        """
        if points is None or len(points) < 3:
            return None
        
        try:
            # Fit 2nd order polynomial: y = ax² + bx + c
            # We fit x as function of y (inverse) for better stability
            y = points[:, 1]
            x = points[:, 0]
            
            # Filter out outliers
            if len(x) < 3:
                return None
            
            coeffs = np.polyfit(y, x, self.poly_order)  # Returns [a, b, c] for x = ay² + by + c
            return coeffs
        except:
            return None
    
    def calculate_lane_center(self, left_coeffs: Optional[np.ndarray], 
                            right_coeffs: Optional[np.ndarray], 
                            y_position: int) -> Optional[float]:
        """
        Calculate lane center position at given y position
        
        Args:
            left_coeffs: Left lane polynomial coefficients
            right_coeffs: Right lane polynomial coefficients
            y_position: Y position in image (row)
            
        Returns:
            Lane center x position, or None if can't calculate
        """
        if left_coeffs is None or right_coeffs is None:
            return None
        
        try:
            # Calculate x positions for both lanes
            left_x = np.polyval(left_coeffs, y_position)
            right_x = np.polyval(right_coeffs, y_position)
            
            # Lane center is midpoint
            center_x = (left_x + right_x) / 2.0
            return center_x
        except:
            return None
    
    def calculate_curvature(self, coeffs: Optional[np.ndarray], y_position: int) -> Optional[float]:
        """
        Calculate road curvature from polynomial coefficients
        
        Args:
            coeffs: Polynomial coefficients [a, b, c] for x = ay² + by + c
            y_position: Y position to calculate curvature at
            
        Returns:
            Curvature (1/radius) in pixels, or None
        """
        if coeffs is None:
            return None
        
        try:
            # For polynomial x = ay² + by + c
            # First derivative: dx/dy = 2ay + b
            # Second derivative: d²x/dy² = 2a
            
            first_deriv = 2 * coeffs[0] * y_position + coeffs[1]
            second_deriv = 2 * coeffs[0]
            
            # Curvature formula: |d²x/dy²| / (1 + (dx/dy)²)^(3/2)
            curvature = abs(second_deriv) / ((1 + first_deriv**2)**1.5)
            
            return curvature
        except:
            return None
    
    def estimate_waypoint(self, left_coeffs: Optional[np.ndarray],
                         right_coeffs: Optional[np.ndarray],
                         vehicle_center_x: float) -> Tuple[Optional[float], Optional[float]]:
        """
        Estimate waypoint direction from lane detection
        
        Args:
            left_coeffs: Left lane polynomial coefficients
            right_coeffs: Right lane polynomial coefficients
            vehicle_center_x: Vehicle center x position in image
            
        Returns:
            (waypoint_x, waypoint_y) in image coordinates, or (None, None)
        """
        if left_coeffs is None or right_coeffs is None:
            return None, None
        
        try:
            # Project waypoint ahead (at ROI top, which is ahead of vehicle)
            waypoint_y = self.roi_top
            
            # Calculate lane center at waypoint position
            waypoint_x = self.calculate_lane_center(left_coeffs, right_coeffs, waypoint_y)
            
            if waypoint_x is None:
                return None, None
            
            # Calculate direction vector (relative to vehicle center)
            dx = waypoint_x - vehicle_center_x
            dy = waypoint_y - self.image_height  # Vehicle is at bottom
            
            return dx, dy
        except:
            return None, None
    
    def detect_lane_change_availability(self, left_coeffs: Optional[np.ndarray],
                                      right_coeffs: Optional[np.ndarray],
                                      current_y: int) -> Tuple[bool, bool]:
        """
        Detect if lane change is available (simplified)
        
        Args:
            left_coeffs: Left lane polynomial coefficients
            right_coeffs: Right lane polynomial coefficients
            current_y: Current y position
            
        Returns:
            (can_change_left, can_change_right)
        """
        # Simplified: assume lane change is available if we can detect lanes
        # In real-world, would check for dashed vs solid lines
        can_change_left = left_coeffs is not None
        can_change_right = right_coeffs is not None
        
        return can_change_left, can_change_right
    
    def process_image(self, image: np.ndarray) -> dict:
        """
        Process image and extract all lane/waypoint features
        
        Args:
            image: RGB image (H, W, 3)
            
        Returns:
            Dictionary with:
            - lane_center_offset: Distance from vehicle center to lane center
            - waypoint_direction: (dx, dy) direction to waypoint
            - curvature: Road curvature
            - lane_change_left: Can change left?
            - lane_change_right: Can change right?
        """
        # Detect lanes
        left_lane, right_lane = self.detect_lanes(image)
        
        # Fit polynomials
        left_coeffs = self.fit_polynomial(left_lane) if left_lane is not None else None
        right_coeffs = self.fit_polynomial(right_lane) if right_lane is not None else None
        
        # Vehicle center (assume image center)
        vehicle_center_x = self.image_width / 2.0
        vehicle_center_y = self.image_height
        
        # Calculate lane center offset
        lane_center = self.calculate_lane_center(
            left_coeffs, right_coeffs, vehicle_center_y
        )
        lane_center_offset = (lane_center - vehicle_center_x) if lane_center is not None else 0.0
        
        # Estimate waypoint
        waypoint_dx, waypoint_dy = self.estimate_waypoint(
            left_coeffs, right_coeffs, vehicle_center_x
        )
        
        # Calculate curvature (use average of left and right)
        curvature = None
        if left_coeffs is not None and right_coeffs is not None:
            left_curv = self.calculate_curvature(left_coeffs, vehicle_center_y)
            right_curv = self.calculate_curvature(right_coeffs, vehicle_center_y)
            if left_curv is not None and right_curv is not None:
                curvature = (left_curv + right_curv) / 2.0
        
        # Lane change availability
        can_change_left, can_change_right = self.detect_lane_change_availability(
            left_coeffs, right_coeffs, vehicle_center_y
        )
        
        # Normalize features
        # Lane center offset: normalize by image width
        normalized_offset = np.clip(lane_center_offset / (self.image_width / 2.0), -1.0, 1.0)
        
        # Waypoint direction: normalize
        if waypoint_dx is not None and waypoint_dy is not None:
            # Normalize by image dimensions
            normalized_dx = np.clip(waypoint_dx / (self.image_width / 2.0), -1.0, 1.0)
            normalized_dy = np.clip(waypoint_dy / self.waypoint_ahead_pixels, -1.0, 1.0)
        else:
            normalized_dx = 0.0
            normalized_dy = 0.0
        
        # Curvature: normalize (typical range: 0-0.01)
        normalized_curvature = np.clip(curvature / 0.01 if curvature is not None else 0.0, -1.0, 1.0)
        
        return {
            'lane_center_offset': normalized_offset,
            'waypoint_dx': normalized_dx,
            'waypoint_dy': normalized_dy,
            'curvature': normalized_curvature,
            'lane_change_left': 1.0 if can_change_left else 0.0,
            'lane_change_right': 1.0 if can_change_right else 0.0,
            'detection_success': 1.0 if (left_coeffs is not None and right_coeffs is not None) else 0.0
        }

