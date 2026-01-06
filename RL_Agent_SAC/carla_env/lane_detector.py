import numpy as np
import cv2
from typing import Tuple, Optional, List
import logging
class LaneDetector:
    
    def __init__(self, image_width: int = 160, image_height: int = 90,
                 edge_detection_method: str = 'multiscale_canny'):
        self.image_width = image_width
        self.image_height = image_height
        self.edge_detection_method = edge_detection_method
        self.roi_top = int(image_height * 0.5)
        self.roi_bottom = image_height
        self.roi_left = 0
        self.roi_right = image_width
        self.canny_low = 50
        self.canny_high = 150
        self.multiscale_scales = [1.0, 0.5, 1.25, 2.0]
        self.enhanced_clahe_clip_limit = 2.0
        self.enhanced_clahe_tile_size = (8, 8)
        self.enhanced_bilateral_d = 9
        self.enhanced_bilateral_sigma_color = 75
        self.enhanced_bilateral_sigma_space = 75
        self.hough_rho = 1
        self.hough_theta = np.pi / 180
        self.hough_threshold = 30
        self.hough_min_line_len = 20
        self.hough_max_line_gap = 10
        self.poly_order = 2
        self.waypoint_ahead_pixels = int(image_height * 0.3)
    def detect_lanes(self, image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        
        if len(image.shape) == 3:
            if image.max() <= 1.0:
                gray = (image.mean(axis=2) * 255).astype(np.uint8)
            else:
                gray = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_RGB2GRAY)
        else:
            gray = image.astype(np.uint8) if image.max() <= 1.0 else image
        if self.edge_detection_method == 'multiscale_canny':
            edges = self._multiscale_canny(gray)
        elif self.edge_detection_method == 'enhanced_canny':
            edges = self._enhanced_canny(gray)
        else:
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        roi_mask = np.zeros_like(edges)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
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
        left_points = []
        right_points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.3:
                continue
            if slope < 0:
                left_points.extend([(x1, y1), (x2, y2)])
            else:
                right_points.extend([(x1, y1), (x2, y2)])
        left_lane = np.array(left_points) if left_points else None
        right_lane = np.array(right_points) if right_points else None
        return left_lane, right_lane
    def _multiscale_canny(self, gray: np.ndarray) -> np.ndarray:
        
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
        combined = np.maximum.reduce(all_edges)
        return combined
    def _enhanced_canny(self, gray: np.ndarray) -> np.ndarray:
        
        clahe = cv2.createCLAHE(
            clipLimit=self.enhanced_clahe_clip_limit,
            tileGridSize=self.enhanced_clahe_tile_size
        )
        equalized = clahe.apply(gray)
        filtered = cv2.bilateralFilter(
            equalized,
            self.enhanced_bilateral_d,
            self.enhanced_bilateral_sigma_color,
            self.enhanced_bilateral_sigma_space
        )
        median = np.median(filtered)
        low = int(max(0, 0.5 * median))
        high = int(min(255, 1.5 * median))
        edges = cv2.Canny(filtered, low, high)
        return edges
    def fit_polynomial(self, points: np.ndarray) -> Optional[np.ndarray]:
        
        if points is None or len(points) < 3:
            return None
        try:
            y = points[:, 1]
            x = points[:, 0]
            if len(x) < 3:
                return None
            coeffs = np.polyfit(y, x, self.poly_order)
            return coeffs
        except:
            return None
    def calculate_lane_center(self, left_coeffs: Optional[np.ndarray],
                            right_coeffs: Optional[np.ndarray],
                            y_position: int) -> Optional[float]:
        
        if left_coeffs is None or right_coeffs is None:
            return None
        try:
            left_x = np.polyval(left_coeffs, y_position)
            right_x = np.polyval(right_coeffs, y_position)
            center_x = (left_x + right_x) / 2.0
            return center_x
        except:
            return None
    def calculate_curvature(self, coeffs: Optional[np.ndarray], y_position: int) -> Optional[float]:
        
        if coeffs is None:
            return None
        try:
            first_deriv = 2 * coeffs[0] * y_position + coeffs[1]
            second_deriv = 2 * coeffs[0]
            curvature = abs(second_deriv) / ((1 + first_deriv**2)**1.5)
            return curvature
        except:
            return None
    def estimate_waypoint(self, left_coeffs: Optional[np.ndarray],
                         right_coeffs: Optional[np.ndarray],
                         vehicle_center_x: float) -> Tuple[Optional[float], Optional[float]]:
        
        if left_coeffs is None or right_coeffs is None:
            return None, None
        try:
            waypoint_y = self.roi_top
            waypoint_x = self.calculate_lane_center(left_coeffs, right_coeffs, waypoint_y)
            if waypoint_x is None:
                return None, None
            dx = waypoint_x - vehicle_center_x
            dy = waypoint_y - self.image_height
            return dx, dy
        except:
            return None, None
    def detect_lane_change_availability(self, left_coeffs: Optional[np.ndarray],
                                      right_coeffs: Optional[np.ndarray],
                                      current_y: int) -> Tuple[bool, bool]:
        
        can_change_left = left_coeffs is not None
        can_change_right = right_coeffs is not None
        return can_change_left, can_change_right
    def process_image(self, image: np.ndarray) -> dict:
        
        left_lane, right_lane = self.detect_lanes(image)
        left_coeffs = self.fit_polynomial(left_lane) if left_lane is not None else None
        right_coeffs = self.fit_polynomial(right_lane) if right_lane is not None else None
        vehicle_center_x = self.image_width / 2.0
        vehicle_center_y = self.image_height
        lane_center = self.calculate_lane_center(
            left_coeffs, right_coeffs, vehicle_center_y
        )
        lane_center_offset = (lane_center - vehicle_center_x) if lane_center is not None else 0.0
        waypoint_dx, waypoint_dy = self.estimate_waypoint(
            left_coeffs, right_coeffs, vehicle_center_x
        )
        curvature = None
        if left_coeffs is not None and right_coeffs is not None:
            left_curv = self.calculate_curvature(left_coeffs, vehicle_center_y)
            right_curv = self.calculate_curvature(right_coeffs, vehicle_center_y)
            if left_curv is not None and right_curv is not None:
                curvature = (left_curv + right_curv) / 2.0
        can_change_left, can_change_right = self.detect_lane_change_availability(
            left_coeffs, right_coeffs, vehicle_center_y
        )
        normalized_offset = np.clip(lane_center_offset / (self.image_width / 2.0), -1.0, 1.0)
        if waypoint_dx is not None and waypoint_dy is not None:
            normalized_dx = np.clip(waypoint_dx / (self.image_width / 2.0), -1.0, 1.0)
            normalized_dy = np.clip(waypoint_dy / self.waypoint_ahead_pixels, -1.0, 1.0)
        else:
            normalized_dx = 0.0
            normalized_dy = 0.0
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