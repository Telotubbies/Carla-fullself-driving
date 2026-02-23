"""
Lane Post-Processing Module

ใช้ math algorithms เพื่อ:
1. กำจัด noise
2. แบ่ง lanes เป็น 1, 2, 3, 4
3. เพิ่มความแม่นยำด้วย polynomial fitting และ temporal smoothing
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional, Dict
from sklearn.cluster import DBSCAN
from scipy.interpolate import interp1d
from scipy.ndimage import median_filter
import logging

logger = logging.getLogger(__name__)


class LanePostProcessor:
    """
    Post-process lane detection results to:
    - Remove noise
    - Separate lanes into individual lanes (1, 2, 3, 4)
    - Smooth lanes using polynomial fitting
    - Apply temporal smoothing across frames
    """
    
    def __init__(
        self,
        image_height: int = 480,
        image_width: int = 640,
        min_lane_points: int = 20,
        polynomial_order: int = 2,
        temporal_smoothing: bool = True,
        temporal_window: int = 5,
        enable_numbering: bool = False  # Disable numbering by default (causes overlapping)
    ):
        """
        Initialize lane post-processor.
        
        Args:
            image_height: Image height
            image_width: Image width
            min_lane_points: Minimum points required for a valid lane
            polynomial_order: Order of polynomial for fitting (2 or 3)
            temporal_smoothing: Enable temporal smoothing across frames
            temporal_window: Number of frames for temporal smoothing
        """
        self.image_height = image_height
        self.image_width = image_width
        self.min_lane_points = min_lane_points
        self.polynomial_order = polynomial_order
        self.temporal_smoothing = temporal_smoothing
        self.temporal_window = temporal_window
        self.enable_numbering = enable_numbering
        
        # Temporal smoothing buffers (only if numbering enabled)
        self.lane_history: Dict[int, List[np.ndarray]] = {}  # lane_id -> list of polynomial coeffs
        
        logger.info(f"LanePostProcessor initialized (poly_order={polynomial_order}, temporal={temporal_smoothing}, numbering={enable_numbering})")
    
    def process(
        self,
        lane_mask: np.ndarray,
        lane_coords: Optional[List[List[Tuple[int, int]]]] = None
    ) -> Tuple[np.ndarray, List[np.ndarray], Dict[int, np.ndarray]]:
        """
        Process lane detection results.
        
        Args:
            lane_mask: Binary lane mask (H, W)
            lane_coords: Optional list of lane coordinates from model
            
        Returns:
            Tuple of:
            - cleaned_mask: Cleaned binary mask
            - lane_polynomials: List of polynomial coefficients for each lane
            - numbered_lanes: Dict mapping lane_id (1,2,3,4) to lane points
        """
        # Step 1: Noise reduction
        cleaned_mask = self._reduce_noise(lane_mask)
        
        # Step 2: Extract lane points
        if lane_coords is not None and len(lane_coords) > 0:
            # Use model's lane coordinates if available
            lane_points_list = self._extract_points_from_coords(lane_coords)
        else:
            # Extract from mask
            lane_points_list = self._extract_points_from_mask(cleaned_mask)
        
        if len(lane_points_list) == 0:
            logger.debug("No lanes detected after processing")
            return cleaned_mask, [], {}
        
        # Step 3: Cluster lanes (separate into individual lanes)
        separated_lanes = self._cluster_lanes(lane_points_list)
        
        if len(separated_lanes) == 0:
            logger.debug("No valid lanes after clustering")
            return cleaned_mask, [], {}
        
        # Step 4: Fit polynomials to each lane
        lane_polynomials = []
        lane_points_dict = {}
        
        for lane_id, lane_points in enumerate(separated_lanes):
            if len(lane_points) < self.min_lane_points:
                continue
            
            # Fit polynomial
            poly_coeffs = self._fit_polynomial(lane_points)
            if poly_coeffs is not None:
                lane_polynomials.append(poly_coeffs)
                lane_points_dict[lane_id] = lane_points
        
        # Step 5: Number lanes (optional - disabled by default to avoid overlapping)
        if self.enable_numbering:
            numbered_lanes = self._number_lanes(lane_points_dict, lane_polynomials)
            
            # Step 6: Temporal smoothing (if enabled)
            if self.temporal_smoothing:
                numbered_lanes = self._apply_temporal_smoothing(numbered_lanes)
            
            # Step 7: Reconstruct cleaned mask from processed lanes
            cleaned_mask = self._reconstruct_mask_from_lanes(numbered_lanes)
            
            return cleaned_mask, lane_polynomials, numbered_lanes
        else:
            # No numbering - return simple list of polynomials
            # Reconstruct mask from polynomial list
            cleaned_mask = self._reconstruct_mask_from_polynomials(lane_polynomials)
            
            return cleaned_mask, lane_polynomials, lane_polynomials  # Return polynomials as "processed_lanes"
    
    def _reduce_noise(self, mask: np.ndarray) -> np.ndarray:
        """
        Reduce noise in lane mask using morphological operations and median filter.
        
        Args:
            mask: Binary lane mask
            
        Returns:
            Cleaned mask
        """
        # 1. Median filter to remove salt-and-pepper noise
        mask_filtered = median_filter(mask, size=3)
        
        # 2. Morphological opening to remove small noise
        kernel_open = np.ones((3, 3), np.uint8)
        mask_opened = cv2.morphologyEx(mask_filtered, cv2.MORPH_OPEN, kernel_open, iterations=1)
        
        # 3. Morphological closing to fill small gaps
        kernel_close = np.ones((5, 5), np.uint8)
        mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel_close, iterations=1)
        
        # 4. Remove small connected components (noise)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_closed, connectivity=8)
        
        # Filter out small components (less than 50 pixels)
        min_area = 50
        cleaned_mask = np.zeros_like(mask_closed)
        
        for label_id in range(1, num_labels):  # Skip background (label 0)
            area = stats[label_id, cv2.CC_STAT_AREA]
            if area >= min_area:
                cleaned_mask[labels == label_id] = 255
        
        return cleaned_mask
    
    def _extract_points_from_mask(self, mask: np.ndarray) -> List[np.ndarray]:
        """
        Extract lane points from binary mask.
        
        Args:
            mask: Binary lane mask
            
        Returns:
            List of point arrays (each array is Nx2)
        """
        # Find all lane pixels
        y_coords, x_coords = np.where(mask > 0)
        
        if len(y_coords) == 0:
            return []
        
        # Group points by y-coordinate (horizontal lines)
        points_by_y = {}
        for y, x in zip(y_coords, x_coords):
            if y not in points_by_y:
                points_by_y[y] = []
            points_by_y[y].append(x)
        
        # Extract points (sample every few rows to reduce computation)
        all_points = []
        for y in sorted(points_by_y.keys())[::2]:  # Sample every 2 rows
            x_list = sorted(points_by_y[y])
            for x in x_list[::2]:  # Sample every 2 points in row
                all_points.append([x, y])
        
        if len(all_points) == 0:
            return []
        
        return [np.array(all_points, dtype=np.float32)]
    
    def _extract_points_from_coords(self, lane_coords: List[List[Tuple[int, int]]]) -> List[np.ndarray]:
        """
        Extract points from model's lane coordinates.
        
        Args:
            lane_coords: List of lane coordinate lists
            
        Returns:
            List of point arrays
        """
        point_arrays = []
        
        for lane in lane_coords:
            if len(lane) >= 2:
                points = np.array(lane, dtype=np.float32)
                point_arrays.append(points)
        
        return point_arrays
    
    def _cluster_lanes(self, lane_points_list: List[np.ndarray]) -> List[np.ndarray]:
        """
        Separate lanes using DBSCAN clustering.
        
        Args:
            lane_points_list: List of point arrays
            
        Returns:
            List of separated lane point arrays
        """
        if len(lane_points_list) == 0:
            return []
        
        # Combine all points
        all_points = np.vstack(lane_points_list)
        
        if len(all_points) < self.min_lane_points:
            return []
        
        # DBSCAN parameters
        # eps: maximum distance between points in same cluster (in pixels)
        # min_samples: minimum points to form a cluster
        eps = 30.0  # 30 pixels
        min_samples = self.min_lane_points // 2
        
        # Apply DBSCAN
        clustering = DBSCAN(eps=eps, min_samples=min_samples, metric='euclidean')
        labels = clustering.fit_predict(all_points)
        
        # Separate into clusters
        unique_labels = np.unique(labels)
        separated_lanes = []
        
        for label in unique_labels:
            if label == -1:  # Noise points (outliers)
                continue
            
            cluster_points = all_points[labels == label]
            if len(cluster_points) >= self.min_lane_points:
                separated_lanes.append(cluster_points)
        
        logger.debug(f"Clustered {len(lane_points_list)} input lanes into {len(separated_lanes)} separated lanes")
        
        return separated_lanes
    
    def _fit_polynomial(self, points: np.ndarray) -> Optional[np.ndarray]:
        """
        Fit polynomial to lane points.
        
        Args:
            points: Lane points (Nx2) as [x, y] or [y, x]
            
        Returns:
            Polynomial coefficients (highest order first) or None if fitting fails
        """
        if len(points) < self.polynomial_order + 1:
            return None
        
        # Sort points by y-coordinate (top to bottom)
        # For lane detection, we fit x = f(y) (x as function of y)
        y_coords = points[:, 1]  # Row (y)
        x_coords = points[:, 0]  # Column (x)
        
        # Sort by y
        sort_idx = np.argsort(y_coords)
        y_sorted = y_coords[sort_idx]
        x_sorted = x_coords[sort_idx]
        
        # Remove duplicates in y
        unique_y, unique_idx = np.unique(y_sorted, return_index=True)
        if len(unique_y) < self.polynomial_order + 1:
            return None
        
        x_unique = x_sorted[unique_idx]
        
        try:
            # Fit polynomial: x = a*y^2 + b*y + c (for order 2)
            # or x = a*y^3 + b*y^2 + c*y + d (for order 3)
            coeffs = np.polyfit(unique_y, x_unique, self.polynomial_order)
            return coeffs
        except np.linalg.LinAlgError:
            logger.warning("Polynomial fitting failed (singular matrix)")
            return None
    
    def _number_lanes(
        self,
        lane_points_dict: Dict[int, np.ndarray],
        lane_polynomials: List[np.ndarray]
    ) -> Dict[int, np.ndarray]:
        """
        Number lanes from left to right (1, 2, 3, 4).
        
        Args:
            lane_points_dict: Dict mapping lane_id to points
            lane_polynomials: List of polynomial coefficients
            
        Returns:
            Dict mapping numbered lane_id (1,2,3,4) to polynomial coefficients
        """
        if len(lane_points_dict) == 0:
            return {}
        
        # Evaluate polynomial at bottom of image to get x-position
        # Lane with smallest x (leftmost) = lane 1
        bottom_y = self.image_height - 1
        
        lane_x_positions = []
        lane_indices = []
        
        for lane_id, poly_coeffs in enumerate(lane_polynomials):
            if poly_coeffs is None:
                continue
            
            # Evaluate polynomial at bottom: x = poly(bottom_y)
            x_at_bottom = np.polyval(poly_coeffs, bottom_y)
            lane_x_positions.append(x_at_bottom)
            lane_indices.append(lane_id)
        
        if len(lane_x_positions) == 0:
            return {}
        
        # Sort by x-position (left to right)
        sorted_indices = np.argsort(lane_x_positions)
        
        # Number lanes (1, 2, 3, 4)
        numbered_lanes = {}
        for numbered_id, original_idx in enumerate(sorted_indices, start=1):
            if numbered_id > 4:  # Maximum 4 lanes
                break
            
            original_lane_id = lane_indices[original_idx]
            poly_coeffs = lane_polynomials[original_lane_id]
            numbered_lanes[numbered_id] = poly_coeffs
        
        logger.debug(f"Numbered {len(numbered_lanes)} lanes: {list(numbered_lanes.keys())}")
        
        return numbered_lanes
    
    def _apply_temporal_smoothing(
        self,
        numbered_lanes: Dict[int, np.ndarray]
    ) -> Dict[int, np.ndarray]:
        """
        Apply temporal smoothing across frames using moving average.
        
        Args:
            numbered_lanes: Current frame's numbered lanes
            
        Returns:
            Smoothed numbered lanes
        """
        smoothed_lanes = {}
        
        for lane_id, poly_coeffs in numbered_lanes.items():
            # Add to history
            if lane_id not in self.lane_history:
                self.lane_history[lane_id] = []
            
            self.lane_history[lane_id].append(poly_coeffs)
            
            # Keep only recent history
            if len(self.lane_history[lane_id]) > self.temporal_window:
                self.lane_history[lane_id].pop(0)
            
            # Average coefficients
            history = np.array(self.lane_history[lane_id])
            smoothed_coeffs = np.mean(history, axis=0)
            smoothed_lanes[lane_id] = smoothed_coeffs
        
        return smoothed_lanes
    
    def _reconstruct_mask_from_lanes(
        self,
        numbered_lanes: Dict[int, np.ndarray]
    ) -> np.ndarray:
        """
        Reconstruct binary mask from processed lanes.
        
        Args:
            numbered_lanes: Dict mapping lane_id to polynomial coefficients
            
        Returns:
            Binary mask
        """
        mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
        
        for lane_id, poly_coeffs in numbered_lanes.items():
            # Generate points from polynomial
            y_coords = np.arange(0, self.image_height, 2)  # Sample every 2 rows
            x_coords = np.polyval(poly_coeffs, y_coords).astype(int)
            
            # Draw lane line
            for i in range(len(y_coords) - 1):
                x1, y1 = x_coords[i], y_coords[i]
                x2, y2 = x_coords[i + 1], y_coords[i + 1]
                
                # Check bounds
                if 0 <= x1 < self.image_width and 0 <= y1 < self.image_height:
                    if 0 <= x2 < self.image_width and 0 <= y2 < self.image_height:
                        cv2.line(mask, (x1, y1), (x2, y2), 255, 3)
        
        return mask
    
    def reset_temporal_buffer(self):
        """Reset temporal smoothing buffers (call when scene changes)."""
        self.lane_history.clear()
        logger.debug("Temporal smoothing buffers reset")

