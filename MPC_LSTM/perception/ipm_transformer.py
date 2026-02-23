"""
Inverse Perspective Mapping (IPM) / Bird's Eye View (BEV) Transformer

Transforms lane points from camera view to BEV space for accurate far-field detection.

Mathematical Foundation:
- Homography: [x', y', 1]ᵀ = H · [x, y, 1]ᵀ
- Direct Linear Transform (DLT) for homography estimation
"""

import numpy as np
import cv2
from typing import Tuple, List, Optional
import logging

logger = logging.getLogger(__name__)


class IPMTransformer:
    """
    Transforms points from camera view to Bird's Eye View (BEV) using homography.
    
    This enables:
    - Uniform scale (1 meter = constant pixel distance)
    - Stable polynomial fitting for far lanes
    - Direct real-world metric computation
    """
    
    def __init__(
        self,
        image_width: int = 640,
        image_height: int = 480,
        camera_fov: float = 90.0,
        camera_height: float = 1.4,
        camera_pitch: float = 0.0,
        bev_x_range: Tuple[float, float] = (-3.0, 3.0),
        bev_y_range: Tuple[float, float] = (0.0, 100.0),
        roi_top_ratio: float = 0.4,
        roi_bottom_ratio: float = 1.0,
        roi_left_ratio: float = 0.1,
        roi_right_ratio: float = 0.9
    ):
        """
        Initialize IPM transformer.
        
        Args:
            image_width: Camera image width (pixels)
            image_height: Camera image height (pixels)
            camera_fov: Camera field of view (degrees)
            camera_height: Camera height above ground (meters)
            camera_pitch: Camera pitch angle (radians, positive = down)
            bev_x_range: BEV lateral range (meters) - (min, max)
            bev_y_range: BEV forward range (meters) - (min, max)
            roi_top_ratio: ROI top boundary (ratio of image height)
            roi_bottom_ratio: ROI bottom boundary (ratio of image height)
            roi_left_ratio: ROI left boundary (ratio of image width)
            roi_right_ratio: ROI right boundary (ratio of image width)
        """
        self.image_width = image_width
        self.image_height = image_height
        self.camera_fov = camera_fov
        self.camera_height = camera_height
        self.camera_pitch = camera_pitch
        self.bev_x_range = bev_x_range
        self.bev_y_range = bev_y_range
        
        # ROI in camera view
        self.roi_top = int(roi_top_ratio * image_height)
        self.roi_bottom = int(roi_bottom_ratio * image_height)
        self.roi_left = int(roi_left_ratio * image_width)
        self.roi_right = int(roi_right_ratio * image_width)
        
        # Compute homography matrix
        self.H, self.H_inv = self._compute_homography()
        
        logger.info(f"IPMTransformer initialized: BEV range x={bev_x_range}, y={bev_y_range}")
    
    def _compute_homography(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute homography matrix from camera view to BEV.
        
        Uses 4-point correspondence method.
        
        Returns:
            Tuple of (H, H_inv) - forward and inverse homography matrices
        """
        # Source points (camera view - ROI corners)
        src_points = np.array([
            [self.roi_left, self.roi_top],      # Top-left
            [self.roi_right, self.roi_top],     # Top-right
            [self.roi_right, self.roi_bottom],  # Bottom-right
            [self.roi_left, self.roi_bottom]    # Bottom-left
        ], dtype=np.float32)
        
        # Destination points (BEV - meters)
        # Top edge = far distance, Bottom edge = near distance
        dst_points = np.array([
            [self.bev_x_range[0], self.bev_y_range[1]],  # Far-left
            [self.bev_x_range[1], self.bev_y_range[1]],  # Far-right
            [self.bev_x_range[1], self.bev_y_range[0]],  # Near-right
            [self.bev_x_range[0], self.bev_y_range[0]]   # Near-left
        ], dtype=np.float32)
        
        # Compute homography using OpenCV
        H = cv2.getPerspectiveTransform(src_points, dst_points)
        H_inv = np.linalg.inv(H)
        
        logger.debug(f"Homography matrix computed: H shape={H.shape}")
        
        return H, H_inv
    
    def transform_points_to_bev(
        self,
        points: np.ndarray
    ) -> np.ndarray:
        """
        Transform points from camera view to BEV.
        
        Args:
            points: Array of shape (N, 2) with pixel coordinates [x, y]
            
        Returns:
            Array of shape (N, 2) with BEV coordinates [x', y'] in meters
        """
        if len(points) == 0:
            return np.array([], dtype=np.float32).reshape(0, 2)
        
        # Convert to homogeneous coordinates
        points_homogeneous = np.column_stack([points, np.ones(len(points))])
        
        # Transform: [x', y', w']ᵀ = H · [x, y, 1]ᵀ
        transformed = (self.H @ points_homogeneous.T).T
        
        # Normalize homogeneous coordinates
        transformed = transformed / transformed[:, 2:3]
        
        # Extract x, y coordinates (drop w)
        bev_points = transformed[:, :2]
        
        return bev_points
    
    def transform_points_to_camera(
        self,
        bev_points: np.ndarray
    ) -> np.ndarray:
        """
        Transform points from BEV to camera view (inverse transformation).
        
        Args:
            bev_points: Array of shape (N, 2) with BEV coordinates [x', y'] in meters
            
        Returns:
            Array of shape (N, 2) with pixel coordinates [x, y]
        """
        if len(bev_points) == 0:
            return np.array([], dtype=np.float32).reshape(0, 2)
        
        # Convert to homogeneous coordinates
        points_homogeneous = np.column_stack([bev_points, np.ones(len(bev_points))])
        
        # Transform: [x, y, w]ᵀ = H_inv · [x', y', 1]ᵀ
        transformed = (self.H_inv @ points_homogeneous.T).T
        
        # Normalize homogeneous coordinates
        transformed = transformed / transformed[:, 2:3]
        
        # Extract x, y coordinates (drop w)
        camera_points = transformed[:, :2]
        
        return camera_points
    
    def transform_polynomial_to_bev(
        self,
        polynomial_camera: np.ndarray,
        y_range: Tuple[float, float] = (0.0, 100.0),
        num_samples: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Transform polynomial from camera view to BEV by sampling and transforming points.
        
        Args:
            polynomial_camera: Polynomial coefficients [a, b, c] for x = a·y² + b·y + c in camera space
            y_range: Y range in camera space (pixels)
            num_samples: Number of sample points
            
        Returns:
            Tuple of (bev_points, polynomial_bev)
            - bev_points: (N, 2) array of BEV coordinates
            - polynomial_bev: Polynomial coefficients in BEV space
        """
        # Sample points in camera space
        y_camera = np.linspace(y_range[0], y_range[1], num_samples)
        x_camera = np.polyval(polynomial_camera, y_camera)
        camera_points = np.column_stack([x_camera, y_camera])
        
        # Transform to BEV
        bev_points = self.transform_points_to_bev(camera_points)
        
        # Fit polynomial in BEV space
        if len(bev_points) >= 3:
            # Fit x = f(y) in BEV
            y_bev = bev_points[:, 1]  # Forward distance
            x_bev = bev_points[:, 0]  # Lateral position
            
            # Fit polynomial: x = a·y² + b·y + c
            polynomial_bev = np.polyfit(y_bev, x_bev, 2)
        else:
            polynomial_bev = None
        
        return bev_points, polynomial_bev
    
    def get_bev_roi_mask(self) -> np.ndarray:
        """
        Get mask for valid BEV region.
        
        Returns:
            Binary mask (same size as input image) indicating valid ROI
        """
        mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
        mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 255
        return mask
    
    def is_point_in_bev_bounds(self, bev_point: np.ndarray) -> bool:
        """
        Check if BEV point is within valid bounds.
        
        Args:
            bev_point: BEV point [x, y] in meters
            
        Returns:
            True if point is within bounds
        """
        x, y = bev_point
        return (self.bev_x_range[0] <= x <= self.bev_x_range[1] and
                self.bev_y_range[0] <= y <= self.bev_y_range[1])
    
    def filter_points_in_bev_bounds(self, bev_points: np.ndarray) -> np.ndarray:
        """
        Filter BEV points to keep only those within valid bounds.
        
        Args:
            bev_points: Array of shape (N, 2) with BEV coordinates
            
        Returns:
            Filtered array of BEV points
        """
        if len(bev_points) == 0:
            return bev_points
        
        mask = np.array([
            self.is_point_in_bev_bounds(pt) for pt in bev_points
        ])
        
        return bev_points[mask]


def create_default_ipm_transformer(
    image_width: int = 640,
    image_height: int = 480
) -> IPMTransformer:
    """
    Create default IPM transformer with standard CARLA parameters.
    
    Args:
        image_width: Image width
        image_height: Image height
        
    Returns:
        Configured IPMTransformer instance
    """
    return IPMTransformer(
        image_width=image_width,
        image_height=image_height,
        camera_fov=90.0,
        camera_height=1.4,
        camera_pitch=0.0,
        bev_x_range=(-3.0, 3.0),
        bev_y_range=(0.0, 100.0),
        roi_top_ratio=0.4,
        roi_bottom_ratio=1.0,
        roi_left_ratio=0.1,
        roi_right_ratio=0.9
    )

