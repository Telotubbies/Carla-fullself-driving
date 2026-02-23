"""
Far-Field Accurate Lane Detection System

Production-grade lane detection pipeline with:
- IPM/BEV transformation
- Robust polynomial fitting (RANSAC + WLS)
- Kalman filtering for temporal smoothing
- Comprehensive confidence scoring
- Fail-safe validation
- Real-world metrics computation
- MPC-ready output

Author: Senior Autonomous Driving Engineer
"""

import numpy as np
import cv2
import logging
from typing import Tuple, Optional, List, Dict, Any
from pathlib import Path

from perception.ultra_fast_lane_detector import UltraFastLaneDetector
from perception.ipm_transformer import IPMTransformer, create_default_ipm_transformer
from perception.lane_fitter import RobustLaneFitter
from perception.kalman_lane_tracker import DualLaneTracker
from perception.lane_confidence import LaneConfidenceScorer
from perception.lane_validator import LaneValidator
from perception.lane_metrics import LaneMetricsCalculator

logger = logging.getLogger(__name__)


class FarFieldLaneDetector:
    """
    Production-grade far-field accurate lane detection system.
    
    Pipeline:
    1. Raw detection (Ultra-Fast-Lane-Detection-v2)
    2. IPM transformation (camera → BEV)
    3. Multi-zone detection
    4. Robust polynomial fitting (RANSAC + WLS)
    5. Kalman filtering (temporal smoothing)
    6. Confidence scoring
    7. Fail-safe validation
    8. Real-world metrics
    9. MPC-ready output
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        image_width: int = 640,
        image_height: int = 480,
        dataset: str = "tusimple",
        backbone: str = "18",
        enable_ipm: bool = True,
        enable_kalman: bool = True,
        enable_validation: bool = True
    ):
        """
        Initialize far-field lane detector.
        
        Args:
            model_path: Path to Ultra-Fast-Lane-Detection-v2 model
            image_width: Image width
            image_height: Image height
            dataset: Dataset name for model
            backbone: Backbone architecture
            enable_ipm: Enable IPM transformation
            enable_kalman: Enable Kalman filtering
            enable_validation: Enable fail-safe validation
        """
        self.image_width = image_width
        self.image_height = image_height
        self.enable_ipm = enable_ipm
        self.enable_kalman = enable_kalman
        self.enable_validation = enable_validation
        
        # Initialize raw detector
        self.raw_detector = UltraFastLaneDetector(
            model_path=model_path,
            dataset=dataset,
            backbone=backbone
        )
        
        # Initialize IPM transformer
        if enable_ipm:
            self.ipm_transformer = create_default_ipm_transformer(
                image_width=image_width,
                image_height=image_height
            )
        else:
            self.ipm_transformer = None
        
        # Initialize robust fitter
        self.lane_fitter = RobustLaneFitter(
            polynomial_order=2,
            ransac_threshold=0.5,
            ransac_max_iterations=100,
            distance_decay_sigma=25.0,
            use_ransac=True
        )
        
        # Initialize Kalman tracker
        if enable_kalman:
            self.kalman_tracker = DualLaneTracker(
                polynomial_order=2
            )
        else:
            self.kalman_tracker = None
        
        # Initialize confidence scorer
        self.confidence_scorer = LaneConfidenceScorer(
            coverage_weight=0.25,
            stability_weight=0.25,
            symmetry_weight=0.20,
            temporal_weight=0.15,
            distance_weight=0.15
        )
        
        # Initialize validator
        if enable_validation:
            self.validator = LaneValidator(
                max_curvature=0.1,
                max_lateral_shift=0.5,
                expected_lane_width=3.5
            )
        else:
            self.validator = None
        
        # Initialize metrics calculator
        self.metrics_calculator = LaneMetricsCalculator(
            waypoint_spacing=1.0,
            waypoint_range=(0.0, 100.0)
        )
        
        # Zone definitions (in BEV, meters)
        self.zones = {
            'near': (0.0, 20.0),
            'mid': (20.0, 50.0),
            'far': (50.0, 100.0)
        }
        
        logger.info(f"FarFieldLaneDetector initialized: IPM={enable_ipm}, Kalman={enable_kalman}, Validation={enable_validation}")
    
    def detect_lanes(
        self,
        image: np.ndarray
    ) -> Dict[str, Any]:
        """
        Detect lanes with full pipeline.
        
        Args:
            image: Input RGB image (H, W, 3) uint8
            
        Returns:
            Dict with detection results:
            - left_coefficients: Left lane polynomial [a, b, c]
            - right_coefficients: Right lane polynomial [a, b, c]
            - centerline_coefficients: Centerline polynomial [a, b, c]
            - left_confidence: Left lane confidence [0, 1]
            - right_confidence: Right lane confidence [0, 1]
            - centerline_confidence: Centerline confidence [0, 1]
            - waypoints: MPC waypoints (N, 2)
            - mpc_reference: MPC reference trajectory
            - metrics: Real-world metrics
            - validation_result: Validation results
        """
        # Step 1: Raw detection
        lane_mask, lane_features, lane_coords = self.raw_detector.detect_lanes(
            image, use_post_processing=False
        )
        
        # DEBUG: Log raw detection
        logger.debug(f"Raw detector: {len(lane_coords)} lanes detected")
        for i, lane in enumerate(lane_coords):
            if len(lane) > 0:
                x_coords = [p[0] for p in lane]
                y_coords = [p[1] for p in lane]
                logger.debug(f"  Lane {i}: {len(lane)} points, x_range=[{min(x_coords):.0f}, {max(x_coords):.0f}], y_range=[{min(y_coords):.0f}, {max(y_coords):.0f}]")
        
        if len(lane_coords) == 0:
            logger.warning("No lanes detected by raw detector")
            return self._create_empty_result()
        
        # Step 2: Transform to BEV and separate lanes intelligently
        # NOTE: If IPM is disabled, we work directly in camera space
        # This is useful when IPM transformation causes misalignment
        if self.enable_ipm and self.ipm_transformer is not None:
            # Convert lane coordinates to BEV
            all_bev_points = []
            lane_bev_mapping = []  # Track which BEV lane corresponds to which raw lane
            
            for lane_idx, lane in enumerate(lane_coords):
                if len(lane) > 0:
                    lane_points = np.array(lane, dtype=np.float32)
                    bev_points = self.ipm_transformer.transform_points_to_bev(lane_points)
                    # Filter points in BEV bounds
                    bev_points = self.ipm_transformer.filter_points_in_bev_bounds(bev_points)
                    if len(bev_points) > 0:
                        all_bev_points.append(bev_points)
                        lane_bev_mapping.append(lane_idx)
        else:
            # Use camera coordinates directly (raw detection - no IPM transformation)
            # Convert to numpy arrays for processing
            # Note: In camera space, y increases downward, x increases rightward
            # For lane separation, we'll use x-coordinate (left = smaller x, right = larger x)
            all_bev_points = []
            lane_bev_mapping = []
            
            for lane_idx, lane in enumerate(lane_coords):
                if len(lane) > 0:
                    # Convert to numpy array: [x, y] format
                    lane_points = np.array(lane, dtype=np.float32)
                    all_bev_points.append(lane_points)
                    lane_bev_mapping.append(lane_idx)
            
            logger.debug(f"IPM disabled: Using {len(all_bev_points)} lanes in camera space")
        
        if len(all_bev_points) == 0:
            logger.warning("No valid BEV points after transformation")
            return self._create_empty_result()
        
        # Step 3: Separate left and right lanes intelligently
        # Strategy: Select the leftmost and rightmost lanes as boundaries
        # This ensures we get actual road boundaries, not center lines
        
        # Get image center for reference (when IPM disabled)
        image_center_x = self.image_width / 2.0
        
        # Calculate mean x for each lane
        lane_mean_x = []
        for bev_lane in all_bev_points:
            if len(bev_lane) > 0:
                mean_x = np.mean(bev_lane[:, 0])
                lane_mean_x.append(mean_x)
            else:
                lane_mean_x.append(float('inf'))  # Skip empty lanes
        
        if len(lane_mean_x) == 0:
            logger.warning("No valid lanes for separation")
            return self._create_empty_result()
        
        # Sort lanes by mean x-coordinate
        sorted_indices = np.argsort(lane_mean_x)
        
        # Strategy: Select leftmost and rightmost lanes as boundaries
        # This works better than simple left/right split because:
        # - Model may detect center lines, multiple lane markings, etc.
        # - We want the actual road boundaries (leftmost and rightmost)
        
        left_points_list = []
        right_points_list = []
        
        if self.enable_ipm and self.ipm_transformer is not None:
            # BEV space: leftmost = smallest x, rightmost = largest x
            if len(sorted_indices) >= 2:
                # Left boundary: leftmost lane
                left_idx = sorted_indices[0]
                left_points_list.append(all_bev_points[left_idx])
                logger.debug(f"Left boundary: lane {left_idx}, mean_x={lane_mean_x[left_idx]:.2f}")
                
                # Right boundary: rightmost lane
                right_idx = sorted_indices[-1]
                right_points_list.append(all_bev_points[right_idx])
                logger.debug(f"Right boundary: lane {right_idx}, mean_x={lane_mean_x[right_idx]:.2f}")
            elif len(sorted_indices) == 1:
                # Only one lane detected - assign based on position
                idx = sorted_indices[0]
                if lane_mean_x[idx] < 0:
                    left_points_list.append(all_bev_points[idx])
                    logger.debug(f"Single lane assigned LEFT: mean_x={lane_mean_x[idx]:.2f}")
                else:
                    right_points_list.append(all_bev_points[idx])
                    logger.debug(f"Single lane assigned RIGHT: mean_x={lane_mean_x[idx]:.2f}")
        else:
            # Camera space: leftmost = smallest x, rightmost = largest x
            # IMPORTANT: TuSimple model detects lane markings (อาจเป็น center lines หรือ lane markings)
            # เราต้องการ road boundaries (ขอบถนน) ไม่ใช่ center lines
            # Strategy: เลือก lanes ที่อยู่ใกล้ขอบภาพมากที่สุด (extreme positions)
            
            if len(sorted_indices) >= 2:
                # Filter: ต้องมี lanes ที่อยู่ใกล้ขอบภาพจริงๆ (ไม่ใช่ center lines)
                # Left boundary: เลือก lane ที่อยู่ซ้ายสุด และอยู่ใกล้ขอบซ้ายของภาพ
                # Right boundary: เลือก lane ที่อยู่ขวาสุด และอยู่ใกล้ขอบขวาของภาพ
                
                # Calculate distance from image edges
                left_candidates = []
                right_candidates = []
                
                for idx in sorted_indices:
                    mean_x = lane_mean_x[idx]
                    # Distance from left edge
                    dist_from_left = mean_x
                    # Distance from right edge
                    dist_from_right = self.image_width - mean_x
                    
                    # Left candidates: lanes on left side, prefer those closer to left edge
                    if mean_x < image_center_x:
                        left_candidates.append((idx, dist_from_left, mean_x))
                    # Right candidates: lanes on right side, prefer those closer to right edge
                    if mean_x > image_center_x:
                        right_candidates.append((idx, dist_from_right, mean_x))
                
                # Select leftmost boundary (closest to left edge)
                if left_candidates:
                    left_candidates.sort(key=lambda x: x[1])  # Sort by distance from left edge
                    left_idx = left_candidates[0][0]
                    left_points_list.append(all_bev_points[left_idx])
                    logger.debug(f"Left boundary: lane {left_idx}, mean_x={lane_mean_x[left_idx]:.1f}, dist_from_left={left_candidates[0][1]:.1f}")
                else:
                    # Fallback: use leftmost lane
                    left_idx = sorted_indices[0]
                    left_points_list.append(all_bev_points[left_idx])
                    logger.debug(f"Left boundary (fallback): lane {left_idx}, mean_x={lane_mean_x[left_idx]:.1f} (leftmost)")
                
                # Select rightmost boundary (closest to right edge)
                if right_candidates:
                    right_candidates.sort(key=lambda x: x[1])  # Sort by distance from right edge
                    right_idx = right_candidates[0][0]
                    right_points_list.append(all_bev_points[right_idx])
                    logger.debug(f"Right boundary: lane {right_idx}, mean_x={lane_mean_x[right_idx]:.1f}, dist_from_right={right_candidates[0][1]:.1f}")
                else:
                    # Fallback: use rightmost lane
                    right_idx = sorted_indices[-1]
                    right_points_list.append(all_bev_points[right_idx])
                    logger.debug(f"Right boundary (fallback): lane {right_idx}, mean_x={lane_mean_x[right_idx]:.1f} (rightmost)")
            elif len(sorted_indices) == 1:
                # Only one lane detected - assign based on position relative to center
                idx = sorted_indices[0]
                if lane_mean_x[idx] < image_center_x:
                    left_points_list.append(all_bev_points[idx])
                    logger.debug(f"Single lane assigned LEFT: mean_x={lane_mean_x[idx]:.1f} < center={image_center_x:.1f}")
                else:
                    right_points_list.append(all_bev_points[idx])
                    logger.debug(f"Single lane assigned RIGHT: mean_x={lane_mean_x[idx]:.1f} >= center={image_center_x:.1f}")
            
            # Log all detected lanes for debugging
            logger.debug(f"All detected lanes (sorted by x):")
            for i, idx in enumerate(sorted_indices):
                logger.debug(f"  Lane {idx}: mean_x={lane_mean_x[idx]:.1f}, points={len(all_bev_points[idx])}")
        
        # Combine points and convert back to camera coordinates for visualization
        left_points = None
        right_points = None
        left_lane_coords = []
        right_lane_coords = []
        
        if left_points_list:
            left_points = np.vstack(left_points_list)
            # Convert BEV points back to camera coordinates for visualization
            if self.enable_ipm and self.ipm_transformer is not None:
                left_lane_coords = self.ipm_transformer.transform_points_to_camera(left_points).tolist()
            else:
                # Already in camera space
                left_lane_coords = left_points.tolist()
        else:
            left_points = np.array([]).reshape(0, 2)
        
        if right_points_list:
            right_points = np.vstack(right_points_list)
            # Convert BEV points back to camera coordinates for visualization
            if self.enable_ipm and self.ipm_transformer is not None:
                right_lane_coords = self.ipm_transformer.transform_points_to_camera(right_points).tolist()
            else:
                # Already in camera space
                right_lane_coords = right_points.tolist()
        else:
            right_points = np.array([]).reshape(0, 2)
        
        # Fallback: use clustering if separation failed
        if len(left_points) == 0 and len(right_points) == 0:
            logger.warning("Lane separation failed, using clustering fallback")
            left_points, right_points = self._separate_left_right(all_bev_points)
        
        # Step 4: Robust polynomial fitting
        left_coeffs, right_coeffs = None, None
        left_inliers, right_inliers = None, None
        
        if len(left_points) >= 3:
            # Compute distance weights
            left_weights = self.lane_fitter.compute_distance_weights(left_points)
            # Fit polynomial
            left_coeffs, left_inliers, left_mse = self.lane_fitter.fit_polynomial(
                left_points, left_weights
            )
        
        if len(right_points) >= 3:
            # Compute distance weights
            right_weights = self.lane_fitter.compute_distance_weights(right_points)
            # Fit polynomial
            right_coeffs, right_inliers, right_mse = self.lane_fitter.fit_polynomial(
                right_points, right_weights
            )
        
        # Step 5: Kalman filtering (temporal smoothing)
        if self.enable_kalman and self.kalman_tracker is not None:
            # Predict
            self.kalman_tracker.predict()
            
            # Update with measurements
            left_coeffs, right_coeffs = self.kalman_tracker.update(
                left_coeffs, right_coeffs
            )
        
        # Step 6: Compute centerline
        centerline_coeffs = None
        if left_coeffs is not None and right_coeffs is not None:
            centerline_coeffs = self.metrics_calculator.compute_centerline(
                left_coeffs, right_coeffs
            )
        elif left_coeffs is not None:
            # Estimate right from left
            if self.validator is not None:
                right_coeffs = self.validator.estimate_missing_boundary(
                    left_coeffs, is_left_detected=True
                )
                centerline_coeffs = self.metrics_calculator.compute_centerline(
                    left_coeffs, right_coeffs
                )
        elif right_coeffs is not None:
            # Estimate left from right
            if self.validator is not None:
                left_coeffs = self.validator.estimate_missing_boundary(
                    right_coeffs, is_left_detected=False
                )
                centerline_coeffs = self.metrics_calculator.compute_centerline(
                    left_coeffs, right_coeffs
                )
        
        # Step 7: Confidence scoring
        left_confidence = 0.0
        right_confidence = 0.0
        centerline_confidence = 0.0
        
        if left_coeffs is not None:
            left_points_for_conf = left_points[left_inliers] if left_inliers is not None else left_points
            left_confidence, _ = self.confidence_scorer.compute_comprehensive_confidence(
                num_valid_points=len(left_points_for_conf),
                coefficients=left_coeffs,
                points=left_points_for_conf,
                left_coefficients=left_coeffs,
                right_coefficients=right_coeffs
            )
        
        if right_coeffs is not None:
            right_points_for_conf = right_points[right_inliers] if right_inliers is not None else right_points
            right_confidence, _ = self.confidence_scorer.compute_comprehensive_confidence(
                num_valid_points=len(right_points_for_conf),
                coefficients=right_coeffs,
                points=right_points_for_conf,
                left_coefficients=left_coeffs,
                right_coefficients=right_coeffs
            )
        
        if centerline_coeffs is not None:
            centerline_confidence = (left_confidence + right_confidence) / 2.0
        
        # Step 8: Fail-safe validation
        validation_result = None
        if self.enable_validation and self.validator is not None:
            validation_result = self.validator.apply_fail_safe(
                current_coefficients=centerline_coeffs,
                confidence=centerline_confidence,
                left_coefficients=left_coeffs,
                right_coefficients=right_coeffs
            )
            
            # Apply validation corrections
            if not validation_result['valid']:
                logger.warning(f"Validation failed: {validation_result['warnings']}")
                if 'final_coefficients' in validation_result:
                    centerline_coeffs = validation_result['final_coefficients']
                centerline_confidence = validation_result.get('confidence_adjusted', centerline_confidence)
        
        # Step 9: Real-world metrics
        metrics = {}
        if centerline_coeffs is not None:
            metrics = self.metrics_calculator.compute_lane_statistics(
                left_coefficients=left_coeffs,
                right_coefficients=right_coeffs,
                centerline_coefficients=centerline_coeffs
            )
        
        # Step 10: MPC-ready output
        waypoints = None
        mpc_reference = None
        
        if centerline_coeffs is not None:
            waypoints = self.metrics_calculator.generate_waypoints(centerline_coeffs)
            mpc_reference = self.metrics_calculator.generate_mpc_reference(centerline_coeffs)
        
        # Compile results
        result = {
            'left_coefficients': left_coeffs,
            'right_coefficients': right_coeffs,
            'centerline_coefficients': centerline_coeffs,
            'left_confidence': left_confidence,
            'right_confidence': right_confidence,
            'centerline_confidence': centerline_confidence,
            'waypoints': waypoints,
            'mpc_reference': mpc_reference,
            'metrics': metrics,
            'validation_result': validation_result,
            'raw_lane_coords': lane_coords,
            'bev_points': all_bev_points,
            'left_lane_coords': left_lane_coords,  # Camera coordinates for visualization
            'right_lane_coords': right_lane_coords  # Camera coordinates for visualization
        }
        
        return result
    
    def _separate_left_right(
        self,
        all_bev_points: List[np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Separate lane points into left and right using intelligent clustering.
        
        Strategy:
        1. Use DBSCAN to cluster lanes
        2. Separate by x-coordinate (left = negative, right = positive)
        3. Handle multiple lanes per side
        
        Args:
            all_bev_points: List of BEV point arrays
            
        Returns:
            Tuple of (left_points, right_points)
        """
        if len(all_bev_points) == 0:
            return np.array([]).reshape(0, 2), np.array([]).reshape(0, 2)
        
        # Combine all points
        combined_points = np.vstack(all_bev_points)
        
        if len(combined_points) == 0:
            return np.array([]).reshape(0, 2), np.array([]).reshape(0, 2)
        
        # Use DBSCAN clustering to separate lanes
        try:
            from sklearn.cluster import DBSCAN
            
            # Cluster parameters
            eps = 1.0  # Maximum distance between points in same cluster (meters)
            min_samples = 5  # Minimum points per cluster
            
            clustering = DBSCAN(eps=eps, min_samples=min_samples, metric='euclidean')
            labels = clustering.fit_predict(combined_points)
            
            # Separate clusters into left and right
            left_points_list = []
            right_points_list = []
            
            unique_labels = np.unique(labels)
            for label in unique_labels:
                if label == -1:  # Noise points
                    continue
                
                cluster_points = combined_points[labels == label]
                
                # Determine if left or right based on x-coordinate
                # In BEV: x < 0 = left, x > 0 = right
                mean_x = np.mean(cluster_points[:, 0])
                
                if mean_x < 0:
                    left_points_list.append(cluster_points)
                else:
                    right_points_list.append(cluster_points)
            
            # Combine left and right points
            if left_points_list:
                left_points = np.vstack(left_points_list)
            else:
                left_points = np.array([]).reshape(0, 2)
            
            if right_points_list:
                right_points = np.vstack(right_points_list)
            else:
                right_points = np.array([]).reshape(0, 2)
            
            logger.debug(f"Separated lanes: left={len(left_points)} points, right={len(right_points)} points")
            
        except ImportError:
            # Fallback: simple median split
            logger.warning("sklearn not available, using simple median split")
            sorted_indices = np.argsort(combined_points[:, 0])
            sorted_points = combined_points[sorted_indices]
            median_idx = len(sorted_points) // 2
            left_points = sorted_points[:median_idx]
            right_points = sorted_points[median_idx:]
        
        return left_points, right_points
    
    def _create_empty_result(self) -> Dict[str, Any]:
        """Create empty result dict."""
        return {
            'left_coefficients': None,
            'right_coefficients': None,
            'centerline_coefficients': None,
            'left_confidence': 0.0,
            'right_confidence': 0.0,
            'centerline_confidence': 0.0,
            'waypoints': None,
            'mpc_reference': None,
            'metrics': {},
            'validation_result': None,
            'raw_lane_coords': [],
            'bev_points': []
        }
    
    def reset(self):
        """Reset temporal buffers."""
        if self.kalman_tracker is not None:
            self.kalman_tracker.reset()
        if self.confidence_scorer is not None:
            self.confidence_scorer.reset_history()
        if self.validator is not None:
            self.validator.reset()

