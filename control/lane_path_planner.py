"""
Lane-based path planner for MPC reference trajectory generation.

Uses lane detection to generate smooth reference trajectories.
"""

import numpy as np
import logging
from typing import Dict, Any, Optional, Tuple, List
import cv2

logger = logging.getLogger(__name__)


class LanePathPlanner:
    """Lane-based path planner for generating reference trajectories."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize lane path planner.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.mpc_config = config.get('mpc', {})
        self.horizon = self.mpc_config.get('horizon', 10)
        self.dt = self.mpc_config.get('dt', 0.05)
        
        # Lane following parameters
        self.lookahead_distance = 20.0  # meters
        self.lane_width = 3.5  # meters (typical lane width)
        self.lateral_offset = 0.0  # meters (0 = center, positive = right)
        
        # Smoothing parameters
        self.smoothing_factor = 0.3
        
        # Previous trajectory for smoothing
        self.prev_trajectory: Optional[np.ndarray] = None
        
        logger.info("✅ LanePathPlanner initialized")
    
    def generate_reference_from_lanes(
        self,
        current_state: np.ndarray,
        lane_info: Optional[Dict[str, Any]] = None,
        waypoints: Optional[List] = None
    ) -> np.ndarray:
        """
        Generate reference trajectory from lane information.
        
        Args:
            current_state: Current state [x, y, yaw, v]
            lane_info: Lane information dict with 'left_lane', 'right_lane', 'center_line'
            waypoints: CARLA waypoints (optional)
            
        Returns:
            Reference trajectory (N+1, 4): [x, y, yaw, v]
        """
        x, y, yaw, v = current_state
        
        # If waypoints available, use them
        if waypoints is not None and len(waypoints) > 0:
            return self._generate_from_waypoints(current_state, waypoints)
        
        # If lane info available, use it
        if lane_info is not None:
            return self._generate_from_lane_info(current_state, lane_info)
        
        # Fallback: straight ahead
        return self._generate_straight_ahead(current_state)
    
    def _generate_from_waypoints(
        self,
        current_state: np.ndarray,
        waypoints: List
    ) -> np.ndarray:
        """Generate trajectory from CARLA waypoints."""
        x, y, yaw, v = current_state
        
        # Target speed
        target_v = min(8.0, max(v, 5.0)) if v > 0.5 else 5.0
        
        # Generate trajectory from waypoints
        ref_traj = np.zeros((self.horizon + 1, 4))
        
        # Find closest waypoint
        closest_wp = None
        min_dist = float('inf')
        for wp in waypoints[:50]:  # Check first 50 waypoints
            wp_loc = wp.transform.location
            dist = np.sqrt((wp_loc.x - x)**2 + (wp_loc.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_wp = wp
        
        if closest_wp is None:
            return self._generate_straight_ahead(current_state)
        
        # Follow waypoints
        current_wp = closest_wp
        for i in range(self.horizon + 1):
            if current_wp is None:
                # Extrapolate from last waypoint
                if i > 0:
                    prev_state = ref_traj[i-1]
                    ref_traj[i, 0] = prev_state[0] + target_v * np.cos(prev_state[2]) * self.dt
                    ref_traj[i, 1] = prev_state[1] + target_v * np.sin(prev_state[2]) * self.dt
                    ref_traj[i, 2] = prev_state[2]
                    ref_traj[i, 3] = target_v
                else:
                    ref_traj[i] = current_state
                continue
            
            wp_loc = current_wp.transform.location
            wp_rot = current_wp.transform.rotation
            
            ref_traj[i, 0] = wp_loc.x
            ref_traj[i, 1] = wp_loc.y
            ref_traj[i, 2] = np.deg2rad(wp_rot.yaw)
            ref_traj[i, 3] = target_v
            
            # Get next waypoint
            next_wps = current_wp.next(2.0)  # 2 meters ahead
            if next_wps and len(next_wps) > 0:
                current_wp = next_wps[0]
            else:
                current_wp = None
        
        # Smooth trajectory
        ref_traj = self._smooth_trajectory(ref_traj)
        
        return ref_traj
    
    def _generate_from_lane_info(
        self,
        current_state: np.ndarray,
        lane_info: Dict[str, Any]
    ) -> np.ndarray:
        """Generate trajectory from lane detection info."""
        x, y, yaw, v = current_state
        
        target_v = min(8.0, max(v, 5.0)) if v > 0.5 else 5.0
        
        # Extract lane center from lane info
        center_line = lane_info.get('center_line')
        left_lane = lane_info.get('left_lane')
        right_lane = lane_info.get('right_lane')
        
        ref_traj = np.zeros((self.horizon + 1, 4))
        
        if center_line is not None and len(center_line) > 0:
            # Use center line
            for i in range(self.horizon + 1):
                t = i * self.dt
                lookahead_dist = target_v * t
                
                # Find point on center line at lookahead distance
                if lookahead_dist < len(center_line):
                    idx = int(lookahead_dist * 10)  # Assuming 0.1m spacing
                    idx = min(idx, len(center_line) - 1)
                    point = center_line[idx]
                    
                    ref_traj[i, 0] = point[0]
                    ref_traj[i, 1] = point[1]
                    ref_traj[i, 2] = yaw  # Use current yaw for now
                    ref_traj[i, 3] = target_v
                else:
                    # Extrapolate
                    if i > 0:
                        prev_state = ref_traj[i-1]
                        ref_traj[i, 0] = prev_state[0] + target_v * np.cos(prev_state[2]) * self.dt
                        ref_traj[i, 1] = prev_state[1] + target_v * np.sin(prev_state[2]) * self.dt
                        ref_traj[i, 2] = prev_state[2]
                        ref_traj[i, 3] = target_v
                    else:
                        ref_traj[i] = current_state
        else:
            # Fallback: use left/right lanes to estimate center
            return self._generate_straight_ahead(current_state)
        
        # Smooth trajectory
        ref_traj = self._smooth_trajectory(ref_traj)
        
        return ref_traj
    
    def _generate_straight_ahead(self, current_state: np.ndarray) -> np.ndarray:
        """Generate straight-ahead trajectory."""
        x, y, yaw, v = current_state
        
        target_v = min(8.0, max(v, 5.0)) if v > 0.5 else 5.0
        
        ref_traj = np.zeros((self.horizon + 1, 4))
        for i in range(self.horizon + 1):
            t = i * self.dt
            ref_traj[i, 0] = x + target_v * np.cos(yaw) * t
            ref_traj[i, 1] = y + target_v * np.sin(yaw) * t
            ref_traj[i, 2] = yaw
            ref_traj[i, 3] = min(target_v, v + (target_v - v) * (i / (self.horizon + 1)))
        
        return ref_traj
    
    def _smooth_trajectory(self, trajectory: np.ndarray) -> np.ndarray:
        """Smooth trajectory using moving average."""
        if self.prev_trajectory is None:
            self.prev_trajectory = trajectory.copy()
            return trajectory
        
        # Smooth with previous trajectory
        smoothed = (1 - self.smoothing_factor) * trajectory + self.smoothing_factor * self.prev_trajectory
        
        self.prev_trajectory = smoothed.copy()
        return smoothed
    
    def extract_lane_center_from_image(
        self,
        lane_mask: np.ndarray,
        camera_image: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        Extract lane center line from lane mask.
        
        Args:
            lane_mask: Binary lane mask (H, W)
            camera_image: Camera image for reference
            
        Returns:
            Lane center points in image coordinates (N, 2) or None
        """
        if lane_mask is None or lane_mask.size == 0:
            return None
        
        h, w = lane_mask.shape
        
        # Find lane pixels
        lane_pixels = np.where(lane_mask > 0)
        
        if len(lane_pixels[0]) == 0:
            return None
        
        # Extract center line (simplified: use middle row)
        center_points = []
        for y in range(h // 2, h, 10):  # Sample every 10 pixels
            row_pixels = lane_pixels[1][lane_pixels[0] == y]
            if len(row_pixels) > 0:
                center_x = np.mean(row_pixels)
                center_points.append([center_x, y])
        
        if len(center_points) == 0:
            return None
        
        return np.array(center_points)
    
    def project_lane_to_world(
        self,
        lane_points_image: np.ndarray,
        vehicle_transform,
        camera_transform,
        image_width: int = 640,
        image_height: int = 480,
        fov: float = 90.0
    ) -> Optional[np.ndarray]:
        """
        Project lane points from image to world coordinates.
        
        Args:
            lane_points_image: Lane points in image (N, 2)
            vehicle_transform: Vehicle transform
            camera_transform: Camera transform
            image_width: Image width
            image_height: Image height
            fov: Camera FOV
            
        Returns:
            Lane points in world coordinates (N, 2) or None
        """
        if lane_points_image is None or len(lane_points_image) == 0:
            return None
        
        # Camera intrinsic matrix
        f = image_width / (2.0 * np.tan(np.radians(fov / 2.0)))
        K = np.array([
            [f, 0, image_width / 2.0],
            [0, f, image_height / 2.0],
            [0, 0, 1]
        ])
        
        # Camera world transform
        camera_world_transform = vehicle_transform * camera_transform
        
        # Project points (simplified: assume ground plane)
        world_points = []
        for point_img in lane_points_image:
            u, v = point_img
            
            # Back-project to 3D (simplified: assume fixed height)
            z = 1.0  # Assume 1m ahead
            x_img = (u - image_width / 2.0) * z / f
            y_img = (v - image_height / 2.0) * z / f
            
            # Transform to world (simplified)
            # In real implementation, would use proper camera projection
            world_points.append([x_img, y_img])
        
        return np.array(world_points) if world_points else None

