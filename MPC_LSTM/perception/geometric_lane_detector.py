"""
Geometric Lane Detection using CARLA waypoints and camera projection.

No ML model needed - uses CARLA map API to get road topology,
projects 3D lane boundaries to 2D camera image using pinhole camera model.
"""

import numpy as np
import cv2
import logging
from typing import Tuple, Optional, List, Dict, Any

logger = logging.getLogger(__name__)


class GeometricLaneDetector:
    """
    Detect lanes by projecting CARLA road boundaries to camera image.

    Pipeline:
    1. Get current waypoint from CARLA map
    2. Walk forward along the road to collect waypoints
    3. Compute left/right lane boundaries from waypoint + lane_width
    4. Project 3D boundaries to 2D image using camera intrinsics/extrinsics
    5. Draw lane mask and extract features
    """

    LOOKAHEAD_DISTANCE = 60.0
    WAYPOINT_SPACING = 1.0
    LANE_LINE_THICKNESS = 3

    def __init__(self, camera_config: Dict[str, Any]):
        """
        Initialize with camera parameters from config.yaml.

        Args:
            camera_config: dict with width, height, fov, location, rotation
        """
        self.img_w = camera_config.get('width', 640)
        self.img_h = camera_config.get('height', 480)
        self.fov = camera_config.get('fov', 90)

        cam_loc = camera_config.get('location', {})
        self.cam_x = cam_loc.get('x', 2.0)
        self.cam_y = cam_loc.get('y', 0.0)
        self.cam_z = cam_loc.get('z', 1.4)

        cam_rot = camera_config.get('rotation', {})
        self.cam_pitch = cam_rot.get('pitch', 0.0)
        self.cam_yaw = cam_rot.get('yaw', 0.0)
        self.cam_roll = cam_rot.get('roll', 0.0)

        f = self.img_w / (2.0 * np.tan(np.radians(self.fov / 2.0)))
        self.K = np.array([
            [f,   0.0, self.img_w / 2.0],
            [0.0, f,   self.img_h / 2.0],
            [0.0, 0.0, 1.0]
        ])

        logger.info(
            f"GeometricLaneDetector initialized "
            f"(img={self.img_w}x{self.img_h}, fov={self.fov})"
        )

    def detect(
        self, world, vehicle, image: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, List[List[Tuple[int, int]]]]:
        """
        Detect lanes using CARLA map geometry.

        Returns:
            lane_mask (H,W) uint8, features (128,) float32, lane_coords list
        """
        h = self.img_h if image is None else image.shape[0]
        w = self.img_w if image is None else image.shape[1]
        lane_mask = np.zeros((h, w), dtype=np.uint8)
        empty_feat = np.zeros(128, dtype=np.float32)

        if world is None or vehicle is None:
            return lane_mask, empty_feat, []

        try:
            import carla
            carla_map = world.get_map()
            vehicle_tf = vehicle.get_transform()
            current_wp = carla_map.get_waypoint(
                vehicle_tf.location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
            if current_wp is None:
                return lane_mask, empty_feat, []

            forward_wps = self._collect_forward_waypoints(current_wp)

            world2cam = self._compute_world2cam(vehicle_tf)

            left_3d, right_3d, center_3d = self._compute_lane_boundaries(forward_wps)

            left_2d = self._project_points(left_3d, world2cam, w, h)
            right_2d = self._project_points(right_3d, world2cam, w, h)
            center_2d = self._project_points(center_3d, world2cam, w, h)

            all_coords = []
            for pts in (left_2d, right_2d):
                if len(pts) >= 2:
                    self._draw_polyline(lane_mask, pts, 255)
                    all_coords.append(pts)

            if len(left_2d) >= 2 and len(right_2d) >= 2:
                self._fill_lane_area(lane_mask, left_2d, right_2d)

            adj = self._detect_adjacent_lanes(current_wp, world2cam, w, h, lane_mask)
            all_coords.extend(adj)

            features = self._extract_features(lane_mask, center_2d, left_2d, right_2d)
            return lane_mask, features, all_coords

        except Exception as e:
            logger.warning(f"Geometric lane detection failed: {e}")
            return lane_mask, empty_feat, []

    def _compute_world2cam(self, vehicle_tf):
        """Compute world-to-camera transform matrix manually (CARLA 0.9.16 compat)."""
        import carla
        # Build camera world transform using matrix multiplication
        veh_mat = np.array(vehicle_tf.get_matrix())
        
        cam_tf = carla.Transform(
            carla.Location(x=self.cam_x, y=self.cam_y, z=self.cam_z),
            carla.Rotation(pitch=self.cam_pitch, yaw=self.cam_yaw, roll=self.cam_roll),
        )
        cam_mat = np.array(cam_tf.get_matrix())
        
        # Compose: camera_world = vehicle * camera_relative
        cam_world_mat = veh_mat @ cam_mat
        
        # Invert to get world-to-camera
        return np.linalg.inv(cam_world_mat)

    def _collect_forward_waypoints(self, start_wp, distance=None):
        """Walk forward along the lane."""
        distance = distance or self.LOOKAHEAD_DISTANCE
        wps = [start_wp]
        wp = start_wp
        traveled = 0.0
        while traveled < distance:
            nxt = wp.next(self.WAYPOINT_SPACING)
            if not nxt:
                break
            wp = nxt[0]
            wps.append(wp)
            traveled += self.WAYPOINT_SPACING
        return wps

    def _compute_lane_boundaries(self, waypoints):
        """Compute left/right/center 3D boundary points from waypoints."""
        left, right, center = [], [], []
        for wp in waypoints:
            loc = wp.transform.location
            yaw = np.deg2rad(wp.transform.rotation.yaw)
            hw = wp.lane_width / 2.0
            px, py = -np.sin(yaw), np.cos(yaw)
            left.append([loc.x + px * hw, loc.y + py * hw, loc.z])
            right.append([loc.x - px * hw, loc.y - py * hw, loc.z])
            center.append([loc.x, loc.y, loc.z])
        return np.array(left), np.array(right), np.array(center)

    def _project_points(self, pts_3d, world2cam, img_w, img_h):
        """Project 3D world points to 2D image using CARLA camera model."""
        if len(pts_3d) == 0:
            return []
        result = []
        for pt in pts_3d:
            pw = np.array([pt[0], pt[1], pt[2], 1.0])
            pc = world2cam @ pw
            x_cam, y_cam, z_cam = pc[1], -pc[2], pc[0]
            if z_cam < 0.5:
                continue
            u = int(self.K[0, 0] * x_cam / z_cam + self.K[0, 2])
            v = int(self.K[1, 1] * y_cam / z_cam + self.K[1, 2])
            if 0 <= u < img_w and 0 <= v < img_h:
                result.append((u, v))
        return result

    def _draw_polyline(self, mask, pts, color=255):
        """Draw a connected polyline on the mask."""
        if len(pts) < 2:
            return
        arr = np.array(pts, dtype=np.int32)
        cv2.polylines(mask, [arr], isClosed=False, color=color,
                      thickness=self.LANE_LINE_THICKNESS)

    def _fill_lane_area(self, mask, left_2d, right_2d):
        """Fill polygon between left and right boundaries (faint)."""
        if len(left_2d) < 2 or len(right_2d) < 2:
            return
        polygon = np.array(left_2d + right_2d[::-1], dtype=np.int32)
        overlay = np.zeros_like(mask)
        cv2.fillPoly(overlay, [polygon], 80)
        np.maximum(mask, overlay, out=mask)

    def _detect_adjacent_lanes(self, current_wp, world2cam, img_w, img_h, mask):
        """Detect boundaries of adjacent lanes."""
        import carla
        coords = []
        for get_adj in [current_wp.get_left_lane, current_wp.get_right_lane]:
            adj = get_adj()
            if adj is None or adj.lane_type != carla.LaneType.Driving:
                continue
            fwd = self._collect_forward_waypoints(adj, distance=40.0)
            lb, rb, _ = self._compute_lane_boundaries(fwd)
            for boundary in (lb, rb):
                pts = self._project_points(boundary, world2cam, img_w, img_h)
                if len(pts) >= 2:
                    self._draw_polyline(mask, pts, 180)
                    coords.append(pts)
        return coords

    def _extract_features(self, lane_mask, center_2d, left_2d, right_2d):
        """Extract 128-dim feature vector from lane geometry."""
        h, w = lane_mask.shape
        f = []

        if len(center_2d) > 0:
            xs = [p[0] for p in center_2d]
            ys = [p[1] for p in center_2d]
            f.extend([np.mean(xs) / w, np.mean(ys) / h])
        else:
            f.extend([0.5, 0.5])

        if len(left_2d) > 0 and len(right_2d) > 0:
            mean_w = abs(np.mean([p[0] for p in left_2d]) - np.mean([p[0] for p in right_2d]))
            f.append(mean_w / w)
        else:
            f.append(0.0)

        if len(center_2d) > 5:
            xs = np.array([p[0] for p in center_2d], dtype=float)
            ys = np.array([p[1] for p in center_2d], dtype=float)
            if len(np.unique(ys)) > 3:
                coeffs = np.polyfit(ys, xs, deg=2)
                f.extend(coeffs.tolist())
            else:
                f.extend([0.0, 0.0, 0.0])
        else:
            f.extend([0.0, 0.0, 0.0])

        for y1, y2 in [(0, h//3), (h//3, 2*h//3), (2*h//3, h)]:
            region = lane_mask[y1:y2, :]
            f.append(np.sum(region > 0) / (region.size + 1e-6))

        if len(center_2d) > 2:
            top = min(center_2d, key=lambda p: p[1])
            f.extend([top[0] / w, top[1] / h])
        else:
            f.extend([0.5, 0.0])

        while len(f) < 128:
            f.append(0.0)
        return np.array(f[:128], dtype=np.float32)
