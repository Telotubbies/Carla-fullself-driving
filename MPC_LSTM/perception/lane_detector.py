"""
Lane Detection Model using U-Net architecture.

Detects lane markings from RGB images and extracts lane features.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import cv2
import logging
from typing import Tuple, Optional, List
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.device_utils import get_device

logger = logging.getLogger(__name__)


class UNetBlock(nn.Module):
    """U-Net building block."""
    
    def __init__(self, in_channels: int, out_channels: int):
        super(UNetBlock, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )
    
    def forward(self, x):
        return self.conv(x)


class LaneUNet(nn.Module):
    """
    U-Net for lane detection.
    
    Architecture:
    - Encoder: Downsampling path
    - Decoder: Upsampling path with skip connections
    - Output: Lane segmentation mask
    """
    
    def __init__(self, in_channels: int = 3, num_classes: int = 2):
        """
        Initialize U-Net for lane detection.
        
        Args:
            in_channels: Input channels (3 for RGB)
            num_classes: Number of classes (2: background, lane)
        """
        super(LaneUNet, self).__init__()
        
        # Encoder (downsampling)
        self.enc1 = UNetBlock(in_channels, 64)
        self.enc2 = UNetBlock(64, 128)
        self.enc3 = UNetBlock(128, 256)
        self.enc4 = UNetBlock(256, 512)
        
        # Bottleneck
        self.bottleneck = UNetBlock(512, 1024)
        
        # Decoder (upsampling)
        self.up4 = nn.ConvTranspose2d(1024, 512, 2, stride=2)
        self.dec4 = UNetBlock(1024, 512)
        
        self.up3 = nn.ConvTranspose2d(512, 256, 2, stride=2)
        self.dec3 = UNetBlock(512, 256)
        
        self.up2 = nn.ConvTranspose2d(256, 128, 2, stride=2)
        self.dec2 = UNetBlock(256, 128)
        
        self.up1 = nn.ConvTranspose2d(128, 64, 2, stride=2)
        self.dec1 = UNetBlock(128, 64)
        
        # Output layer
        self.final = nn.Conv2d(64, num_classes, 1)
        
        self.device = get_device()
        self.to(self.device)
        
        logger.info(f"✅ LaneUNet initialized (device={self.device})")
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.
        
        Args:
            x: Input image (B, 3, H, W)
            
        Returns:
            Lane segmentation mask (B, num_classes, H, W)
        """
        # Encoder
        e1 = self.enc1(x)
        p1 = F.max_pool2d(e1, 2)
        
        e2 = self.enc2(p1)
        p2 = F.max_pool2d(e2, 2)
        
        e3 = self.enc3(p2)
        p3 = F.max_pool2d(e3, 2)
        
        e4 = self.enc4(p3)
        p4 = F.max_pool2d(e4, 2)
        
        # Bottleneck
        b = self.bottleneck(p4)
        
        # Decoder with skip connections
        d4 = self.up4(b)
        d4 = torch.cat([d4, e4], dim=1)
        d4 = self.dec4(d4)
        
        d3 = self.up3(d4)
        d3 = torch.cat([d3, e3], dim=1)
        d3 = self.dec3(d3)
        
        d2 = self.up2(d3)
        d2 = torch.cat([d2, e2], dim=1)
        d2 = self.dec2(d2)
        
        d1 = self.up1(d2)
        d1 = torch.cat([d1, e1], dim=1)
        d1 = self.dec1(d1)
        
        # Output
        output = self.final(d1)
        
        return output


class LaneDetector:
    """
    Lane detection using U-Net model or Ultra-Fast-Lane-Detection-v2.
    
    Can also use CARLA's built-in lane detection for labeling.
    """
    
    def __init__(self, model_path: Optional[str] = None, use_carla: bool = True, 
                 model_type: str = "unet"):
        """
        Initialize lane detector.
        
        Args:
            model_path: Path to trained model (None = use CARLA)
            use_carla: Use CARLA's lane detection if model not available
            model_type: Model type ("unet" or "ultra_fast")
        """
        self.use_carla = use_carla
        self.model = None
        self.model_type = model_type
        self.ultra_fast_detector = None
        
        if model_path and Path(model_path).exists():
            if model_type == "ultra_fast":
                # Use Ultra-Fast-Lane-Detection-v2
                try:
                    from perception.ultra_fast_lane_detector import UltraFastLaneDetector
                    # Detect dataset from model path
                    if "tusimple" in model_path.lower():
                        dataset = "tusimple"
                        backbone = "18"  # Default, can be extracted from path
                    elif "culane" in model_path.lower():
                        dataset = "culane"
                        backbone = "18"
                    else:
                        dataset = "tusimple"  # Default
                        backbone = "18"
                    
                    self.ultra_fast_detector = UltraFastLaneDetector(
                        model_path=model_path,
                        dataset=dataset,
                        backbone=backbone
                    )
                    self.use_carla = False
                    logger.info(f"✅ Loaded Ultra-Fast-Lane-Detection-v2 model from {model_path} (dataset={dataset}, backbone={backbone})")
                except Exception as e:
                    logger.warning(f"Failed to load Ultra-Fast-Lane-Detection-v2: {e}, using CARLA detection")
                    import traceback
                    logger.debug(traceback.format_exc())
                    self.use_carla = True
            else:
                # Use U-Net (default)
                try:
                    self.model = LaneUNet()
                    self.model.load_state_dict(torch.load(model_path, map_location=get_device()))
                    self.model.eval()
                    self.use_carla = False
                    logger.info(f"✅ Loaded U-Net lane detection model from {model_path}")
                except Exception as e:
                    logger.warning(f"Failed to load U-Net model: {e}, using CARLA detection")
                    self.use_carla = True
        else:
            if use_carla:
                logger.info("Using CARLA's built-in lane detection")
            else:
                logger.warning("No model provided, using CARLA detection")
                self.use_carla = True
    
    def detect_lanes_carla(self, image: np.ndarray, world, vehicle, camera_transform=None) -> np.ndarray:
        """
        Detect lanes using CARLA's built-in lane detection with proper camera projection.
        
        Args:
            image: Input image (H, W, 3)
            world: CARLA world object
            vehicle: CARLA vehicle object
            camera_transform: Camera transform relative to vehicle (optional)
            
        Returns:
            Lane mask (H, W) binary
        """
        try:
            import carla
            
            h, w = image.shape[:2]
            lane_mask = np.zeros((h, w), dtype=np.uint8)
            
            # Get vehicle transform
            vehicle_transform = vehicle.get_transform()
            
            # Default camera transform if not provided (typical front camera)
            if camera_transform is None:
                camera_transform = carla.Transform(
                    carla.Location(x=2.0, z=1.4),  # Front of vehicle, above ground
                    carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                )
            
            # Calculate camera world transform
            # CARLA Transform multiplication: vehicle_transform * camera_transform
            # This combines the transforms correctly
            try:
                camera_world_transform = vehicle_transform * camera_transform
            except Exception as e:
                # Fallback: manually combine transforms
                logger.debug(f"Transform multiplication failed: {e}, using manual calculation")
                import carla
                # Get vehicle location and rotation
                vehicle_loc = vehicle_transform.location
                vehicle_rot = vehicle_transform.rotation
                
                # Get camera relative transform
                cam_rel_loc = camera_transform.location
                cam_rel_rot = camera_transform.rotation
                
                # Manual transform combination (simplified)
                camera_world_transform = carla.Transform(
                    carla.Location(
                        x=vehicle_loc.x + cam_rel_loc.x,
                        y=vehicle_loc.y + cam_rel_loc.y,
                        z=vehicle_loc.z + cam_rel_loc.z
                    ),
                    carla.Rotation(
                        pitch=vehicle_rot.pitch + cam_rel_rot.pitch,
                        yaw=vehicle_rot.yaw + cam_rel_rot.yaw,
                        roll=vehicle_rot.roll + cam_rel_rot.roll
                    )
                )
            
            # Get map
            carla_map = world.get_map()
            
            # Get waypoints (lane centers) - use smaller distance for better performance
            waypoints = carla_map.generate_waypoints(distance=2.0)
            
            # Get vehicle location
            vehicle_location = vehicle_transform.location
            
            # Find nearby waypoints (within 50m)
            nearby_waypoints = []
            for wp in waypoints:
                dist = vehicle_location.distance(wp.transform.location)
                if dist < 50.0:
                    nearby_waypoints.append(wp)
            
            if len(nearby_waypoints) == 0:
                logger.debug("No nearby waypoints found")
                return lane_mask
            
            # Camera intrinsic matrix (assume 90 FOV, can be passed as parameter)
            fov = 90.0
            f = w / (2.0 * np.tan(np.radians(fov / 2.0)))
            K = np.array([
                [f, 0, w / 2.0],
                [0, f, h / 2.0],
                [0, 0, 1.0]
            ])
            
            # Use CARLA's built-in transform matrix (more accurate)
            # Get world-to-camera transformation matrix
            world_2_camera = np.array(camera_world_transform.get_inverse_matrix())
            
            # Project waypoints to image
            projected_points = []
            for wp in nearby_waypoints:
                # World point in homogeneous coordinates
                world_point = np.array([
                    wp.transform.location.x,
                    wp.transform.location.y,
                    wp.transform.location.z,
                    1.0
                ])
                
                # Transform to camera coordinates using CARLA's matrix
                point_camera = np.dot(world_2_camera, world_point)
                
                # CARLA/UE4 coordinate system transformation
                # UE4: (x, y, z) -> Standard camera: (y, -z, x)
                x_cam = point_camera[1]   # y -> x (right)
                y_cam = -point_camera[2]  # -z -> y (down)
                z_cam = point_camera[0]   # x -> z (forward)
                
                # Skip if behind camera
                if z_cam <= 0.1:  # Minimum depth
                    continue
                
                # Normalize (perspective projection)
                x_norm = x_cam / z_cam
                y_norm = y_cam / z_cam
                
                # Project using camera intrinsic matrix
                point_img = np.dot(K, np.array([x_norm, y_norm, 1.0]))
                
                # Normalize homogeneous coordinates
                u = int(point_img[0] / point_img[2])
                v = int(point_img[1] / point_img[2])
                
                # Check bounds
                if 0 <= u < w and 0 <= v < h:
                    projected_points.append((u, v))
            
            # Draw lane lines connecting waypoints
            if len(projected_points) > 1:
                # Sort by distance from vehicle (bottom of image = closer)
                projected_points.sort(key=lambda p: -p[1])  # Sort by v (y coordinate, descending)
                
                # Draw lines between consecutive waypoints
                for i in range(len(projected_points) - 1):
                    pt1 = projected_points[i]
                    pt2 = projected_points[i + 1]
                    
                    # Only draw if points are reasonably close (avoid long lines across image)
                    dist = np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
                    if dist < 300:  # Max pixel distance (increased for smoother lines)
                        cv2.line(lane_mask, pt1, pt2, 255, 4)
                
                # Draw waypoint markers for better visibility (every 3rd point to avoid clutter)
                for pt in projected_points[::3]:
                    cv2.circle(lane_mask, pt, 4, 255, -1)
            
            return lane_mask
            
        except Exception as e:
            logger.warning(f"CARLA lane detection failed: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return np.zeros(image.shape[:2], dtype=np.uint8)
    
    def detect_lanes(self, image: np.ndarray, world=None, vehicle=None) -> Tuple[np.ndarray, np.ndarray, List[List[Tuple[int, int]]]]:
        """
        Detect lanes in image.
        
        Args:
            image: Input image (H, W, 3) RGB, uint8
            world: CARLA world (if using CARLA detection)
            vehicle: CARLA vehicle (if using CARLA detection)
            
        Returns:
            Tuple of:
            - lane_mask: Binary lane mask (H, W)
            - lane_features: Lane feature vector (128,)
            - lane_coords: List of lane coordinate lists (for visualization)
        """
        # Use Ultra-Fast-Lane-Detection-v2 if available
        if self.ultra_fast_detector is not None:
            mask, features, coords = self.ultra_fast_detector.detect_lanes(image, use_post_processing=True)
            return mask, features, coords
        
        # Use trained U-Net if available
        if self.model is not None:
            with torch.no_grad():
                # Preprocess
                img_tensor = torch.FloatTensor(image).permute(2, 0, 1).unsqueeze(0) / 255.0
                img_tensor = img_tensor.to(self.model.device)
                
                # Resize to model input size
                img_tensor = F.interpolate(img_tensor, size=(256, 256), mode='bilinear', align_corners=False)
                
                # Predict
                output = self.model(img_tensor)
                mask = torch.argmax(output, dim=1).squeeze().cpu().numpy()
                
                # Resize back
                mask = cv2.resize(mask.astype(np.uint8), (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
                
                # Extract features from mask
                lane_features = self._extract_lane_features(mask)
                
                return (mask * 255).astype(np.uint8), lane_features, []
        else:
            # Use CARLA detection
            if world is not None and vehicle is not None:
                mask = self.detect_lanes_carla(image, world, vehicle)
                lane_features = self._extract_lane_features(mask)
                return mask, lane_features, []
            else:
                # Fallback: simple edge detection
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                edges = cv2.Canny(gray, 50, 150)
                # Simple lane detection (horizontal lines in lower half)
                h, w = image.shape[:2]
                mask = np.zeros((h, w), dtype=np.uint8)
                mask[h//2:, :] = edges[h//2:, :]
                lane_features = self._extract_lane_features(mask)
                return mask, lane_features, []
    
    def _extract_lane_features(self, lane_mask: np.ndarray) -> np.ndarray:
        """
        Extract lane features from mask.
        
        Args:
            lane_mask: Binary lane mask (H, W)
            
        Returns:
            Feature vector (128,)
        """
        h, w = lane_mask.shape
        
        # Features: lane position, curvature, width, etc.
        features = []
        
        # Lane position (horizontal center)
        lane_pixels = np.where(lane_mask > 0)
        if len(lane_pixels[0]) > 0:
            center_x = np.mean(lane_pixels[1])
            center_y = np.mean(lane_pixels[0])
            features.extend([center_x / w, center_y / h])
        else:
            features.extend([0.5, 0.5])  # Default center
        
        # Lane width (approximate)
        if len(lane_pixels[0]) > 0:
            width = np.std(lane_pixels[1]) * 2
            features.append(width / w)
        else:
            features.append(0.0)
        
        # Lane curvature (simplified)
        if len(lane_pixels[0]) > 10:
            # Fit polynomial to lane points
            y_coords = lane_pixels[0]
            x_coords = lane_pixels[1]
            if len(np.unique(y_coords)) > 3:
                coeffs = np.polyfit(y_coords, x_coords, 2)
                features.extend([coeffs[0], coeffs[1], coeffs[2]])
            else:
                features.extend([0.0, 0.0, 0.0])
        else:
            features.extend([0.0, 0.0, 0.0])
        
        # Lane density (pixels per region)
        regions = [
            (0, h//3, 0, w),           # Top
            (h//3, 2*h//3, 0, w),      # Middle
            (2*h//3, h, 0, w),         # Bottom
        ]
        for y1, y2, x1, x2 in regions:
            region_mask = lane_mask[y1:y2, x1:x2]
            density = np.sum(region_mask > 0) / (region_mask.size + 1e-6)
            features.append(density)
        
        # Pad to 128 dimensions
        while len(features) < 128:
            features.append(0.0)
        
        return np.array(features[:128], dtype=np.float32)


def create_lane_labels_from_carla(images_dir: str, output_dir: str, world, vehicle):
    """
    Create lane labels from CARLA for training.
    
    Args:
        images_dir: Directory with images
        output_dir: Output directory for masks
        world: CARLA world
        vehicle: CARLA vehicle
    """
    from pathlib import Path
    import cv2
    
    images_dir = Path(images_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    detector = LaneDetector(use_carla=True)
    
    image_files = list(images_dir.glob("*.png"))
    logger.info(f"Processing {len(image_files)} images...")
    
    for img_path in image_files:
        image = cv2.imread(str(img_path))
        if image is None:
            continue
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask, _ = detector.detect_lanes(image, world, vehicle)
        
        # Save mask
        mask_path = output_dir / f"{img_path.stem}_lane.png"
        cv2.imwrite(str(mask_path), mask)
    
    logger.info(f"✅ Created {len(image_files)} lane masks")

