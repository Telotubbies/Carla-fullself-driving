"""
Create lane labels from CARLA for training.

Uses CARLA's map API to extract lane markings and project to camera view.
"""

import sys
import argparse
import logging
import numpy as np
import cv2
from pathlib import Path
from tqdm import tqdm
import carla
from typing import Tuple, List

sys.path.insert(0, str(Path(__file__).parent.parent))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def get_lane_markings_from_carla(
    world: carla.World,
    vehicle: carla.Vehicle,
    camera_transform: carla.Transform,
    image_width: int = 640,
    image_height: int = 480,
    fov: float = 90.0,
    distance: float = 50.0
) -> np.ndarray:
    """
    Extract lane markings from CARLA and project to camera view.
    
    Args:
        world: CARLA world
        vehicle: CARLA vehicle
        camera_transform: Camera transform relative to vehicle
        image_width: Image width
        image_height: Image height
        fov: Camera FOV
        distance: Maximum distance to extract lanes
        
    Returns:
        Lane mask (H, W) binary
    """
    # Get vehicle transform
    vehicle_transform = vehicle.get_transform()
    
    # Calculate camera world transform
    camera_world_transform = vehicle_transform * camera_transform
    
    # Get map
    carla_map = world.get_map()
    
    # Get waypoints (lane centers)
    waypoints = carla_map.generate_waypoints(distance=1.0)
    
    # Get vehicle location
    vehicle_location = vehicle_transform.location
    
    # Find nearby waypoints
    nearby_waypoints = []
    for wp in waypoints:
        dist = vehicle_location.distance(wp.transform.location)
        if dist < distance:
            nearby_waypoints.append(wp)
    
    # Create lane mask
    lane_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    
    if len(nearby_waypoints) == 0:
        return lane_mask
    
    # Project waypoints to image
    # Camera intrinsic matrix (simplified)
    f = image_width / (2.0 * np.tan(np.radians(fov / 2.0)))
    K = np.array([
        [f, 0, image_width / 2.0],
        [0, f, image_height / 2.0],
        [0, 0, 1.0]
    ])
    
    # Camera rotation and translation
    camera_rotation = camera_world_transform.rotation
    camera_location = camera_world_transform.location
    
    # Convert CARLA rotation to rotation matrix
    roll = np.radians(camera_rotation.roll)
    pitch = np.radians(camera_rotation.pitch)
    yaw = np.radians(camera_rotation.yaw)
    
    # Rotation matrices
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    R = R_z @ R_y @ R_x
    
    # Project waypoints
    projected_points = []
    for wp in nearby_waypoints:
        # World to camera coordinates
        world_point = np.array([
            wp.transform.location.x,
            wp.transform.location.y,
            wp.transform.location.z
        ])
        
        # Transform to camera frame
        cam_point = world_point - np.array([
            camera_location.x,
            camera_location.y,
            camera_location.z
        ])
        cam_point = R @ cam_point
        
        # Skip if behind camera
        if cam_point[2] <= 0:
            continue
        
        # Project to image
        x = cam_point[0] / cam_point[2]
        y = cam_point[1] / cam_point[2]
        
        pixel = K @ np.array([x, y, 1])
        u, v = int(pixel[0]), int(pixel[1])
        
        # Check bounds
        if 0 <= u < image_width and 0 <= v < image_height:
            projected_points.append((u, v))
    
    # Draw lane lines
    if len(projected_points) > 1:
        # Sort by distance from vehicle
        projected_points.sort(key=lambda p: p[1], reverse=True)  # Sort by v (bottom to top)
        
        # Draw lines connecting waypoints
        for i in range(len(projected_points) - 1):
            pt1 = projected_points[i]
            pt2 = projected_points[i + 1]
            cv2.line(lane_mask, pt1, pt2, 255, 3)
        
        # Also draw lane boundaries (left and right)
        # Get left and right lane markings
        for wp in nearby_waypoints[:100]:  # Limit for performance
            # Get left and right lane markings
            left_wp = wp.get_left_lane()
            right_wp = wp.get_right_lane()
            
            if left_wp:
                left_loc = left_wp.transform.location
                left_cam = np.array([left_loc.x, left_loc.y, left_loc.z]) - np.array([
                    camera_location.x, camera_location.y, camera_location.z
                ])
                left_cam = R @ left_cam
                if left_cam[2] > 0:
                    x = left_cam[0] / left_cam[2]
                    y = left_cam[1] / left_cam[2]
                    pixel = K @ np.array([x, y, 1])
                    u, v = int(pixel[0]), int(pixel[1])
                    if 0 <= u < image_width and 0 <= v < image_height:
                        cv2.circle(lane_mask, (u, v), 2, 255, -1)
            
            if right_wp:
                right_loc = right_wp.transform.location
                right_cam = np.array([right_loc.x, right_loc.y, right_loc.z]) - np.array([
                    camera_location.x, camera_location.y, camera_location.z
                ])
                right_cam = R @ right_cam
                if right_cam[2] > 0:
                    x = right_cam[0] / right_cam[2]
                    y = right_cam[1] / right_cam[2]
                    pixel = K @ np.array([x, y, 1])
                    u, v = int(pixel[0]), int(pixel[1])
                    if 0 <= u < image_width and 0 <= v < image_height:
                        cv2.circle(lane_mask, (u, v), 2, 255, -1)
    
    # Apply morphological operations to clean up
    kernel = np.ones((3, 3), np.uint8)
    lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
    lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
    
    return lane_mask


def create_lane_labels_batch(
    images_dir: str,
    output_dir: str,
    carla_host: str = "localhost",
    carla_port: int = 2000,
    camera_location: Tuple[float, float, float] = (2.0, 0.0, 1.4),
    camera_rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    image_width: int = 640,
    image_height: int = 480,
    fov: float = 90.0
):
    """
    Create lane labels for all images in directory.
    
    Args:
        images_dir: Directory with images
        output_dir: Output directory for masks
        carla_host: CARLA host
        carla_port: CARLA port
        camera_location: Camera location relative to vehicle
        camera_rotation: Camera rotation relative to vehicle
        image_width: Image width
        image_height: Image height
        fov: Camera FOV
    """
    images_dir = Path(images_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Connect to CARLA
    logger.info(f"Connecting to CARLA at {carla_host}:{carla_port}...")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    world = client.get_world()
    
    # Get vehicle
    vehicles = world.get_actors().filter("vehicle.*")
    if len(vehicles) == 0:
        logger.error("No vehicle found in CARLA!")
        return
    
    vehicle = vehicles[0]
    logger.info(f"Using vehicle: {vehicle.type_id}")
    
    # Camera transform
    camera_transform = carla.Transform(
        carla.Location(camera_location[0], camera_location[1], camera_location[2]),
        carla.Rotation(camera_rotation[0], camera_rotation[1], camera_rotation[2])
    )
    
    # Get image files
    image_files = sorted(list(images_dir.glob("*.png")))
    logger.info(f"Processing {len(image_files)} images...")
    
    # Process each image
    success_count = 0
    for img_path in tqdm(image_files, desc="Creating lane labels"):
        try:
            # Read image to get metadata (if stored in filename or we need to sync)
            # For now, we'll generate mask based on current vehicle position
            # In production, you'd want to store vehicle state with each image
            
            # Get lane mask
            lane_mask = get_lane_markings_from_carla(
                world=world,
                vehicle=vehicle,
                camera_transform=camera_transform,
                image_width=image_width,
                image_height=image_height,
                fov=fov
            )
            
            # Save mask
            mask_path = output_dir / f"{img_path.stem}_lane.png"
            cv2.imwrite(str(mask_path), lane_mask)
            success_count += 1
            
        except Exception as e:
            logger.warning(f"Failed to process {img_path}: {e}")
            continue
    
    logger.info(f"✅ Created {success_count}/{len(image_files)} lane masks")
    logger.info(f"   Output: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description='Create lane labels from CARLA')
    parser.add_argument('--images-dir', type=str, required=True, help='Images directory')
    parser.add_argument('--output-dir', type=str, required=True, help='Output directory for masks')
    parser.add_argument('--carla-host', type=str, default='localhost', help='CARLA host')
    parser.add_argument('--carla-port', type=int, default=2000, help='CARLA port')
    parser.add_argument('--image-width', type=int, default=640, help='Image width')
    parser.add_argument('--image-height', type=int, default=480, help='Image height')
    parser.add_argument('--fov', type=float, default=90.0, help='Camera FOV')
    
    args = parser.parse_args()
    
    create_lane_labels_batch(
        images_dir=args.images_dir,
        output_dir=args.output_dir,
        carla_host=args.carla_host,
        carla_port=args.carla_port,
        image_width=args.image_width,
        image_height=args.image_height,
        fov=args.fov
    )


if __name__ == '__main__':
    main()

