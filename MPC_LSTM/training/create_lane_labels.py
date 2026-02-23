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


def get_lane_mask_from_semantic_camera(
    world: carla.World,
    vehicle: carla.Vehicle,
    camera_transform: carla.Transform,
    image_width: int = 640,
    image_height: int = 480,
    fov: float = 90.0
) -> np.ndarray:
    """
    Get lane mask from CARLA semantic segmentation camera.
    This shows only lane markings on the GUI, making it perfect for training.
    
    Args:
        world: CARLA world
        vehicle: CARLA vehicle
        camera_transform: Camera transform relative to vehicle
        image_width: Image width
        image_height: Image height
        fov: Camera FOV
        
    Returns:
        Lane mask (H, W) binary (255 for lanes, 0 for background)
    """
    import queue
    import time
    
    # Get blueprint for semantic segmentation camera
    bp_library = world.get_blueprint_library()
    camera_bp = bp_library.find('sensor.camera.semantic_segmentation')
    camera_bp.set_attribute('image_size_x', str(image_width))
    camera_bp.set_attribute('image_size_y', str(image_height))
    camera_bp.set_attribute('fov', str(fov))
    
    # Calculate camera world transform
    vehicle_transform = vehicle.get_transform()
    # CARLA Transform multiplication: vehicle_transform * camera_transform
    # Use manual calculation (CARLA 0.9.16 Transform multiplication may not work as expected)
    import math
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation
    
    # Convert camera relative location to world coordinates
    cam_rel_loc = camera_transform.location
    yaw_rad = math.radians(vehicle_rotation.yaw)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    
    # Rotate camera location by vehicle yaw (simplified - only yaw rotation)
    world_x = vehicle_location.x + cam_rel_loc.x * cos_yaw - cam_rel_loc.y * sin_yaw
    world_y = vehicle_location.y + cam_rel_loc.x * sin_yaw + cam_rel_loc.y * cos_yaw
    world_z = vehicle_location.z + cam_rel_loc.z
    
    world_location = carla.Location(world_x, world_y, world_z)
    world_rotation = carla.Rotation(
        vehicle_rotation.pitch + camera_transform.rotation.pitch,
        vehicle_rotation.roll + camera_transform.rotation.roll,
        vehicle_rotation.yaw + camera_transform.rotation.yaw
    )
    camera_world_transform = carla.Transform(world_location, world_rotation)
    
    # Queue for receiving images
    image_queue = queue.Queue()
    
    def image_callback(image):
        """Callback to receive semantic segmentation image."""
        image_queue.put(image)
    
    # Spawn camera
    camera = world.spawn_actor(camera_bp, camera_world_transform, attach_to=vehicle)
    
    try:
        # Listen for images
        camera.listen(image_callback)
        
        # Wait for camera to initialize and get first image
        # Tick multiple times to ensure camera captures correct view
        for _ in range(5):
            world.tick()
            time.sleep(0.05)
        
        # Get image from queue (with timeout)
        try:
            image = image_queue.get(timeout=3.0)
        except queue.Empty:
            logger.warning("Timeout waiting for semantic segmentation image")
            return np.zeros((image_height, image_width), dtype=np.uint8)
        
        # Convert to numpy array
        # CARLA semantic segmentation image format: BGRA
        # In CARLA 0.9.16, class IDs are stored in the R channel (B and G are 0)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))  # BGRA
        
        # Extract class IDs from R channel (channel index 2 in BGRA format)
        class_ids = array[:, :, 2]  # R channel contains class IDs
        
        # Check which classes are present (for debugging)
        unique_classes = np.unique(class_ids)
        logger.debug(f"Semantic segmentation classes found: {unique_classes}")
        
        # Create lane mask
        # Class 24: RoadLine (lane markings)
        lane_mask = np.zeros((image_height, image_width), dtype=np.uint8)
        lane_mask[class_ids == 24] = 255
        
        # Log for debugging
        lane_pixels = np.sum(lane_mask > 0)
        if lane_pixels == 0:
            logger.warning(f"No lane markings found (class 24). Available classes: {unique_classes}")
            if 24 not in unique_classes:
                logger.warning("Class 24 (RoadLine) not found in semantic segmentation image")
                # Don't use fallback - return empty mask instead of wrong mask
                return lane_mask
            # If class 24 exists but no pixels found, it might be a different issue
            # Return empty mask rather than using fallback that might be wrong
            return lane_mask
        
        logger.debug(f"Lane mask: {np.sum(lane_mask > 0)} pixels ({100*np.sum(lane_mask > 0)/(image_height*image_width):.2f}%)")
        
        return lane_mask
        
    finally:
        # Stop listening and destroy camera
        camera.stop()
        camera.destroy()


def get_lane_markings_from_carla(
    world: carla.World,
    vehicle: carla.Vehicle,
    camera_transform: carla.Transform,
    image_width: int = 640,
    image_height: int = 480,
    fov: float = 90.0,
    distance: float = 50.0,
    use_semantic_camera: bool = True
) -> np.ndarray:
    """
    Extract lane markings from CARLA.
    
    If use_semantic_camera=True, uses semantic segmentation camera (recommended).
    Otherwise, uses map API projection (legacy method).
    
    Args:
        world: CARLA world
        vehicle: CARLA vehicle
        camera_transform: Camera transform relative to vehicle
        image_width: Image width
        image_height: Image height
        fov: Camera FOV
        distance: Maximum distance to extract lanes (only for map API method)
        use_semantic_camera: If True, use semantic segmentation camera (recommended)
        
    Returns:
        Lane mask (H, W) binary
    """
    if use_semantic_camera:
        return get_lane_mask_from_semantic_camera(
            world=world,
            vehicle=vehicle,
            camera_transform=camera_transform,
            image_width=image_width,
            image_height=image_height,
            fov=fov
        )
    
    # Legacy method: project waypoints (kept for backward compatibility)
    # Get vehicle transform
    vehicle_transform = vehicle.get_transform()
    
    # Calculate camera world transform
    # Use manual calculation (CARLA 0.9.16 Transform multiplication may not work)
    import math
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation
    cam_rel_loc = camera_transform.location
    yaw_rad = math.radians(vehicle_rotation.yaw)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    world_x = vehicle_location.x + cam_rel_loc.x * cos_yaw - cam_rel_loc.y * sin_yaw
    world_y = vehicle_location.y + cam_rel_loc.x * sin_yaw + cam_rel_loc.y * cos_yaw
    world_z = vehicle_location.z + cam_rel_loc.z
    world_location = carla.Location(world_x, world_y, world_z)
    world_rotation = carla.Rotation(
        vehicle_rotation.pitch + camera_transform.rotation.pitch,
        vehicle_rotation.roll + camera_transform.rotation.roll,
        vehicle_rotation.yaw + camera_transform.rotation.yaw
    )
    camera_world_transform = carla.Transform(world_location, world_rotation)
    
    # Get map
    carla_map = world.get_map()
    
    # Get vehicle location
    vehicle_location = vehicle_transform.location
    
    # Get waypoint at vehicle location
    waypoint_at_vehicle = carla_map.get_waypoint(vehicle_location)
    
    # Create lane mask
    lane_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    
    if waypoint_at_vehicle is None:
        # Try to find nearest waypoint if exact match fails
        logger.warning(f"No waypoint found at vehicle location {vehicle_location}, trying nearest waypoint")
        # Generate waypoints and find nearest
        all_waypoints = carla_map.generate_waypoints(distance=5.0)
        if len(all_waypoints) > 0:
            min_dist = float('inf')
            nearest_wp = None
            for wp in all_waypoints[:1000]:  # Check first 1000 waypoints
                dist = vehicle_location.distance(wp.transform.location)
                if dist < min_dist:
                    min_dist = dist
                    nearest_wp = wp
            if nearest_wp and min_dist < 10.0:  # Within 10 meters
                waypoint_at_vehicle = nearest_wp
                logger.info(f"Found nearest waypoint at distance {min_dist:.2f}m")
            else:
                logger.warning(f"No nearby waypoint found (nearest: {min_dist:.2f}m)")
                return lane_mask
        else:
            logger.warning("No waypoints available in map")
            return lane_mask
    
    # Get waypoints along the road from vehicle position
    nearby_waypoints = []
    current_wp = waypoint_at_vehicle
    
    # Get waypoints ahead (forward)
    for _ in range(100):  # Get 100 waypoints ahead (200m at 2m spacing)
        nearby_waypoints.append(current_wp)
        next_wps = current_wp.next(2.0)  # 2 meters ahead
        if len(next_wps) > 0:
            current_wp = next_wps[0]
        else:
            break
    
    # Get waypoints behind (backward)
    current_wp = waypoint_at_vehicle
    for _ in range(50):  # Get 50 waypoints behind (100m)
        prev_wps = current_wp.previous(2.0)  # 2 meters behind
        if len(prev_wps) > 0:
            current_wp = prev_wps[0]
            nearby_waypoints.insert(0, current_wp)  # Insert at beginning
        else:
            break
    
    if len(nearby_waypoints) == 0:
        logger.warning(f"No nearby waypoints found from vehicle waypoint")
        return lane_mask
    
    logger.debug(f"Found {len(nearby_waypoints)} nearby waypoints for projection")
    
    # Project waypoints to image using CARLA's built-in projection
    # Create a temporary camera to get proper transform matrix (same as lane_detector.py)
    bp_library = world.get_blueprint_library()
    temp_camera_bp = bp_library.find('sensor.camera.rgb')
    temp_camera_bp.set_attribute('image_size_x', str(image_width))
    temp_camera_bp.set_attribute('image_size_y', str(image_height))
    temp_camera_bp.set_attribute('fov', str(fov))
    
    # Spawn temporary camera to get transform matrix
    temp_camera = world.spawn_actor(temp_camera_bp, camera_world_transform, attach_to=vehicle)
    
    try:
        # Get world-to-camera matrix (4x4) from camera actor
        world_2_camera = np.array(temp_camera.get_transform().get_inverse_matrix())
        
        # Build projection matrix (CARLA's standard method)
        focal = image_width / (2.0 * np.tan(np.radians(fov) / 2.0))
        K = np.identity(3)
        K[0, 0] = K[1, 1] = focal
        K[0, 2] = image_width / 2.0
        K[1, 2] = image_height / 2.0
        
        # Project waypoints using CARLA's coordinate system (same as lane_detector.py)
        projected_points = []
        points_in_view = 0
        points_behind = 0
        points_out_of_bounds = 0
        
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
                points_behind += 1
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
            if 0 <= u < image_width and 0 <= v < image_height:
                projected_points.append((u, v))
                points_in_view += 1
            else:
                points_out_of_bounds += 1
        
        # Debug logging (log first few failures)
        if len(nearby_waypoints) > 0 and len(projected_points) == 0:
            logger.warning(f"Waypoint projection failed: {len(nearby_waypoints)} waypoints, {points_in_view} in view, {points_behind} behind, {points_out_of_bounds} out of bounds")
    
    finally:
        temp_camera.destroy()
    
    # Draw lane lines
    if len(projected_points) > 1:
        # Sort by distance from vehicle
        projected_points.sort(key=lambda p: p[1], reverse=True)  # Sort by v (bottom to top)
        
        # Draw lines connecting waypoints
        for i in range(len(projected_points) - 1):
            pt1 = projected_points[i]
            pt2 = projected_points[i + 1]
            cv2.line(lane_mask, pt1, pt2, 255, 3)
        
        # Also draw lane boundaries (left and right) using same projection
        for wp in nearby_waypoints[:50]:  # Limit for performance
            # Get left and right lane markings
            left_wp = wp.get_left_lane()
            right_wp = wp.get_right_lane()
            
            for boundary_wp in [left_wp, right_wp]:
                if boundary_wp is None:
                    continue
                
                # Project boundary waypoint using same method
                boundary_point = np.array([
                    boundary_wp.transform.location.x,
                    boundary_wp.transform.location.y,
                    boundary_wp.transform.location.z,
                    1.0
                ])
                
                point_camera = np.dot(world_2_camera, boundary_point)
                x_cam = point_camera[1]
                y_cam = -point_camera[2]
                z_cam = point_camera[0]
                
                if z_cam <= 0.1:
                    continue
                
                x_norm = x_cam / z_cam
                y_norm = y_cam / z_cam
                point_img = np.dot(K, np.array([x_norm, y_norm, 1.0]))
                u = int(point_img[0] / point_img[2])
                v = int(point_img[1] / point_img[2])
                
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
    fov: float = 90.0,
    data_csv: str = None
):
    """
    Create lane labels for all images in directory.
    
    Uses vehicle state from CSV file to generate accurate lane masks.
    
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
        data_csv: Path to CSV file with vehicle states (x, y, yaw)
    """
    import csv
    
    images_dir = Path(images_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Connect to CARLA
    logger.info(f"Connecting to CARLA at {carla_host}:{carla_port}...")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    world = client.get_world()
    
    # Get vehicle (spawn if not exists)
    vehicles = world.get_actors().filter("vehicle.*")
    if len(vehicles) == 0:
        logger.warning("No vehicle found in CARLA! Spawning vehicle...")
        # Spawn a vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
        
        # Get spawn points
        spawn_points = world.get_map().get_spawn_points()
        if len(spawn_points) == 0:
            logger.error("No spawn points available!")
            return
        
        # Spawn vehicle at first spawn point
        spawn_point = spawn_points[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        logger.info(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
        
        # Set autopilot to keep vehicle in place (optional)
        vehicle.set_autopilot(False)
    else:
        vehicle = vehicles[0]
        logger.info(f"Using existing vehicle: {vehicle.type_id}")
    
    # Camera transform
    camera_transform = carla.Transform(
        carla.Location(camera_location[0], camera_location[1], camera_location[2]),
        carla.Rotation(camera_rotation[0], camera_rotation[1], camera_rotation[2])
    )
    
    # Load vehicle states from CSV if provided
    vehicle_states = {}
    if data_csv and Path(data_csv).exists():
        logger.info(f"Loading vehicle states from {data_csv}...")
        with open(data_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                step = int(row['step'])
                vehicle_states[step] = {
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw'])
                }
        logger.info(f"Loaded {len(vehicle_states)} vehicle states")
    else:
        logger.warning("No CSV file provided or file not found. Using current vehicle position (may be inaccurate).")
    
    # Get image files
    image_files = sorted(list(images_dir.glob("*.png")))
    logger.info(f"Processing {len(image_files)} images...")
    
    # Process each image
    success_count = 0
    for img_path in tqdm(image_files, desc="Creating lane labels"):
        try:
            # Extract step number from filename (image_XXXXXX.png)
            step_str = img_path.stem.replace('image_', '')
            try:
                step = int(step_str)
            except ValueError:
                logger.warning(f"Could not extract step from {img_path.name}, skipping...")
                continue
            
            # Teleport vehicle to saved position if available
            if step in vehicle_states:
                state = vehicle_states[step]
                
                # Get map to find nearest waypoint for correct z (height)
                carla_map = world.get_map()
                location_2d = carla.Location(state['x'], state['y'], 0.0)
                waypoint = carla_map.get_waypoint(location_2d)
                
                # Use waypoint z if available, otherwise use default
                if waypoint:
                    z_height = waypoint.transform.location.z + 0.5
                else:
                    # If no waypoint, try to get z from current vehicle position
                    current_transform = vehicle.get_transform()
                    z_height = current_transform.location.z
                
                # Create new transform with saved x, y, yaw
                new_transform = carla.Transform(
                    carla.Location(state['x'], state['y'], z_height),
                    carla.Rotation(
                        0.0,  # pitch
                        0.0,  # roll
                        np.degrees(state['yaw'])  # Convert radians to degrees
                    )
                )
                
                # Teleport vehicle
                vehicle.set_transform(new_transform)
                
                # Wait for physics to settle - tick multiple times
                import time
                for _ in range(5):
                    world.tick()
                    time.sleep(0.05)
                
                # Additional delay to ensure camera sees correct view
                time.sleep(0.2)
            
            # Get lane mask using waypoint projection (more reliable than semantic camera)
            # Semantic camera may not work correctly after teleportation
            lane_mask = get_lane_markings_from_carla(
                world=world,
                vehicle=vehicle,
                camera_transform=camera_transform,
                image_width=image_width,
                image_height=image_height,
                fov=fov,
                use_semantic_camera=False  # Use waypoint projection instead
            )
            
            # Debug: Check lane mask before saving
            lane_pixels = np.sum(lane_mask > 0)
            total_pixels = lane_mask.size
            unique_values = np.unique(lane_mask)
            
            if lane_pixels == total_pixels:
                logger.error(f"ERROR: {img_path.name} - Lane mask is 100% white! This should not happen.")
                logger.error(f"  Unique values: {unique_values}")
                logger.error(f"  Skipping this mask to avoid corrupting dataset.")
                continue
            
            if lane_pixels == 0:
                logger.warning(f"WARNING: {img_path.name} - No lane pixels found. Available classes may not include class 24.")
            else:
                logger.info(f"OK: {img_path.name} - Found {lane_pixels:,}/{total_pixels:,} lane pixels ({100*lane_pixels/total_pixels:.2f}%)")
            
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
    parser.add_argument('--data-csv', type=str, default=None, help='Path to CSV file with vehicle states (x, y, yaw)')
    
    args = parser.parse_args()
    
    create_lane_labels_batch(
        images_dir=args.images_dir,
        output_dir=args.output_dir,
        carla_host=args.carla_host,
        carla_port=args.carla_port,
        image_width=args.image_width,
        image_height=args.image_height,
        fov=args.fov,
        data_csv=args.data_csv
    )


if __name__ == '__main__':
    main()

