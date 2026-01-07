"""
Capture image from CARLA for lane detection testing
"""

import sys
import os
import numpy as np
import cv2

# Add CARLA Python API to path
carla_path = os.path.join(os.path.dirname(__file__), '..', 'PythonAPI', 'carla')
if os.path.exists(carla_path):
    sys.path.insert(0, carla_path)

try:
    import carla
except ImportError:
    print("❌ CARLA Python API not found")
    sys.exit(1)


def capture_image_from_carla(port=2000, timeout=10.0):
    """Capture current image from CARLA"""
    try:
        # Connect to CARLA
        client = carla.Client('localhost', port)
        client.set_timeout(timeout)
        
        # Get world
        world = client.get_world()
        
        # Find a vehicle
        vehicles = world.get_actors().filter('vehicle.*')
        if len(vehicles) == 0:
            print("⚠️  No vehicles found, spawning one...")
            # Spawn a vehicle
            blueprint_library = world.get_blueprint_library()
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            
            # Get spawn points
            spawn_points = world.get_map().get_spawn_points()
            if len(spawn_points) == 0:
                print("❌ No spawn points available")
                return None
            
            spawn_point = spawn_points[0]
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            vehicles = [vehicle]
        
        vehicle = vehicles[0]
        print(f"✅ Found vehicle: {vehicle.type_id}")
        
        # Create camera
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '160')
        camera_bp.set_attribute('image_size_y', '90')
        camera_bp.set_attribute('fov', '110')
        
        # Attach camera to vehicle
        transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        
        print("📸 Capturing image...")
        
        # Capture image
        image_data = None
        image_captured = False
        
        def callback(image):
            nonlocal image_data, image_captured
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]  # Remove alpha channel
            image_data = array
            image_captured = True
        
        camera.listen(callback)
        
        # Wait for image
        import time
        max_wait = 5.0
        start_time = time.time()
        while not image_captured and (time.time() - start_time) < max_wait:
            time.sleep(0.1)
        
        # Cleanup
        camera.destroy()
        
        if image_data is not None:
            print(f"✅ Image captured: {image_data.shape}")
            return image_data
        else:
            print("❌ Failed to capture image")
            return None
            
    except Exception as e:
        print(f"❌ Error capturing from CARLA: {e}")
        import traceback
        traceback.print_exc()
        return None


def save_image(image, output_path):
    """Save image to file"""
    if image is None:
        return False
    
    # Convert RGB to BGR for OpenCV
    bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(output_path, bgr_image)
    print(f"✅ Image saved to {output_path}")
    return True


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Capture image from CARLA')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port')
    parser.add_argument('--output', type=str, default='carla_captured_image.png', help='Output image path')
    args = parser.parse_args()
    
    print("=" * 80)
    print("📸 Capturing Image from CARLA")
    print("=" * 80)
    print(f"Port: {args.port}")
    print(f"Output: {args.output}")
    print()
    
    image = capture_image_from_carla(port=args.port)
    
    if image is not None:
        save_image(image, args.output)
        print()
        print("✅ Success! Image captured and saved.")
        print(f"   Use: python3 test_lane_detection.py --image {args.output}")
    else:
        print()
        print("❌ Failed to capture image from CARLA")
        sys.exit(1)

