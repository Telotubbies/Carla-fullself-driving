#!/usr/bin/env python3
"""
Demo: Tesla-like Perception System
แสดงการทำงานของระบบ perception แบบ Tesla
ตรวจจับ: เลน, รถ, ไฟแดง, คน ทุกอย่าง
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import cv2
import numpy as np
import carla
from src.perception import PerceptionFusion
import time


def main():
    print("=" * 80)
    print("🚗 Tesla-like Perception System Demo")
    print("=" * 80)
    print("")
    
    # Connect to CARLA
    print("🔄 Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    print(f"✅ Connected to CARLA: {world.get_map().name}")
    
    # Spawn vehicle
    print("🔄 Spawning vehicle...")
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print("✅ Vehicle spawned")
    
    # Setup camera
    print("🔄 Setting up camera...")
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '480')
    camera_bp.set_attribute('fov', '90')
    
    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4),
        carla.Rotation(pitch=-15)
    )
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    
    # Camera data storage
    camera_data = {'image': None}
    
    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha
        camera_data['image'] = array
    
    camera.listen(camera_callback)
    print("✅ Camera ready")
    
    # Initialize Perception System
    print("")
    print("🔄 Initializing Perception System...")
    perception = PerceptionFusion(use_gpu=True, image_size=(640, 480))
    print("")
    
    # Enable autopilot for demo
    vehicle.set_autopilot(True)
    print("✅ Autopilot enabled (for demo)")
    
    print("")
    print("=" * 80)
    print("🎥 Perception Demo Running")
    print("=" * 80)
    print("Press 'q' to quit")
    print("Press 's' to save screenshot")
    print("")
    
    frame_count = 0
    fps_time = time.time()
    fps = 0.0
    
    try:
        while True:
            # Wait for camera data
            if camera_data['image'] is None:
                time.sleep(0.01)
                continue
            
            # Get camera image
            image = camera_data['image'].copy()
            
            # Process with perception system
            perception_output = perception.process(image)
            
            # Visualize
            vis = perception.visualize(image, perception_output)
            
            # Add FPS
            frame_count += 1
            if frame_count % 10 == 0:
                current_time = time.time()
                fps = 10.0 / (current_time - fps_time)
                fps_time = current_time
            
            cv2.putText(vis, f"FPS: {fps:.1f}", (10, vis.shape[0]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Display
            cv2.imshow('Tesla-like Perception System', vis)
            
            # Print status
            if frame_count % 30 == 0:
                print(f"\nFrame {frame_count}:")
                print(f"  Lane: {'✅ Detected' if perception_output.lane_detected else '❌ Lost'}")
                print(f"  Offset: {perception_output.lane_center_offset:+.2f}m")
                print(f"  Objects: {len(perception_output.objects)}")
                print(f"  Traffic Lights: {len(perception_output.traffic_lights)}")
                if perception_output.active_traffic_light:
                    print(f"  Active Light: {perception_output.active_traffic_light.state.name}")
                print(f"  Safe: {'✅ Yes' if perception_output.safe_to_proceed else '❌ No'}")
                print(f"  Recommended Speed: {perception_output.recommended_speed:.0f} km/h")
            
            # Handle keys
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("\n⏹️  Quit requested")
                break
            elif key == ord('s'):
                filename = f"perception_screenshot_{frame_count}.png"
                cv2.imwrite(filename, vis)
                print(f"💾 Screenshot saved: {filename}")
    
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()
        print("✅ Cleanup complete")


if __name__ == '__main__':
    main()
