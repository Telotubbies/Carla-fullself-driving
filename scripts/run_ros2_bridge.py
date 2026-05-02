#!/usr/bin/env python3
"""
ROS2 CARLA Bridge Runner
เชื่อมต่อ CARLA กับ ROS2 และ publish sensor data
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import rclpy
from rclpy.executors import MultiThreadedExecutor
from src.ros2_bridge.carla_ros_node import CarlaRosNode
from src.carla_gym_env import CarlaEnv
import time


def main():
    print("=" * 80)
    print("🤖 ROS2 CARLA Bridge")
    print("=" * 80)
    
    # สร้าง CARLA environment
    print("\n1. Connecting to CARLA...")
    env_config = {
        'host': 'localhost',
        'port': 2000,
        'timeout': 10.0,
        'map': 'Town01',
        'delta_seconds': 0.05,
        'max_episode_steps': 10000,
        'use_camera': True,
        'use_fixed_spawn': True,
        'fixed_spawn_indices': [0],
        'sensor_config': {
            'lidar_channels': 32,
            'lidar_range': 50,
            'bev_range': 25.0,
            'bev_resolution': 256,
            'camera_width': 640,
            'camera_height': 480,
            'camera_fov': 90,
        },
        'reward_config': {
            'w_progress': 1.0,
            'w_comfort': 0.1,
            'w_collision': 200.0,
            'w_lane_deviation': 0.5,
            'w_speed': 0.2,
            'target_speed': 30.0 / 3.6,
        }
    }
    
    try:
        env = CarlaEnv(env_config)
        print("   ✅ Connected to CARLA")
        
        # Reset environment
        print("\n2. Spawning vehicle...")
        obs, info = env.reset()
        print("   ✅ Vehicle spawned")
        
        # Initialize ROS2
        print("\n3. Initializing ROS2...")
        rclpy.init()
        
        # สร้าง ROS2 node
        ros_node = CarlaRosNode(env.world, env.vehicle)
        print("   ✅ ROS2 node created")
        
        # สร้าง executor
        executor = MultiThreadedExecutor()
        executor.add_node(ros_node)
        
        print("\n" + "=" * 80)
        print("✅ ROS2 CARLA Bridge Running!")
        print("=" * 80)
        print("\nPublishing topics:")
        print("  📡 /carla/ego_vehicle/lidar/point_cloud2")
        print("  📷 /carla/ego_vehicle/camera/rgb/image_raw")
        print("  🧭 /carla/ego_vehicle/odometry")
        print("  📊 /carla/ego_vehicle/imu")
        print("  🚗 /carla/ego_vehicle/velocity")
        print("\nSubscribing to:")
        print("  🎮 /carla/ego_vehicle/control")
        print("\nPress Ctrl+C to stop")
        print("=" * 80 + "\n")
        
        # รัน executor
        try:
            executor.spin()
        except KeyboardInterrupt:
            print("\n⏹️  Stopping ROS2 bridge...")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        
        if 'ros_node' in locals():
            ros_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        if 'env' in locals():
            env.close()
        
        print("✅ Cleanup complete")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
