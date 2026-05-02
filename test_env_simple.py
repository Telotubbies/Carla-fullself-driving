#!/usr/bin/env python3
"""Simple test script to verify CARLA environment works."""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

from src.carla_gym_env import CarlaEnv

def test_environment():
    """Test basic environment functionality."""
    
    print("=" * 80)
    print("Testing CARLA Gym Environment")
    print("=" * 80)
    
    # Environment configuration
    env_config = {
        'host': 'localhost',
        'port': 2000,
        'timeout': 10.0,
        'map': 'Town01',
        'delta_seconds': 0.05,
        'max_episode_steps': 100,
        'sensor_config': {
            'lidar_channels': 32,
            'lidar_range': 50,
            'bev_range': 25.0,
            'bev_resolution': 128,
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
        # Create environment
        print("\n1. Creating environment...")
        env = CarlaEnv(env_config)
        print("   ✅ Environment created successfully")
        
        # Reset environment
        print("\n2. Resetting environment...")
        obs, info = env.reset()
        print("   ✅ Environment reset successfully")
        print(f"   Observation keys: {obs.keys()}")
        print(f"   LiDAR BEV shape: {obs['lidar_bev'].shape}")
        print(f"   Ego state shape: {obs['ego_state'].shape}")
        
        # Take a few steps
        print("\n3. Taking 10 steps...")
        for step in range(10):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            print(f"   Step {step+1}: reward={reward:.2f}, terminated={terminated}, truncated={truncated}")
            
            if terminated or truncated:
                print(f"   Episode ended at step {step+1}")
                break
        
        print("\n4. Closing environment...")
        env.close()
        print("   ✅ Environment closed successfully")
        
        print("\n" + "=" * 80)
        print("✅ ALL TESTS PASSED!")
        print("=" * 80)
        
        return True
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_environment()
    sys.exit(0 if success else 1)
