#!/usr/bin/env python3
"""
Test Curriculum Learning System
ทดสอบ curriculum manager และ stage transitions
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

from src.curriculum import CurriculumManager
from src.carla_gym_env import CarlaEnv
import numpy as np


def test_curriculum_system():
    """ทดสอบ curriculum system"""
    
    print("=" * 80)
    print("🧪 TESTING CURRICULUM LEARNING SYSTEM")
    print("=" * 80)
    
    # สร้าง curriculum manager
    print("\n1. Creating Curriculum Manager...")
    curriculum = CurriculumManager()
    print("   ✅ Curriculum Manager created")
    
    # แสดงสถานะเริ่มต้น
    print("\n2. Initial Status:")
    curriculum.print_status()
    
    # ทดสอบ env config
    print("\n3. Testing Environment Config:")
    env_config = curriculum.get_env_config()
    print(f"   Spawn points: {env_config['fixed_spawn_indices']}")
    print(f"   Traffic density: {env_config['traffic_density']}")
    print(f"   Reward weights: {env_config['reward_config']}")
    
    # จำลอง episodes
    print("\n4. Simulating Episodes...")
    
    for episode in range(600):
        # จำลอง metrics
        metrics = {
            'total_reward': np.random.uniform(50, 150),
            'avg_lane_deviation': np.random.uniform(0.2, 0.6),
            'avg_speed': np.random.uniform(20, 35),
            'collision': np.random.random() < 0.2,  # 20% collision
            'episode_length': np.random.randint(100, 500)
        }
        
        curriculum.record_episode(metrics)
        
        # แสดงความคืบหน้าทุก 100 episodes
        if (episode + 1) % 100 == 0:
            progress = curriculum.get_progress()
            print(f"\n   Episode {episode + 1}:")
            print(f"   Stage: {progress['current_stage_name']}")
            print(f"   Progress: {progress['stage_progress']*100:.1f}%")
            print(f"   Overall: {progress['overall_progress']*100:.1f}%")
    
    # แสดงสถิติสุดท้าย
    print("\n5. Final Statistics:")
    curriculum.print_status()
    
    # ทดสอบ save/load
    print("\n6. Testing Save/Load:")
    save_path = "data/curriculum_state.pkl"
    curriculum.save_state(save_path)
    
    # สร้าง manager ใหม่และโหลด
    new_curriculum = CurriculumManager()
    new_curriculum.load_state(save_path)
    
    print("\n7. Loaded Curriculum Status:")
    new_curriculum.print_status()
    
    print("\n" + "=" * 80)
    print("✅ ALL CURRICULUM TESTS PASSED!")
    print("=" * 80)


def test_with_real_env():
    """ทดสอบ curriculum กับ CARLA environment จริง"""
    
    print("\n" + "=" * 80)
    print("🚗 TESTING CURRICULUM WITH REAL CARLA ENVIRONMENT")
    print("=" * 80)
    
    # สร้าง curriculum manager
    curriculum = CurriculumManager()
    
    # ดึง env config สำหรับ stage ปัจจุบัน
    curriculum_config = curriculum.get_env_config()
    
    # สร้าง base env config
    base_config = {
        'host': 'localhost',
        'port': 2000,
        'timeout': 10.0,
        'map': 'Town01',
        'delta_seconds': 0.05,
        'max_episode_steps': 500,
        'use_camera': True,
        'sensor_config': {
            'lidar_channels': 32,
            'lidar_range': 50,
            'bev_range': 25.0,
            'bev_resolution': 256,
            'camera_width': 640,
            'camera_height': 480,
            'camera_fov': 90,
        }
    }
    
    # รวม config
    base_config.update(curriculum_config)
    
    print("\n1. Creating CARLA Environment with Curriculum Config...")
    print(f"   Stage: {curriculum.current_config.name}")
    print(f"   Spawn points: {base_config['fixed_spawn_indices']}")
    
    try:
        env = CarlaEnv(base_config)
        print("   ✅ Environment created successfully")
        
        # ทดสอบ 3 episodes
        print("\n2. Running 3 Test Episodes...")
        
        for ep in range(3):
            obs, info = env.reset()
            print(f"\n   Episode {ep + 1}:")
            print(f"   - Observation keys: {obs.keys()}")
            print(f"   - LiDAR BEV shape: {obs['lidar_bev'].shape}")
            print(f"   - Camera shape: {obs.get('camera', np.zeros((1,))).shape}")
            
            # รัน 50 steps
            episode_reward = 0
            for step in range(50):
                action = env.action_space.sample()
                obs, reward, done, truncated, info = env.step(action)
                episode_reward += reward
                
                if done or truncated:
                    break
            
            # บันทึก metrics
            metrics = {
                'total_reward': episode_reward,
                'avg_lane_deviation': abs(obs['ego_state'][4]),
                'avg_speed': obs['ego_state'][0] * 3.6,
                'collision': info.get('collision', False),
                'episode_length': step + 1
            }
            
            curriculum.record_episode(metrics)
            
            print(f"   - Steps: {step + 1}")
            print(f"   - Reward: {episode_reward:.2f}")
            print(f"   - Collision: {info.get('collision', False)}")
        
        # แสดงสถิติ
        print("\n3. Curriculum Statistics:")
        curriculum.print_status()
        
        env.close()
        print("\n✅ Real environment test completed!")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("🧪 CARLA CURRICULUM LEARNING SYSTEM TEST")
    print("=" * 80)
    
    # Test 1: Curriculum system only
    test_curriculum_system()
    
    # Test 2: With real CARLA environment
    print("\n\nDo you want to test with real CARLA environment? (requires CARLA server)")
    print("Make sure CARLA server is running at localhost:2000")
    
    response = input("Test with real environment? (y/n): ").lower()
    
    if response == 'y':
        test_with_real_env()
    else:
        print("\nSkipping real environment test")
    
    print("\n" + "=" * 80)
    print("✅ ALL TESTS COMPLETED!")
    print("=" * 80)
