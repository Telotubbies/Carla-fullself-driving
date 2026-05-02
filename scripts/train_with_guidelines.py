#!/usr/bin/env python3
"""
Training Script with Guidelines + Full System
เทรน SAC agent ด้วย:
- Training Guidelines
- Curriculum Learning
- Enhanced Rewards
- Perception System (Lane + Object + Traffic Light)
- Expert Controller
- MLflow Tracking
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import yaml
from pathlib import Path
from src.carla_gym_env import CarlaEnv
from src.carla_gym_env.enhanced_rewards import EnhancedRewardCalculator
from src.curriculum import CurriculumManager
from src.mlflow_integration import MLflowTracker, MetricsLogger
from src.imitation import ExpertController
from src.perception import PerceptionFusion
import matplotlib.pyplot as plt
import time


class GuidedTrainer:
    """Trainer ที่ใช้ training guidelines"""
    
    def __init__(self, config_path: str = "config/training_guidelines.yaml"):
        # โหลด guidelines
        with open(config_path, 'r') as f:
            self.guidelines = yaml.safe_load(f)
        
        # สร้าง curriculum manager
        self.curriculum = CurriculumManager()
        
        # สร้าง MLflow tracker
        self.mlflow = MLflowTracker(
            experiment_name="carla_guided_training",
            run_name=f"guided_run_{int(time.time())}"
        )
        
        # สร้าง metrics logger
        self.metrics = MetricsLogger(window_size=100)
        
        # Episode tracking
        self.total_episodes = 0
        self.total_steps = 0
        
    def create_env(self, stage: str = "stage1"):
        """สร้าง environment ตาม curriculum stage"""
        
        # ดึง curriculum config
        curriculum_config = self.curriculum.get_env_config()
        
        # Get current stage name from curriculum
        current_stage_name = self.curriculum.current_config.name
        
        # Map stage name to guidelines key
        stage_map = {
            'basic_control': 'stage1',
            'navigation': 'stage2',
            'complex_scenarios': 'stage3'
        }
        guidelines_stage = stage_map.get(current_stage_name, 'stage1')
        stage_config = self.guidelines['curriculum'][guidelines_stage]
        
        # สร้าง base config
        env_config = {
            'host': 'localhost',
            'port': 2000,
            'timeout': 10.0,
            'map': 'Town01',
            'delta_seconds': 0.05,
            'max_episode_steps': self.guidelines['termination']['max_steps'],
            'use_camera': True,
            'use_fixed_spawn': True,
            'fixed_spawn_indices': stage_config['spawn_points'],
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
        
        # รวม curriculum config
        env_config.update(curriculum_config)
        
        return CarlaEnv(env_config)
    
    def run_episode(self, env, reward_calculator, use_expert: bool = False):
        """รัน 1 episode พร้อม guidelines"""
        
        # Reset
        obs, info = env.reset()
        reward_calculator.reset()
        
        # Expert controller (ถ้าใช้)
        expert = None
        if use_expert:
            expert = ExpertController(
                target_speed=self.guidelines['speed']['target_speed'] / 3.6
            )
        
        # Episode data
        episode_reward = 0
        episode_length = 0
        distance_traveled = 0
        collision_count = 0
        
        # Reward components tracking
        reward_components = {
            'progress': [],
            'lane_keeping': [],
            'speed_tracking': [],
            'smooth_control': [],
            'heading_alignment': [],
        }
        
        done = False
        truncated = False
        
        while not (done or truncated):
            # Get action
            if expert:
                action = expert.get_action(env.vehicle, env.map)
            else:
                # Random action (จะเปลี่ยนเป็น policy ภายหลัง)
                action = env.action_space.sample()
            
            # Execute action
            next_obs, _, done, truncated, info = env.step(action)
            
            # คำนวณ reward ด้วย enhanced calculator
            ego_state = obs['ego_state']
            speed = ego_state[0]  # m/s
            steering = action[0]
            throttle = action[1]
            brake = action[2]
            lateral_offset = ego_state[4]
            heading_error = ego_state[5]
            
            # คำนวณ progress
            current_location = env.vehicle.get_location()
            if episode_length > 0:
                prev_location = env.prev_location
                progress = np.sqrt(
                    (current_location.x - prev_location.x)**2 +
                    (current_location.y - prev_location.y)**2
                )
                distance_traveled += progress
            else:
                progress = 0
            
            # คำนวณ acceleration
            acceleration = (speed - reward_calculator.prev_speed) / env.delta_seconds
            
            # คำนวณ reward components
            rewards = reward_calculator.calculate_reward(
                speed=speed,
                steering=steering,
                throttle=throttle,
                brake=brake,
                lateral_offset=lateral_offset,
                heading_error=heading_error,
                progress=progress,
                collision=info.get('collision', False),
                acceleration=acceleration
            )
            
            # เก็บ reward components
            for key in reward_components.keys():
                if key in rewards:
                    reward_components[key].append(rewards[key])
            
            # Update episode stats
            episode_reward += rewards['total']
            episode_length += 1
            
            if info.get('collision', False):
                collision_count += 1
            
            # ตรวจสอบการจบ episode ตาม guidelines
            should_terminate, reason = reward_calculator.should_terminate(
                lateral_offset=lateral_offset,
                heading_error=heading_error,
                speed=speed,
                steps=episode_length,
                collision=info.get('collision', False)
            )
            
            if should_terminate:
                done = True
                info['termination_reason'] = reason
            
            obs = next_obs
        
        # ตรวจสอบความสำเร็จ
        success = reward_calculator.is_success(
            distance_traveled=distance_traveled,
            collision_count=collision_count
        )
        
        # สร้าง episode summary
        episode_summary = {
            'total_reward': episode_reward,
            'episode_length': episode_length,
            'distance_traveled': distance_traveled,
            'collision_count': collision_count,
            'success': success,
            'avg_lane_deviation': np.mean([abs(ego_state[4]) for _ in range(episode_length)]),
            'avg_speed': np.mean([ego_state[0] * 3.6 for _ in range(episode_length)]),
            'termination_reason': info.get('termination_reason', 'max_steps'),
        }
        
        # เพิ่ม reward components averages
        for key, values in reward_components.items():
            if values:
                episode_summary[f'avg_{key}_reward'] = np.mean(values)
        
        return episode_summary
    
    def train(self, num_episodes: int = 1000, use_expert: bool = False):
        """เริ่มการเทรน"""
        
        print("=" * 80)
        print("🎓 GUIDED TRAINING WITH GUIDELINES")
        print("=" * 80)
        print(f"Total episodes: {num_episodes}")
        print(f"Use expert: {use_expert}")
        print(f"Guidelines: config/training_guidelines.yaml")
        print("=" * 80 + "\n")
        
        # เริ่ม MLflow run
        self.mlflow.start_run(tags={
            'training_type': 'guided',
            'use_expert': str(use_expert),
            'curriculum': 'enabled'
        })
        
        # Log parameters
        self.mlflow.log_params({
            'num_episodes': num_episodes,
            'use_expert': use_expert,
            'curriculum_enabled': True,
        })
        
        try:
            # สร้าง environment
            current_stage = self.curriculum.current_config.name
            env = self.create_env(current_stage)
            
            # สร้าง reward calculator
            reward_calc = EnhancedRewardCalculator(
                guidelines_path="config/training_guidelines.yaml",
                stage=f"stage{self.curriculum.current_stage_idx + 1}"
            )
            
            print(f"✅ Environment created for stage: {current_stage}\n")
            
            # Training loop
            for episode in range(num_episodes):
                self.total_episodes += 1
                
                # รัน episode
                summary = self.run_episode(env, reward_calc, use_expert)
                
                # Log metrics
                self.metrics.log_dict({
                    'episode_reward': summary['total_reward'],
                    'episode_length': summary['episode_length'],
                    'distance_traveled': summary['distance_traveled'],
                    'collision_count': summary['collision_count'],
                    'success_rate': 1.0 if summary['success'] else 0.0,
                })
                
                # Log to MLflow
                self.mlflow.log_episode_metrics(
                    episode=self.total_episodes,
                    reward=summary['total_reward'],
                    length=summary['episode_length'],
                    collision=summary['collision_count'] > 0,
                    success=summary['success'],
                    distance=summary['distance_traveled'],
                    avg_speed=summary['avg_speed'],
                    avg_lane_dev=summary['avg_lane_deviation']
                )
                
                # บันทึกลง curriculum
                curriculum_metrics = {
                    'total_reward': summary['total_reward'],
                    'avg_lane_deviation': summary['avg_lane_deviation'],
                    'avg_speed': summary['avg_speed'],
                    'collision': summary['collision_count'] > 0,
                    'episode_length': summary['episode_length']
                }
                self.curriculum.record_episode(curriculum_metrics)
                
                # แสดงความคืบหน้า
                if (episode + 1) % 10 == 0:
                    print(f"\nEpisode {episode + 1}/{num_episodes}:")
                    print(f"  Reward: {summary['total_reward']:.2f}")
                    print(f"  Length: {summary['episode_length']}")
                    print(f"  Distance: {summary['distance_traveled']:.1f}m")
                    print(f"  Success: {'✅' if summary['success'] else '❌'}")
                    print(f"  Reason: {summary['termination_reason']}")
                    
                    # แสดง curriculum progress
                    progress = self.curriculum.get_progress()
                    print(f"  Stage: {progress['current_stage_name']}")
                    print(f"  Progress: {progress['stage_progress']*100:.1f}%")
                
                # ตรวจสอบ curriculum transition
                if not self.curriculum.is_final_stage():
                    old_stage = self.curriculum.current_stage_idx
                    
                    # Curriculum จะ transition อัตโนมัติ
                    if self.curriculum.current_stage_idx != old_stage:
                        # Stage เปลี่ยน - สร้าง env ใหม่
                        print(f"\n🎓 Curriculum transition detected!")
                        env.close()
                        
                        current_stage = self.curriculum.current_config.name
                        env = self.create_env(current_stage)
                        
                        # อัพเดท reward calculator
                        reward_calc = EnhancedRewardCalculator(
                            guidelines_path="config/training_guidelines.yaml",
                            stage=f"stage{self.curriculum.current_stage_idx + 1}"
                        )
                        
                        print(f"✅ New environment created for stage: {current_stage}\n")
                
                # Save checkpoint ทุก 100 episodes
                if (episode + 1) % 100 == 0:
                    self.curriculum.save_state(f"data/curriculum_ep{episode+1}.pkl")
                    print(f"💾 Checkpoint saved at episode {episode+1}")
            
            # Training complete
            print("\n" + "=" * 80)
            print("✅ TRAINING COMPLETE!")
            print("=" * 80)
            
            # แสดงสถิติสุดท้าย
            self.metrics.print_summary()
            self.curriculum.print_status()
            
        except KeyboardInterrupt:
            print("\n⏹️  Training interrupted by user")
        
        except Exception as e:
            print(f"\n❌ Error during training: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Cleanup
            if 'env' in locals():
                env.close()
            
            self.mlflow.end_run()
            print("\n✅ Cleanup complete")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Train with guidelines')
    parser.add_argument('--episodes', type=int, default=1000, help='Number of episodes')
    parser.add_argument('--expert', action='store_true', help='Use expert controller')
    
    args = parser.parse_args()
    
    # สร้าง trainer
    trainer = GuidedTrainer()
    
    # เริ่มเทรน
    trainer.train(num_episodes=args.episodes, use_expert=args.expert)


if __name__ == "__main__":
    main()
