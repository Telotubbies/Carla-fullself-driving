#!/usr/bin/env python3
"""
RLlib Training with Full System Integration
ใช้ทุกระบบที่เขียนไว้แล้ว:
- Perception (Lane, Object, Traffic Light Detection)
- Training Guidelines
- Curriculum Learning
- Expert Controller
- MLflow Tracking
- Enhanced Rewards
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import yaml
from pathlib import Path
import ray
from ray import tune
from ray.rllib.algorithms.sac import SACConfig
from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
import gymnasium as gym
from gymnasium import spaces

from src.carla_gym_env import CarlaEnv
from src.perception import PerceptionFusion
from src.curriculum import CurriculumManager
from src.carla_gym_env.enhanced_rewards import EnhancedRewardCalculator
from src.mlflow_integration import MLflowTracker, MetricsLogger
from src.sac_trainer.callbacks import CarlaCallbacks

class FullSystemWrapper(gym.Wrapper):
    """
    Wrapper ที่รวมทุกระบบเข้าด้วยกัน:
    - Perception system
    - Enhanced rewards
    - Curriculum learning
    """
    
    def __init__(self, env, guidelines_path='config/training_guidelines.yaml'):
        super().__init__(env)
        
        print("🔧 Initializing Full System Wrapper...")
        
        # Load guidelines
        with open(guidelines_path, 'r') as f:
            self.guidelines = yaml.safe_load(f)
        
        # Initialize perception system
        print("  🎯 Loading Perception System...")
        self.perception = PerceptionFusion(
            use_gpu=True,
            image_size=(640, 480)
        )
        
        # Initialize enhanced reward calculator
        print("  💰 Loading Enhanced Reward Calculator...")
        self.reward_calculator = EnhancedRewardCalculator(guidelines_path)
        
        # Initialize curriculum manager
        print("  📚 Loading Curriculum Manager...")
        self.curriculum = CurriculumManager()
        
        # Metrics
        self.episode_step = 0
        self.episode_reward = 0.0
        self.perception_outputs = []
        
        # Extended observation space with perception
        self.observation_space = spaces.Dict({
            'ego_state': spaces.Box(
                low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
            ),
            'perception': spaces.Box(
                low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32
            )
        })
        
        print("✅ Full System Wrapper initialized")
    
    def reset(self, **kwargs):
        """Reset with curriculum stage update"""
        obs, info = self.env.reset(**kwargs)
        
        # Update curriculum if needed
        if hasattr(self, 'episode_reward'):
            success = self.episode_reward > 0
            self.curriculum.update(success, self.episode_step)
        
        self.episode_step = 0
        self.episode_reward = 0.0
        self.perception_outputs = []
        
        # Process perception
        perception_features = self._process_perception(obs)
        
        # Enhanced observation
        enhanced_obs = {
            'ego_state': obs['ego_state'].astype(np.float32),
            'perception': perception_features
        }
        
        info['curriculum_stage'] = self.curriculum.current_config.name
        
        return enhanced_obs, info
    
    def step(self, action):
        """Step with enhanced rewards and perception"""
        obs, reward, done, truncated, info = self.env.step(action)
        
        # Process perception
        perception_features = self._process_perception(obs)
        perception_output = self.perception_outputs[-1] if self.perception_outputs else None
        
        # Calculate enhanced reward
        enhanced_reward = self.reward_calculator.calculate_reward(
            obs=obs,
            action=action,
            info=info,
            perception_output=perception_output
        )
        
        # Enhanced observation
        enhanced_obs = {
            'ego_state': obs['ego_state'].astype(np.float32),
            'perception': perception_features
        }
        
        # Update metrics
        self.episode_step += 1
        self.episode_reward += enhanced_reward
        
        # Add perception info
        if perception_output:
            info['perception'] = {
                'lane_detected': perception_output.lane_detected,
                'objects_count': len(perception_output.objects),
                'safe_to_proceed': perception_output.safe_to_proceed,
                'recommended_speed': perception_output.recommended_speed
            }
        
        info['curriculum_stage'] = self.curriculum.current_config.name
        info['enhanced_reward'] = enhanced_reward
        info['original_reward'] = reward
        
        return enhanced_obs, enhanced_reward, done, truncated, info
    
    def _process_perception(self, obs):
        """Process perception and extract features"""
        if 'camera' not in obs:
            return np.zeros(10, dtype=np.float32)
        
        try:
            # Run perception
            perception_output = self.perception.process(obs['camera'])
            self.perception_outputs.append(perception_output)
            
            # Extract features
            features = np.array([
                1.0 if perception_output.lane_detected else 0.0,
                perception_output.lane_center_offset,
                perception_output.lane_heading_error,
                len(perception_output.objects),
                len(perception_output.objects_in_path),
                perception_output.closest_vehicle.distance if perception_output.closest_vehicle else 100.0,
                1.0 if perception_output.should_stop_for_light else 0.0,
                1.0 if perception_output.collision_warning else 0.0,
                1.0 if perception_output.safe_to_proceed else 0.0,
                perception_output.recommended_speed / 50.0  # Normalize
            ], dtype=np.float32)
            
            return features
            
        except Exception as e:
            print(f"⚠️  Perception error: {e}")
            return np.zeros(10, dtype=np.float32)


def make_env(config):
    """Create environment with full system"""
    env_config = {
        'host': config.get('host', 'localhost'),
        'port': config.get('port', 2000),
        'timeout': 10.0,
        'map': 'Town01',
        'delta_seconds': 0.05,
        'max_episode_steps': 1000,
        'use_camera': True,
        'use_fixed_spawn': True,
        'fixed_spawn_indices': [0, 1, 2],
        'sensor_config': {
            'camera_width': 640,
            'camera_height': 480,
            'lidar_channels': 32,
            'lidar_range': 50,
            'bev_range': 25.0,
            'bev_resolution': 256,
        }
    }
    
    env = CarlaEnv(env_config)
    env = FullSystemWrapper(env)
    
    return env


def main():
    print("=" * 80)
    print("🚀 RLlib Training with Full System Integration")
    print("=" * 80)
    print("")
    print("Systems:")
    print("  ✅ Perception (Lane + Object + Traffic Light)")
    print("  ✅ Training Guidelines")
    print("  ✅ Curriculum Learning")
    print("  ✅ Enhanced Rewards")
    print("  ✅ MLflow Tracking")
    print("  ✅ RLlib SAC")
    print("")
    
    # Initialize Ray
    print("🔄 Initializing Ray...")
    ray.init(ignore_reinit_error=True, num_gpus=1)
    print("✅ Ray initialized")
    
    # Initialize MLflow
    print("\n📊 Initializing MLflow...")
    mlflow_tracker = MLflowTracker(
        experiment_name="carla_rllib_full_system",
        tracking_uri="./mlruns"
    )
    mlflow_tracker.start_run(run_name="rllib_sac_full")
    print("✅ MLflow initialized")
    
    # Configure SAC
    print("\n🧠 Configuring SAC...")
    config = (
        SACConfig()
        .environment(
            env=make_env,
            env_config={
                'host': 'localhost',
                'port': 2000
            }
        )
        .framework("torch")
        .training(
            actor_lr=3e-4,
            critic_lr=3e-4,
            entropy_coeff=0.2,
            target_entropy="auto",
            tau=0.005,
            gamma=0.99,
            train_batch_size=256,
            replay_buffer_config={
                "type": "MultiAgentReplayBuffer",
                "capacity": 100000,
            }
        )
        .rollouts(
            num_rollout_workers=1,
            num_envs_per_worker=1,
        )
        .resources(
            num_gpus=1,
        )
        .callbacks(CarlaCallbacks)
        .debugging(seed=42)
    )
    
    print("✅ SAC configured")
    
    # Build algorithm
    print("\n🔨 Building SAC algorithm...")
    algo = config.build()
    print("✅ Algorithm built")
    
    # Training loop
    print("\n" + "=" * 80)
    print("🎓 Starting Training")
    print("=" * 80)
    print("Iterations: 100")
    print("Press Ctrl+C to stop and save")
    print("")
    
    try:
        for i in range(100):
            print(f"\n{'='*60}")
            print(f"Iteration {i+1}/100")
            print(f"{'='*60}")
            
            # Train
            result = algo.train()
            
            # Print metrics
            print(f"\n📊 Metrics:")
            print(f"  Episode Reward Mean:  {result.get('episode_reward_mean', 0):.2f}")
            print(f"  Episode Length Mean:  {result.get('episode_len_mean', 0):.0f}")
            print(f"  Episodes This Iter:   {result.get('episodes_this_iter', 0)}")
            print(f"  Timesteps Total:      {result.get('timesteps_total', 0)}")
            
            # Log to MLflow
            mlflow_tracker.log_metrics({
                'episode_reward_mean': result.get('episode_reward_mean', 0),
                'episode_len_mean': result.get('episode_len_mean', 0),
                'timesteps_total': result.get('timesteps_total', 0),
            }, step=i)
            
            # Save checkpoint every 10 iterations
            if (i + 1) % 10 == 0:
                checkpoint_dir = algo.save(checkpoint_dir="./checkpoints")
                print(f"\n💾 Checkpoint saved: {checkpoint_dir}")
        
        print("\n✅ Training complete!")
        
    except KeyboardInterrupt:
        print("\n⏹️  Training stopped by user")
    
    finally:
        # Save final model
        print("\n💾 Saving final model...")
        checkpoint_dir = algo.save(checkpoint_dir="./checkpoints/final")
        print(f"✅ Model saved: {checkpoint_dir}")
        
        # Cleanup
        print("\n🧹 Cleaning up...")
        algo.stop()
        mlflow_tracker.end_run()
        ray.shutdown()
        print("✅ Cleanup complete")


if __name__ == '__main__':
    main()
