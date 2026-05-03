#!/usr/bin/env python3
"""
SAC Training - Simple Version
ใช้แค่ ego_state (ไม่ใช้ camera) เพื่อประหยัด memory
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import matplotlib.pyplot as plt
import torch
import yaml
from pathlib import Path
import gymnasium as gym
from gymnasium import spaces

from src.carla_gym_env import CarlaEnv
from src.mlflow_integration import MLflowTracker
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

class EgoStateWrapper(gym.Wrapper):
    """Wrapper ที่ใช้แค่ ego_state (ไม่ใช้ camera/lidar)"""
    
    def __init__(self, env):
        super().__init__(env)
        # Observation space = ego_state only (6 dimensions)
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(6,),
            dtype=np.float32
        )
    
    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        return obs['ego_state'].astype(np.float32), info
    
    def step(self, action):
        obs, reward, done, truncated, info = self.env.step(action)
        return obs['ego_state'].astype(np.float32), reward, done, truncated, info


class TrainingCallback(BaseCallback):
    """Callback สำหรับ visualization และ logging"""
    
    def __init__(self, check_freq=100):
        super().__init__()
        self.check_freq = check_freq
        self.episode_rewards = []
        self.episode_lengths = []
        self.episode_count = 0
        
        # Setup visualization
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('SAC Training Progress', fontsize=14, fontweight='bold')
        
        # Episode rewards
        self.ax_rewards = self.axes[0, 0]
        self.ax_rewards.set_title('Episode Rewards')
        self.ax_rewards.set_xlabel('Episode')
        self.ax_rewards.set_ylabel('Total Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        self.reward_line, = self.ax_rewards.plot([], [], 'b-', linewidth=2)
        
        # Episode lengths
        self.ax_lengths = self.axes[0, 1]
        self.ax_lengths.set_title('Episode Lengths')
        self.ax_lengths.set_xlabel('Episode')
        self.ax_lengths.set_ylabel('Steps')
        self.ax_lengths.grid(True, alpha=0.3)
        self.length_line, = self.ax_lengths.plot([], [], 'g-', linewidth=2)
        
        # Training info
        self.ax_info = self.axes[1, 0]
        self.ax_info.set_title('Training Info')
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.1, 0.5, '', fontsize=11, 
                                          verticalalignment='center', family='monospace')
        
        # Success rate
        self.ax_success = self.axes[1, 1]
        self.ax_success.set_title('Recent Performance')
        self.ax_success.set_xlabel('Episode')
        self.ax_success.set_ylabel('Avg Reward (last 10)')
        self.ax_success.grid(True, alpha=0.3)
        self.success_line, = self.ax_success.plot([], [], 'r-', linewidth=2)
        
        plt.tight_layout()
        
    def _on_step(self) -> bool:
        return True
    
    def _on_rollout_end(self) -> None:
        """Called at end of rollout"""
        if len(self.model.ep_info_buffer) > 0:
            for ep_info in self.model.ep_info_buffer:
                if ep_info not in [{'r': r, 'l': l} for r, l in zip(self.episode_rewards, self.episode_lengths)]:
                    self.episode_rewards.append(ep_info['r'])
                    self.episode_lengths.append(ep_info['l'])
                    self.episode_count += 1
            
            self.update_plots()
    
    def update_plots(self):
        """Update plots"""
        if len(self.episode_rewards) == 0:
            return
        
        # Rewards
        self.reward_line.set_data(range(len(self.episode_rewards)), self.episode_rewards)
        self.ax_rewards.relim()
        self.ax_rewards.autoscale_view()
        
        # Lengths
        self.length_line.set_data(range(len(self.episode_lengths)), self.episode_lengths)
        self.ax_lengths.relim()
        self.ax_lengths.autoscale_view()
        
        # Info
        avg_reward = np.mean(self.episode_rewards[-10:]) if len(self.episode_rewards) >= 10 else np.mean(self.episode_rewards)
        avg_length = np.mean(self.episode_lengths[-10:]) if len(self.episode_lengths) >= 10 else np.mean(self.episode_lengths)
        
        info_str = f"""
Algorithm:    SAC
Timesteps:    {self.num_timesteps}
Episodes:     {self.episode_count}

Recent Performance (last 10):
  Avg Reward: {avg_reward:+.2f}
  Avg Length: {avg_length:.0f}

Latest Episode:
  Reward:     {self.episode_rewards[-1]:+.2f}
  Length:     {self.episode_lengths[-1]}
        """
        self.info_text.set_text(info_str.strip())
        
        # Success rate (moving average of rewards)
        if len(self.episode_rewards) >= 10:
            moving_avg = [np.mean(self.episode_rewards[max(0, i-9):i+1]) 
                         for i in range(len(self.episode_rewards))]
            self.success_line.set_data(range(len(moving_avg)), moving_avg)
            self.ax_success.relim()
            self.ax_success.autoscale_view()
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main():
    print("=" * 80)
    print("🎓 SAC RL Training - Simple Version (ego_state only)")
    print("=" * 80)
    print("")
    
    # Load guidelines
    print("📋 Loading training guidelines...")
    guidelines_path = Path('config/training_guidelines.yaml')
    with open(guidelines_path, 'r') as f:
        guidelines = yaml.safe_load(f)
    print("✅ Guidelines loaded")
    
    # Create environment
    print("\n🔄 Creating CARLA environment...")
    env_config = {
        'host': 'localhost',
        'port': 2000,
        'timeout': 10.0,
        'map': 'Town01',
        'delta_seconds': 0.05,
        'max_episode_steps': 1000,
        'use_camera': False,  # ไม่ใช้ camera เพื่อประหยัด memory
        'use_fixed_spawn': True,
        'fixed_spawn_indices': [0, 1, 2],
    }
    
    env = CarlaEnv(env_config)
    env = EgoStateWrapper(env)  # Wrap เพื่อใช้แค่ ego_state
    print("✅ Environment created (using ego_state only)")
    
    # Initialize MLflow
    print("\n📊 Initializing MLflow...")
    mlflow_tracker = MLflowTracker(
        experiment_name="carla_sac_training",
        tracking_uri="./mlruns"
    )
    mlflow_tracker.start_run(run_name="sac_ego_state")
    print("✅ MLflow initialized")
    
    # Create SAC agent
    print("\n🧠 Creating SAC agent...")
    model = SAC(
        "MlpPolicy",  # MLP สำหรับ vector observation
        env,
        learning_rate=3e-4,
        buffer_size=100000,
        learning_starts=1000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        ent_coef='auto',
        verbose=1,
        device='cuda' if torch.cuda.is_available() else 'cpu',
        tensorboard_log="./data/tensorboard"
    )
    print(f"✅ SAC agent created (device: {model.device})")
    
    # Create callback
    print("\n🎨 Setting up visualization...")
    callback = TrainingCallback(check_freq=100)
    print("✅ Visualization ready")
    
    # Start training
    print("\n" + "=" * 80)
    print("🚀 Starting SAC Training")
    print("=" * 80)
    print("Total timesteps: 100,000")
    print("Observation: ego_state (6D vector)")
    print("  [speed, x, y, z, lateral_offset, heading_error]")
    print("")
    print("Press Ctrl+C to stop and save model")
    print("")
    
    try:
        model.learn(
            total_timesteps=100000,
            callback=callback,
            log_interval=10,
            progress_bar=True
        )
        
        print("\n✅ Training complete!")
        
        # Save model
        Path("models").mkdir(exist_ok=True)
        model_path = "models/sac_carla_final.zip"
        model.save(model_path)
        print(f"💾 Model saved to {model_path}")
        
        print("\nClose matplotlib window to exit...")
        plt.show(block=True)
        
    except KeyboardInterrupt:
        print("\n⏹️  Training stopped by user")
        
        # Save model
        Path("models").mkdir(exist_ok=True)
        model_path = "models/sac_carla_interrupted.zip"
        model.save(model_path)
        print(f"💾 Model saved to {model_path}")
    
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        mlflow_tracker.end_run()
        env.close()
        print("✅ Cleanup complete")


if __name__ == '__main__':
    main()
