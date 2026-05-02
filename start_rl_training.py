#!/usr/bin/env python3
"""
RL Training with SAC Algorithm
เริ่ม training จริงๆ ด้วย Soft Actor-Critic
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import torch
import yaml
from pathlib import Path

from src.carla_gym_env import CarlaEnv
from src.curriculum import CurriculumManager
from src.mlflow_integration import MLflowTracker
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback

class RLTrainingVisualizer(BaseCallback):
    """Callback สำหรับ visualization ระหว่าง training"""
    
    def __init__(self, env, update_freq=10):
        super().__init__()
        self.env = env
        self.update_freq = update_freq
        self.episode_rewards = []
        self.episode_lengths = []
        self.episode_count = 0
        
        # Setup visualization
        self.setup_visualization()
        
    def setup_visualization(self):
        """Setup matplotlib"""
        plt.ion()
        self.fig = plt.figure(figsize=(16, 10))
        
        # Camera
        self.ax_camera = plt.subplot(2, 4, 1)
        self.ax_camera.set_title('Camera View', fontsize=12, fontweight='bold')
        self.ax_camera.axis('off')
        self.img_camera = None
        
        # LiDAR
        self.ax_lidar = plt.subplot(2, 4, 2)
        self.ax_lidar.set_title('LiDAR BEV', fontsize=12, fontweight='bold')
        self.ax_lidar.axis('off')
        self.img_lidar = None
        
        # Episode Rewards
        self.ax_rewards = plt.subplot(2, 4, 3)
        self.ax_rewards.set_title('Episode Rewards', fontsize=12, fontweight='bold')
        self.ax_rewards.set_xlabel('Episode')
        self.ax_rewards.set_ylabel('Total Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        self.reward_line, = self.ax_rewards.plot([], [], 'b-', linewidth=2)
        
        # Episode Lengths
        self.ax_lengths = plt.subplot(2, 4, 4)
        self.ax_lengths.set_title('Episode Lengths', fontsize=12, fontweight='bold')
        self.ax_lengths.set_xlabel('Episode')
        self.ax_lengths.set_ylabel('Steps')
        self.ax_lengths.grid(True, alpha=0.3)
        self.length_line, = self.ax_lengths.plot([], [], 'g-', linewidth=2)
        
        # Training Info
        self.ax_info = plt.subplot(2, 4, 5)
        self.ax_info.set_title('Training Info', fontsize=12, fontweight='bold')
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.1, 0.5, '', fontsize=10, 
                                          verticalalignment='center', family='monospace')
        
        # Vehicle State
        self.ax_state = plt.subplot(2, 4, 6)
        self.ax_state.set_title('Vehicle State', fontsize=12, fontweight='bold')
        self.ax_state.axis('off')
        self.state_text = self.ax_state.text(0.1, 0.5, '', fontsize=10,
                                            verticalalignment='center', family='monospace')
        
        # Loss Plot (placeholder)
        self.ax_loss = plt.subplot(2, 4, 7)
        self.ax_loss.set_title('Learning Progress', fontsize=12, fontweight='bold')
        self.ax_loss.set_xlabel('Timestep')
        self.ax_loss.set_ylabel('Value')
        self.ax_loss.grid(True, alpha=0.3)
        
        # Success Rate
        self.ax_success = plt.subplot(2, 4, 8)
        self.ax_success.set_title('Success Rate', fontsize=12, fontweight='bold')
        self.ax_success.set_xlabel('Episode')
        self.ax_success.set_ylabel('Success %')
        self.ax_success.set_ylim(0, 100)
        self.ax_success.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
    def _on_step(self) -> bool:
        """Called at each step"""
        if self.n_calls % self.update_freq == 0:
            self.update_visualization()
        return True
    
    def _on_rollout_end(self) -> None:
        """Called at end of rollout"""
        # Get episode info
        if len(self.model.ep_info_buffer) > 0:
            ep_info = self.model.ep_info_buffer[-1]
            self.episode_rewards.append(ep_info['r'])
            self.episode_lengths.append(ep_info['l'])
            self.episode_count += 1
            
            # Update plots
            self.update_plots()
    
    def update_visualization(self):
        """Update visualization"""
        try:
            # Get current observation
            obs = self.training_env.get_attr('obs')[0]
            
            if obs is None:
                return
            
            # Camera
            if 'camera' in obs:
                if self.img_camera is None:
                    self.img_camera = self.ax_camera.imshow(obs['camera'])
                else:
                    self.img_camera.set_data(obs['camera'])
            
            # LiDAR
            if 'lidar_bev' in obs:
                if self.img_lidar is None:
                    self.img_lidar = self.ax_lidar.imshow(obs['lidar_bev'])
                else:
                    self.img_lidar.set_data(obs['lidar_bev'])
            
            # Vehicle state
            if 'ego_state' in obs:
                ego_state = obs['ego_state']
                speed = ego_state[0] * 3.6
                x, y, z = ego_state[1:4]
                
                state_str = f"""
Speed:    {speed:6.1f} km/h
Position: ({x:6.1f}, {y:6.1f})
Timestep: {self.num_timesteps}
Episode:  {self.episode_count}
                """
                self.state_text.set_text(state_str.strip())
            
            # Training info
            info_str = f"""
Algorithm: SAC
Timesteps: {self.num_timesteps}
Episodes:  {self.episode_count}
FPS:       ~20

Status: TRAINING
            """
            self.info_text.set_text(info_str.strip())
            
            # Update display
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
            
        except Exception as e:
            pass
    
    def update_plots(self):
        """Update reward and length plots"""
        if len(self.episode_rewards) > 0:
            # Rewards
            self.reward_line.set_data(range(len(self.episode_rewards)), self.episode_rewards)
            self.ax_rewards.relim()
            self.ax_rewards.autoscale_view()
            
            # Lengths
            self.length_line.set_data(range(len(self.episode_lengths)), self.episode_lengths)
            self.ax_lengths.relim()
            self.ax_lengths.autoscale_view()
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


def main():
    print("=" * 80)
    print("🎓 RL Training with SAC Algorithm")
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
        'max_episode_steps': guidelines['termination']['max_steps'],
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
    print("✅ Environment created")
    
    # Initialize MLflow
    print("\n📊 Initializing MLflow...")
    mlflow_tracker = MLflowTracker(
        experiment_name="carla_sac_training",
        tracking_uri="./mlruns"
    )
    mlflow_tracker.start_run(run_name="sac_rl_training")
    print("✅ MLflow initialized")
    
    # Create SAC agent
    print("\n🧠 Creating SAC agent...")
    model = SAC(
        "MultiInputPolicy",
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
        device='auto',
        tensorboard_log="./data/tensorboard"
    )
    print("✅ SAC agent created")
    
    # Create callback
    print("\n🎨 Setting up visualization...")
    callback = RLTrainingVisualizer(env, update_freq=10)
    print("✅ Visualization ready")
    
    # Start training
    print("\n" + "=" * 80)
    print("🚀 Starting RL Training")
    print("=" * 80)
    print("Total timesteps: 100,000")
    print("Press Ctrl+C to stop")
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
        model_path = "models/sac_carla_final.zip"
        model.save(model_path)
        print(f"💾 Model saved to {model_path}")
        
    except KeyboardInterrupt:
        print("\n⏹️  Training stopped by user")
        
        # Save model
        model_path = "models/sac_carla_interrupted.zip"
        model.save(model_path)
        print(f"💾 Model saved to {model_path}")
    
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        mlflow_tracker.end_run()
        env.close()
        plt.close('all')
        print("✅ Cleanup complete")


if __name__ == '__main__':
    main()
