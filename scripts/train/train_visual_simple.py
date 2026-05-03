#!/usr/bin/env python3
"""
Training with Simple Visualization
แสดง visualization real-time:
- Camera view
- LiDAR BEV
- Training metrics (rewards, success rate)
- Vehicle state
- Actions
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import yaml
from pathlib import Path
import time
import cv2

from src.carla_gym_env import CarlaEnv
from src.curriculum import CurriculumManager
from src.mlflow_integration import MLflowTracker
from src.imitation import ExpertController


class SimpleVisualTrainer:
    """Trainer with simple real-time visualization"""
    
    def __init__(self):
        print("=" * 80)
        print("🎨 TRAINING WITH VISUALIZATION")
        print("=" * 80)
        print("")
        
        # Load guidelines
        with open('config/training_guidelines.yaml', 'r') as f:
            self.guidelines = yaml.safe_load(f)
        
        # Initialize curriculum
        self.curriculum = CurriculumManager()
        
        # Initialize MLflow
        self.mlflow = MLflowTracker(
            experiment_name="carla_visual_training",
            run_name=f"visual_run_{int(time.time())}"
        )
        self.mlflow.start_run()
        
        # Create environment
        print("🔄 Creating environment...")
        curriculum_config = self.curriculum.get_env_config()
        env_config = {
            'host': 'localhost',
            'port': 2000,
            'timeout': 10.0,
            'map': 'Town01',
            'delta_seconds': 0.05,
            'max_episode_steps': 1000,
            'use_camera': True,
            'use_fixed_spawn': True,
            'fixed_spawn_indices': curriculum_config.get('fixed_spawn_indices', [0, 1, 2]),
            'sensor_config': {
                'camera_width': 640,
                'camera_height': 480,
                'lidar_channels': 32,
                'lidar_range': 50,
                'bev_range': 25.0,
                'bev_resolution': 256,
            }
        }
        
        self.env = CarlaEnv(env_config)
        print("✅ Environment created")
        
        # Initialize expert controller
        print("🤖 Initializing expert controller...")
        self.expert = ExpertController(
            target_speed=30.0 / 3.6,
            lateral_kp=1.0,
            lateral_ki=0.0,
            lateral_kd=0.1,
            longitudinal_kp=0.5,
            longitudinal_ki=0.0,
            longitudinal_kd=0.1
        )
        print("✅ Expert ready")
        
        # Setup visualization
        self.setup_visualization()
        
        # Metrics
        self.episode_rewards = []
        self.episode_lengths = []
        self.success_rates = []
        self.episode_count = 0
        
    def setup_visualization(self):
        """Setup matplotlib visualization"""
        plt.ion()
        self.fig = plt.figure(figsize=(18, 10))
        gs = GridSpec(3, 3, figure=self.fig, hspace=0.3, wspace=0.3)
        
        # Row 1: Camera and LiDAR
        self.ax_camera = self.fig.add_subplot(gs[0, 0:2])
        self.ax_camera.set_title('🎥 Camera View', fontsize=14, fontweight='bold')
        self.ax_camera.axis('off')
        self.img_camera = None
        
        self.ax_lidar = self.fig.add_subplot(gs[0, 2])
        self.ax_lidar.set_title('📡 LiDAR BEV', fontsize=14, fontweight='bold')
        self.ax_lidar.axis('off')
        self.img_lidar = None
        
        # Row 2: Metrics
        self.ax_rewards = self.fig.add_subplot(gs[1, 0])
        self.ax_rewards.set_title('📊 Episode Rewards', fontsize=12, fontweight='bold')
        self.ax_rewards.set_xlabel('Episode')
        self.ax_rewards.set_ylabel('Total Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        self.reward_line, = self.ax_rewards.plot([], [], 'b-', linewidth=2, label='Reward')
        self.reward_ma_line, = self.ax_rewards.plot([], [], 'r--', linewidth=2, label='MA(10)')
        self.ax_rewards.legend(loc='upper left', fontsize=8)
        
        self.ax_lengths = self.fig.add_subplot(gs[1, 1])
        self.ax_lengths.set_title('📏 Episode Lengths', fontsize=12, fontweight='bold')
        self.ax_lengths.set_xlabel('Episode')
        self.ax_lengths.set_ylabel('Steps')
        self.ax_lengths.grid(True, alpha=0.3)
        self.length_line, = self.ax_lengths.plot([], [], 'g-', linewidth=2)
        
        self.ax_success = self.fig.add_subplot(gs[1, 2])
        self.ax_success.set_title('✅ Success Rate', fontsize=12, fontweight='bold')
        self.ax_success.set_xlabel('Episode')
        self.ax_success.set_ylabel('Success %')
        self.ax_success.set_ylim(0, 100)
        self.ax_success.grid(True, alpha=0.3)
        self.success_line, = self.ax_success.plot([], [], 'orange', linewidth=2)
        
        # Row 3: State and Actions
        self.ax_state = self.fig.add_subplot(gs[2, 0])
        self.ax_state.set_title('🚗 Vehicle State', fontsize=12, fontweight='bold')
        self.ax_state.axis('off')
        self.state_text = self.ax_state.text(0.05, 0.5, '', fontsize=11, 
                                            verticalalignment='center', family='monospace')
        
        self.ax_actions = self.fig.add_subplot(gs[2, 1])
        self.ax_actions.set_title('🎮 Current Actions', fontsize=12, fontweight='bold')
        
        self.ax_info = self.fig.add_subplot(gs[2, 2])
        self.ax_info.set_title('ℹ️  Training Info', fontsize=12, fontweight='bold')
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.5, '', fontsize=11,
                                          verticalalignment='center', family='monospace')
        
        plt.tight_layout()
        
    def train(self, num_episodes=100):
        """Train with visualization"""
        print("\n" + "=" * 80)
        print("🚀 Starting Training with Visualization")
        print("=" * 80)
        print(f"Episodes: {num_episodes}")
        print("Press Ctrl+C to stop")
        print("")
        
        try:
            for episode in range(num_episodes):
                self.run_episode(episode)
                
        except KeyboardInterrupt:
            print("\n⏹️  Training stopped by user")
        finally:
            self.cleanup()
    
    def run_episode(self, episode_num):
        """Run one episode with visualization"""
        obs, info = self.env.reset()
        done = False
        truncated = False
        step = 0
        episode_reward = 0.0
        
        print(f"\n🎬 Episode {episode_num + 1} starting...")
        
        while not done and not truncated and step < 1000:
            # Get expert action
            try:
                action = self.expert.get_action(
                    self.env.vehicle,
                    self.env.map,
                    dt=0.05
                )
            except Exception as e:
                # Fallback: simple forward action
                action = np.array([0.0, 0.5, 0.0])
            
            # Step environment
            next_obs, reward, done, truncated, info = self.env.step(action)
            
            episode_reward += reward
            
            # Update visualization every 3 steps
            if step % 3 == 0:
                self.update_visualization(obs, action, episode_reward, step, episode_num, info)
            
            obs = next_obs
            step += 1
        
        # Episode complete
        self.episode_count += 1
        self.episode_rewards.append(episode_reward)
        self.episode_lengths.append(step)
        
        # Calculate success
        success = not info.get('collision', False) and step > 100
        if len(self.success_rates) >= 10:
            recent_successes = [not info.get('collision', False) for _ in range(min(10, len(self.success_rates)))]
            recent_success = sum(recent_successes) / len(recent_successes)
        else:
            recent_success = float(success)
        self.success_rates.append(recent_success * 100)
        
        # Update curriculum
        self.curriculum.record_episode({
            'total_reward': episode_reward,
            'collision': info.get('collision', False),
            'episode_length': step,
            'avg_speed': info.get('speed', 0),
            'avg_lane_deviation': 0.5
        })
        
        # Log to MLflow
        self.mlflow.log_metrics({
            'episode_reward': episode_reward,
            'episode_length': step,
            'success': float(success),
            'curriculum_stage': self.curriculum.current_stage_idx
        }, step=episode_num)
        
        # Update final plots
        self.update_plots()
        
        status = "✅ Success" if success else "❌ Failed"
        reason = "collision" if info.get('collision', False) else "timeout" if truncated else "stuck"
        print(f"{status} Episode {episode_num + 1}: "
              f"Reward={episode_reward:.2f}, Steps={step}, Reason={reason}")
    
    def update_visualization(self, obs, action, episode_reward, step, episode_num, info):
        """Update all visualization panels"""
        # Camera view
        if 'camera' in obs and obs['camera'] is not None:
            camera_img = obs['camera'].copy()
            
            # Add simple overlay
            h, w = camera_img.shape[:2]
            
            # Speed overlay
            speed = obs['ego_state'][0] * 3.6
            cv2.putText(camera_img, f"Speed: {speed:.1f} km/h", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Step overlay
            cv2.putText(camera_img, f"Step: {step}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Reward overlay
            cv2.putText(camera_img, f"Reward: {episode_reward:.1f}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Center crosshair
            cv2.line(camera_img, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 255), 2)
            cv2.line(camera_img, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 255), 2)
            
            if self.img_camera is None:
                self.img_camera = self.ax_camera.imshow(camera_img)
            else:
                self.img_camera.set_data(camera_img)
        
        # LiDAR BEV
        if 'lidar_bev' in obs:
            if self.img_lidar is None:
                self.img_lidar = self.ax_lidar.imshow(obs['lidar_bev'])
            else:
                self.img_lidar.set_data(obs['lidar_bev'])
        
        # Vehicle state
        ego_state = obs['ego_state']
        speed = ego_state[0] * 3.6
        heading = np.degrees(ego_state[1]) if len(ego_state) > 1 else 0
        steering = ego_state[2] if len(ego_state) > 2 else 0
        lateral_offset = ego_state[4] if len(ego_state) > 4 else 0.0
        heading_error = np.degrees(ego_state[5]) if len(ego_state) > 5 else 0.0
        
        state_str = f"""
Speed:      {speed:7.1f} km/h
Heading:    {heading:7.1f}°
Steering:   {steering:+7.3f}
Lateral:    {lateral_offset:+7.3f} m
Head Error: {heading_error:+7.1f}°

Step:       {step:7d}
Reward:     {episode_reward:+9.2f}
        """
        self.state_text.set_text(state_str.strip())
        
        # Actions
        action_labels = ['Steer', 'Throttle', 'Brake']
        colors = ['#3498db', '#2ecc71', '#e74c3c']
        
        self.ax_actions.clear()
        self.ax_actions.set_title('🎮 Current Actions', fontsize=12, fontweight='bold')
        bars = self.ax_actions.bar(action_labels, action, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
        self.ax_actions.set_ylim(-1, 1)
        self.ax_actions.axhline(y=0, color='k', linestyle='-', linewidth=1)
        self.ax_actions.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bar, val in zip(bars, action):
            height = bar.get_height()
            self.ax_actions.text(bar.get_x() + bar.get_width()/2., height,
                               f'{val:.2f}', ha='center', va='bottom' if height > 0 else 'top',
                               fontsize=9, fontweight='bold')
        
        # Training info
        progress = self.curriculum.get_progress()
        avg_reward = np.mean(self.episode_rewards[-10:]) if self.episode_rewards else 0
        avg_length = np.mean(self.episode_lengths[-10:]) if self.episode_lengths else 0
        success_rate = self.success_rates[-1] if self.success_rates else 0
        
        info_str = f"""
Episode:    {episode_num + 1:7d}
Stage:      {progress['current_stage_name']}
Progress:   {progress['stage_progress']*100:6.1f}%
Overall:    {progress['overall_progress']*100:6.1f}%

Avg Reward: {avg_reward:+9.2f}
Avg Length: {avg_length:7.0f}
Success:    {success_rate:6.1f}%
        """
        self.info_text.set_text(info_str.strip())
        
        # Update display
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def update_plots(self):
        """Update metric plots"""
        if not self.episode_rewards:
            return
        
        episodes = range(len(self.episode_rewards))
        
        # Rewards
        self.reward_line.set_data(episodes, self.episode_rewards)
        if len(self.episode_rewards) >= 10:
            ma = np.convolve(self.episode_rewards, np.ones(10)/10, mode='valid')
            self.reward_ma_line.set_data(range(len(ma)), ma)
        self.ax_rewards.relim()
        self.ax_rewards.autoscale_view()
        
        # Lengths
        self.length_line.set_data(episodes, self.episode_lengths)
        self.ax_lengths.relim()
        self.ax_lengths.autoscale_view()
        
        # Success rate
        if self.success_rates:
            self.success_line.set_data(range(len(self.success_rates)), self.success_rates)
            self.ax_success.relim()
            self.ax_success.autoscale_view()
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def cleanup(self):
        """Cleanup resources"""
        print("\n🧹 Cleaning up...")
        self.mlflow.end_run()
        self.env.close()
        print("✅ Cleanup complete")
        print("\nClose matplotlib window to exit...")
        plt.show(block=True)


def main():
    trainer = SimpleVisualTrainer()
    trainer.train(num_episodes=100)


if __name__ == '__main__':
    main()
