#!/usr/bin/env python3
"""
Training with Full Visualization
แสดง visualization ทุกอย่าง real-time:
- Camera view
- Perception overlay (lanes, objects, traffic lights)
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

from src.carla_gym_env import CarlaEnv
from src.carla_gym_env.enhanced_rewards import EnhancedRewardCalculator
from src.curriculum import CurriculumManager
from src.mlflow_integration import MLflowTracker
from src.imitation import ExpertController
from src.perception import PerceptionFusion

class FullVisualizationTrainer:
    """Trainer with complete real-time visualization"""
    
    def __init__(self):
        print("=" * 80)
        print("🎨 TRAINING WITH FULL VISUALIZATION")
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
        
        # Initialize perception
        print("🎯 Initializing perception system...")
        self.perception = PerceptionFusion(use_gpu=True, image_size=(640, 480))
        print("✅ Perception ready")
        
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
        
        # Enhanced reward calculator
        self.reward_calculator = EnhancedRewardCalculator('config/training_guidelines.yaml')
        
        # Setup visualization
        self.setup_visualization()
        
        # Metrics
        self.episode_rewards = []
        self.episode_lengths = []
        self.success_rates = []
        self.episode_count = 0
        
    def setup_visualization(self):
        """Setup comprehensive matplotlib visualization"""
        plt.ion()
        self.fig = plt.figure(figsize=(20, 12))
        gs = GridSpec(3, 4, figure=self.fig, hspace=0.3, wspace=0.3)
        
        # Row 1: Camera and Perception
        self.ax_camera = self.fig.add_subplot(gs[0, 0:2])
        self.ax_camera.set_title('Camera + Perception Overlay', fontsize=14, fontweight='bold')
        self.ax_camera.axis('off')
        self.img_camera = None
        
        self.ax_lidar = self.fig.add_subplot(gs[0, 2])
        self.ax_lidar.set_title('LiDAR BEV', fontsize=14, fontweight='bold')
        self.ax_lidar.axis('off')
        self.img_lidar = None
        
        self.ax_state = self.fig.add_subplot(gs[0, 3])
        self.ax_state.set_title('Vehicle State', fontsize=14, fontweight='bold')
        self.ax_state.axis('off')
        self.state_text = self.ax_state.text(0.1, 0.5, '', fontsize=10, 
                                            verticalalignment='center', family='monospace')
        
        # Row 2: Metrics
        self.ax_rewards = self.fig.add_subplot(gs[1, 0:2])
        self.ax_rewards.set_title('Episode Rewards', fontsize=14, fontweight='bold')
        self.ax_rewards.set_xlabel('Episode')
        self.ax_rewards.set_ylabel('Total Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        self.reward_line, = self.ax_rewards.plot([], [], 'b-', linewidth=2, label='Reward')
        self.reward_ma_line, = self.ax_rewards.plot([], [], 'r-', linewidth=2, label='MA(10)')
        self.ax_rewards.legend()
        
        self.ax_lengths = self.fig.add_subplot(gs[1, 2])
        self.ax_lengths.set_title('Episode Lengths', fontsize=14, fontweight='bold')
        self.ax_lengths.set_xlabel('Episode')
        self.ax_lengths.set_ylabel('Steps')
        self.ax_lengths.grid(True, alpha=0.3)
        self.length_line, = self.ax_lengths.plot([], [], 'g-', linewidth=2)
        
        self.ax_success = self.fig.add_subplot(gs[1, 3])
        self.ax_success.set_title('Success Rate', fontsize=14, fontweight='bold')
        self.ax_success.set_xlabel('Episode')
        self.ax_success.set_ylabel('Success %')
        self.ax_success.set_ylim(0, 100)
        self.ax_success.grid(True, alpha=0.3)
        self.success_line, = self.ax_success.plot([], [], 'orange', linewidth=2)
        
        # Row 3: Actions and Info
        self.ax_actions = self.fig.add_subplot(gs[2, 0])
        self.ax_actions.set_title('Current Actions', fontsize=14, fontweight='bold')
        
        self.ax_perception_info = self.fig.add_subplot(gs[2, 1])
        self.ax_perception_info.set_title('Perception Info', fontsize=14, fontweight='bold')
        self.ax_perception_info.axis('off')
        self.perception_text = self.ax_perception_info.text(0.1, 0.5, '', fontsize=10,
                                                           verticalalignment='center', family='monospace')
        
        self.ax_training_info = self.fig.add_subplot(gs[2, 2:4])
        self.ax_training_info.set_title('Training Info', fontsize=14, fontweight='bold')
        self.ax_training_info.axis('off')
        self.training_text = self.ax_training_info.text(0.1, 0.5, '', fontsize=10,
                                                        verticalalignment='center', family='monospace')
        
        plt.tight_layout()
        
    def train(self, num_episodes=100):
        """Train with full visualization"""
        print("\n" + "=" * 80)
        print("🚀 Starting Training with Full Visualization")
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
        perception_outputs = []
        
        print(f"\n🎬 Episode {episode_num + 1} starting...")
        
        while not done and not truncated and step < 1000:
            # Get expert action
            try:
                action = self.expert.get_action(
                    self.env.vehicle,
                    self.env.map,
                    dt=0.05
                )
            except:
                # Fallback: simple forward action
                action = np.array([0.0, 0.5, 0.0])
            
            # Process perception if camera available
            perception_output = None
            if 'camera' in obs and obs['camera'] is not None:
                try:
                    perception_output = self.perception.process(obs['camera'])
                    perception_outputs.append(perception_output)
                except Exception as e:
                    pass
            
            # Step environment
            next_obs, reward, done, truncated, info = self.env.step(action)
            
            # Calculate enhanced reward
            if perception_output:
                enhanced_reward = self.reward_calculator.calculate_reward(
                    obs=obs,
                    action=action,
                    info=info,
                    perception_output=perception_output
                )
            else:
                enhanced_reward = reward
            
            episode_reward += enhanced_reward
            
            # Update visualization every 5 steps
            if step % 5 == 0:
                self.update_visualization(obs, action, perception_output, 
                                        episode_reward, step, episode_num)
            
            obs = next_obs
            step += 1
        
        # Episode complete
        self.episode_count += 1
        self.episode_rewards.append(episode_reward)
        self.episode_lengths.append(step)
        
        # Calculate success
        success = not info.get('collision', False) and step > 100
        if len(self.success_rates) >= 10:
            recent_success = sum(self.success_rates[-10:]) / 10.0
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
        
        print(f"✅ Episode {episode_num + 1} complete: "
              f"Reward={episode_reward:.2f}, Steps={step}, "
              f"Success={'✓' if success else '✗'}")
    
    def update_visualization(self, obs, action, perception_output, 
                           episode_reward, step, episode_num):
        """Update all visualization panels"""
        # Camera with perception overlay
        if 'camera' in obs and obs['camera'] is not None:
            if perception_output:
                vis_img = self.perception.visualize(obs['camera'], perception_output)
            else:
                vis_img = obs['camera']
            
            if self.img_camera is None:
                self.img_camera = self.ax_camera.imshow(vis_img)
            else:
                self.img_camera.set_data(vis_img)
        
        # LiDAR BEV
        if 'lidar_bev' in obs:
            if self.img_lidar is None:
                self.img_lidar = self.ax_lidar.imshow(obs['lidar_bev'])
            else:
                self.img_lidar.set_data(obs['lidar_bev'])
        
        # Vehicle state
        ego_state = obs['ego_state']
        speed = ego_state[0] * 3.6
        x, y, z = ego_state[1:4]
        lateral_offset = ego_state[4] if len(ego_state) > 4 else 0.0
        heading_error = ego_state[5] if len(ego_state) > 5 else 0.0
        
        state_str = f"""
Speed:     {speed:6.1f} km/h
Position:  ({x:6.1f}, {y:6.1f})
Lateral:   {lateral_offset:+6.3f} m
Heading:   {np.degrees(heading_error):+6.1f}°
Step:      {step}
Reward:    {episode_reward:+8.2f}
        """
        self.state_text.set_text(state_str.strip())
        
        # Actions
        action_labels = ['Steering', 'Throttle', 'Brake']
        colors = ['blue', 'green', 'red']
        
        self.ax_actions.clear()
        self.ax_actions.set_title('Current Actions', fontsize=14, fontweight='bold')
        self.ax_actions.bar(action_labels, action, color=colors, alpha=0.7)
        self.ax_actions.set_ylim(-1, 1)
        self.ax_actions.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax_actions.grid(True, alpha=0.3, axis='y')
        
        # Perception info
        if perception_output:
            perc_str = f"""
Lane Detected: {'✓' if perception_output.lane_detected else '✗'}
Lane Offset:   {perception_output.lane_center_offset:+.3f} m
Objects:       {len(perception_output.objects)}
In Path:       {len(perception_output.objects_in_path)}
Traffic Light: {perception_output.active_traffic_light.state.name if perception_output.active_traffic_light else 'None'}
Safe:          {'✓' if perception_output.safe_to_proceed else '✗'}
Rec. Speed:    {perception_output.recommended_speed:.0f} km/h
            """
            self.perception_text.set_text(perc_str.strip())
        
        # Training info
        progress = self.curriculum.get_progress()
        training_str = f"""
Episode:       {episode_num + 1}
Stage:         {progress['current_stage_name']}
Stage Progress: {progress['stage_progress']*100:.1f}%
Overall:       {progress['overall_progress']*100:.1f}%

Recent Avg Reward: {np.mean(self.episode_rewards[-10:]) if self.episode_rewards else 0:.2f}
Recent Avg Length: {np.mean(self.episode_lengths[-10:]) if self.episode_lengths else 0:.0f}
Success Rate:      {self.success_rates[-1] if self.success_rates else 0:.1f}%
        """
        self.training_text.set_text(training_str.strip())
        
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
    trainer = FullVisualizationTrainer()
    trainer.train(num_episodes=100)


if __name__ == '__main__':
    main()
