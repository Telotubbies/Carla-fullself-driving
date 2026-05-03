#!/usr/bin/env python3
"""
Demo: Moving Car with Visualization
รถจะเคลื่อนที่จริงๆ ด้วย Expert Controller หรือ Autopilot
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import carla
from src.carla_gym_env import CarlaEnv
from src.imitation import ExpertController
import time

class MovingCarDemo:
    def __init__(self):
        print("=" * 80)
        print("🚗 CARLA Moving Car Demo")
        print("=" * 80)
        print("")
        
        # Create environment
        print("🔄 Creating environment...")
        config = {
            'host': 'localhost',
            'port': 2000,
            'timeout': 10.0,
            'map': 'Town01',
            'delta_seconds': 0.05,
            'max_episode_steps': 2000,
            'use_camera': True,
            'use_fixed_spawn': True,
            'fixed_spawn_indices': [0],
            'sensor_config': {
                'camera_width': 640,
                'camera_height': 480,
                'lidar_channels': 32,
                'lidar_range': 50,
                'bev_range': 25.0,
                'bev_resolution': 256,
            }
        }
        
        self.env = CarlaEnv(config)
        print("✅ Environment created")
        
        # Create expert controller
        print("🎮 Creating expert controller...")
        self.expert = ExpertController(
            target_speed=30.0 / 3.6,  # Convert to m/s
            lateral_kp=1.0,
            lateral_ki=0.0,
            lateral_kd=0.1,
            longitudinal_kp=0.5,
            longitudinal_ki=0.0,
            longitudinal_kd=0.1
        )
        print("✅ Expert controller ready")
        
        # Setup visualization
        self.setup_visualization()
        
        # State
        self.obs = None
        self.step_count = 0
        self.total_distance = 0.0
        self.episode = 0
        
    def setup_visualization(self):
        """Setup matplotlib visualization"""
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10))
        
        # Camera
        self.ax_camera = plt.subplot(2, 3, 1)
        self.ax_camera.set_title('📷 Camera View')
        self.ax_camera.axis('off')
        self.img_camera = None
        
        # LiDAR BEV
        self.ax_lidar = plt.subplot(2, 3, 2)
        self.ax_lidar.set_title('🎯 LiDAR BEV')
        self.ax_lidar.axis('off')
        self.img_lidar = None
        
        # Speed
        self.ax_speed = plt.subplot(2, 3, 3)
        self.ax_speed.set_title('🚗 Speed')
        self.ax_speed.set_xlabel('Step')
        self.ax_speed.set_ylabel('km/h')
        self.speed_data = []
        self.speed_line, = self.ax_speed.plot([], [], 'b-', linewidth=2)
        
        # Actions
        self.ax_actions = plt.subplot(2, 3, 4)
        self.ax_actions.set_title('🎮 Actions')
        self.action_bars = None
        
        # Distance
        self.ax_distance = plt.subplot(2, 3, 5)
        self.ax_distance.set_title('📏 Distance Traveled')
        self.ax_distance.set_xlabel('Step')
        self.ax_distance.set_ylabel('Meters')
        self.distance_data = []
        self.distance_line, = self.ax_distance.plot([], [], 'g-', linewidth=2)
        
        # Info
        self.ax_info = plt.subplot(2, 3, 6)
        self.ax_info.set_title('ℹ️ Info')
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.1, 0.5, '', fontsize=12, verticalalignment='center')
        
        plt.tight_layout()
        
    def reset(self):
        """Reset environment"""
        print(f"\n🎬 Episode {self.episode + 1} starting...")
        self.obs, info = self.env.reset()
        self.step_count = 0
        self.total_distance = 0.0
        self.speed_data = []
        self.distance_data = []
        self.episode += 1
        
    def step(self):
        """Execute one step"""
        # Get expert action
        ego_state = self.obs['ego_state']
        action = self.expert.get_action(ego_state)
        
        # Make sure action has throttle
        if action[1] < 0.3:  # If throttle too low
            action[1] = 0.5  # Set minimum throttle
        
        # Execute action
        next_obs, reward, done, truncated, info = self.env.step(action)
        
        # Update state
        prev_pos = self.obs['ego_state'][1:4]  # x, y, z
        curr_pos = next_obs['ego_state'][1:4]
        distance = np.linalg.norm(curr_pos - prev_pos)
        self.total_distance += distance
        
        self.obs = next_obs
        self.step_count += 1
        
        # Update visualization
        self.update_visualization(action, reward, done or truncated)
        
        return done or truncated
        
    def update_visualization(self, action, reward, done):
        """Update visualization"""
        # Camera
        if 'camera' in self.obs:
            if self.img_camera is None:
                self.img_camera = self.ax_camera.imshow(self.obs['camera'])
            else:
                self.img_camera.set_data(self.obs['camera'])
        
        # LiDAR
        if 'lidar_bev' in self.obs:
            if self.img_lidar is None:
                self.img_lidar = self.ax_lidar.imshow(self.obs['lidar_bev'])
            else:
                self.img_lidar.set_data(self.obs['lidar_bev'])
        
        # Speed
        speed = self.obs['ego_state'][0] * 3.6  # m/s to km/h
        self.speed_data.append(speed)
        self.speed_line.set_data(range(len(self.speed_data)), self.speed_data)
        self.ax_speed.relim()
        self.ax_speed.autoscale_view()
        
        # Distance
        self.distance_data.append(self.total_distance)
        self.distance_line.set_data(range(len(self.distance_data)), self.distance_data)
        self.ax_distance.relim()
        self.ax_distance.autoscale_view()
        
        # Actions
        action_labels = ['Steering', 'Throttle', 'Brake']
        action_values = action
        colors = ['blue', 'green', 'red']
        
        self.ax_actions.clear()
        self.ax_actions.set_title('🎮 Actions')
        self.ax_actions.bar(action_labels, action_values, color=colors, alpha=0.7)
        self.ax_actions.set_ylim(-1, 1)
        self.ax_actions.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        
        # Info
        info_str = f"""
Episode: {self.episode}
Step: {self.step_count}
Speed: {speed:.1f} km/h
Distance: {self.total_distance:.1f} m
Reward: {reward:.2f}
Status: {'Done ✓' if done else 'Running...'}

Actions:
  Steering: {action[0]:+.2f}
  Throttle: {action[1]:.2f}
  Brake: {action[2]:.2f}
        """
        self.info_text.set_text(info_str.strip())
        
        # Update display
        plt.pause(0.001)
        
    def run(self, episodes=5):
        """Run demo"""
        print("\n" + "=" * 80)
        print("🚀 Starting Demo")
        print("=" * 80)
        print(f"Episodes: {episodes}")
        print("Press Ctrl+C to stop")
        print("")
        
        try:
            for ep in range(episodes):
                self.reset()
                done = False
                
                while not done and self.step_count < 1000:
                    done = self.step()
                    
                    if self.step_count % 100 == 0:
                        speed = self.obs['ego_state'][0] * 3.6
                        print(f"  Step {self.step_count}: Speed={speed:.1f} km/h, Distance={self.total_distance:.1f}m")
                
                print(f"✅ Episode {self.episode} complete: {self.step_count} steps, {self.total_distance:.1f}m")
                
        except KeyboardInterrupt:
            print("\n⏹️  Stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup"""
        print("\n🧹 Cleaning up...")
        if self.env:
            self.env.close()
        plt.close('all')
        print("✅ Cleanup complete")


def main():
    demo = MovingCarDemo()
    demo.run(episodes=5)


if __name__ == '__main__':
    main()
