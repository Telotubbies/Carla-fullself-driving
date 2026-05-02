#!/usr/bin/env python3
"""
Real-time visualization สำหรับ CARLA SAC Training
แสดง sensor data, rewards, และ training metrics แบบ real-time
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.carla_gym_env import CarlaEnv
import time

class TrainingVisualizer:
    """Visualizer สำหรับแสดงผลการเทรนแบบ real-time"""
    
    def __init__(self):
        # สร้าง figure สำหรับแสดงผล
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('🚗 CARLA SAC Training - Real-time Visualization', 
                         fontsize=16, fontweight='bold')
        
        # สร้าง subplots
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # LiDAR BEV view
        self.ax_lidar = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax_lidar.set_title('🎯 LiDAR Bird\'s Eye View', fontweight='bold')
        self.ax_lidar.axis('off')
        
        # Ego state
        self.ax_state = self.fig.add_subplot(gs[0, 2])
        self.ax_state.set_title('📊 Vehicle State', fontweight='bold')
        self.ax_state.axis('off')
        
        # Rewards
        self.ax_rewards = self.fig.add_subplot(gs[1, 2])
        self.ax_rewards.set_title('💰 Rewards', fontweight='bold')
        self.ax_rewards.set_xlabel('Step')
        self.ax_rewards.set_ylabel('Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        
        # Actions
        self.ax_actions = self.fig.add_subplot(gs[2, 0])
        self.ax_actions.set_title('🎮 Actions', fontweight='bold')
        self.ax_actions.set_xlabel('Step')
        self.ax_actions.set_ylabel('Value')
        self.ax_actions.grid(True, alpha=0.3)
        
        # Speed
        self.ax_speed = self.fig.add_subplot(gs[2, 1])
        self.ax_speed.set_title('⚡ Speed (km/h)', fontweight='bold')
        self.ax_speed.set_xlabel('Step')
        self.ax_speed.set_ylabel('Speed')
        self.ax_speed.grid(True, alpha=0.3)
        
        # Episode info
        self.ax_info = self.fig.add_subplot(gs[2, 2])
        self.ax_info.set_title('ℹ️ Episode Info', fontweight='bold')
        self.ax_info.axis('off')
        
        # Data storage
        self.rewards = []
        self.speeds = []
        self.steering = []
        self.throttle = []
        self.brake = []
        self.steps = []
        
        # Environment
        self.env = None
        self.current_step = 0
        self.episode_reward = 0
        self.episode_num = 0
        
    def create_env(self):
        """สร้าง CARLA environment"""
        env_config = {
            'host': 'localhost',
            'port': 2000,
            'timeout': 10.0,
            'map': 'Town01',
            'delta_seconds': 0.05,
            'max_episode_steps': 1000,
            'sensor_config': {
                'lidar_channels': 32,
                'lidar_range': 50,
                'bev_range': 25.0,
                'bev_resolution': 256,
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
        
        print("🔄 กำลังเชื่อมต่อกับ CARLA server...")
        self.env = CarlaEnv(env_config)
        print("✅ เชื่อมต่อสำเร็จ!")
        
    def update_visualization(self, frame):
        """Update visualization แบบ real-time"""
        
        if self.env is None:
            return
        
        # ถ้าเป็น step แรก ให้ reset environment
        if self.current_step == 0:
            obs, info = self.env.reset()
            self.episode_num += 1
            self.episode_reward = 0
            print(f"\n🎬 Episode {self.episode_num} เริ่มต้น!")
        
        # Sample random action (ในการเทรนจริงจะใช้ policy)
        action = self.env.action_space.sample()
        
        # Execute action
        obs, reward, terminated, truncated, info = self.env.step(action)
        
        # Update data
        self.current_step += 1
        self.episode_reward += reward
        self.steps.append(self.current_step)
        self.rewards.append(reward)
        self.speeds.append(info.get('speed', 0) * 3.6)  # m/s to km/h
        self.steering.append(action[0])
        self.throttle.append(action[1])
        self.brake.append(action[2])
        
        # Keep only last 100 steps
        if len(self.steps) > 100:
            self.steps = self.steps[-100:]
            self.rewards = self.rewards[-100:]
            self.speeds = self.speeds[-100:]
            self.steering = self.steering[-100:]
            self.throttle = self.throttle[-100:]
            self.brake = self.brake[-100:]
        
        # Update LiDAR BEV
        self.ax_lidar.clear()
        self.ax_lidar.set_title('🎯 LiDAR Bird\'s Eye View', fontweight='bold')
        lidar_bev = obs['lidar_bev']
        self.ax_lidar.imshow(lidar_bev)
        self.ax_lidar.axis('off')
        
        # Update ego state
        self.ax_state.clear()
        self.ax_state.set_title('📊 Vehicle State', fontweight='bold')
        ego_state = obs['ego_state']
        state_text = f"""
Speed: {info.get('speed', 0)*3.6:.1f} km/h
Heading: {np.rad2deg(ego_state[1]):.1f}°
Steering: {ego_state[2]:.2f}
Accel: {ego_state[3]:.2f} m/s²
Lateral: {ego_state[4]:.2f} m
Heading Err: {np.rad2deg(ego_state[5]):.1f}°
        """
        self.ax_state.text(0.1, 0.5, state_text, fontsize=10, 
                          verticalalignment='center', family='monospace')
        self.ax_state.axis('off')
        
        # Update rewards
        self.ax_rewards.clear()
        self.ax_rewards.set_title('💰 Rewards', fontweight='bold')
        self.ax_rewards.plot(self.steps, self.rewards, 'g-', linewidth=2, label='Reward')
        self.ax_rewards.axhline(y=0, color='r', linestyle='--', alpha=0.3)
        self.ax_rewards.set_xlabel('Step')
        self.ax_rewards.set_ylabel('Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        self.ax_rewards.legend()
        
        # Update actions
        self.ax_actions.clear()
        self.ax_actions.set_title('🎮 Actions', fontweight='bold')
        self.ax_actions.plot(self.steps, self.steering, 'b-', label='Steering', linewidth=1.5)
        self.ax_actions.plot(self.steps, self.throttle, 'g-', label='Throttle', linewidth=1.5)
        self.ax_actions.plot(self.steps, self.brake, 'r-', label='Brake', linewidth=1.5)
        self.ax_actions.set_xlabel('Step')
        self.ax_actions.set_ylabel('Value')
        self.ax_actions.grid(True, alpha=0.3)
        self.ax_actions.legend(loc='upper right')
        
        # Update speed
        self.ax_speed.clear()
        self.ax_speed.set_title('⚡ Speed (km/h)', fontweight='bold')
        self.ax_speed.plot(self.steps, self.speeds, 'purple', linewidth=2)
        self.ax_speed.axhline(y=30, color='orange', linestyle='--', 
                             alpha=0.5, label='Target (30 km/h)')
        self.ax_speed.set_xlabel('Step')
        self.ax_speed.set_ylabel('Speed')
        self.ax_speed.grid(True, alpha=0.3)
        self.ax_speed.legend()
        
        # Update episode info
        self.ax_info.clear()
        self.ax_info.set_title('ℹ️ Episode Info', fontweight='bold')
        info_text = f"""
Episode: {self.episode_num}
Step: {self.current_step}
Total Reward: {self.episode_reward:.1f}
Avg Reward: {np.mean(self.rewards[-10:]):.2f}
Collision: {'❌ Yes' if info.get('collision', False) else '✅ No'}
        """
        self.ax_info.text(0.1, 0.5, info_text, fontsize=11, 
                         verticalalignment='center', family='monospace',
                         fontweight='bold')
        self.ax_info.axis('off')
        
        # Reset if episode ended
        if terminated or truncated:
            print(f"✅ Episode {self.episode_num} จบ! Total Reward: {self.episode_reward:.1f}")
            self.current_step = 0
            time.sleep(1)  # Pause before next episode
        
    def run(self):
        """เริ่มการ visualization"""
        try:
            self.create_env()
            
            # Create animation
            ani = FuncAnimation(self.fig, self.update_visualization, 
                              interval=50, blit=False, cache_frame_data=False)
            
            plt.tight_layout()
            plt.show()
            
        except KeyboardInterrupt:
            print("\n⏹️ หยุดการ visualization")
        finally:
            if self.env:
                self.env.close()
                print("🔒 ปิด environment เรียบร้อย")


if __name__ == "__main__":
    print("=" * 80)
    print("🚗 CARLA SAC Training - Real-time Visualization")
    print("=" * 80)
    print("\n📌 คำแนะนำ:")
    print("  - กด Ctrl+C เพื่อหยุดการ visualization")
    print("  - หน้าต่างจะแสดง LiDAR, rewards, actions, และ metrics แบบ real-time")
    print("  - ตรวจสอบให้แน่ใจว่า CARLA server กำลังรันอยู่\n")
    
    visualizer = TrainingVisualizer()
    visualizer.run()
