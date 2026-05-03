#!/usr/bin/env python3
"""
Advanced Real-time Visualization สำหรับ CARLA SAC Training
แสดง Camera + LiDAR BEV + Metrics + Curriculum Progress
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from src.carla_gym_env import CarlaEnv
import time

class AdvancedVisualizer:
    """Advanced Visualizer แสดงผล Camera + LiDAR + Metrics แบบครบครัน"""
    
    def __init__(self, enable_spectator=True):
        self.enable_spectator = enable_spectator
        
        # สร้าง figure ขนาดใหญ่
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.suptitle('🚗 CARLA Advanced Training - Multi-View Visualization', 
                         fontsize=18, fontweight='bold')
        
        # สร้าง grid layout
        gs = self.fig.add_gridspec(3, 4, hspace=0.35, wspace=0.35)
        
        # Row 1: Camera + LiDAR BEV + Curriculum Progress
        self.ax_camera = self.fig.add_subplot(gs[0, 0:2])
        self.ax_camera.set_title('📷 Front Camera View', fontweight='bold', fontsize=12)
        self.ax_camera.axis('off')
        
        self.ax_lidar = self.fig.add_subplot(gs[0, 2])
        self.ax_lidar.set_title('🎯 LiDAR BEV', fontweight='bold', fontsize=12)
        self.ax_lidar.axis('off')
        
        self.ax_curriculum = self.fig.add_subplot(gs[0, 3])
        self.ax_curriculum.set_title('📚 Training Stage', fontweight='bold', fontsize=12)
        self.ax_curriculum.axis('off')
        
        # Row 2: Rewards + Actions + Speed
        self.ax_rewards = self.fig.add_subplot(gs[1, 0:2])
        self.ax_rewards.set_title('💰 Rewards Timeline', fontweight='bold', fontsize=11)
        self.ax_rewards.set_xlabel('Step')
        self.ax_rewards.set_ylabel('Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        
        self.ax_actions = self.fig.add_subplot(gs[1, 2])
        self.ax_actions.set_title('🎮 Control Actions', fontweight='bold', fontsize=11)
        self.ax_actions.set_xlabel('Step')
        self.ax_actions.set_ylabel('Value')
        self.ax_actions.grid(True, alpha=0.3)
        
        self.ax_speed = self.fig.add_subplot(gs[1, 3])
        self.ax_speed.set_title('⚡ Speed (km/h)', fontweight='bold', fontsize=11)
        self.ax_speed.set_xlabel('Step')
        self.ax_speed.set_ylabel('Speed')
        self.ax_speed.grid(True, alpha=0.3)
        
        # Row 3: Vehicle State + Episode Info + Performance Metrics
        self.ax_state = self.fig.add_subplot(gs[2, 0])
        self.ax_state.set_title('📊 Vehicle State', fontweight='bold', fontsize=11)
        self.ax_state.axis('off')
        
        self.ax_episode = self.fig.add_subplot(gs[2, 1])
        self.ax_episode.set_title('ℹ️ Episode Info', fontweight='bold', fontsize=11)
        self.ax_episode.axis('off')
        
        self.ax_performance = self.fig.add_subplot(gs[2, 2:4])
        self.ax_performance.set_title('📈 Performance Metrics', fontweight='bold', fontsize=11)
        self.ax_performance.axis('off')
        
        # Data storage
        self.rewards = []
        self.speeds = []
        self.steering = []
        self.throttle = []
        self.brake = []
        self.steps = []
        self.lane_deviations = []
        
        # Episode tracking
        self.env = None
        self.current_step = 0
        self.episode_reward = 0
        self.episode_num = 0
        self.total_collisions = 0
        self.total_success = 0
        
        # Curriculum stage (mock for now)
        self.current_stage = "Stage 1: Basic Control"
        self.stage_progress = 0.0
        
    def create_env(self):
        """สร้าง CARLA environment พร้อม camera"""
        env_config = {
            'host': 'localhost',
            'port': 2000,
            'timeout': 10.0,
            'map': 'Town01',
            'delta_seconds': 0.05,
            'max_episode_steps': 1000,
            'use_camera': True,  # เปิดใช้ camera
            'use_fixed_spawn': True,  # ใช้ fixed spawn points
            'fixed_spawn_indices': [0, 1, 2],  # spawn points ที่จะใช้
            'sensor_config': {
                'lidar_channels': 32,
                'lidar_range': 50,
                'bev_range': 25.0,
                'bev_resolution': 256,
                'camera_width': 640,
                'camera_height': 480,
                'camera_fov': 90,
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
        
        # เปิด spectator mode ถ้าต้องการ
        if self.enable_spectator:
            self._setup_spectator()
    
    def _setup_spectator(self):
        """ตั้งค่า CARLA spectator camera ให้ติดตามรถ"""
        try:
            spectator = self.env.world.get_spectator()
            print("✅ CARLA Spectator Mode เปิดใช้งาน - เปิด CARLA window เพื่อดู 3D view")
        except Exception as e:
            print(f"⚠️ ไม่สามารถเปิด spectator mode: {e}")
    
    def _update_spectator(self):
        """อัพเดทตำแหน่ง spectator camera ให้ติดตามรถ"""
        if self.enable_spectator and self.env and self.env.vehicle:
            try:
                spectator = self.env.world.get_spectator()
                transform = self.env.vehicle.get_transform()
                
                # ตั้งกล้องด้านหลังรถ สูงขึ้นเล็กน้อย
                spectator.set_transform(
                    carla.Transform(
                        transform.location + carla.Location(x=-8, z=4),
                        carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
                    )
                )
            except:
                pass
    
    def update_visualization(self, frame):
        """Update visualization แบบ real-time"""
        
        if self.env is None:
            return
        
        # Reset episode ถ้าเป็น step แรก
        if self.current_step == 0:
            obs, info = self.env.reset()
            self.episode_num += 1
            self.episode_reward = 0
            print(f"\n🎬 Episode {self.episode_num} เริ่มต้น!")
        
        # Sample action (ในการเทรนจริงจะใช้ policy)
        action = self.env.action_space.sample()
        
        # Execute action
        obs, reward, terminated, truncated, info = self.env.step(action)
        
        # Update spectator camera
        self._update_spectator()
        
        # Update data
        self.current_step += 1
        self.episode_reward += reward
        self.steps.append(self.current_step)
        self.rewards.append(reward)
        self.speeds.append(info.get('speed', 0) * 3.6)
        self.steering.append(action[0])
        self.throttle.append(action[1])
        self.brake.append(action[2])
        self.lane_deviations.append(abs(obs['ego_state'][4]))
        
        # Keep only last 100 steps
        max_history = 100
        if len(self.steps) > max_history:
            self.steps = self.steps[-max_history:]
            self.rewards = self.rewards[-max_history:]
            self.speeds = self.speeds[-max_history:]
            self.steering = self.steering[-max_history:]
            self.throttle = self.throttle[-max_history:]
            self.brake = self.brake[-max_history:]
            self.lane_deviations = self.lane_deviations[-max_history:]
        
        # Update visualizations
        self._update_camera(obs)
        self._update_lidar(obs)
        self._update_curriculum()
        self._update_rewards()
        self._update_actions()
        self._update_speed()
        self._update_state(obs, info)
        self._update_episode_info(info)
        self._update_performance()
        
        # Check episode end
        if terminated or truncated:
            if info.get('collision', False):
                self.total_collisions += 1
                print(f"❌ Episode {self.episode_num} จบ (ชน)! Reward: {self.episode_reward:.1f}")
            else:
                self.total_success += 1
                print(f"✅ Episode {self.episode_num} จบ (สำเร็จ)! Reward: {self.episode_reward:.1f}")
            
            self.current_step = 0
            time.sleep(1)
    
    def _update_camera(self, obs):
        """อัพเดท camera view"""
        self.ax_camera.clear()
        self.ax_camera.set_title('📷 Front Camera View', fontweight='bold', fontsize=12)
        
        if 'camera' in obs and obs['camera'] is not None:
            # แสดงภาพจากกล้อง
            camera_img = obs['camera']
            self.ax_camera.imshow(camera_img)
            
            # เพิ่ม overlay ข้อมูล
            speed_kmh = obs['ego_state'][0] * 3.6
            self.ax_camera.text(10, 30, f'Speed: {speed_kmh:.1f} km/h', 
                              color='lime', fontsize=10, fontweight='bold',
                              bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
        else:
            self.ax_camera.text(0.5, 0.5, 'Camera Not Available', 
                              ha='center', va='center', fontsize=14)
        
        self.ax_camera.axis('off')
    
    def _update_lidar(self, obs):
        """อัพเดท LiDAR BEV"""
        self.ax_lidar.clear()
        self.ax_lidar.set_title('🎯 LiDAR BEV', fontweight='bold', fontsize=12)
        
        lidar_bev = obs['lidar_bev']
        self.ax_lidar.imshow(lidar_bev)
        
        # วาดตำแหน่งรถ (จุดกึ่งกลาง)
        center = lidar_bev.shape[0] // 2
        self.ax_lidar.plot(center, center, 'r+', markersize=15, markeredgewidth=2)
        
        self.ax_lidar.axis('off')
    
    def _update_curriculum(self):
        """อัพเดท curriculum progress"""
        self.ax_curriculum.clear()
        self.ax_curriculum.set_title('📚 Training Stage', fontweight='bold', fontsize=12)
        
        # Mock curriculum progress
        self.stage_progress = min((self.episode_num % 500) / 500.0, 1.0)
        
        curriculum_text = f"""
Stage: {self.current_stage}

Progress: {self.stage_progress*100:.1f}%

Episodes: {self.episode_num}

Next Stage:
{500 - (self.episode_num % 500)} episodes
        """
        
        self.ax_curriculum.text(0.1, 0.5, curriculum_text, fontsize=9,
                               verticalalignment='center', family='monospace')
        
        # Progress bar
        bar_y = 0.15
        bar_height = 0.05
        self.ax_curriculum.add_patch(Rectangle((0.1, bar_y), 0.8, bar_height, 
                                               fill=True, facecolor='lightgray', edgecolor='black'))
        self.ax_curriculum.add_patch(Rectangle((0.1, bar_y), 0.8 * self.stage_progress, bar_height,
                                               fill=True, facecolor='green', edgecolor='black'))
        
        self.ax_curriculum.set_xlim(0, 1)
        self.ax_curriculum.set_ylim(0, 1)
        self.ax_curriculum.axis('off')
    
    def _update_rewards(self):
        """อัพเดท rewards graph"""
        self.ax_rewards.clear()
        self.ax_rewards.set_title('💰 Rewards Timeline', fontweight='bold', fontsize=11)
        
        if len(self.steps) > 0:
            self.ax_rewards.plot(self.steps, self.rewards, 'g-', linewidth=2, label='Reward')
            self.ax_rewards.axhline(y=0, color='r', linestyle='--', alpha=0.3)
            
            # แสดง moving average
            if len(self.rewards) >= 10:
                ma = np.convolve(self.rewards, np.ones(10)/10, mode='valid')
                ma_steps = self.steps[9:]
                self.ax_rewards.plot(ma_steps, ma, 'b-', linewidth=1.5, 
                                    alpha=0.7, label='MA(10)')
        
        self.ax_rewards.set_xlabel('Step')
        self.ax_rewards.set_ylabel('Reward')
        self.ax_rewards.grid(True, alpha=0.3)
        self.ax_rewards.legend(loc='upper left', fontsize=8)
    
    def _update_actions(self):
        """อัพเดท actions graph"""
        self.ax_actions.clear()
        self.ax_actions.set_title('🎮 Control Actions', fontweight='bold', fontsize=11)
        
        if len(self.steps) > 0:
            self.ax_actions.plot(self.steps, self.steering, 'b-', label='Steering', linewidth=1.5, alpha=0.8)
            self.ax_actions.plot(self.steps, self.throttle, 'g-', label='Throttle', linewidth=1.5, alpha=0.8)
            self.ax_actions.plot(self.steps, self.brake, 'r-', label='Brake', linewidth=1.5, alpha=0.8)
        
        self.ax_actions.set_xlabel('Step')
        self.ax_actions.set_ylabel('Value')
        self.ax_actions.set_ylim(-1.1, 1.1)
        self.ax_actions.grid(True, alpha=0.3)
        self.ax_actions.legend(loc='upper right', fontsize=8)
    
    def _update_speed(self):
        """อัพเดท speed graph"""
        self.ax_speed.clear()
        self.ax_speed.set_title('⚡ Speed (km/h)', fontweight='bold', fontsize=11)
        
        if len(self.steps) > 0:
            self.ax_speed.plot(self.steps, self.speeds, 'purple', linewidth=2)
            self.ax_speed.axhline(y=30, color='orange', linestyle='--', 
                                 alpha=0.5, label='Target (30 km/h)')
        
        self.ax_speed.set_xlabel('Step')
        self.ax_speed.set_ylabel('Speed')
        self.ax_speed.grid(True, alpha=0.3)
        self.ax_speed.legend(loc='upper right', fontsize=8)
    
    def _update_state(self, obs, info):
        """อัพเดท vehicle state"""
        self.ax_state.clear()
        self.ax_state.set_title('📊 Vehicle State', fontweight='bold', fontsize=11)
        
        ego_state = obs['ego_state']
        state_text = f"""
Speed: {ego_state[0]*3.6:.1f} km/h
Heading: {np.rad2deg(ego_state[1]):.1f}°
Steering: {ego_state[2]:.2f}
Accel: {ego_state[3]:.2f} m/s²
Lateral: {ego_state[4]:.2f} m
Head Err: {np.rad2deg(ego_state[5]):.1f}°
        """
        
        self.ax_state.text(0.1, 0.5, state_text, fontsize=9,
                          verticalalignment='center', family='monospace')
        self.ax_state.axis('off')
    
    def _update_episode_info(self, info):
        """อัพเดท episode info"""
        self.ax_episode.clear()
        self.ax_episode.set_title('ℹ️ Episode Info', fontweight='bold', fontsize=11)
        
        collision_icon = '❌' if info.get('collision', False) else '✅'
        
        episode_text = f"""
Episode: {self.episode_num}
Step: {self.current_step}

Total Reward: {self.episode_reward:.1f}
Avg Reward: {np.mean(self.rewards[-10:]) if self.rewards else 0:.2f}

Collision: {collision_icon}
        """
        
        self.ax_episode.text(0.1, 0.5, episode_text, fontsize=9,
                            verticalalignment='center', family='monospace',
                            fontweight='bold')
        self.ax_episode.axis('off')
    
    def _update_performance(self):
        """อัพเดท performance metrics"""
        self.ax_performance.clear()
        self.ax_performance.set_title('📈 Performance Metrics', fontweight='bold', fontsize=11)
        
        # คำนวณ metrics
        total_episodes = max(self.episode_num, 1)
        success_rate = (self.total_success / total_episodes) * 100
        collision_rate = (self.total_collisions / total_episodes) * 100
        avg_lane_dev = np.mean(self.lane_deviations[-50:]) if self.lane_deviations else 0
        avg_speed = np.mean(self.speeds[-50:]) if self.speeds else 0
        
        perf_text = f"""
Success Rate: {success_rate:.1f}%  |  Collision Rate: {collision_rate:.1f}%  |  Avg Lane Dev: {avg_lane_dev:.2f}m  |  Avg Speed: {avg_speed:.1f} km/h

Total Episodes: {total_episodes}  |  Successful: {self.total_success}  |  Collisions: {self.total_collisions}
        """
        
        self.ax_performance.text(0.05, 0.5, perf_text, fontsize=9,
                                verticalalignment='center', family='monospace')
        self.ax_performance.axis('off')
    
    def run(self):
        """เริ่มการ visualization"""
        try:
            self.create_env()
            
            # สร้าง animation
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
    print("🚗 CARLA Advanced Training - Multi-View Visualization")
    print("=" * 80)
    print("\n📌 คำแนะนำ:")
    print("  - กด Ctrl+C เพื่อหยุดการ visualization")
    print("  - หน้าต่างจะแสดง Camera + LiDAR + Metrics แบบ real-time")
    print("  - CARLA Spectator window จะเปิดอัตโนมัติ (ถ้ามี CARLA GUI)")
    print("  - ตรวจสอบให้แน่ใจว่า CARLA server กำลังรันอยู่\n")
    
    # เปิด spectator mode
    visualizer = AdvancedVisualizer(enable_spectator=True)
    visualizer.run()
