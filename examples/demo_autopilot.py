#!/usr/bin/env python3
"""
Demo: CARLA Autopilot with Visualization
รถจะเคลื่อนที่ด้วย CARLA Autopilot และแสดง visualization
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import matplotlib.pyplot as plt
import carla
from src.carla_gym_env import CarlaEnv
import time

class AutopilotDemo:
    def __init__(self):
        print("=" * 80)
        print("🚗 CARLA Autopilot Demo with Visualization")
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
        
        # Reset to spawn vehicle
        print("🔄 Spawning vehicle...")
        self.obs, _ = self.env.reset()
        
        # Enable autopilot
        print("🤖 Enabling autopilot...")
        self.env.vehicle.set_autopilot(True)
        print("✅ Autopilot enabled - รถจะขับเองอัตโนมัติ!")
        
        # Setup visualization
        self.setup_visualization()
        
        # State
        self.obs = None
        self.step_count = 0
        self.total_distance = 0.0
        self.speed_data = []
        self.distance_data = []
        
    def setup_visualization(self):
        """Setup matplotlib visualization"""
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10))
        
        # Camera
        self.ax_camera = plt.subplot(2, 3, 1)
        self.ax_camera.set_title('Camera View', fontsize=14, fontweight='bold')
        self.ax_camera.axis('off')
        self.img_camera = None
        
        # LiDAR BEV
        self.ax_lidar = plt.subplot(2, 3, 2)
        self.ax_lidar.set_title('LiDAR BEV', fontsize=14, fontweight='bold')
        self.ax_lidar.axis('off')
        self.img_lidar = None
        
        # Speed
        self.ax_speed = plt.subplot(2, 3, 3)
        self.ax_speed.set_title('Speed (km/h)', fontsize=14, fontweight='bold')
        self.ax_speed.set_xlabel('Step')
        self.ax_speed.set_ylabel('km/h')
        self.ax_speed.grid(True, alpha=0.3)
        self.speed_line, = self.ax_speed.plot([], [], 'b-', linewidth=2)
        
        # Vehicle State
        self.ax_state = plt.subplot(2, 3, 4)
        self.ax_state.set_title('Vehicle State', fontsize=14, fontweight='bold')
        self.ax_state.axis('off')
        self.state_text = self.ax_state.text(0.1, 0.5, '', fontsize=11, 
                                            verticalalignment='center', family='monospace')
        
        # Distance
        self.ax_distance = plt.subplot(2, 3, 5)
        self.ax_distance.set_title('Distance Traveled (m)', fontsize=14, fontweight='bold')
        self.ax_distance.set_xlabel('Step')
        self.ax_distance.set_ylabel('Meters')
        self.ax_distance.grid(True, alpha=0.3)
        self.distance_line, = self.ax_distance.plot([], [], 'g-', linewidth=2)
        
        # Info
        self.ax_info = plt.subplot(2, 3, 6)
        self.ax_info.set_title('System Info', fontsize=14, fontweight='bold')
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.1, 0.5, '', fontsize=11, 
                                          verticalalignment='center', family='monospace')
        
        plt.tight_layout()
        
    def reset(self):
        """Reset environment"""
        print(f"\nResetting environment...")
        self.obs, info = self.env.reset()
        
        # Re-enable autopilot after reset
        self.env.vehicle.set_autopilot(True)
        
        self.step_count = 0
        self.total_distance = 0.0
        self.speed_data = []
        self.distance_data = []
        print("✅ Reset complete, autopilot re-enabled")
        
    def step(self):
        """Execute one step"""
        # Autopilot is controlling the vehicle, we just observe
        # Use zero action (autopilot overrides it)
        action = np.array([0.0, 0.0, 0.0])
        
        # Execute step
        next_obs, reward, done, truncated, info = self.env.step(action)
        
        # Calculate distance
        if self.obs is not None:
            prev_pos = self.obs['ego_state'][1:4]
            curr_pos = next_obs['ego_state'][1:4]
            distance = np.linalg.norm(curr_pos - prev_pos)
            self.total_distance += distance
        
        self.obs = next_obs
        self.step_count += 1
        
        # Update visualization
        self.update_visualization()
        
        return done or truncated
        
    def update_visualization(self):
        """Update visualization"""
        if self.obs is None:
            return
        
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
        
        # Get vehicle state
        ego_state = self.obs['ego_state']
        speed_ms = ego_state[0]
        speed_kmh = speed_ms * 3.6
        x, y, z = ego_state[1:4]
        heading = ego_state[6]
        
        # Speed
        self.speed_data.append(speed_kmh)
        if len(self.speed_data) > 200:
            self.speed_data = self.speed_data[-200:]
        self.speed_line.set_data(range(len(self.speed_data)), self.speed_data)
        self.ax_speed.relim()
        self.ax_speed.autoscale_view()
        
        # Distance
        self.distance_data.append(self.total_distance)
        if len(self.distance_data) > 200:
            self.distance_data = self.distance_data[-200:]
        self.distance_line.set_data(range(len(self.distance_data)), self.distance_data)
        self.ax_distance.relim()
        self.ax_distance.autoscale_view()
        
        # Vehicle State
        state_str = f"""
Speed:    {speed_kmh:6.1f} km/h
Position: ({x:6.1f}, {y:6.1f}, {z:5.1f})
Heading:  {np.degrees(heading):6.1f}°
Distance: {self.total_distance:6.1f} m
        """
        self.state_text.set_text(state_str.strip())
        
        # Info
        info_str = f"""
Step:     {self.step_count}
Mode:     AUTOPILOT
Status:   RUNNING
FPS:      ~20

Autopilot is controlling
the vehicle automatically!
        """
        self.info_text.set_text(info_str.strip())
        
        # Update display
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
        
    def run(self, max_steps=1000):
        """Run demo"""
        print("\n" + "=" * 80)
        print("Starting Autopilot Demo")
        print("=" * 80)
        print(f"Max steps: {max_steps}")
        print("Press Ctrl+C to stop")
        print("Close matplotlib window to exit")
        print("")
        
        try:
            self.reset()
            done = False
            
            while not done and self.step_count < max_steps:
                done = self.step()
                
                if self.step_count % 100 == 0:
                    speed = self.obs['ego_state'][0] * 3.6
                    print(f"Step {self.step_count:4d}: Speed={speed:5.1f} km/h, Distance={self.total_distance:6.1f}m")
            
            print(f"\n✅ Demo complete: {self.step_count} steps, {self.total_distance:.1f}m traveled")
            print("Close the matplotlib window to exit...")
            plt.show(block=True)
                
        except KeyboardInterrupt:
            print("\n⏹️  Stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup"""
        print("\n🧹 Cleaning up...")
        if self.env:
            self.env.close()
        print("✅ Cleanup complete")


def main():
    demo = AutopilotDemo()
    demo.run(max_steps=1000)


if __name__ == '__main__':
    main()
