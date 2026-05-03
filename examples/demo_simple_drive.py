#!/usr/bin/env python3
"""
Simple Demo: รถขับตรงด้วย constant throttle
รถจะเคลื่อนที่แน่นอน!
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import numpy as np
import matplotlib.pyplot as plt
from src.carla_gym_env import CarlaEnv

class SimpleDriveDemo:
    def __init__(self):
        print("=" * 80)
        print("🚗 Simple Drive Demo - รถจะขับตรงด้วย Constant Throttle")
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
        
        # Setup visualization
        self.setup_visualization()
        
        # State
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
        
        # Actions
        self.ax_actions = plt.subplot(2, 3, 6)
        self.ax_actions.set_title('Actions', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
    def run(self, max_steps=500):
        """Run demo"""
        print("\n" + "=" * 80)
        print("🚀 Starting Simple Drive Demo")
        print("=" * 80)
        print("Action: Constant throttle (0.5), no steering")
        print(f"Max steps: {max_steps}")
        print("Press Ctrl+C to stop")
        print("")
        
        try:
            # Reset
            print("🔄 Resetting environment...")
            obs, _ = self.env.reset()
            print("✅ Reset complete")
            print("")
            
            prev_pos = obs['ego_state'][1:4]
            
            for step in range(max_steps):
                # Simple action: drive straight with constant throttle
                action = np.array([
                    0.0,   # steering = 0 (straight)
                    0.6,   # throttle = 0.6 (moderate speed)
                    0.0    # brake = 0
                ])
                
                # Execute
                obs, reward, done, truncated, info = self.env.step(action)
                
                # Calculate distance
                curr_pos = obs['ego_state'][1:4]
                distance = np.linalg.norm(curr_pos - prev_pos)
                self.total_distance += distance
                prev_pos = curr_pos
                
                self.step_count = step + 1
                
                # Update visualization
                self.update_visualization(obs, action)
                
                # Print progress
                if (step + 1) % 50 == 0:
                    speed = obs['ego_state'][0] * 3.6
                    print(f"Step {step+1:4d}: Speed={speed:5.1f} km/h, Distance={self.total_distance:6.1f}m")
                
                if done or truncated:
                    print(f"\n⚠️  Episode ended: {info.get('termination_reason', 'unknown')}")
                    break
            
            print(f"\n✅ Demo complete!")
            print(f"   Total steps: {self.step_count}")
            print(f"   Distance traveled: {self.total_distance:.1f}m")
            print(f"   Average speed: {(self.total_distance / (self.step_count * 0.05)) * 3.6:.1f} km/h")
            print("\nClose the matplotlib window to exit...")
            plt.show(block=True)
                
        except KeyboardInterrupt:
            print("\n⏹️  Stopped by user")
        finally:
            self.cleanup()
    
    def update_visualization(self, obs, action):
        """Update visualization"""
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
        
        # Get vehicle state
        ego_state = obs['ego_state']
        speed_ms = ego_state[0]
        speed_kmh = speed_ms * 3.6
        x, y, z = ego_state[1:4]
        # ego_state: [speed, x, y, z, lateral_offset, heading_error]
        heading = ego_state[5] if len(ego_state) > 5 else 0.0
        
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
Steps:    {self.step_count}
        """
        self.state_text.set_text(state_str.strip())
        
        # Actions
        action_labels = ['Steering', 'Throttle', 'Brake']
        action_values = action
        colors = ['blue', 'green', 'red']
        
        self.ax_actions.clear()
        self.ax_actions.set_title('Actions', fontsize=14, fontweight='bold')
        self.ax_actions.bar(action_labels, action_values, color=colors, alpha=0.7)
        self.ax_actions.set_ylim(-1, 1)
        self.ax_actions.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax_actions.grid(True, alpha=0.3, axis='y')
        
        # Update display
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def cleanup(self):
        """Cleanup"""
        print("\n🧹 Cleaning up...")
        if hasattr(self, 'env'):
            self.env.close()
        print("✅ Cleanup complete")


def main():
    demo = SimpleDriveDemo()
    demo.run(max_steps=500)


if __name__ == '__main__':
    main()
