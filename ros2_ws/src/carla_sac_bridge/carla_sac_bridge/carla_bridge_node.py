#!/usr/bin/env python3
"""
CARLA Bridge Node - ROS2
เชื่อมต่อ CARLA กับ ROS2 พร้อม Training Guidelines
"""

import sys
sys.path.insert(0, '/home/supawich/Desktop/carla_sac_ros2_training')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge

import carla
import numpy as np
import yaml
from pathlib import Path

from src.carla_gym_env import CarlaEnv
from src.carla_gym_env.enhanced_rewards import EnhancedRewardCalculator
from src.curriculum import CurriculumManager


class CarlaBridgeNode(Node):
    """
    CARLA Bridge Node with Training Guidelines
    - Publish sensor data to ROS2
    - Subscribe to control commands
    - Apply training guidelines
    - Manage curriculum learning
    """
    
    def __init__(self):
        super().__init__('carla_bridge')
        
        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('use_guidelines', True)
        self.declare_parameter('curriculum_enabled', True)
        self.declare_parameter('publish_rate', 20.0)
        
        # Get parameters
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        self.use_guidelines = self.get_parameter('use_guidelines').value
        self.curriculum_enabled = self.get_parameter('curriculum_enabled').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.get_logger().info('='*80)
        self.get_logger().info('🚗 CARLA Bridge Node Starting...')
        self.get_logger().info('='*80)
        self.get_logger().info(f'Host: {host}:{port}')
        self.get_logger().info(f'Guidelines: {self.use_guidelines}')
        self.get_logger().info(f'Curriculum: {self.curriculum_enabled}')
        self.get_logger().info('='*80)
        
        # QoS Profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.camera_pub = self.create_publisher(
            Image, '/carla/ego_vehicle/camera/rgb/image_raw', self.sensor_qos)
        
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/carla/ego_vehicle/lidar/point_cloud2', self.sensor_qos)
        
        self.odom_pub = self.create_publisher(
            Odometry, '/carla/ego_vehicle/odometry', self.sensor_qos)
        
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/carla/ego_vehicle/velocity', self.sensor_qos)
        
        self.metrics_pub = self.create_publisher(
            Float32MultiArray, '/carla/training/metrics', self.sensor_qos)
        
        self.status_pub = self.create_publisher(
            String, '/carla/training/status', self.sensor_qos)
        
        # Subscribers
        self.control_sub = self.create_subscription(
            Twist, '/carla/ego_vehicle/control', 
            self.control_callback, self.control_qos)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize CARLA environment
        self.env = None
        self.curriculum = None
        self.reward_calculator = None
        
        try:
            self.initialize_carla(host, port)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CARLA: {e}')
            raise
        
        # Timers
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_sensor_data)
        
        self.status_timer = self.create_timer(
            1.0, self.publish_status)
        
        # State
        self.episode_count = 0
        self.step_count = 0
        self.current_action = np.array([0.0, 0.0, 0.0])
        
        self.get_logger().info('✅ CARLA Bridge Node Ready!')
    
    def initialize_carla(self, host: str, port: int):
        """Initialize CARLA environment"""
        self.get_logger().info('🔄 Initializing CARLA environment...')
        
        # Load guidelines if enabled
        if self.use_guidelines:
            guidelines_path = '/home/supawich/Desktop/carla_sac_ros2_training/config/training_guidelines.yaml'
            with open(guidelines_path, 'r') as f:
                self.guidelines = yaml.safe_load(f)
            self.get_logger().info('✅ Training guidelines loaded')
        
        # Initialize curriculum if enabled
        if self.curriculum_enabled:
            self.curriculum = CurriculumManager()
            curriculum_config = self.curriculum.get_env_config()
            self.get_logger().info(f'✅ Curriculum initialized: {self.curriculum.current_config.name}')
        else:
            curriculum_config = {}
        
        # Create environment config
        env_config = {
            'host': host,
            'port': port,
            'timeout': 10.0,
            'map': 'Town01',
            'delta_seconds': 0.05,
            'max_episode_steps': 1000,
            'use_camera': True,
            'use_fixed_spawn': True,
            'fixed_spawn_indices': [0, 1, 2],
            'sensor_config': {
                'lidar_channels': 32,
                'lidar_range': 50,
                'bev_range': 25.0,
                'bev_resolution': 256,
                'camera_width': 640,
                'camera_height': 480,
                'camera_fov': 90,
            }
        }
        
        # Merge curriculum config
        env_config.update(curriculum_config)
        
        # Create environment
        self.env = CarlaEnv(env_config)
        self.obs, self.info = self.env.reset()
        
        # Initialize reward calculator if using guidelines
        if self.use_guidelines:
            stage = f"stage{self.curriculum.current_stage_idx + 1}" if self.curriculum else "stage1"
            self.reward_calculator = EnhancedRewardCalculator(
                guidelines_path=guidelines_path,
                stage=stage
            )
            self.get_logger().info(f'✅ Reward calculator initialized: {stage}')
        
        self.get_logger().info('✅ CARLA environment ready')
    
    def control_callback(self, msg: Twist):
        """Handle control commands from ROS2"""
        # Extract control values
        steering = msg.angular.z  # Use angular.z for steering
        throttle = msg.linear.x   # Use linear.x for throttle
        brake = msg.linear.y      # Use linear.y for brake
        
        # Clip values
        steering = np.clip(steering, -1.0, 1.0)
        throttle = np.clip(throttle, 0.0, 1.0)
        brake = np.clip(brake, 0.0, 1.0)
        
        self.current_action = np.array([steering, throttle, brake])
    
    def publish_sensor_data(self):
        """Publish sensor data to ROS2"""
        if self.env is None:
            return
        
        try:
            # Execute action in CARLA
            next_obs, reward, done, truncated, info = self.env.step(self.current_action)
            
            # Publish camera
            if 'camera' in next_obs:
                camera_msg = self.bridge.cv2_to_imgmsg(next_obs['camera'], 'rgb8')
                camera_msg.header.stamp = self.get_clock().now().to_msg()
                camera_msg.header.frame_id = 'ego_vehicle/camera'
                self.camera_pub.publish(camera_msg)
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'ego_vehicle'
            
            # Get vehicle transform
            transform = self.env.vehicle.get_transform()
            odom_msg.pose.pose.position.x = transform.location.x
            odom_msg.pose.pose.position.y = transform.location.y
            odom_msg.pose.pose.position.z = transform.location.z
            
            self.odom_pub.publish(odom_msg)
            
            # Publish velocity
            velocity = self.env.vehicle.get_velocity()
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = 'ego_vehicle'
            vel_msg.twist.linear.x = velocity.x
            vel_msg.twist.linear.y = velocity.y
            vel_msg.twist.linear.z = velocity.z
            
            self.velocity_pub.publish(vel_msg)
            
            # Publish metrics (if using guidelines)
            if self.use_guidelines and self.reward_calculator:
                ego_state = next_obs['ego_state']
                speed = ego_state[0]
                lateral_offset = ego_state[4]
                heading_error = ego_state[5]
                
                metrics_msg = Float32MultiArray()
                metrics_msg.data = [
                    float(speed * 3.6),  # km/h
                    float(self.current_action[0]),  # steering
                    float(self.current_action[1]),  # throttle
                    float(lateral_offset),
                    float(reward),
                    20.0  # FPS placeholder
                ]
                self.metrics_pub.publish(metrics_msg)
            
            # Update observation
            self.obs = next_obs
            self.step_count += 1
            
            # Check if episode done
            if done or truncated:
                self.get_logger().info(f'Episode {self.episode_count} done: {info.get("termination_reason", "unknown")}')
                self.obs, self.info = self.env.reset()
                self.episode_count += 1
                
                # Update curriculum if enabled
                if self.curriculum_enabled and self.curriculum:
                    self.curriculum.record_episode({
                        'total_reward': reward,
                        'avg_lane_deviation': abs(ego_state[4]),
                        'avg_speed': speed * 3.6,
                        'collision': info.get('collision', False),
                        'episode_length': self.step_count
                    })
                    self.step_count = 0
        
        except Exception as e:
            self.get_logger().error(f'Error in publish_sensor_data: {e}')
    
    def publish_status(self):
        """Publish training status"""
        status_msg = String()
        
        if self.curriculum_enabled and self.curriculum:
            progress = self.curriculum.get_progress()
            status = f"Stage: {progress['current_stage_name']}, "
            status += f"Progress: {progress['stage_progress']*100:.1f}%, "
            status += f"Episode: {self.episode_count}"
        else:
            status = f"Episode: {self.episode_count}, Steps: {self.step_count}"
        
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('🧹 Cleaning up CARLA Bridge Node...')
        if self.env:
            self.env.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CarlaBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
