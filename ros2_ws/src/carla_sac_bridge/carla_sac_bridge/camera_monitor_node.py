#!/usr/bin/env python3
"""
Camera Monitor Node - ROS2
ดูกล้องและ metrics จาก CARLA Bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime


class CameraMonitorNode(Node):
    """Monitor node สำหรับดูกล้องและ metrics"""
    
    def __init__(self):
        super().__init__('camera_monitor')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/carla/ego_vehicle/camera/rgb/image_raw',
            self.camera_callback,
            10
        )
        
        # Subscribe to metrics
        self.metrics_sub = self.create_subscription(
            Float32MultiArray,
            '/carla/training/metrics',
            self.metrics_callback,
            10
        )
        
        # Subscribe to status
        self.status_sub = self.create_subscription(
            String,
            '/carla/training/status',
            self.status_callback,
            10
        )
        
        # State
        self.metrics = {
            'speed': 0.0,
            'steering': 0.0,
            'throttle': 0.0,
            'lane_deviation': 0.0,
            'reward': 0.0,
            'fps': 0.0
        }
        self.status = 'Waiting...'
        self.last_update = datetime.now()
        self.frame_count = 0
        
        # Heartbeat check
        self.create_timer(2.0, self.check_heartbeat)
        
        self.get_logger().info('🎥 Camera Monitor Node Started')
        self.get_logger().info('📡 Subscribing to:')
        self.get_logger().info('   - /carla/ego_vehicle/camera/rgb/image_raw')
        self.get_logger().info('   - /carla/training/metrics')
        self.get_logger().info('   - /carla/training/status')
        self.get_logger().info('🎮 Press "q" to quit')
    
    def camera_callback(self, msg):
        """Display camera feed"""
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Draw overlay
            self.draw_overlay(cv_image)
            
            # Display
            cv2.imshow('CARLA Training Monitor', cv_image)
            key = cv2.waitKey(1)
            
            if key == ord('q'):
                self.get_logger().info('Quit requested')
                rclpy.shutdown()
            
            self.last_update = datetime.now()
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def metrics_callback(self, msg):
        """Update metrics"""
        if len(msg.data) >= 6:
            self.metrics['speed'] = msg.data[0]
            self.metrics['steering'] = msg.data[1]
            self.metrics['throttle'] = msg.data[2]
            self.metrics['lane_deviation'] = msg.data[3]
            self.metrics['reward'] = msg.data[4]
            self.metrics['fps'] = msg.data[5]
    
    def status_callback(self, msg):
        """Update status"""
        self.status = msg.data
    
    def draw_overlay(self, image):
        """Draw overlay on camera image"""
        h, w = image.shape[:2]
        
        # Main info box
        cv2.rectangle(image, (10, 10), (400, 250), (0, 0, 0), -1)
        cv2.rectangle(image, (10, 10), (400, 250), (0, 255, 0), 2)
        
        # Title
        cv2.putText(image, "CARLA Training Monitor", 
                   (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Metrics
        y = 65
        line_height = 28
        
        # Speed
        speed_color = (0, 255, 0) if self.metrics['speed'] < 40 else (0, 165, 255)
        cv2.putText(image, f"Speed: {self.metrics['speed']:.1f} km/h", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, speed_color, 2)
        y += line_height
        
        # Steering
        steering_bar = int(self.metrics['steering'] * 100)
        cv2.putText(image, f"Steering: {self.metrics['steering']:+.2f}", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.rectangle(image, (200, y-15), (200 + steering_bar, y-5), (0, 255, 0), -1)
        y += line_height
        
        # Throttle
        throttle_bar = int(self.metrics['throttle'] * 100)
        cv2.putText(image, f"Throttle: {self.metrics['throttle']:.2f}", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.rectangle(image, (200, y-15), (200 + throttle_bar, y-5), (0, 255, 0), -1)
        y += line_height
        
        # Lane deviation
        lane_color = (0, 255, 0) if abs(self.metrics['lane_deviation']) < 0.5 else (0, 165, 255)
        cv2.putText(image, f"Lane Dev: {self.metrics['lane_deviation']:+.2f}m", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, lane_color, 2)
        y += line_height
        
        # Reward
        reward_color = (0, 255, 0) if self.metrics['reward'] > 0 else (0, 0, 255)
        cv2.putText(image, f"Reward: {self.metrics['reward']:+.2f}", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, reward_color, 2)
        y += line_height
        
        # FPS
        cv2.putText(image, f"FPS: {self.metrics['fps']:.1f}", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Status box
        cv2.rectangle(image, (10, h-80), (w-10, h-10), (0, 0, 0), -1)
        cv2.rectangle(image, (10, h-80), (w-10, h-10), (0, 255, 0), 2)
        
        # Status text
        cv2.putText(image, self.status, 
                   (20, h-50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(image, timestamp, 
                   (20, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Status indicator
        status_color = (0, 255, 0)
        cv2.circle(image, (w - 30, 30), 15, status_color, -1)
        cv2.putText(image, "LIVE", (w - 80, 38), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Instructions
        cv2.putText(image, "Press 'q' to quit", (w - 180, h - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    def check_heartbeat(self):
        """Check if still receiving data"""
        elapsed = (datetime.now() - self.last_update).total_seconds()
        
        if elapsed > 3.0:
            self.get_logger().warn(f'⚠️  No camera data for {elapsed:.1f}s')
        else:
            self.get_logger().info(f'✅ System OK - FPS: {self.metrics["fps"]:.1f}, Speed: {self.metrics["speed"]:.1f} km/h')
    
    def destroy_node(self):
        """Cleanup"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    print("=" * 80)
    print("🎥 CARLA Camera Monitor - ROS2")
    print("=" * 80)
    print("")
    
    rclpy.init(args=args)
    
    try:
        node = CameraMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  Shutting down...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Camera monitor stopped")


if __name__ == '__main__':
    main()
