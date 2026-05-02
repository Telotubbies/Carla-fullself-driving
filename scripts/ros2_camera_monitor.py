#!/usr/bin/env python3
"""
ROS2 Camera Monitor
ดูกล้องจาก CARLA ผ่าน ROS2 เพื่อเช็คว่าระบบยังทำงานอยู่
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime


class CameraMonitor(Node):
    """Monitor CARLA ผ่านกล้อง - เช็คว่ายังทำงานอยู่ไหม"""
    
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
        
        # Subscribe to odometry (speed, position)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            10
        )
        
        # Subscribe to velocity
        self.vel_sub = self.create_subscription(
            TwistStamped,
            '/carla/ego_vehicle/velocity',
            self.velocity_callback,
            10
        )
        
        # State
        self.latest_image = None
        self.speed = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.last_update = datetime.now()
        self.frame_count = 0
        self.fps = 0.0
        
        # FPS calculation
        self.fps_timer = self.create_timer(1.0, self.calculate_fps)
        self.frames_in_second = 0
        
        # Heartbeat check
        self.heartbeat_timer = self.create_timer(2.0, self.check_heartbeat)
        
        self.get_logger().info('🎥 Camera Monitor Started')
        self.get_logger().info('📡 Waiting for camera data...')
    
    def camera_callback(self, msg):
        """แสดงกล้อง"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Draw overlay
            self.draw_overlay(cv_image)
            
            # Display
            cv2.imshow('CARLA Camera Monitor', cv_image)
            key = cv2.waitKey(1)
            
            # Press 'q' to quit
            if key == ord('q'):
                self.get_logger().info('Quit requested')
                rclpy.shutdown()
            
            # Update state
            self.last_update = datetime.now()
            self.frame_count += 1
            self.frames_in_second += 1
            
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def odom_callback(self, msg):
        """รับข้อมูล odometry"""
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
    
    def velocity_callback(self, msg):
        """รับข้อมูลความเร็ว"""
        # คำนวณความเร็ว (m/s -> km/h)
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        speed_ms = np.sqrt(vx**2 + vy**2)
        self.speed = speed_ms * 3.6
    
    def draw_overlay(self, image):
        """วาด overlay บนกล้อง"""
        h, w = image.shape[:2]
        
        # Background box
        cv2.rectangle(image, (10, 10), (350, 180), (0, 0, 0), -1)
        cv2.rectangle(image, (10, 10), (350, 180), (0, 255, 0), 2)
        
        # Title
        cv2.putText(image, "CARLA System Monitor", 
                   (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Metrics
        y = 65
        line_height = 25
        
        # Speed
        cv2.putText(image, f"Speed: {self.speed:.1f} km/h", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y += line_height
        
        # Position
        cv2.putText(image, f"Pos: ({self.position[0]:.1f}, {self.position[1]:.1f})", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y += line_height
        
        # FPS
        cv2.putText(image, f"FPS: {self.fps:.1f}", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y += line_height
        
        # Frame count
        cv2.putText(image, f"Frames: {self.frame_count}", 
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Status indicator (top right)
        status_color = (0, 255, 0)  # Green = OK
        cv2.circle(image, (w - 30, 30), 15, status_color, -1)
        cv2.putText(image, "LIVE", (w - 80, 38), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Timestamp (bottom)
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(image, timestamp, (10, h - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Instructions
        cv2.putText(image, "Press 'q' to quit", (w - 180, h - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    def calculate_fps(self):
        """คำนวณ FPS"""
        self.fps = self.frames_in_second
        self.frames_in_second = 0
    
    def check_heartbeat(self):
        """ตรวจสอบว่ายังได้รับข้อมูลอยู่ไหม"""
        elapsed = (datetime.now() - self.last_update).total_seconds()
        
        if elapsed > 3.0:
            self.get_logger().warn(f'⚠️  No camera data for {elapsed:.1f}s')
            self.get_logger().warn('   Check if CARLA and ROS2 bridge are running!')
        else:
            self.get_logger().info(f'✅ System OK - FPS: {self.fps:.1f}, Speed: {self.speed:.1f} km/h')


def main(args=None):
    print("=" * 80)
    print("🎥 CARLA Camera Monitor - ROS2")
    print("=" * 80)
    print("📡 Topics:")
    print("   - /carla/ego_vehicle/camera/rgb/image_raw")
    print("   - /carla/ego_vehicle/odometry")
    print("   - /carla/ego_vehicle/velocity")
    print("")
    print("🎮 Controls:")
    print("   - Press 'q' to quit")
    print("=" * 80)
    print("")
    
    rclpy.init(args=args)
    node = CameraMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  Shutting down...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Camera monitor stopped")


if __name__ == '__main__':
    main()
