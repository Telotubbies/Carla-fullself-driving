import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
from typing import Callable, Optional


class ControlSubscriber(Node):
    """Subscriber for vehicle control commands."""
    
    def __init__(self, control_callback: Optional[Callable] = None):
        super().__init__('control_subscriber')
        
        self.control_callback = control_callback
        
        # QoS for control commands
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to control commands
        self.control_sub = self.create_subscription(
            TwistStamped,
            '/carla/ego_vehicle/control_cmd',
            self.control_msg_callback,
            control_qos
        )
        
        # Alternative: subscribe to raw control array [steering, throttle, brake]
        self.raw_control_sub = self.create_subscription(
            Float32MultiArray,
            '/carla/ego_vehicle/raw_control',
            self.raw_control_callback,
            control_qos
        )
        
        self.latest_control = None
        
        self.get_logger().info('Control Subscriber initialized')
    
    def control_msg_callback(self, msg: TwistStamped):
        """Callback for TwistStamped control messages."""
        # Extract control: linear.x = throttle/brake, angular.z = steering
        linear_x = msg.twist.linear.x
        steering = msg.twist.angular.z
        
        # Convert to [steering, throttle, brake]
        if linear_x >= 0:
            control = np.array([steering, linear_x, 0.0], dtype=np.float32)
        else:
            control = np.array([steering, 0.0, -linear_x], dtype=np.float32)
        
        self.latest_control = control
        
        if self.control_callback is not None:
            self.control_callback(control)
    
    def raw_control_callback(self, msg: Float32MultiArray):
        """Callback for raw control array [steering, throttle, brake]."""
        if len(msg.data) >= 3:
            control = np.array(msg.data[:3], dtype=np.float32)
            self.latest_control = control
            
            if self.control_callback is not None:
                self.control_callback(control)
    
    def get_latest_control(self) -> Optional[np.ndarray]:
        """Get the latest control command."""
        return self.latest_control
    
    def set_control_callback(self, callback: Callable):
        """Set the control callback function."""
        self.control_callback = callback


def main(args=None):
    rclpy.init(args=args)
    
    def print_control(control):
        print(f"Received control: steering={control[0]:.3f}, throttle={control[1]:.3f}, brake={control[2]:.3f}")
    
    node = ControlSubscriber(control_callback=print_control)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
