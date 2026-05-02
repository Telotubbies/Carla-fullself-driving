import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import numpy as np


class SensorPublisher(Node):
    """Standalone sensor publisher for CARLA data."""
    
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.lidar_pub = self.create_publisher(
            PointCloud2,
            '/carla/ego_vehicle/lidar/point_cloud2',
            sensor_qos
        )
        
        self.camera_pub = self.create_publisher(
            Image,
            '/carla/ego_vehicle/camera/rgb/image_raw',
            sensor_qos
        )
        
        self.get_logger().info('Sensor Publisher initialized')
    
    def publish_lidar(self, points: np.ndarray, frame_id: str = 'lidar_link'):
        """Publish LiDAR point cloud."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        
        self.lidar_pub.publish(msg)
    
    def publish_camera(self, image: np.ndarray, frame_id: str = 'camera_link'):
        """Publish camera image."""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8' if image.shape[2] == 3 else 'bgra8'
        msg.is_bigendian = False
        msg.step = image.shape[1] * image.shape[2]
        msg.data = image.tobytes()
        
        self.camera_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
