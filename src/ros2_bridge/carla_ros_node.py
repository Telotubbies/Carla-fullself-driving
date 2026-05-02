import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, Image, Imu, NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import carla
import numpy as np
from typing import Optional
import weakref


class CarlaRosNode(Node):
    """
    ROS2 node for CARLA integration.
    Publishes sensor data and subscribes to control commands.
    """
    
    def __init__(self, world: carla.World, vehicle: carla.Actor):
        super().__init__('carla_ros_node')
        
        self.world = world
        self.vehicle = vehicle
        
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
        self.lidar_pub = self.create_publisher(
            PointCloud2,
            '/carla/ego_vehicle/lidar/point_cloud2',
            self.sensor_qos
        )
        
        self.camera_pub = self.create_publisher(
            Image,
            '/carla/ego_vehicle/camera/rgb/image_raw',
            self.sensor_qos
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/carla/ego_vehicle/imu',
            self.sensor_qos
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.sensor_qos
        )
        
        self.speed_pub = self.create_publisher(
            Float32,
            '/carla/ego_vehicle/speedometer',
            self.sensor_qos
        )
        
        # Subscribers
        self.control_sub = self.create_subscription(
            TwistStamped,
            '/carla/ego_vehicle/control_cmd',
            self.control_callback,
            self.control_qos
        )
        
        # CARLA sensors
        self.sensors = {}
        self._setup_sensors()
        
        # Timer for publishing odometry and speed
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        self.get_logger().info('CARLA ROS2 Node initialized')
    
    def _setup_sensors(self):
        """Setup CARLA sensors with ROS2 callbacks."""
        blueprint_library = self.world.get_blueprint_library()
        
        # LiDAR
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('range', '100')
        lidar_bp.set_attribute('points_per_second', '500000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        
        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.5))
        lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        
        weak_self = weakref.ref(self)
        lidar.listen(lambda data: CarlaRosNode._lidar_callback(weak_self, data))
        self.sensors['lidar'] = lidar
        
        # Camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '90')
        
        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=2.4),
            carla.Rotation(pitch=-15)
        )
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        camera.listen(lambda data: CarlaRosNode._camera_callback(weak_self, data))
        self.sensors['camera'] = camera
        
        # IMU
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(x=0.0, z=0.0))
        imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
        imu.listen(lambda data: CarlaRosNode._imu_callback(weak_self, data))
        self.sensors['imu'] = imu
    
    @staticmethod
    def _lidar_callback(weak_self, data):
        """Callback for LiDAR data."""
        self = weak_self()
        if not self:
            return
        
        # Convert CARLA LiDAR data to ROS2 PointCloud2
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        
        # Parse point cloud data
        points = np.frombuffer(data.raw_data, dtype=np.float32)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        
        self.lidar_pub.publish(msg)
    
    @staticmethod
    def _camera_callback(weak_self, data):
        """Callback for camera data."""
        self = weak_self()
        if not self:
            return
        
        # Convert CARLA image to ROS2 Image
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = data.height
        msg.width = data.width
        msg.encoding = 'bgra8'
        msg.is_bigendian = False
        msg.step = data.width * 4
        msg.data = data.raw_data
        
        self.camera_pub.publish(msg)
    
    @staticmethod
    def _imu_callback(weak_self, data):
        """Callback for IMU data."""
        self = weak_self()
        if not self:
            return
        
        # Convert CARLA IMU to ROS2 Imu
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientation (quaternion)
        # Note: CARLA uses different coordinate system
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # Angular velocity
        msg.angular_velocity.x = data.gyroscope.x
        msg.angular_velocity.y = data.gyroscope.y
        msg.angular_velocity.z = data.gyroscope.z
        
        # Linear acceleration
        msg.linear_acceleration.x = data.accelerometer.x
        msg.linear_acceleration.y = data.accelerometer.y
        msg.linear_acceleration.z = data.accelerometer.z
        
        self.imu_pub.publish(msg)
    
    def timer_callback(self):
        """Publish odometry and speed at regular intervals."""
        # Get vehicle state
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        angular_velocity = self.vehicle.get_angular_velocity()
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = transform.location.x
        odom_msg.pose.pose.position.y = transform.location.y
        odom_msg.pose.pose.position.z = transform.location.z
        
        # Orientation (convert from Euler to quaternion)
        roll = np.radians(transform.rotation.roll)
        pitch = np.radians(transform.rotation.pitch)
        yaw = np.radians(transform.rotation.yaw)
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        odom_msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        odom_msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        odom_msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        odom_msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Velocity
        odom_msg.twist.twist.linear.x = velocity.x
        odom_msg.twist.twist.linear.y = velocity.y
        odom_msg.twist.twist.linear.z = velocity.z
        
        odom_msg.twist.twist.angular.x = angular_velocity.x
        odom_msg.twist.twist.angular.y = angular_velocity.y
        odom_msg.twist.twist.angular.z = angular_velocity.z
        
        self.odom_pub.publish(odom_msg)
        
        # Publish speed
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
    
    def control_callback(self, msg: TwistStamped):
        """Callback for control commands."""
        # Extract control from TwistStamped
        # Linear.x = throttle/brake, Angular.z = steering
        
        linear_x = msg.twist.linear.x
        steering = msg.twist.angular.z
        
        # Convert to CARLA control
        control = carla.VehicleControl()
        
        if linear_x >= 0:
            control.throttle = min(linear_x, 1.0)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(-linear_x, 1.0)
        
        control.steer = np.clip(steering, -1.0, 1.0)
        
        self.vehicle.apply_control(control)
    
    def destroy(self):
        """Clean up sensors."""
        for sensor in self.sensors.values():
            if sensor is not None and sensor.is_alive:
                sensor.destroy()
        
        self.sensors.clear()
        self.get_logger().info('CARLA ROS2 Node destroyed')
