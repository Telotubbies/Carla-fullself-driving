from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for CARLA SAC training with ROS2 integration."""
    
    # Declare arguments
    carla_host = DeclareLaunchArgument(
        'carla_host',
        default_value='localhost',
        description='CARLA server host'
    )
    
    carla_port = DeclareLaunchArgument(
        'carla_port',
        default_value='2000',
        description='CARLA server port'
    )
    
    carla_map = DeclareLaunchArgument(
        'carla_map',
        default_value='Town01',
        description='CARLA map to load'
    )
    
    # CARLA server (optional - if not already running)
    # Uncomment if you want to start CARLA server automatically
    # carla_server = ExecuteProcess(
    #     cmd=[
    #         '/opt/carla-simulator/CarlaUE4.sh',
    #         '-RenderOffScreen',
    #         '-carla-rpc-port=2000'
    #     ],
    #     output='screen'
    # )
    
    # CARLA ROS2 bridge node
    carla_ros_node = Node(
        package='carla_sac_ros2_training',
        executable='carla_ros_node',
        name='carla_ros_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('carla_host'),
            'port': LaunchConfiguration('carla_port'),
            'map': LaunchConfiguration('carla_map'),
        }]
    )
    
    # Static transform publishers for sensor frames
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '2.5', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['1.5', '0', '2.4', '0', '-0.26', '0', 'base_link', 'camera_link']
    )
    
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    # RViz2 for visualization (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '$(find carla_sac_ros2_training)/config/carla_viz.rviz'],
        condition='IfCondition(False)'  # Set to True to enable
    )
    
    return LaunchDescription([
        carla_host,
        carla_port,
        carla_map,
        # carla_server,  # Uncomment if needed
        carla_ros_node,
        tf_base_to_lidar,
        tf_base_to_camera,
        tf_base_to_imu,
        # rviz,  # Uncomment if needed
    ])
