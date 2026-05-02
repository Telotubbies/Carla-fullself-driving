from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for CARLA SAC evaluation."""
    
    # Declare arguments
    checkpoint_path = DeclareLaunchArgument(
        'checkpoint_path',
        default_value='data/checkpoints/best_model',
        description='Path to model checkpoint for evaluation'
    )
    
    num_episodes = DeclareLaunchArgument(
        'num_episodes',
        default_value='10',
        description='Number of episodes to evaluate'
    )
    
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
    
    # Evaluation node
    eval_node = Node(
        package='carla_sac_ros2_training',
        executable='evaluate_model',
        name='evaluation_node',
        output='screen',
        parameters=[{
            'checkpoint_path': LaunchConfiguration('checkpoint_path'),
            'num_episodes': LaunchConfiguration('num_episodes'),
            'host': LaunchConfiguration('carla_host'),
            'port': LaunchConfiguration('carla_port'),
        }]
    )
    
    # Static transform publishers
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
    
    return LaunchDescription([
        checkpoint_path,
        num_episodes,
        carla_host,
        carla_port,
        eval_node,
        tf_base_to_lidar,
        tf_base_to_camera,
    ])
