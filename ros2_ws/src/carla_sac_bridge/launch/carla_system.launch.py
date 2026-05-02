#!/usr/bin/env python3
"""
CARLA System Launch File
เปิดระบบทั้งหมด: Bridge + Camera Monitor
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('carla_sac_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'carla_config.yaml')
    
    # Launch arguments
    carla_host_arg = DeclareLaunchArgument(
        'carla_host',
        default_value='localhost',
        description='CARLA server host'
    )
    
    carla_port_arg = DeclareLaunchArgument(
        'carla_port',
        default_value='2000',
        description='CARLA server port'
    )
    
    use_guidelines_arg = DeclareLaunchArgument(
        'use_guidelines',
        default_value='true',
        description='Use training guidelines'
    )
    
    curriculum_enabled_arg = DeclareLaunchArgument(
        'curriculum_enabled',
        default_value='true',
        description='Enable curriculum learning'
    )
    
    # CARLA Bridge Node
    carla_bridge_node = Node(
        package='carla_sac_bridge',
        executable='carla_bridge',
        name='carla_bridge',
        output='screen',
        parameters=[{
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            'use_guidelines': LaunchConfiguration('use_guidelines'),
            'curriculum_enabled': LaunchConfiguration('curriculum_enabled'),
            'publish_rate': 20.0,
        }],
        emulate_tty=True,
    )
    
    # Camera Monitor Node
    camera_monitor_node = Node(
        package='carla_sac_bridge',
        executable='camera_monitor',
        name='camera_monitor',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        carla_host_arg,
        carla_port_arg,
        use_guidelines_arg,
        curriculum_enabled_arg,
        carla_bridge_node,
        camera_monitor_node,
    ])
