"""Launch RViz2 with the preset config for RL debugging."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..", "..", "..", "..", "config", "rviz", "debug.rviz",
        )
    )
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rl_debug_rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
