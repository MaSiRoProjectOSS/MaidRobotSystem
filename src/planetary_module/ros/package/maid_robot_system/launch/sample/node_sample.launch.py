import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    my_system_name = 'maid_robot_system/'

    sample_node = Node(
        namespace=my_system_name,
        package='maid_robot_system',
        executable='sample_node',
        parameters=[{
            "param": {
                "times": 20.0,
                "offset": 3.0
            }
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        sample_node
    ])
