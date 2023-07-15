import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    my_system_name = 'maid_robot_system/'

    template_node = Node(
        namespace=my_system_name,
        package='maid_robot_system_develop',
        executable='template_node',
        parameters=[{
            "template/param/times"  :10,
            "template/param/offset":  1,
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        template_node
    ])
