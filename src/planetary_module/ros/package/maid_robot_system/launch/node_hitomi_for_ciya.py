import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    my_system_name = '/maid_robot_system/'

    hitomi_node = Node(
        namespace=my_system_name,
        package='maid_robot_system',
        executable='hitomi_node',
        parameters=[{
            "skin_name": "ciya",
            "l_x": 145.0,
            "l_y": 265.0,
            "r_x": -186.0,
            "r_y": 261.0,
            "eyeball_left_x": -25.0,
            "eyeball_left_y": 145.0,
            "eyeball_right_x": 110.0,
            "eyeball_right_y": 185.0,
            "eyeball_angle": 0.0,
            "eyelid_size_width": 1480,
            "eyelid_size_height": 1350,
            "blink_time_ms_quickly": 150,
            "blink_time_ms_min": 450,
            "blink_time_ms_max": 550,
            "blink_time_ms_limit": 15000,
            "blink_time_ms_offset": 0,
            "tiredness": 25,
            "l_angle": -2,
            "r_angle": -5.3
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        hitomi_node
    ])
