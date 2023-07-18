import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    hitomi_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system',
        executable='hitomi_node',
        output=_output_type,
        parameters=[{
            "skin_name": "miko",
            "l_x": 135.0,
            "l_y": 455.0,
            "r_x": -46.0,
            "r_y": 455.0,
            "eyeball_left_x": -100.0,
            "eyeball_left_y": 100.0,
            "eyeball_right_x": 100.0,
            "eyeball_right_y": 100.0,
            "eyeball_angle": 0.0,
            "eyelid_size_width": 1520,
            "eyelid_size_height": 2190,
            "blink_time_ms_quickly": 150,
            "blink_time_ms_min": 400,
            "blink_time_ms_max": 600,
            "blink_time_ms_limit": 15000,
            "blink_time_ms_offset": 0,
            "tiredness": 25,
            "l_angle": 0,
            "r_angle": 0,
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        hitomi_node
    ])
