import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/device'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]
    _skin_name = os.environ.get('MRS_CAST_NAME', 'miko')

    _ros_sub_input = '/head_unit' + '/logic'

    launch_eye_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system',
        executable='eye_node',
        output=_output_type,
        remappings=[
            ('in', _ros_namespace + _ros_sub_input + '/eye')
        ],
        parameters=[{
            "skin_name": _skin_name,
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
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        launch_eye_node
    ])
