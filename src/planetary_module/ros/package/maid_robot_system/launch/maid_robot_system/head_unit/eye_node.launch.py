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
    _skin_name = os.environ.get('MRS_CAST_NAME', 'Iris')
    _setting_file = os.environ.get('MRS_CAST_DATA', '/opt/MaidRobotSystem/data/cast/Iris/settings.json')

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
            "setting_file": _setting_file,
            "skin_name": _skin_name,

            "left/width": 1520,
            "left/height": 2190,
            "left/center/x": 135.0,
            "left/center/y": 455.0,
            "left/center/angle": 0,
            "left/eyeball/x": -100.0,
            "left/eyeball/y": 100.0,
            "left/eyeball/angle": 0.0,

            "right/width": 1520,
            "right/height": 2190,
            "right/center/x": -46.0,
            "right/center/y": 455.0,
            "right/center/angle": 0,
            "right/eyeball/x": 100.0,
            "right/eyeball/y": 100.0,
            "right/eyeball/angle": 0.0,

            "eyelid/width": 1520,
            "eyelid/height": 2190,

            "blink_time/quickly_ms": 150,
            "blink_time/min_ms": 400,
            "blink_time/max_ms": 600,
            "blink_time/limit_ms": 15000,
            "blink_time/offset_ms": 0
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        launch_eye_node
    ])
