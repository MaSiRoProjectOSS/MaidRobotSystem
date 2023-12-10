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
    _skin_name = os.environ.get('MRS_CAST_NAME', 'Ayame')
    _setting_file = os.environ.get('MRS_CAST_DATA', '/opt/MaidRobotSystem/data/cast/Ayame/settings.json')

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
            "notify/message/enable": False,
            "notify/message/verbose": False,
            "setting_file": _setting_file,
            "brightness": 100,
            "eyelid/color/r": 231,
            "eyelid/color/g": 183,
            "eyelid/color/b": 147,
            "ciliary/color/r": 255,
            "ciliary/color/g": 255,
            "ciliary/color/b": 255
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        launch_eye_node
    ])
