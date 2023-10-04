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
            "brightness": 100,
            "color/r": 255,
            "color/g": 255,
            "color/b": 255
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        launch_eye_node
    ])
