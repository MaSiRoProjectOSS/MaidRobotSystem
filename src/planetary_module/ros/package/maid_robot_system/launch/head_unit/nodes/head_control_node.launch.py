import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/logic'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    _ros_sub_input_pose = '/head_unit' + '/logic'
    _ros_sub_input_ar = '/head_unit' + '/logic'
    _ros_sub_input_voice = '/head_unit' + '/controller' + '/microphone'

    launch_head_control_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system',
        executable='head_control_node',
        output=_output_type,
        remappings=[
            ('in/pose/left', _ros_namespace + _ros_sub_input_pose + '/in/pose/left'),
            ('in/pose/right', _ros_namespace + _ros_sub_input_pose + '/in/pose/right'),
            ('in/ar/left', _ros_namespace + _ros_sub_input_ar + '/in/ar/left'),
            ('in/ar/right', _ros_namespace + _ros_sub_input_ar + '/in/ar/right'),
            ('in/voice', _ros_namespace + _ros_sub_input_voice + '/data/voice'),
            ('out/eye', _ros_namespace + _ros_sub_namespace + '/data/eye'),
            ('out/neck', _ros_namespace + _ros_sub_namespace + '/data/neck'),
            ('out/lip', _ros_namespace + _ros_sub_namespace + '/data/lip')
        ],
        parameters=[{
            "id": -1
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        launch_head_control_node
    ])
