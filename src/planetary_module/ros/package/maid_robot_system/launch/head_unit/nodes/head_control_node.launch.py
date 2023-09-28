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

    _ros_sub_input_posture = '/head_unit' + '/logic' + '/posture'
    _ros_sub_input_ar = '/head_unit' + '/logic' + '/marker'
    _ros_sub_input_voice = '/head_unit' + '/controller' + '/microphone'
    _ros_sub_input_wattmeter = '/power_unit' + '/controller' + '/wattmeter'

    launch_head_control_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system',
        executable='head_control_node',
        output=_output_type,
        remappings=[
            ('in/posture/left', _ros_namespace + _ros_sub_input_posture + '/mediapipe/landmarks/left'),
            ('in/posture/right', _ros_namespace + _ros_sub_input_posture + '/mediapipe/landmarks/right'),
            ('in/marks/left', _ros_namespace + _ros_sub_input_ar + '/ar/left'),
            ('in/marks/right', _ros_namespace + _ros_sub_input_ar + '/ar/right'),
            ('in/voice_text', _ros_namespace + _ros_sub_input_voice + '/voice_text'),
            ('in/voltage', _ros_namespace + _ros_sub_input_wattmeter + '/voltage'),
            ('out/eye', _ros_namespace + _ros_sub_namespace + '/eye'),
            ('out/neck', _ros_namespace + _ros_sub_namespace + '/neck'),
            ('out/lip', _ros_namespace + _ros_sub_namespace + '/lip')
        ],
        parameters=[{
            "eye/left/center/x": 0,
            "eye/left/center/y": 0,
            "eye/left/offset/up": 0,
            "eye/left/offset/down": 0,

            "eye/right/center/x": 0,
            "eye/right/center/y": 0,
            "eye/right/offset/up": 0,
            "eye/right/offset/down": 0,

            "neck/pitch/min": 0,
            "neck/pitch/max": 0,
            "neck/yaw/min": 0,
            "neck/yaw/max": 0,
            "neck/roll/min": 0,
            "neck/roll/max": 0,

            "lip/min": 0,
            "lip/max": 0,
            "tiredness": 25
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        launch_head_control_node
    ])
