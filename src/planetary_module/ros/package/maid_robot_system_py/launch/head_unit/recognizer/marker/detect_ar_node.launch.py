import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/logic' + '/marker'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    _ros_sub_input = '/head_unit' + '/controller' + os.environ.get('MRS_TYPE_INPUT_VIDEO_DEVICE', '/camera')

    left_detect_ar_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='detect_ar_node',
        name='left_detect_ar_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + _ros_sub_input + '/image/inquiry/left'),
            ('out', _ros_namespace + _ros_sub_namespace + '/ar/left')
        ],
        parameters=[{
            "INTERVAL_MS": 500,
            "timeout_ms": 5000,
            "mirror": False,
            "upside_down": False,
            "width": 320,
            "height": 256,
            "notify/message/verbose": False
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    right_detect_ar_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='detect_ar_node',
        name='right_detect_ar_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + _ros_sub_input + '/image/inquiry/right'),
            ('out', _ros_namespace + _ros_sub_namespace + '/ar/right')
        ],
        parameters=[{
            "INTERVAL_MS": 500,
            "timeout_ms": 5000,
            "mirror": False,
            "upside_down": False,
            "width": 320,
            "height": 256,
            "notify/message/verbose": False
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_detect_ar_node,
        right_detect_ar_node
    ])
