import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/view'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    _ros_sub_input = '/head_unit' + '/logic'

    left_mediapipe_ext_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_ext_node',
        name='left_mediapipe_ext_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + _ros_sub_input + '/mediapipe/left/pose'),
            ('out', _ros_namespace + _ros_sub_namespace + '/image/left/pose')
        ],
        parameters=[{
            "INTERVAL_MS": 500,
            "timeout_ms": 5000,
            "drawing_box": True,
            "publisher/resize/width": 640,
            "publisher/resize/height": 512
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    right_mediapipe_ext_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_ext_node',
        name='right_mediapipe_ext_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + _ros_sub_input + '/mediapipe/right/pose'),
            ('out', _ros_namespace + _ros_sub_namespace + '/image/right/pose')
        ],
        parameters=[{
            "INTERVAL_MS": 500,
            "timeout_ms": 5000,
            "drawing_box": True,
            "publisher/resize/width": 640,
            "publisher/resize/height": 512
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_mediapipe_ext_node,
        right_mediapipe_ext_node
    ])
