import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE') + '/head_unit'

    left_detect_ar_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='detect_ar_node',
        name='left_detect_ar_node',
        output=_output_type,
        remappings=[
            ('in', _ros_namespace + '/image/raw/left'),
            ('out', _ros_namespace + '/data/ar/left')
        ],
        parameters=[{
            "confidence/fps": 0.5,
            "preference/video/mirror": False,
            "preference/info/verbose": False
        }],
        respawn=False,
        respawn_delay=2.0
    )

    right_detect_ar_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='detect_ar_node',
        name='right_detect_ar_node',
        output=_output_type,
        remappings=[
            ('in', _ros_namespace + '/image/raw/right'),
            ('out', _ros_namespace + '/data/ar/right')
        ],
        parameters=[{
            "confidence/fps": 0.5,
            "preference/video/mirror": False,
            "preference/info/verbose": False
        }],
        respawn=False,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_detect_ar_node,
        right_detect_ar_node
    ])
