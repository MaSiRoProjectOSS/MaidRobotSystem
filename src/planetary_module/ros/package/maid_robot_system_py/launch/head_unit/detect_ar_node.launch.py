import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = "{}{}".format(os.environ.get('MRS_ROS_NAMESPACE'), '/head_unit')

    left_detect_ar_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='detect_ar_node',
        name='left_detect_ar_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + '/video/left/image'),
            ('out', _ros_namespace + '/data/left/ar')
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
        respawn=True,
        respawn_delay=2.0
    )

    right_detect_ar_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='detect_ar_node',
        name='right_detect_ar_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + '/video/right/image'),
            ('out', _ros_namespace + '/data/right/ar')
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
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_detect_ar_node,
        right_detect_ar_node
    ])
