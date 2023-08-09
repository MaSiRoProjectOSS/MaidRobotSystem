import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = "{}{}".format(os.environ.get('MRS_ROS_NAMESPACE'), '/head_unit')

    left_mediapipe_ext_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_ext_node',
        name='left_mediapipe_ext_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + '/mediapipe/pose/left'),
            ('out', _ros_namespace + '/image/pose/left')
        ],
        parameters=[{
            "INTERVAL_MS": 500,
            "timeout_ms": 5000,
            "drawing_box": True,
            "publisher/resize/width": 640,
            "publisher/resize/height": 512
        }],
        respawn=False,
        respawn_delay=2.0
    )

    right_mediapipe_ext_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_ext_node',
        name='right_mediapipe_ext_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + '/image/add_pose/right'),
            ('out', _ros_namespace + '/image/pose/right')
        ],
        parameters=[{
            "INTERVAL_MS": 500,
            "timeout_ms": 5000,
            "drawing_box": True,
            "publisher/resize/width": 640,
            "publisher/resize/height": 512
        }],
        respawn=False,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_mediapipe_ext_node,
        right_mediapipe_ext_node
    ])
