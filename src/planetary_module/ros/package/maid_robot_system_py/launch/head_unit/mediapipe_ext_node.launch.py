import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE') + '/head_unit'

    right_mediapipe_ext_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_ext_node',
        name='right_mediapipe_ext_node',
        output=_output_type,
        remappings=[
            ('in/image', _ros_namespace + '/image/raw/right'),
            ('in/pose_data', _ros_namespace + '/data/pose/right'),
            ('out/image', _ros_namespace + '/image/pose/right'),
        ],
        parameters=[{
            "configuration/publisher/interval_fps": 4.0,
            "preference/info/verbose": True,
            "preference/image/drawing_box": True,
            "preference/image/width": 320,
            "preference/image/height": 256
        }],
        respawn=False,
        respawn_delay=2.0
    )

    return LaunchDescription([
        # left_mediapipe_ext_node,
        right_mediapipe_ext_node
    ])
