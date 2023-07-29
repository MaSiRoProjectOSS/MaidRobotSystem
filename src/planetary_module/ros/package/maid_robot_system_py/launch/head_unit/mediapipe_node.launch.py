import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE') + '/head_unit'

    right_mediapipe_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_node',
        name='right_mediapipe_node',
        output=_output_type,
        remappings=[
            ('in', _ros_namespace + '/image/raw/right'),
            ('out1', _ros_namespace + '/image/raw/right'),
            ('out2', _ros_namespace + '/data/right')
        ],
        parameters=[{
            "configuration/publisher/interval_fps": 10.0,
            "preference/info/verbose": True,
            "preference/confidence/min_detection": 0.5,
            "preference/confidence/min_tracking": 0.5,
            "preference/confidence/visibility_th": 0.5,
            "preference/video/area/center_x": 0,
            "preference/video/area/center_y": 0,
            "preference/video/area/width": 0,
            "preference/video/area/height": 0,
            "preference/image/publish": True
        }],
        respawn=False,
        respawn_delay=2.0
    )
    return LaunchDescription([
        right_mediapipe_node
    ])
