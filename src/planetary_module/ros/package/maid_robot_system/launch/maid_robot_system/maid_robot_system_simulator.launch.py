import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # mediapipe
    launch_mediapipe_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/mediapipe_node.launch.py'))
    )
    # video
    launch_video_topic_to_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/video_topic_to_service.launch.py'))
    )

    return LaunchDescription([
        # # video
        launch_video_topic_to_service,
        # # mediapipe
        launch_mediapipe_node
    ])
