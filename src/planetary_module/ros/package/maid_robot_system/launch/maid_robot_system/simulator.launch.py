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
    launch_mediapipe_ext_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/mediapipe_ext_node.launch.py'))
    )
    # video
    launch_video_topic_to_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/video_topic_to_service.launch.py'))
    )
    # head unit controller
    launch_head_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/head_unit/head_control_node.launch.py'))
    )
    # waist down unit controller
    launch_waist_down_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/waist_down_unit/waist_down_control_node.launch.py'))
    )

    return LaunchDescription([
        # # video
        launch_video_topic_to_service,
        # # mediapipe
        launch_mediapipe_node,
        launch_mediapipe_ext_node,
        # # controller
        launch_head_control_node,
        launch_waist_down_control_node
    ])
