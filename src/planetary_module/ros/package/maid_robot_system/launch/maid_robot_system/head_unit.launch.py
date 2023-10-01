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
    launch_video_capture_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/video_capture_node.launch.py'))
    )
    # voice
    launch_voice_recognition_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/head_unit/voice_recognition_node.launch.py'))
    )
    # controller
    launch_head_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/head_unit/head_control_node.launch.py'))
    )
    # device driver
    launch_eye_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/head_unit/eye_node.launch.py'))
    )
    launch_neck_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/head_unit/neck_node.launch.py'))
    )
    launch_lip_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/maid_robot_system/head_unit/lip_node.launch.py'))
    )

    return LaunchDescription([
        # # mediapipe
        launch_mediapipe_node,
        # # video
        launch_video_capture_node,
        # # voice
        launch_voice_recognition_node,
        # # controller
        launch_head_control_node,
        # # device driver
        launch_eye_node,
        launch_neck_node,
        launch_lip_node
    ])
