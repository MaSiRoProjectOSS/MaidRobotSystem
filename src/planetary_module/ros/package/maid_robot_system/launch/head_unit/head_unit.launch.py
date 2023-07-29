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

    launch_nodes_hitomi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/head_unit/nodes/hitomi_node.launch.py'))
    )

    launch_nodes_mitumeru = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/head_unit/nodes/mitumeru_node.launch.py'))
    )

    launch_nodes_kubi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/head_unit/nodes/kubi_node.launch.py'))
    )

    launch_nodes_video_capture_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/video_capture_node.launch.py'))
    )

    launch_nodes_detect_ar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/detect_ar_node.launch.py'))
    )

    launch_nodes_mediapipe_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/mediapipe_node.launch.py'))
    )

    return LaunchDescription([
        # launch_nodes_hitomi,
        # launch_nodes_mitumeru,
        # launch_nodes_kubi,
        launch_nodes_mediapipe_node,
        launch_nodes_detect_ar_node,
        launch_nodes_video_capture_node
    ])
