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
    output_type = 'screen'

    launch_node_hitomi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/head_unit/node_hitomi.launch.py'))
    )

    launch_node_mitumeru = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/head_unit/node_mitumeru.launch.py'))
    )

    launch_node_kubi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system'),
                'launch/head_unit/node_kubi.launch.py'))
    )

    return LaunchDescription([
        launch_node_hitomi,
        launch_node_mitumeru,
        launch_node_kubi
    ])
