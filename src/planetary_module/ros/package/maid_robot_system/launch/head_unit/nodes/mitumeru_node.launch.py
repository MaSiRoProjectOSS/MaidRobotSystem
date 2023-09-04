import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    mitumeru_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system',
        executable='mitumeru_node',
        output=_output_type,
        remappings=[
            ('in/pose/left', _ros_namespace + '/in/pose/left'),
            ('in/pose/right', _ros_namespace + '/in/pose/right'),
            ('in/ar/left', _ros_namespace + '/in/ar/left'),
            ('in/ar/right', _ros_namespace + '/in/ar/right'),
            ('in/voice', _ros_namespace + '/data/voice'),
            ('out/hitomi', _ros_namespace + '/data/hitomi'),
            ('out/kubi', _ros_namespace + '/data/kubi'),
            ('out/kuchibiru', _ros_namespace + '/data/kuchibiru')
        ],
        parameters=[{
            "id": -1
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        mitumeru_node
    ])
