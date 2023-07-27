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

    recognition_in_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maid_robot_system_py'),
                'launch/recognition_node.launch.py'))
    )
    mitumeru_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system',
        executable='mitumeru_node',
        output=_output_type,
        parameters=[{
            "device": {
                "left": {
                    "type": "v4l",
                    "id": -1,
                    "by_path": "usb-0:1.1:1.0-video-index0",
                    "width": 960,
                    "height": 540
                },
                "right": {
                    "type": "v4l",
                    "id": -1,
                    "by_path": "usb-0:1:1.0-video-index0",
                    "width": 960,
                    "height": 540
                }
            },
            "confidence": {
                "min_detection": 0.5,
                "min_tracking": 0.5
            },
            "update": False
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        mitumeru_node,
        recognition_in_node
    ])
