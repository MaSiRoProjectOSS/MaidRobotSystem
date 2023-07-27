import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    left_video_capture_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='video_capture_node',
        name='left_video_capture_node',
        output=_output_type,
        remappings=[
            ('image', _ros_namespace + '/image/raw/left')
        ],
        parameters=[{
            "info/verbose": True,
            "device/type": "v4l",
            "device/by_path": "",
            "device/id": 1,
            "video/settings/format": "MJPG",
            "video/settings/mirror": True,
            "video/settings/width": 1280,
            "video/settings/height": 1024,
            "video/settings/angle": 140,
            "video/area/center_x": 0,
            "video/area/center_y": 0,
            "video/area/width": 0,
            "video/area/height": 0,
            "publisher/resize/width": 0,
            "publisher/resize/height": 0
        }],
        respawn=True,
        respawn_delay=2.0
    )

    right_video_capture_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='video_capture_node',
        name='right_video_capture_node',
        output=_output_type,
        remappings=[
            ('image', _ros_namespace + '/image/raw/right')
        ],
        parameters=[{
            "info/verbose": True,
            "device/type": "v4l",
            "device/by_path": "",
            "device/id": 3,
            "video/settings/format": "MJPG",
            "video/settings/mirror": True,
            "video/settings/width": 1280,
            "video/settings/height": 1024,
            "video/settings/angle": 140,
            "video/area/center_x": 0,
            "video/area/center_y": 0,
            "video/area/width": 0,
            "video/area/height": 0,
            "publisher/resize/width": 0,
            "publisher/resize/height": 0
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_video_capture_node,
        right_video_capture_node
    ])
