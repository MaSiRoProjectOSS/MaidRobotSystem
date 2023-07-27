import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    left_recognition_in_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='recognition_in_node',
        name='left_recognition_in_node',
        output=_output_type,
        parameters=[{
            "info/verbose": True,
            "topic_sub_name": "left",
            "device/type": "v4l",
            "device/id": -1,
            "device/by_path": "usb-0:1.1:1.0-video-index0",
            "video/settings/width": 1280,
            "video/settings/height": 1024,
            "video/settings/angle": 140,
            "video/area/center_x": 0,
            "video/area/center_y": 0,
            "video/area/width": 0,
            "video/area/height": 0,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/visibility_th": 0.5,
            "image/width": 1280,
            "image/height": 1024,
            "image/overlay_information": False,
            "image/publish": True,
            "features/detect_markers": False,
            "update": True,
        }],
        respawn=False,
        respawn_delay=2.0
    )

    right_recognition_in_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='recognition_in_node',
        name='right_recognition_in_node',
        output=_output_type,
        parameters=[{
            "info/verbose": True,
            "topic_sub_name": "right",
            "device/type": "v4l",
            "device/id": -1,
            "device/by_path": "usb-0:1:1.0-video-index0",
            "video/settings/width": 1280,
            "video/settings/height": 1024,
            "video/settings/angle": 140,
            "video/area/center_x": 0,
            "video/area/center_y": 0,
            "video/area/width": 0,
            "video/area/height": 0,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/visibility_th": 0.5,
            "image/width": 1280,
            "image/height": 1024,
            "image/overlay_information": False,
            "image/publish": True,
            "features/detect_markers": False,
            "update": True,
        }],
        respawn=False,
        respawn_delay=2.0
    )
    return LaunchDescription([
        # left_recognition_in_node,
        right_recognition_in_node
    ])
