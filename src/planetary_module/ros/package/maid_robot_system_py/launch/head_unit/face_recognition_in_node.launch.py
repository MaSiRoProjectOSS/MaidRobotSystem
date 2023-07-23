import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    left_face_recognition_in_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='face_recognition_in_node',
        name='left_face_recognition_in_node',
        output=_output_type,
        parameters=[{
            "info/verbose": True,
            "topic_sub_name": "left",
            "device/type": "v4l",
            "device/id": -1,
            "device/by_path": "usb-0:1.1:1.0-video-index0",
            "video/width": 960,
            "video/height": 540,
            "video/angle": 140,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/visibility_th": 0.2,
            "image/width": 800,
            "image/height": 600,
            "image/overlay_information": False,
            "image/publish": False,
            "features/detect_markers": False,
            "update": True,
        }],
        respawn=True,
        respawn_delay=2.0
    )

    right_face_recognition_in_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='face_recognition_in_node',
        name='right_face_recognition_in_node',
        output=_output_type,
        parameters=[{
            "info/verbose": True,
            "topic_sub_name": "right",
            "device/type": "v4l",
            "device/id": -1,
            "device/by_path": "usb-0:1:1.0-video-index0",
            "video/width": 960,
            "video/height": 540,
            "video/angle": 140,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/visibility_th": 0.2,
            "image/width": 800,
            "image/height": 600,
            "image/overlay_information": False,
            "image/publish": False,
            "features/detect_markers": False,
            "update": True,
        }],
        respawn=True,
        respawn_delay=2.0
    )
    return LaunchDescription([
        left_face_recognition_in_node,
        right_face_recognition_in_node
    ])
