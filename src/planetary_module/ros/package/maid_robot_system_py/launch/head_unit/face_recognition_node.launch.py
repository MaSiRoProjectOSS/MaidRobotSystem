import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    face_recognition_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='face_recognition_node',
        output=_output_type,
        parameters=[{
            "device/left/type": "v4l",
            "device/left/id": -1,
            "device/left/by_path": "usb-0:1.1:1.0-video-index0",
            "device/left/width":  960,
            "device/left/height":  540,
            "device/right/type": "v4l",
            "device/right/id": -1,
            "device/right/by_path": "usb-0:1.1:1.0-video-index1",
            "device/right/width":  960,
            "device/right/height":  540,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/tracking_timeout": 3.0,
            "update": False,
        }],
        respawn=False,
        respawn_delay=2.0
    )

    return LaunchDescription([
        face_recognition_node
    ])
