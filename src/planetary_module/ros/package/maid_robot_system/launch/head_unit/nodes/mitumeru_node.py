from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    my_system_name = '/maid_robot_system/'
    output_type = 'screen'

    face_recognition_node = Node(
        namespace=my_system_name,
        package='maid_robot_system_py',
        executable='face_recognition_node',
        output=output_type,
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
    mitumeru_node = Node(
        namespace=my_system_name,
        package='maid_robot_system',
        executable='mitumeru_node',
        output=output_type,
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
        # mitumeru_node,
        face_recognition_node
    ])
