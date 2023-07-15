import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    my_system_name = '/maid_robot_system'

    node_rosbridge_server_ws = Node(
        namespace=my_system_name+'/rosbridge',
        package='rosbridge_server',
        executable='rosbridge_websocket',
        parameters=[{
            "port": 9090,
            "ssl": False,
            "address": os.environ['ROS_IP'],
            "certfile": "/etc/ssl/localcerts/apache-selfsigned.pem",
            "keyfile": "/etc/ssl/localcerts/apache-selfsigned.key",
            "authenticate": False,
            "websocket_external_port": 9443,
            "status_level": 'warning'  # "error", "warning", "info", "none"; default "error"
        }],
        respawn=True,
        respawn_delay=10.0
    )
    node_rosbridge_server_wss = Node(
        namespace=my_system_name+'/rosbridge',
        package='rosbridge_server',
        executable='rosbridge_websocket',
        parameters=[{
            "port": 9443,
            "ssl": True,
            "address": os.environ['ROS_IP'],
            "certfile": "/etc/ssl/localcerts/apache-selfsigned.pem",
            "keyfile": "/etc/ssl/localcerts/apache-selfsigned.key",
            "authenticate": False,
            "websocket_external_port": 9443,
            "status_level": 'warning'  # "error", "warning", "info", "none"; default "error"
        }],
        respawn=True,
        respawn_delay=10.0
    )

    return LaunchDescription([
        node_rosbridge_server_ws
    ])
