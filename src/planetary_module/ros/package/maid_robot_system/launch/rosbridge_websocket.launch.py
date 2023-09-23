import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    _ros_namespace = "{}{}".format(
        os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system'), '/rosbridge')
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[
        os.getenv('MRS_ROS_SPAWN', 'false')]

    node_rosbridge_server_ws = Node(
        namespace=_ros_namespace,
        package='rosbridge_server',
        executable='rosbridge_websocket',
        parameters=[{
            "port": 9090,
            "ssl": False,
            "address": os.environ.get('ROS_IP', '127.0.0.1'),
            "certfile": "/etc/ssl/localcerts/apache-selfsigned.pem",
            "keyfile": "/etc/ssl/localcerts/apache-selfsigned.key",
            "authenticate": False,
            "websocket_external_port": 9443,
            "status_level": 'warning'  # "error", "warning", "info", "none"; default "error"
        }],
        ros_arguments=['--log-level', _log_level],
        output=_output_type,
        respawn=_res_pawn,
        respawn_delay=10.0
    )
    node_rosbridge_server_wss = Node(
        namespace=_ros_namespace,
        package='rosbridge_server',
        executable='rosbridge_websocket',
        parameters=[{
            "port": 9443,
            "ssl": True,
            "address": os.environ.get('ROS_IP', '127.0.0.1'),
            "certfile": "/etc/ssl/localcerts/apache-selfsigned.pem",
            "keyfile": "/etc/ssl/localcerts/apache-selfsigned.key",
            "authenticate": False,
            "websocket_external_port": 9443,
            "status_level": 'warning'  # "error", "warning", "info", "none"; default "error"
        }],
        ros_arguments=['--log-level', _log_level],
        output=_output_type,
        respawn=_res_pawn,
        respawn_delay=10.0
    )

    return LaunchDescription([
        node_rosbridge_server_ws
    ])
