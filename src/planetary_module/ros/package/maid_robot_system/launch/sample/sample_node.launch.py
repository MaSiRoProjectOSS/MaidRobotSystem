import os
import rclpy
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[
        os.getenv('MRS_ROS_SPAWN', 'false')]

    sample_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system',
        executable='sample_node',
        output=_output_type,
        remappings=[
            ('in',  _ros_namespace + '/messaging/input'),
            ('out', _ros_namespace + '/messaging/output')
        ],
        parameters=[{
            "param": {
                "times": 20.0,
                "offset": 3.0
            }
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        sample_node
    ])
