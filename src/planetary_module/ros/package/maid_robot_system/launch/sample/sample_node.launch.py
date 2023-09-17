import os
import rclpy
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL')
    _ros_namespace = '/maid_robot_system'

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
        ros_arguments=[ '--log-level', _log_level],
        respawn=False,
        respawn_delay=2.0
    )

    return LaunchDescription([
        sample_node
    ])
