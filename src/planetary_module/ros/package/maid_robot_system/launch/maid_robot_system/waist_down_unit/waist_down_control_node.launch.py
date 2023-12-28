import os
import rclpy
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace_waist_down = '/waist_down_unit' + '/controller'
    _ros_sub_namespace_arm = '/arm_unit' + '/controller'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    launch_waist_down_control_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace_waist_down,
        package='maid_robot_system',
        executable='waist_down_control_node',
        output=_output_type,
        remappings=[
            ('in/robot_position_rotation', _ros_namespace + _ros_sub_namespace_waist_down + '/robot_position_rotation'),
            ('in/hand_position', _ros_namespace + _ros_sub_namespace_arm + '/hand_position'),
            ('out', _ros_namespace + _ros_sub_namespace_waist_down + '/move_velocity_reference')
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
        launch_waist_down_control_node
    ])
