import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')

    kubi_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system',
        executable='kubi_node',
        output=_output_type,
        parameters=[{
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        kubi_node
    ])
