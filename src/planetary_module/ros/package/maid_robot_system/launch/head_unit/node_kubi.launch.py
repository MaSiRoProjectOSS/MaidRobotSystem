from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    my_system_name = '/maid_robot_system/'
    output_type = 'screen'

    kubi_node = Node(
        namespace=my_system_name,
        package='maid_robot_system',
        executable='kubi_node',
        output=output_type,
        parameters=[{
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        kubi_node
    ])
