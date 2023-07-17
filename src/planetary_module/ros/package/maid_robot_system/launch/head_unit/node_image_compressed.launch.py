from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    my_system_name = '/maid_robot_system/'
    output_type = 'screen'
    arguments_out = 'compressed'  # 'raw'/'compressed'/'theora'

    image_compressed_left_node = Node(
        namespace=my_system_name,
        package='image_transport',
        executable='republish',
        output=output_type,
        name='image_compressed_left_node',
        arguments=[{'raw', arguments_out}],
        parameters=[{
        }],
        remappings=[
            ('in', my_system_name+'image/left/raw'),
            ('out', my_system_name+'image/left/'+arguments_out)
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_compressed_right_node = Node(
        namespace=my_system_name,
        package='image_transport',
        executable='republish',
        output=output_type,
        name='image_compressed_right_node',
        arguments=[{'raw', arguments_out}],
        parameters=[{
        }],
        remappings=[
            {'in', my_system_name+'image/right/raw'},
            {'out', my_system_name+'image/right/'+arguments_out}
        ],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        image_compressed_left_node,
        image_compressed_right_node
    ])
