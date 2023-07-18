from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    my_system_name = '/maid_robot_system/'
    output_type = 'screen'
    transport_type = 'compressed'  # 'raw'/'compressed'/'theora'

    image_compressed_left_node = Node(
        namespace=my_system_name,
        package='image_transport',
        executable='republish',
        output=output_type,
        name='image_compressed_left_node',
        arguments=['compressed'],
        remappings=[
            ('in', my_system_name+'image/left/raw'),
            ('out', my_system_name+'image/left/compressed')
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
        arguments=['compressed'],
        remappings=[
            ('in', my_system_name+'image/right/raw'),
            ('out', my_system_name+'image/right/compressed')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_theora_left_node = Node(
        namespace=my_system_name,
        package='theora_image_transport',
        executable='ogg_saver',
        output=output_type,
        name='image_theora_left_node',
        arguments=['data/streaming_00.ogv'],
        remappings=[
            ('image', my_system_name+'image/left/raw')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_theora_right_node = Node(
        namespace=my_system_name,
        package='theora_image_transport',
        executable='ogg_saver',
        output=output_type,
        name='image_theora_right_node',
        arguments=['data/streaming_00.ogv'],
        remappings=[
            ('image', my_system_name+'image/left/raw')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_transport_left_node = Node(
        namespace=my_system_name,
        package='image_transport',
        executable='republish',
        output=output_type,
        name='image_transport_left_node',
        arguments=['raw'],
        remappings=[
            ('in', my_system_name+'image/left/raw'),
            ('out', my_system_name+'image/right/raw')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_transport_right_node = Node(
        namespace=my_system_name,
        package='image_transport',
        executable='republish',
        output=output_type,
        name='image_transport_right_node',
        arguments=['raw'],
        remappings=[
            ('in', my_system_name+'image/right/raw'),
            ('out', my_system_name+'image/right/transport')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    if 'raw' == transport_type:
        return LaunchDescription([
            image_transport_left_node,
            image_transport_right_node
        ])
    elif 'compressed' == transport_type:
        return LaunchDescription([
            #   image_compressed_left_node,
            image_compressed_right_node
        ])
    elif 'theora' == transport_type:
        return LaunchDescription([
            image_theora_left_node,
            image_theora_right_node
        ])
