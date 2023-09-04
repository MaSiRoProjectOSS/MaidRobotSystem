import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE')
    transport_type = 'compressed'  # 'raw'/'compressed'/'theora'

    image_compressed_left_node = Node(
        namespace=_ros_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_compressed_left_node',
        arguments=['compressed'],
        remappings=[
            ('in',  _ros_namespace + '/image/left/raw'),
            ('out', _ros_namespace + '/image/left/compressed')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_compressed_right_node = Node(
        namespace=_ros_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_compressed_right_node',
        arguments=['compressed'],
        remappings=[
            ('in',  _ros_namespace + '/image/right/raw'),
            ('out', _ros_namespace + '/image/right/compressed')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_theora_left_node = Node(
        namespace=_ros_namespace,
        package='theora_image_transport',
        executable='ogg_saver',
        output=_output_type,
        name='image_theora_left_node',
        arguments=['data/streaming_00.ogv'],
        remappings=[
            ('image', _ros_namespace + '/image/left/raw')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_theora_right_node = Node(
        namespace=_ros_namespace,
        package='theora_image_transport',
        executable='ogg_saver',
        output=_output_type,
        name='image_theora_right_node',
        arguments=['data/streaming_00.ogv'],
        remappings=[
            ('image', _ros_namespace + '/image/right/raw')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_transport_left_node = Node(
        namespace=_ros_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_transport_left_node',
        arguments=['raw'],
        remappings=[
            ('in',  _ros_namespace + '/image/left/raw'),
            ('out', _ros_namespace + '/image/left/transport')
        ],
        respawn=True,
        respawn_delay=2.0
    )

    image_transport_right_node = Node(
        namespace=_ros_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_transport_right_node',
        arguments=['raw'],
        remappings=[
            ('in',  _ros_namespace + '/image/right/raw'),
            ('out', _ros_namespace + '/image/right/transport')
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
