import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/logic'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    _ros_sub_input = '/head_unit' + '/logic'

    _transport_type = 'raw'  # raw, compressed, theora

    image_compressed_left_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_compressed_left_node',
        arguments=['compressed'],
        remappings=[
            ('in', _ros_namespace + _ros_sub_input + '/image/raw/left'),
            ('out', _ros_namespace + _ros_sub_namespace + '/image/compressed/left')
        ],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    image_compressed_right_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_compressed_right_node',
        arguments=['compressed'],
        remappings=[
            ('in', _ros_namespace + _ros_sub_input + '/image/raw/right'),
            ('out', _ros_namespace + _ros_sub_namespace + '/image/compressed/right')
        ],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    image_theora_left_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='theora_image_transport',
        executable='ogg_saver',
        output=_output_type,
        name='image_theora_left_node',
        arguments=['data/streaming_00.ogv'],
        remappings=[
            ('image', _ros_namespace + _ros_sub_namespace + '/image/raw/left')
        ],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    image_theora_right_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='theora_image_transport',
        executable='ogg_saver',
        output=_output_type,
        name='image_theora_right_node',
        arguments=['data/streaming_00.ogv'],
        remappings=[
            ('image', _ros_namespace + _ros_sub_namespace + '/image/raw/right')
        ],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    image_transport_left_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_transport_left_node',
        arguments=['raw'],
        remappings=[
            ('in', _ros_namespace + _ros_sub_input + '/image/raw/left'),
            ('out', _ros_namespace + _ros_sub_namespace + '/image/transport/left')
        ],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    image_transport_right_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='image_transport',
        executable='republish',
        output=_output_type,
        name='image_transport_right_node',
        arguments=['raw'],
        remappings=[
            ('in', _ros_namespace + _ros_sub_input + '/image/raw/right'),
            ('out', _ros_namespace + _ros_sub_namespace + '/image/transport/right')
        ],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    if 'compressed' == _transport_type:
        return LaunchDescription([
            image_compressed_left_node,
            image_compressed_right_node
        ])
    elif 'theora' == _transport_type:
        return LaunchDescription([
            image_theora_left_node,
            image_theora_right_node
        ])
    else:
        return LaunchDescription([
            image_transport_left_node,
            image_transport_right_node
        ])
