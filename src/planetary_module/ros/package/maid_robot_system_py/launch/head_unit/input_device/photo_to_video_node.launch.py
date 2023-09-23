import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
import maid_robot_system_interfaces.srv as MrsSrv


def generate_launch_description():
    _ros_namespace = "{}{}".format(
        os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system'), '/head_unit')
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[
        os.getenv('MRS_ROS_SPAWN', 'false')]

    left_photo_to_video_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='photo_to_video_node',
        name='left_photo_to_video_node',
        output=_output_type,
        remappings=[
            ('info', _ros_namespace + '/video/left/info'),
            ('out_srv', _ros_namespace + '/video/left/image'),
            ('out_topic', _ros_namespace + '/image/left/raw')
        ],
        parameters=[{
            "photo": "",
            "notify/message/verbose": False,
            "settings/area/mirror": False,
            "settings/area/upside_down": False,
            "settings/area/clockwise": int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK),
            "settings/area/center_x": 0,
            "settings/area/center_y": 0,
            "settings/area/width": 0,
            "settings/area/height": 0,
            "publisher/INTERVAL_FPS": 10.0,
            "publisher/resize/width": 640,
            "publisher/resize/height": 512,
            "publisher/enable": False
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    right_photo_to_video_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='photo_to_video_node',
        name='right_photo_to_video_node',
        output=_output_type,
        remappings=[
            ('info', _ros_namespace + '/video/right/info'),
            ('out_srv', _ros_namespace + '/video/right/image'),
            ('out_topic', _ros_namespace + '/image/right/raw')
        ],
        parameters=[{
            "photo": "",
            "notify/message/verbose": False,
            "settings/area/mirror": False,
            "settings/area/upside_down": False,
            "settings/area/clockwise": int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK),
            "settings/area/center_x": 0,
            "settings/area/center_y": 0,
            "settings/area/width": 0,
            "settings/area/height": 0,
            "publisher/INTERVAL_FPS": 10.0,
            "publisher/resize/width": 640,
            "publisher/resize/height": 512,
            "publisher/enable": False
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_photo_to_video_node,
        right_photo_to_video_node
    ])
