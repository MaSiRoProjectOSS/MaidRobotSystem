import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
import maid_robot_system_interfaces.srv as MrsSrv


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/controller' + '/topic_image'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    _ros_sub_input_left = os.environ.get('MRS_MSG_TOPIC_IMAGE_LEFT',  _ros_namespace + _ros_sub_namespace + '/image/left')
    _ros_sub_input_right = os.environ.get('MRS_MSG_TOPIC_IMAGE_RIGHT', _ros_namespace + _ros_sub_namespace + '/image/right')

    left_video_topic_to_service = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='video_topic_to_service',
        name='left_video_topic_to_service',
        output=_output_type,
        remappings=[
            ('in_topic', _ros_sub_input_left),

            ('out_srv', _ros_namespace + _ros_sub_namespace + '/image/inquiry/left')
        ],
        parameters=[{
            "settings/area/mirror": False,
            "settings/area/upside_down": False,
            "settings/area/clockwise": int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK),
            "settings/area/center_x": 0,
            "settings/area/center_y": 0,
            "settings/area/width": 0,
            "settings/area/height": 0
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    right_video_topic_to_service = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='video_topic_to_service',
        name='right_video_topic_to_service',
        output=_output_type,
        remappings=[
            ('in_topic', _ros_sub_input_right),

            ('out_srv', _ros_namespace + _ros_sub_namespace + '/image/inquiry/right')
        ],
        parameters=[{
            "settings/area/mirror": False,
            "settings/area/upside_down": False,
            "settings/area/clockwise": int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK),
            "settings/area/center_x": 0,
            "settings/area/center_y": 0,
            "settings/area/width": 0,
            "settings/area/height": 0
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_video_topic_to_service,
        right_video_topic_to_service
    ])
