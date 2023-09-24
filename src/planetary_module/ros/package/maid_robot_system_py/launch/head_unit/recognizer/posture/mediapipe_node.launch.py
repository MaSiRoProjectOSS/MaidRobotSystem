import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
import maid_robot_system_interfaces.srv as MrsSrv


def generate_launch_description():
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE', '/maid_robot_system')
    _ros_sub_namespace = '/head_unit' + '/logic'
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE', 'log')
    _log_level = os.environ.get('MRS_ROS_LOG_LEVEL', 'INFO')
    _res_pawn = {'true': True, 'false': False}[os.getenv('MRS_ROS_SPAWN', 'false')]

    _ros_sub_input = '/head_unit' + '/controller' + os.environ.get('MRS_TYPE_INPUT_VIDEO_DEVICE', '/camera')

    left_mediapipe_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_node',
        name='left_mediapipe_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + _ros_sub_input + '/video/left/image'),
            ('in_srv_video', _ros_namespace + _ros_sub_input + '/video/left/info'),
            ('out_srv', _ros_namespace + _ros_sub_namespace + '/mediapipe/left/pose'),
            ('out', _ros_namespace + _ros_sub_namespace + '/data/left/pose')
        ],
        parameters=[{
            "INTERVAL_MS": 100,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/visibility_th": 0.5,
            "area/center_x": 0,
            "area/center_y": 0,
            "area/width": 0,
            "area/height": 0,
            "clockwise": int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK),
            "mirror": False,
            "upside_down": False,
            "notify/message/verbose": True
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )
    right_mediapipe_node = Node(
        namespace=_ros_namespace + _ros_sub_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_node',
        name='right_mediapipe_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + _ros_sub_input + '/video/right/image'),
            ('in_srv_video', _ros_namespace + _ros_sub_input + '/video/right/info'),
            ('out_srv', _ros_namespace + _ros_sub_namespace + '/mediapipe/right/pose'),
            ('out', _ros_namespace + _ros_sub_namespace + '/data/right/pose')
        ],
        parameters=[{
            "INTERVAL_MS": 100,
            "confidence/min_detection": 0.5,
            "confidence/min_tracking": 0.5,
            "confidence/visibility_th": 0.5,
            "area/center_x": 0,
            "area/center_y": 0,
            "area/width": 0,
            "area/height": 0,
            "clockwise": int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK),
            "mirror": False,
            "upside_down": False,
            "notify/message/verbose": True
        }],
        ros_arguments=['--log-level', _log_level],
        respawn=_res_pawn,
        respawn_delay=2.0
    )
    return LaunchDescription([
        left_mediapipe_node,
        right_mediapipe_node
    ])
