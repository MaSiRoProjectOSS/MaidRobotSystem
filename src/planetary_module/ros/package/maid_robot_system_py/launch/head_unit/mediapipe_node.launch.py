import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
import maid_robot_system_interfaces.srv as MrsSrv


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = "{}{}".format(os.environ.get('MRS_ROS_NAMESPACE'), '/head_unit')

    left_mediapipe_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_node',
        name='left_mediapipe_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + '/video/left/image'),
            ('out_srv', _ros_namespace + '/mediapipe/left/pose'),
            ('out', _ros_namespace + '/data/left/pose')
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
        respawn=True,
        respawn_delay=2.0
    )
    right_mediapipe_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='mediapipe_node',
        name='right_mediapipe_node',
        output=_output_type,
        remappings=[
            ('in_srv', _ros_namespace + '/video/right/image'),
            ('out_srv', _ros_namespace + '/mediapipe/right/pose'),
            ('out', _ros_namespace + '/data/right/pose')
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
        respawn=True,
        respawn_delay=2.0
    )
    return LaunchDescription([
        left_mediapipe_node,
        right_mediapipe_node
    ])
