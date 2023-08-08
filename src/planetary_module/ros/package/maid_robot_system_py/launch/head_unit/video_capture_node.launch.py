import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
import maid_robot_system_interfaces.srv as MrsSrv


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = "{}{}".format(os.environ.get('MRS_ROS_NAMESPACE'), '/head_unit')
    _mrs_v4l_left_by_path = str(os.environ.get('MRS_V4L_LEFT_BY_PATH'))
    _mrs_v4l_left_id = int("{}".format(os.environ.get('MRS_V4L_LEFT_ID')))
    _mrs_v4l_right_by_path = str(os.environ.get('MRS_V4L_RIGHT_BY_PATH'))
    _mrs_v4l_right_id = int("{}".format(os.environ.get('MRS_V4L_RIGHT_ID')))

    left_video_capture_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='video_capture_node',
        name='left_video_capture_node',
        output=_output_type,
        remappings=[
            ('info', _ros_namespace + '/video/info/left'),
            ('out_srv', _ros_namespace + '/video/image/left'),
            ('out_topic', _ros_namespace + '/image/raw/left')
        ],
        parameters=[{
            "device/TYPE": "v4l",
            "device/BY_PATH": _mrs_v4l_left_by_path,
            "device/ID": _mrs_v4l_left_id,
            "device/settings/FORMAT": "MJPG",
            "device/settings/WIDTH": 1280,
            "device/settings/HEIGHT": 1024,
            "device/settings/ANGLE": 140,
            "device/settings/FPS": 30.0,
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
        respawn=True,
        respawn_delay=2.0
    )

    right_video_capture_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='video_capture_node',
        name='right_video_capture_node',
        output=_output_type,
        remappings=[
            ('info', _ros_namespace + '/video/info/right'),
            ('out_srv', _ros_namespace + '/video/image/right'),
            ('out_topic', _ros_namespace + '/image/raw/right')
        ],
        parameters=[{
            "device/TYPE": "v4l",
            "device/BY_PATH": _mrs_v4l_right_by_path,
            "device/ID": _mrs_v4l_right_id,
            "device/settings/FORMAT": "MJPG",
            "device/settings/WIDTH": 1280,
            "device/settings/HEIGHT": 1024,
            "device/settings/ANGLE": 140,
            "device/settings/FPS": 30.0,
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
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_video_capture_node,
        right_video_capture_node
    ])
