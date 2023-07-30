import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    _output_type = os.environ.get('MRS_ROS_OUTPUT_TYPE')
    _ros_namespace = os.environ.get('MRS_ROS_NAMESPACE') + '/head_unit'
    _mrs_v4l_left_by_path = str(os.environ.get('MRS_V4L_LEFT_BY_PATH'))
    _mrs_v4l_left_id = int(os.environ.get('MRS_V4L_LEFT_ID'))
    _mrs_v4l_right_by_path = str(os.environ.get('MRS_V4L_RIGHT_BY_PATH'))
    _mrs_v4l_right_id = int(os.environ.get('MRS_V4L_RIGHT_ID'))

    left_video_capture_node = Node(
        namespace=_ros_namespace,
        package='maid_robot_system_py',
        executable='video_capture_node',
        name='left_video_capture_node',
        output=_output_type,
        remappings=[
            ('video_device_info', _ros_namespace + '/video_device_info/left'),
            ('image', _ros_namespace + '/image/raw/left')
        ],
        parameters=[{
            "configuration/device/type": "v4l",
            "configuration/device/by_path": _mrs_v4l_left_by_path,
            "configuration/device/id": _mrs_v4l_left_id,
            "configuration/video/settings/format": "MJPG",
            "configuration/video/settings/width": 1280,
            "configuration/video/settings/height": 1024,
            "configuration/video/settings/angle": 140,
            "configuration/video/settings/fps": 30.0,
            "configuration/publisher/interval_fps": 10.0,
            "preference/info/verbose": False,
            "preference/video/area/mirror": False,
            "preference/video/area/center_x": 0,
            "preference/video/area/center_y": 0,
            "preference/video/area/width": 0,
            "preference/video/area/height": 0,
            "preference/publisher/resize/width": 640,
            "preference/publisher/resize/height": 512
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
            ('video_device_info', _ros_namespace + '/video_device_info/right'),
            ('image', _ros_namespace + '/image/raw/right')
        ],
        parameters=[{
            "configuration/device/type": "v4l",
            "configuration/device/by_path": _mrs_v4l_right_by_path,
            "configuration/device/id": _mrs_v4l_right_id,
            "configuration/video/settings/format": "MJPG",
            "configuration/video/settings/width": 1280,
            "configuration/video/settings/height": 1024,
            "configuration/video/settings/angle": 140,
            "configuration/video/settings/fps": 30.0,
            "configuration/publisher/interval_fps": 10.0,
            "preference/info/verbose": False,
            "preference/video/area/mirror": False,
            "preference/video/area/center_x": 0,
            "preference/video/area/center_y": 0,
            "preference/video/area/width": 0,
            "preference/video/area/height": 0,
            "preference/publisher/resize/width": 640,
            "preference/publisher/resize/height": 512
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        left_video_capture_node,
        right_video_capture_node
    ])
