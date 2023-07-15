import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    my_system_name = '/maid_robot_system/'

    hitomi_node = Node(
        namespace=my_system_name,
        package='maid_robot_system',
        executable='hitomi_node',
        parameters=[{
            "skin_name"  :"ciro",
            "l_x" :120.0 ,
            "l_y" :305.0,
            "r_x" :-95.0 ,
            "r_y" :517.0 ,
            "eyeball_left_x"  :-45.0,
            "eyeball_left_y"  :95.0 ,
            "eyeball_right_x" :45.0 ,
            "eyeball_right_y" :95.0 ,
            "eyeball_angle"   :0.0,
            "eyelid_size_width"  :1480 ,
            "eyelid_size_height" :1350 ,
            "blink_time_ms_quickly" :50 ,
            "blink_time_ms_min"     :500 ,
            "blink_time_ms_max"     :700 ,
            "blink_time_ms_limit"   :15000 ,
            "blink_time_ms_offset"  :0 ,
            "tiredness"  :25 ,
            "l_angle" :0 ,
            "r_angle" :0
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        hitomi_node
    ])
