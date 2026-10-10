import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    teleop_gui_node = Node(
        package="lexium_cobot_l18s_bringup",
        executable="teleop_gui.py",
        name="lexium_cartesian_gui_teleop",
        output="screen",
    )

    return LaunchDescription([teleop_gui_node])
