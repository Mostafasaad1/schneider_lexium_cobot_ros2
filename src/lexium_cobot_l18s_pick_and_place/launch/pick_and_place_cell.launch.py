import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_pnp = get_package_share_directory("lexium_cobot_l18s_pick_and_place")
    pkg_bringup = get_package_share_directory("lexium_cobot_l18s_bringup")

    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_custom = LaunchConfiguration("rviz_config")

    world_path = os.path.join(pkg_pnp, "worlds", "pick_and_place_cell.sdf")
    default_rviz_path = os.path.join(pkg_pnp, "rviz", "pick_and_place.rviz")

    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            description="Execution backend mode: 'mock' (offline/fast) or 'sim' (Gazebo Harmonic).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Custom RViz configuration file path.",
        ),
    ]

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "mode": mode,
            "tool_type": "gripper",
            "use_moveit": "true",
            "use_rviz": use_rviz,
            "gazebo_world": world_path,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [bringup_launch])
