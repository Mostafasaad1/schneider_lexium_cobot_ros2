import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory("lexium_cobot_l18s_bringup")
    bringup_launch = os.path.join(bringup_pkg, "launch", "bringup.launch.py")

    use_moveit = LaunchConfiguration("use_moveit")
    use_rviz = LaunchConfiguration("use_rviz")
    gazebo_world = LaunchConfiguration("gazebo_world")
    gui_config = LaunchConfiguration("gui_config")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_moveit",
            default_value="true",
            description="Whether to launch MoveIt 2 MoveGroup",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2",
        ),
        DeclareLaunchArgument(
            "gazebo_world",
            default_value="",
            description="Gazebo SDF world file path",
        ),
        DeclareLaunchArgument(
            "gui_config",
            default_value="",
            description="Gazebo GUI configuration file path",
        ),
    ]

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "mode": "sim",
            "use_moveit": use_moveit,
            "use_rviz": use_rviz,
            "gazebo_world": gazebo_world,
            "gui_config": gui_config,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [included_launch])
