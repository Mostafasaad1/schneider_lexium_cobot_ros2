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
    controllers_file = LaunchConfiguration("controllers_file")
    tool_type = LaunchConfiguration("tool_type")

    declared_arguments = [
        DeclareLaunchArgument(
            "tool_type",
            default_value="none",
            description="Tool type: 'none', 'gripper', or 'vacuum'",
        ),
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
            "controllers_file",
            default_value="",
            description="Custom ros2_control hardware controllers YAML file",
        ),
    ]

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "mode": "real",
            "tool_type": tool_type,
            "use_moveit": use_moveit,
            "use_rviz": use_rviz,
            "controllers_file": controllers_file,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [included_launch])
