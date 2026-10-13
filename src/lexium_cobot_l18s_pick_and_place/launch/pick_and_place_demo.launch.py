import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_pnp = get_package_share_directory("lexium_cobot_l18s_pick_and_place")

    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")
    continuous = LaunchConfiguration("continuous")

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
            "continuous",
            default_value="false",
            description="Whether to run pick & place continuously in an automated loop.",
        ),
    ]

    cell_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pnp, "launch", "pick_and_place_cell.launch.py")
        ),
        launch_arguments={
            "mode": mode,
            "use_rviz": use_rviz,
        }.items(),
    )

    # Launch pick and place orchestrator node once MoveGroup and controllers are ready
    pnp_node = Node(
        package="lexium_cobot_l18s_pick_and_place",
        executable="pick_and_place_node.py",
        name="pick_and_place_node",
        output="screen",
        arguments=["--continuous"] if continuous == "true" else [],
    )

    # Delay for controller manager and MoveGroup initialization
    delayed_pnp_node = TimerAction(period=8.0, actions=[pnp_node])

    return LaunchDescription(declared_arguments + [cell_launch, delayed_pnp_node])
