import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "Lexium_Cobot_L18S002", package_name="lexium_cobot_l18s_moveit_config"
        )
        .robot_description(
            mappings={
                "use_ros2_control": "true",
                "use_mock_hardware": "false",
                "use_sim": "true",
            }
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("lexium_cobot_l18s_moveit_config"),
        "rviz",
        "moveit.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz2",
    )

    # Gazebo Simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("lexium_cobot_l18s"),
                "launch",
                "gazebo.launch.py",
            )
        ]),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    # Move Group Node (delayed to allow Gazebo & controllers to boot)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    delayed_move_group = TimerAction(
        period=5.0,
        actions=[move_group_node],
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    delayed_rviz = TimerAction(
        period=6.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        use_rviz_arg,
        gazebo_launch,
        delayed_move_group,
        delayed_rviz,
    ])
