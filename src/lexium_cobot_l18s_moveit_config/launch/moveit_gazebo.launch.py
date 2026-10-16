import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    tool_type = LaunchConfiguration("tool_type").perform(context).lower()
    rviz_flag_str = LaunchConfiguration("rviz").perform(context).lower()
    use_rviz = rviz_flag_str in ["true", "1", "yes"]

    moveit_config = (
        MoveItConfigsBuilder(
            "Lexium_Cobot_L18S002", package_name="lexium_cobot_l18s_moveit_config"
        )
        .robot_description(
            mappings={
                "use_ros2_control": "true",
                "use_mock_hardware": "false",
                "use_sim": "true",
                "tool_type": tool_type,
            }
        )
        .robot_description_semantic(
            file_path="config/Lexium_Cobot_L18S002.srdf.xacro",
            mappings={"tool_type": tool_type},
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("lexium_cobot_l18s_moveit_config"),
        "rviz",
        "moveit.rviz",
    )

    # Gazebo Simulation launch (disable raw URDF display in gazebo.launch.py)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("lexium_cobot_l18s"),
                "launch",
                "gazebo.launch.py",
            )
        ]),
        launch_arguments={
            "use_rviz": "false",
            "tool_type": tool_type,
        }.items(),
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
        period=3.0,
        actions=[move_group_node],
    )

    # RViz2 Node with MoveIt MotionPlanning interface
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(str(use_rviz).lower()),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return [
        gazebo_launch,
        delayed_move_group,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "tool_type",
            default_value="gripper",
            description="End-effector tool attachment: 'none', 'gripper', or 'vacuum'",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Whether to launch MoveIt RViz2 interface",
        ),
        OpaqueFunction(function=launch_setup),
    ])
