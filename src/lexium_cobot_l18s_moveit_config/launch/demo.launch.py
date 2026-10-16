import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    tool_type = LaunchConfiguration("tool_type").perform(context).lower()
    rviz_flag_str = LaunchConfiguration("use_rviz").perform(context).lower()
    use_rviz = rviz_flag_str in ["true", "1", "yes"]

    lexium_pkg = get_package_share_directory("lexium_cobot_l18s")
    controllers_file = os.path.join(
        lexium_pkg, "config", "Lexium_Cobot_L18S002_controllers.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "Lexium_Cobot_L18S002", package_name="lexium_cobot_l18s_moveit_config"
        )
        .robot_description(
            mappings={
                "use_ros2_control": "true",
                "use_mock_hardware": "true",
                "use_sim": "false",
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

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # ros2_control node with mock hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_file,
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    delayed_jsb = TimerAction(
        period=1.5,
        actions=[joint_state_broadcaster_spawner],
    )
    delayed_ac = TimerAction(
        period=2.5,
        actions=[arm_controller_spawner],
    )

    nodes = [
        robot_state_publisher_node,
        ros2_control_node,
        delayed_jsb,
        delayed_ac,
    ]

    # Gripper Controller Spawner (only if tool_type is gripper)
    if tool_type == "gripper":
        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        )
        delayed_gc = TimerAction(
            period=3.5,
            actions=[gripper_controller_spawner],
        )
        nodes.append(delayed_gc)

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )
    nodes.append(move_group_node)

    # RViz2 Node
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
            {"use_sim_time": False},
        ],
    )
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "tool_type",
            default_value="gripper",
            description="End-effector tool attachment: 'none', 'gripper', or 'vacuum'",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2",
        ),
        OpaqueFunction(function=launch_setup),
    ])
