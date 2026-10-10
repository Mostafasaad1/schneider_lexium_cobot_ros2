import os
import subprocess
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Retrieve launch configurations
    mode = LaunchConfiguration("mode").perform(context).lower()
    use_moveit_str = LaunchConfiguration("use_moveit").perform(context).lower()
    use_rviz_str = LaunchConfiguration("use_rviz").perform(context).lower()
    rviz_config_custom = LaunchConfiguration("rviz_config").perform(context)
    controllers_file_custom = LaunchConfiguration("controllers_file").perform(context)
    gazebo_world_custom = LaunchConfiguration("gazebo_world").perform(context)
    gui_config_custom = LaunchConfiguration("gui_config").perform(context)

    use_moveit = use_moveit_str in ["true", "1", "yes"]
    use_rviz = use_rviz_str in ["true", "1", "yes"]
    is_sim = mode in ["sim", "gazebo", "simulation"]
    is_mock = mode in ["mock", "fake"]
    is_real = mode in ["real", "hardware"]

    use_sim_time = is_sim

    # Package directories
    lexium_pkg = get_package_share_directory("lexium_cobot_l18s")
    moveit_pkg = get_package_share_directory("lexium_cobot_l18s_moveit_config")

    # Controller configuration file
    controllers_file = (
        controllers_file_custom
        if controllers_file_custom
        else os.path.join(lexium_pkg, "config", "Lexium_Cobot_L18S002_controllers.yaml")
    )

    # RViz configuration file
    if rviz_config_custom:
        rviz_config_file = rviz_config_custom
    elif use_moveit:
        rviz_config_file = os.path.join(moveit_pkg, "rviz", "moveit.rviz")
    else:
        rviz_config_file = os.path.join(lexium_pkg, "rviz", "robot_description.rviz")

    # Build MoveIt Config / Robot Description
    moveit_config = (
        MoveItConfigsBuilder(
            "Lexium_Cobot_L18S002", package_name="lexium_cobot_l18s_moveit_config"
        )
        .robot_description(
            mappings={
                "use_ros2_control": "true",
                "use_mock_hardware": "true" if is_mock else "false",
                "use_sim": "true" if is_sim else "false",
                "controllers_file": controllers_file,
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    actions = []

    # 1. Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )
    actions.append(rsp_node)

    # 2. Hardware / Simulation Backend
    if is_sim:
        # Configure Gazebo environment paths
        resources_package = "lexium_cobot_l18s"
        path_to_share_clipped = "".join(
            get_package_share_directory(resources_package).rsplit("/" + resources_package, 1)
        )
        os.environ["GZ_SIM_RESOURCE_PATH"] = path_to_share_clipped
        if "SDF_PATH" in os.environ:
            os.environ["SDF_PATH"] = os.environ["SDF_PATH"] + ":" + path_to_share_clipped
        else:
            os.environ["SDF_PATH"] = path_to_share_clipped

        # World and GUI config
        gazebo_world = (
            gazebo_world_custom
            if gazebo_world_custom
            else os.path.join(lexium_pkg, "worlds", "empty.sdf")
        )
        gui_config = (
            gui_config_custom
            if gui_config_custom
            else os.path.join(lexium_pkg, "config", "gui.config")
        )

        pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
        gz_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r {gazebo_world} --gui-config {gui_config} --verbose"
            }.items(),
        )
        actions.append(gz_sim_launch)

        # Spawn entity in Gazebo
        spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "Lexium_Cobot_L18S002",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.05",
                "-Y", "0.0",
                "-topic", "/robot_description",
            ],
            output="screen",
        )
        actions.append(spawn_node)

        # Clock Bridge
        clock_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
        actions.append(clock_bridge)

    elif is_mock or is_real:
        # ros2_control_node (Controller Manager) for Mock or Real hardware
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
        actions.append(ros2_control_node)

    # 3. Controller Spawners
    spawner_delay_jsb = 5.0 if is_sim else 1.5
    spawner_delay_ac = 7.0 if is_sim else 2.5

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    ac_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    actions.append(TimerAction(period=spawner_delay_jsb, actions=[jsb_spawner]))
    actions.append(TimerAction(period=spawner_delay_ac, actions=[ac_spawner]))

    # 4. MoveGroup (MoveIt 2)
    if use_moveit:
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
            ],
        )
        move_group_delay = 8.0 if is_sim else 3.0
        actions.append(TimerAction(period=move_group_delay, actions=[move_group_node]))

    # 5. RViz2
    if use_rviz:
        rviz_parameters = [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ] if use_moveit else [
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ]

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=rviz_parameters,
        )
        actions.append(rviz_node)

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            description="Execution backend mode: 'mock' (offline/fake hardware), 'sim' (Gazebo Harmonic), or 'real' (physical hardware).",
        ),
        DeclareLaunchArgument(
            "use_moveit",
            default_value="true",
            description="Whether to launch MoveIt 2 MoveGroup planning node.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2 interface.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Custom RViz configuration file path. Defaults to MoveIt RViz if use_moveit=true, otherwise description RViz.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="Custom ros2_control controllers YAML configuration path.",
        ),
        DeclareLaunchArgument(
            "gazebo_world",
            default_value="",
            description="Gazebo SDF world file path (when mode:=sim).",
        ),
        DeclareLaunchArgument(
            "gui_config",
            default_value="",
            description="Gazebo GUI configuration file path (when mode:=sim).",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
