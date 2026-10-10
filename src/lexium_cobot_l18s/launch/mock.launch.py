from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    resources_package = 'lexium_cobot_l18s'
    pkg_share = Path(FindPackageShare(package=resources_package).find(resources_package))
    default_model_path = pkg_share / 'urdf/Lexium_Cobot_L18S002_wrapper.urdf.xacro'
    default_controllers_file = pkg_share / 'config/Lexium_Cobot_L18S002_controllers.yaml'
    default_rviz_config = pkg_share / 'rviz/robot_description.rviz'

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz',
    )

    controllers_file = LaunchConfiguration('controllers_file')
    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value=str(default_controllers_file),
        description='Path to ros2_control controllers yaml',
    )

    # Robot Description
    robot_description_content = Command([
        'xacro ',
        str(default_model_path),
        ' use_ros2_control:=true',
        ' use_mock_hardware:=true',
        ' use_sim:=false',
        ' controllers_file:=',
        controllers_file,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': False},
        ],
        output='screen',
    )

    # ros2_control node with mock hardware
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_file,
            robot_description,
            {'use_sim_time': False},
        ],
        output='screen',
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # Spawner for arm_controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # Delay spawners to let controller_manager initialize
    delayed_joint_state_broadcaster = TimerAction(
        period=1.5,
        actions=[joint_state_broadcaster_spawner],
    )
    delayed_arm_controller = TimerAction(
        period=2.5,
        actions=[arm_controller_spawner],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(default_rviz_config)],
        condition=IfCondition(use_rviz),
        parameters=[{
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        use_rviz_arg,
        controllers_file_arg,
        robot_state_publisher_node,
        ros2_control_node,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        rviz_node,
    ])
