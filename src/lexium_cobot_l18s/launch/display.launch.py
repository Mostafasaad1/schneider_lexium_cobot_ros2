from pathlib import Path

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = Path(FindPackageShare(package='lexium_cobot_l18s').find('lexium_cobot_l18s'))
    default_model_path = pkg_share / 'urdf/Lexium_Cobot_L18S002_wrapper.urdf.xacro'
    default_rviz_config_path = pkg_share / 'rviz/robot_description.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    tool_type = LaunchConfiguration('tool_type')
    tool_type_arg = DeclareLaunchArgument(
        'tool_type',
        default_value='gripper',
        description="End-effector tool attachment: 'none', 'gripper', or 'vacuum'"
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lexium_cobot_l18s'),
                'launch',
                'description.launch.py',
            ]),
        ]),
        launch_arguments=dict(
            use_sim_time=use_sim_time,
            use_ros2_control=use_ros2_control,
            tool_type=tool_type,
        ).items(),
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # Only for pure visualization: no GUI and no ros2_control (sim provides /joint_states via broadcaster)
        condition=UnlessCondition(PythonExpression(['"', LaunchConfiguration('gui'), '" == "True" or "', use_ros2_control, '" == "true"'])),
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('gui'), '" == "True" and "', use_ros2_control, '" != "true"'])),
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Flag to enable usage of simulation time',
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui',
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=str(default_model_path),
            description='Absolute path to robot urdf file',
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=str(default_rviz_config_path),
            description='Absolute path to rviz config file',
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Include ros2_control configuration in lexium_cobot_l18s'
        ),
        tool_type_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
