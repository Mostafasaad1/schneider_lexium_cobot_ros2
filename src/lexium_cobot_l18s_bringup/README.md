# `lexium_cobot_l18s_bringup`

Unified bringup and top-level launch orchestration for the **Schneider Electric Lexium Cobot L18S** robot arm in ROS 2 Jazzy.

---

## Overview

This package provides a clean, single-entry-point launch architecture to operate the Lexium Cobot L18S across all supported execution backends:

1. **Mock Hardware (`mode:=mock`)**: Fast offline execution with simulated kinematics and zero dependencies on physics engines. Ideal for CI/CD, motion planning testing, and application logic development.
2. **Gazebo Harmonic (`mode:=sim`)**: Full physics simulation with gravity, inertial dynamics, contact modeling, and `gz_ros2_control` hardware interface.
3. **Real Hardware (`mode:=real`)**: Physical robot communication interface with `ros2_control` hardware driver integration.

---

## Package Architecture

```text
lexium_cobot_l18s_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
└── launch/
    ├── bringup.launch.py   # Master unified launch file
    ├── mock.launch.py      # Convenience shortcut for mock hardware
    ├── sim.launch.py       # Convenience shortcut for Gazebo simulation
    └── real.launch.py      # Convenience shortcut for real hardware
```

---

## Launch Arguments

| Argument | Default | Choices / Format | Description |
| :--- | :--- | :--- | :--- |
| `mode` | `mock` | `mock`, `sim`, `real` | Selects the hardware execution backend. |
| `use_moveit` | `true` | `true`, `false` | Launches the MoveIt 2 `move_group` planning node. |
| `use_rviz` | `true` | `true`, `false` | Launches RViz2 with appropriate displays and configuration. |
| `rviz_config` | `""` (auto) | File path | Custom `.rviz` configuration path (auto-selects MoveIt or URDF view). |
| `controllers_file`| `""` (auto) | File path | Custom `ros2_control` controllers YAML configuration file. |
| `gazebo_world` | `""` (auto) | File path | Custom Gazebo SDF world file (only applicable in `sim` mode). |
| `gui_config` | `""` (auto) | File path | Custom Gazebo GUI configuration file (only applicable in `sim` mode). |

---

## Usage Examples

### 1. Unified Master Launch

```bash
# Offline Mock Hardware with MoveIt 2 and RViz
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=mock

# Gazebo Harmonic Simulation with MoveIt 2 and RViz
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=sim

# Physical Hardware with MoveIt 2 and RViz
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=real

# Headless / Background (without RViz)
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=sim use_rviz:=false
```

### 2. Convenience Launch Shortcuts

```bash
# Quick Mock Launch
ros2 launch lexium_cobot_l18s_bringup mock.launch.py

# Quick Gazebo Simulation Launch
ros2 launch lexium_cobot_l18s_bringup sim.launch.py

# Quick Real Hardware Launch
ros2 launch lexium_cobot_l18s_bringup real.launch.py
```

---

## Verified Topics & Interfaces

- `/joint_states` (`sensor_msgs/msg/JointState`): Published at 50 Hz by `joint_state_broadcaster`.
- `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`): Commanded by MoveIt 2.
- `/arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`): Execution action server.
- `/move_action` (`moveit_msgs/action/MoveGroup`): MoveIt 2 programmatic motion planning action server.
- `/planning_scene` (`moveit_msgs/msg/PlanningScene`): Monitored environment and collision models.
