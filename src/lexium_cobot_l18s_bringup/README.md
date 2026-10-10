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
| `use_servo` | `false` | `true`, `false` | Launches the MoveIt 2 Servo real-time streaming node. |
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

# Real-Time MoveIt Servo Streaming
ros2 launch lexium_cobot_l18s_bringup servo.launch.py mode:=mock
```

### 2. Interactive Keyboard Teleoperation

```bash
# In Terminal 1: Launch simulation + MoveIt + Servo
ros2 launch lexium_cobot_l18s_bringup servo.launch.py mode:=mock

# In Terminal 2: Run interactive keyboard teleoperation
ros2 run lexium_cobot_l18s_bringup teleop_keyboard.py
```

Keyboard teleop controls:
- Press `1` for **Joint Jog Mode**: `q/a`, `w/s`, `e/d`, `r/f`, `t/g`, `y/h` jog joints 1 through 6.
- Press `2` for **Cartesian Mode**: `Arrow Keys` (X/Y translation), `Shift+Up/Down` or `w/s` (Z translation).
- Press `0` to **Move to Zero Pose** (`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`).
- Press `9` to **Move to Ready Pose**.
- `+` / `-`: Increase / decrease velocity.
- `Space`: Instant halt.

### 3. Cartesian Joystick GUI Teleoperation

For graphical mouse/touch teleoperation with spring-loaded analog joysticks:

```bash
# In Terminal 1: Launch robot bringup + Servo
ros2 launch lexium_cobot_l18s_bringup servo.launch.py mode:=mock

# In Terminal 2: Run Cartesian Joystick GUI
ros2 run lexium_cobot_l18s_bringup teleop_gui.py
# or
ros2 launch lexium_cobot_l18s_bringup teleop_gui.launch.py
```

GUI Features:
- **Planar XY Joystick**: Spring-loaded 2D joystick controlling $+X/-X$ (Forward/Back) and $+Y/-Y$ (Left/Right) translation.
- **Vertical Z Slider**: Spring-loaded 1D joystick controlling $+Z/-Z$ (Up/Down) translation.
- **Orientation Joystick**: Roll ($\omega_x$), Pitch ($\omega_y$), and Yaw ($\omega_z$) rotational control.
- **Velocity Scaling**: Real-time linear ($0.01-0.50 \text{ m/s}$) and angular ($0.05-1.50 \text{ rad/s}$) sliders.
- **Frame Selection**: Instant toggle between Base frame (`l_Base_Link002`) and Tool frame (`l_Link_017`).
- **Pose Presets**: Quick action buttons for **Move to Zero (0.0)** and **Move to Ready**.
- **Safety Deadman & E-Stop**: Auto-return to zero velocity on pointer release and immediate red Emergency Stop button.

---

## Verified Topics & Interfaces

- `/joint_states` (`sensor_msgs/msg/JointState`): Published at 50 Hz by `joint_state_broadcaster`.
- `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`): Commanded by MoveIt 2 and Servo.
- `/arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`): Execution action server.
- `/move_action` (`moveit_msgs/action/MoveGroup`): MoveIt 2 programmatic motion planning action server.
- `/servo_node/delta_joint_cmds` (`control_msgs/msg/JointJog`): Real-time joint velocity streaming.
- `/servo_node/delta_twist_cmds` (`geometry_msgs/msg/TwistStamped`): Real-time 3D Cartesian velocity streaming.
- `/servo_node/switch_command_type` (`moveit_msgs/srv/ServoCommandType`): Mode switcher (`0=JOINT_JOG`, `1=TWIST`).
