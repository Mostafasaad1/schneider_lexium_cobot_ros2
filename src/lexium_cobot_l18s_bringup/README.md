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
├── scripts/
│   ├── teleop_keyboard.py     # Interactive terminal teleoperation
│   ├── teleop_gui.py          # Modern Cartesian Joystick GUI
│   └── pilz_motion_demo.py     # Pilz industrial motion demo (PTP, LIN, CIRC, Blending)
└── launch/
    ├── bringup.launch.py       # Master unified launch file
    ├── mock.launch.py          # Convenience shortcut for mock hardware
    ├── sim.launch.py           # Convenience shortcut for Gazebo simulation
    ├── real.launch.py          # Convenience shortcut for real hardware
    ├── servo.launch.py         # Convenience shortcut for real-time MoveIt Servo
    └── teleop_gui.launch.py    # Convenience shortcut for Bringup + Joystick GUI
```

---

## Launch Arguments

| Argument | Default | Choices / Format | Description |
| :--- | :--- | :--- | :--- |
| `mode` | `mock` | `mock`, `sim`, `real` | Selects the hardware execution backend. |
| `tool_type` | `none` | `none`, `gripper`, `vacuum` | Attaches end-effector tool (`gripper` for parallel jaw, `vacuum` for suction cup, `none` for bare flange). |
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

# Launch with Generic Parallel Jaw Gripper attached
ros2 launch lexium_cobot_l18s_bringup mock.launch.py tool_type:=gripper
```

### 2. End-Effector & Tool Framework (Parallel Gripper & Vacuum)

The Lexium Cobot L18S includes a modular end-effector tool architecture:
- **Standardized `tool0` Flange**: Standardized mechanical and kinematic interface on the wrist flange (`l_Link_017` -> `tool0` -> `tcp_link`).
- **Parallel Jaw Gripper (`tool_type:=gripper`)**: Symmetric $40\text{ mm}$ stroke prismatic gripper with polyurethane high-grip contact pads, physics friction properties, and dynamic mimic joint transmission.
- **Vacuum Suction Cup (`tool_type:=vacuum`)**: Industrial pneumatic suction tool with mounting manifold and elastomeric suction cup.
- **`ros2_control` Integration**: Controlled via `parallel_gripper_action_controller/GripperActionController` providing the standard `/gripper_controller/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`) interface.
- **MoveIt 2 Integration**: `gripper` planning group with `open` ($40\text{ mm}$) and `close` ($0\text{ mm}$) named poses, end-effector binding to `lexium_arm`, and optimized collision avoidance matrix.

Run the automated pick-and-place demonstration:
```bash
# In Terminal 1: Launch bringup stack with gripper attached
ros2 launch lexium_cobot_l18s_bringup mock.launch.py tool_type:=gripper

# In Terminal 2: Run coordinated pick-and-place sequence
ros2 run lexium_cobot_l18s_bringup gripper_demo.py
```

### 3. Pilz Industrial Motion Planner (PTP, LIN, CIRC & Blending)

The Lexium Cobot L18S is fully configured with the **Pilz Industrial Motion Planner** (`pilz_industrial_motion_planner`) pipeline alongside OMPL, loaded by default in MoveGroup.

Features supported:
- **PTP (Point-to-Point)**: Time-optimal joint-space and Cartesian-space trajectories adhering to velocity & acceleration limits.
- **LIN (Linear Motion)**: Deterministic Cartesian straight-line trajectories with synchronized TCP translation and rotation.
- **CIRC (Circular Motion)**: Deterministic circular arcs defined by center or interim points.
- **MoveGroupSequenceAction & Blending**: Continuous waypoint execution with configurable blend radii (`blend_radius`, e.g., $30-50\text{ mm}$) to eliminate stopping at intermediate via-points.

Run the automated validation & demo suite:
```bash
# In Terminal 1: Launch bringup stack
ros2 launch lexium_cobot_l18s_bringup mock.launch.py

# In Terminal 2: Run Pilz industrial motion demonstration
ros2 run lexium_cobot_l18s_bringup pilz_motion_demo.py
```

### 4. Interactive Keyboard Teleoperation

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

### 5. Cartesian Joystick GUI Teleoperation

For graphical mouse/touch teleoperation with spring-loaded analog joysticks:

```bash
# Launch robot bringup + Servo + Cartesian Joystick GUI in one command:
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
- `/gripper_controller/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`): Gripper action server for parallel jaw actuation.
- `/move_action` (`moveit_msgs/action/MoveGroup`): MoveIt 2 programmatic motion planning action server (supporting `ompl` and `pilz_industrial_motion_planner`).
- `/sequence_move_group` (`moveit_msgs/action/MoveGroupSequence`): Pilz industrial trajectory blending sequence action server.
- `/servo_node/delta_joint_cmds` (`control_msgs/msg/JointJog`): Real-time joint velocity streaming.
- `/servo_node/delta_twist_cmds` (`geometry_msgs/msg/TwistStamped`): Real-time 3D Cartesian velocity streaming.
- `/servo_node/switch_command_type` (`moveit_msgs/srv/ServoCommandType`): Mode switcher (`0=JOINT_JOG`, `1=TWIST`).
