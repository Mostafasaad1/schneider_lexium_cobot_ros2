# Schneider Electric Lexium Cobot L18S — ROS 2 Jazzy Industrial Suite

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-brightgreen.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Industrial-blue.svg)](https://moveit.picknik.ai/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

Comprehensive ROS 2 software stack for the **Schneider Electric Lexium Cobot L18S** 6-DOF industrial collaborative robot. Features physics simulation in Gazebo Harmonic, high-performance offline mock execution, advanced MoveIt 2 motion planning (OMPL & Pilz Industrial Motion Planner with PTP, LIN, CIRC, and trajectory blending), real-time MoveIt Servo streaming, interactive teleoperation GUIs, modular end-effector tooling (parallel jaw and vacuum gripper), and a fully autonomous Table-to-Table Industrial Pick-and-Place workcell.

https://github.com/user-attachments/assets/b81ddbc6-6508-448d-9484-2ec695c9036f

> **Autonomous Pick-and-Place Demonstration**: Live Gazebo Harmonic physics simulation featuring dynamic collision object tracking, vertical descent grasp with physical stall detection, table-to-table transit, and placement.

> [!NOTE]
> **Simulation Scope**: This project was developed, tested, and validated entirely in **simulation** (Gazebo Harmonic physics and offline `ros2_control` mock execution). It has **not** been connected to or operated on physical robot hardware.

---

## Table of Contents

- [Overview & Architecture](#overview--architecture)
- [Workspace Directory Structure](#workspace-directory-structure)
- [Prerequisites & Building](#prerequisites--building)
- [Package 1: `lexium_cobot_l18s` (Robot Description & Models)](#package-1-lexium_cobot_l18s-robot-description--models)
  - [Features](#features)
  - [Launch Commands](#launch-commands)
- [Package 2: `lexium_cobot_l18s_moveit_config` (MoveIt 2 Planning Configuration)](#package-2-lexium_cobot_l18s_moveit_config-moveit-2-planning-configuration)
  - [Features](#features-1)
  - [Launch Commands](#launch-commands-1)
- [Package 3: `lexium_cobot_l18s_bringup` (Master Bringup & Teleoperation)](#package-3-lexium_cobot_l18s_bringup-master-bringup--teleoperation)
  - [Features](#features-2)
  - [Launch Commands](#launch-commands-2)
  - [Executable Scripts & Teleoperation](#executable-scripts--teleoperation)
- [Package 4: `lexium_cobot_l18s_pick_and_place` (Industrial Workcell Application)](#package-4-lexium_cobot_l18s_pick_and_place-industrial-workcell-application)
  - [Features](#features-3)
  - [Launch Commands & Node Execution](#launch-commands--node-execution)
  - [Pick & Place 13-Stage State Machine](#pick--place-13-stage-state-machine)
- [Kinematic Calibration & TF Frame Reference](#kinematic-calibration--tf-frame-reference)
- [Troubleshooting & Best Practices](#troubleshooting--best-practices)

---

## Overview & Architecture

The suite is engineered around a unified, modular architecture separating model definitions, planning pipelines, hardware execution backends, and high-level applications:

| Layer | Primary Packages | Key Responsibilities |
| :--- | :--- | :--- |
| **Application Layer** | [`lexium_cobot_l18s_pick_and_place`](src/lexium_cobot_l18s_pick_and_place), [`lexium_cobot_l18s_bringup`](src/lexium_cobot_l18s_bringup) | Table-to-table pick & place orchestrator, Cartesian Joystick GUI, keyboard teleop, Pilz demos |
| **Motion Planning Layer** | [`lexium_cobot_l18s_moveit_config`](src/lexium_cobot_l18s_moveit_config) | MoveGroup planning server, OMPL pipeline, Pilz Industrial (PTP/LIN/CIRC/Blend), MoveIt Servo |
| **Bringup & Execution** | [`lexium_cobot_l18s_bringup`](src/lexium_cobot_l18s_bringup) | Unified orchestration across Mock Hardware (`mode:=mock`), Gazebo Simulation (`mode:=sim`), and Real Hardware hooks (`mode:=real`) |
| **Model & Tooling** | [`lexium_cobot_l18s`](src/lexium_cobot_l18s) | 6-DOF URDF/Xacro, joint limits, meshes, modular tools (Parallel Jaw & Vacuum Gripper), `ros2_control` |

*(Detailed system architecture flowchart available in [`lexium_cobot_l18s_bringup/README.md`](src/lexium_cobot_l18s_bringup/README.md#system-architecture). Note: All testing and validation are performed in simulation; no physical robot hardware is connected).*

---

## Workspace Directory Structure

```text
schneider_ws/
├── colcon_defaults.yaml
├── README.md                          <-- This repository documentation
└── src/
    ├── lexium_cobot_l18s/             <-- Core description, URDF, meshes, sensors, tools
    │   ├── config/                    <-- Joint controllers YAML
    │   ├── launch/                    <-- Display, description, Gazebo base launch files
    │   ├── meshes/                    <-- Visual & collision OBJ/STL geometries
    │   └── urdf/                      <-- Robot URDF, tools macro, ros2_control Xacro
    │       └── tools/                 <-- Generic Parallel Gripper & Vacuum Gripper
    ├── lexium_cobot_l18s_bringup/     <-- Master unified bringup orchestration
    │   ├── launch/                    <-- bringup, mock, sim, real, servo, teleop_gui launches
    │   └── scripts/                   <-- Joystick GUI, keyboard teleop, Pilz & gripper demos
    ├── lexium_cobot_l18s_moveit_config/ <-- MoveIt 2 configuration package
    │   ├── config/                    <-- SRDF, kinematics, OMPL, Pilz, Cartesian limits, servo
    │   └── launch/                    <-- demo, move_group, moveit_rviz, moveit_gazebo, servo
    └── lexium_cobot_l18s_pick_and_place/ <-- Industrial dual-table pick and place application
        ├── config/                    <-- Planning scene objects & motion waypoints YAML
        ├── launch/                    <-- pick_and_place_cell & pick_and_place_demo launches
        ├── rviz/                      <-- Customized RViz2 layout with planning scene
        ├── scripts/                   <-- Main pick_and_place_node.py orchestrator
        └── worlds/                    <-- pick_and_place_cell.sdf Gazebo Harmonic simulation world
```

---

## Prerequisites & Building

### Operating System & Dependencies

- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**: Jazzy Jalisco (Desktop Install)
- **Gazebo**: Gazebo Harmonic (`ros_gz_sim`, `gz_ros2_control`, `ros_gz_bridge`)
- **MoveIt 2**: `ros-jazzy-moveit`, `ros-jazzy-moveit-servo`, `ros-jazzy-pilz-industrial-motion-planner`
- **Control**: `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-parallel-gripper-action-controller`
- **Python**: Python 3.12 with `PyQt5` or `PySide6` (for GUI teleop)

Install all system ROS 2 packages:
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-gripper-controllers \
  python3-pip python3-pyqt5
```

### Build the Workspace

```bash
cd /home/mox/projects/schneider_ws
source /opt/ros/jazzy/setup.bash

# Build all packages with symlinked install
colcon build --symlink-install

# Source the overlay
source install/setup.bash
```

---

## Package 1: `lexium_cobot_l18s` (Robot Description & Models)

Provides kinematic specifications, inertia tensors, collision bounding boxes, 3D meshes, modular tool attachment macros, and `ros2_control` hardware interface definitions for the Schneider Electric Lexium Cobot L18S.

### Features

- **6-DOF Kinematic Chain**: Joint origins matching the physical robot (`l_Base_Link002` to `l_Link_017` and wrist flange `tool0`).
- **Modular End-Effector Framework (`Lexium_Cobot_L18S002_tools.urdf.xacro`)**:
  - `tool_type:=gripper`: Parallel jaw gripper with $40\,\text{mm}$ stroke per finger ($106\,\text{mm}$ maximum span, $26\,\text{mm}$ closed base span), polyurethane contact pads ($\mu=10.0$ friction), and dynamic mimic joints.
  - `tool_type:=vacuum`: Industrial suction cup tool with mounting flange and elastomeric vacuum seal.
  - `tool_type:=none`: Bare ISO tool flange (`tool0` / `tcp_link`).
- **Gazebo Harmonic Ready**: Tuned inertial properties, ODE contact surfaces, joint damping, friction, and `gz_ros2_control::GazeboSimROS2ControlPlugin` integration.

| Bare Flange (`tool_type:=none`) | Parallel Jaw Gripper (`tool_type:=gripper`) | Vacuum Suction Tool (`tool_type:=vacuum`) |
| :---: | :---: | :---: |
| <img width="260" alt="Gazebo Bare Flange" src="https://github.com/user-attachments/assets/e73106a6-f2a6-4d17-84d3-6606b19013d9" /> | <img width="260" alt="Gazebo Parallel Gripper" src="https://github.com/user-attachments/assets/a0408b53-bc13-463f-9bdf-4e55041a670c" /> | <img width="260" alt="Gazebo Vacuum Tool" src="https://github.com/user-attachments/assets/f6b58c3a-9539-42d7-84f6-bcc5c2502c0b" /> |

### Launch Commands

#### 1. Inspect URDF & Joints in RViz with Sliders (`display.launch.py`)
Launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 to manually articulate every robot joint and verify tool attachments:

<p align="center">
  <img width="800" alt="Lexium Cobot L18S in RViz2" src="https://github.com/user-attachments/assets/9e933355-73cc-41a9-8b50-b22215f76cd6" />
</p>

```bash
# Bare arm flange
ros2 launch lexium_cobot_l18s display.launch.py

# With Parallel Jaw Gripper attached
ros2 launch lexium_cobot_l18s display.launch.py tool_type:=gripper

# With Vacuum Suction Cup attached
ros2 launch lexium_cobot_l18s display.launch.py tool_type:=vacuum
```

#### 2. Publish Robot Description (`description.launch.py`)
Starts `robot_state_publisher` and broadcasts `/robot_description` and TF frames:
```bash
ros2 launch lexium_cobot_l18s description.launch.py tool_type:=gripper
```

#### 3. Base Gazebo Harmonic World (`gazebo.launch.py`)
Spawns the Lexium Cobot inside Gazebo Harmonic with default empty world and controllers:
```bash
ros2 launch lexium_cobot_l18s gazebo.launch.py tool_type:=gripper
```

#### 4. Base Mock Hardware (`mock.launch.py`)
Initializes `ros2_control_node` with simulated mock hardware interfaces:
```bash
ros2 launch lexium_cobot_l18s mock.launch.py tool_type:=gripper
```

---

## Package 2: `lexium_cobot_l18s_moveit_config` (MoveIt 2 Planning Configuration)

Provides complete MoveIt 2 semantic descriptions, collision matrices, kinematics solvers, Cartesian limits, controller bindings, and dual planning pipeline integration (OMPL & Pilz Industrial Motion Planner).

### Features

- **Dual Planning Pipeline Architecture**:
  - **OMPL**: RRTConnect, PRM, and KPIECE for collision-free global trajectory discovery.
  - **Pilz Industrial Motion Planner**: Deterministic industrial trajectories with **PTP** (Point-to-Point), **LIN** (Linear straight-line Cartesian motions), **CIRC** (Circular arcs), and **MoveGroupSequenceAction** blending.
- **Calibrated SRDF (`Lexium_Cobot_L18S002.srdf`)**: Complete allowed collision matrix (ACM) between all 6 links, wrist links, base pedestal, and parallel gripper finger links.
- **MoveIt Servo Real-Time Jogging**: Zero-latency velocity control via Cartesian Twist and Joint Jog inputs.
- **Predefined Named Poses**: `home` (all zeros), `ready` (inspection pose facing workspace center), `open` ($40\,\text{mm}$ gripper stroke), and `close` ($0\,\text{mm}$).

### Launch Commands

#### 1. Interactive MoveIt 2 Demonstration (`demo.launch.py`)
Launches MoveGroup with fake/mock controllers and the full RViz2 Motion Planning display with interactive 6-DOF Cartesian drag markers:

<p align="center">
  <img width="650" alt="MoveIt Trajectory Trail Visualization in RViz" src="https://github.com/user-attachments/assets/eb38aa3e-282e-45f6-8636-e1e4ed54a976" />
</p>

```bash
# Bare arm
ros2 launch lexium_cobot_l18s_moveit_config demo.launch.py

# With Parallel Gripper
ros2 launch lexium_cobot_l18s_moveit_config demo.launch.py tool_type:=gripper
```

#### 2. Standalone MoveGroup Planning Server (`move_group.launch.py`)
Runs the MoveIt 2 `move_group` node without starting RViz:
```bash
ros2 launch lexium_cobot_l18s_moveit_config move_group.launch.py tool_type:=gripper
```

#### 3. Standalone MoveIt RViz Visualization (`moveit_rviz.launch.py`)
Connects an RViz2 instance to an already running MoveGroup server:
```bash
ros2 launch lexium_cobot_l18s_moveit_config moveit_rviz.launch.py
```

#### 4. MoveIt Connected to Gazebo Simulation (`moveit_gazebo.launch.py`)
Launches MoveGroup configured specifically to command `gz_ros2_control` trajectory controllers:
```bash
ros2 launch lexium_cobot_l18s_moveit_config moveit_gazebo.launch.py tool_type:=gripper
```

#### 5. Real-Time MoveIt Servo Node (`servo.launch.py`)
Initializes `moveit_servo` node accepting Twist and Jog commands:
```bash
ros2 launch lexium_cobot_l18s_moveit_config servo.launch.py
```

---

## Package 3: `lexium_cobot_l18s_bringup` (Master Bringup & Teleoperation)

The centralized bringup and control package for the entire robot workcell, providing unified execution mode switching, shortcuts, and interactive teleoperation tools.

### Features

- **Unified Master Launch (`bringup.launch.py`)**: Seamlessly switches between execution environments using `mode:=mock`, `mode:=sim`, or `mode:=real`.
- **Convenience Shortcut Launches**: Dedicated launch files for mock, simulation, real hardware, and servo streaming.
- **Modern Cartesian Joystick GUI (`teleop_gui.py`)**: PyQt-based desktop teleoperation application featuring 6-DOF Cartesian jogging, joint sliders, safety deadman switch, adjustable velocity scaling, preset postures, and live position telemetry.
- **Terminal Keyboard Teleoperation (`teleop_keyboard.py`)**: Responsive keyboard jogging with linear, angular, and joint-level hotkeys.
- **Industrial Motion Demonstration (`pilz_motion_demo.py`)**: Automated verification of PTP, LIN, CIRC, and trajectory blending.

### Launch Commands

#### 1. Master Bringup (`bringup.launch.py`)
```bash
# Mock Hardware Mode (Fast, offline, zero physics overhead)
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=mock tool_type:=gripper

# Gazebo Harmonic Simulation Mode (Full physics, contact dynamics)
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=sim tool_type:=gripper

# Real Hardware Mode (Physical robot driver connection)
ros2 launch lexium_cobot_l18s_bringup bringup.launch.py mode:=real tool_type:=gripper
```

#### 2. Shortcut Launches
```bash
# Convenience launch for Mock mode
ros2 launch lexium_cobot_l18s_bringup mock.launch.py tool_type:=gripper

# Convenience launch for Gazebo simulation
ros2 launch lexium_cobot_l18s_bringup sim.launch.py tool_type:=gripper

# Convenience launch for Real hardware
ros2 launch lexium_cobot_l18s_bringup real.launch.py tool_type:=gripper

# Convenience launch for Real-time MoveIt Servo streaming
ros2 launch lexium_cobot_l18s_bringup servo.launch.py mode:=mock tool_type:=gripper
```

#### 3. Teleoperation GUI Bundle (`teleop_gui.launch.py`)
Launches the complete bringup stack with MoveIt Servo and opens the Cartesian Joystick GUI:
```bash
ros2 launch lexium_cobot_l18s_bringup teleop_gui.launch.py mode:=mock tool_type:=gripper
```

### Executable Scripts & Teleoperation

#### Cartesian Joystick GUI (`teleop_gui.py`)
To run the Joystick GUI against an already running bringup stack:

https://github.com/user-attachments/assets/77d3bd26-471f-493c-81c3-08ace696d276

```bash
# In Terminal 1: Launch bringup with servo enabled
ros2 launch lexium_cobot_l18s_bringup servo.launch.py mode:=sim tool_type:=gripper

# In Terminal 2: Start the GUI
ros2 run lexium_cobot_l18s_bringup teleop_gui.py
```
*GUI Features:*
- **Jogging**: Hold directional buttons for $\pm X, \pm Y, \pm Z$, Roll, Pitch, Yaw.
- **Safety**: Dedicated **HOLD TO JOG** Deadman switch prevents accidental motions.
- **Sliders**: Individual joint position adjustment with interactive feedback.
- **Presets**: Instant recall of `Home`, `Ready`, `Pick Approach`, and `Place Approach`.

#### Interactive Keyboard Teleop (`teleop_keyboard.py`)
```bash
ros2 run lexium_cobot_l18s_bringup teleop_keyboard.py
```
*Keyboard Mapping:*
- `W` / `S`: $\pm X$ (Forward / Backward)
- `A` / `D`: $\pm Y$ (Left / Right)
- `R` / `F`: $\pm Z$ (Up / Down)
- `U` / `J`: $\pm \text{Roll}$
- `I` / `K`: $\pm \text{Pitch}$
- `O` / `L`: $\pm \text{Yaw}$
- `Space`: Emergency Stop (Zero velocity command)

#### Pilz Industrial Motion Demo (`pilz_motion_demo.py`)
Verifies deterministic PTP, LIN, CIRC, and blended sequence trajectory generation:

https://github.com/user-attachments/assets/6361c3aa-ce23-4cde-8aae-f646ef4732f5

```bash
ros2 run lexium_cobot_l18s_bringup pilz_motion_demo.py
```

#### Gripper Action Test (`gripper_demo.py`)
Exercises parallel gripper open, close, and force clamping actions:
```bash
ros2 run lexium_cobot_l18s_bringup gripper_demo.py
```

---

## Package 4: `lexium_cobot_l18s_pick_and_place` (Industrial Workcell Application)

A complete industrial Pick-and-Place workcell application designed for automated Table-to-Table material handling between symmetric workbenches.

https://github.com/user-attachments/assets/b81ddbc6-6508-448d-9484-2ec695c9036f

### Features

- **Dual-Table Workcell (`pick_and_place_cell.sdf`)**:
  - Source Pick Table at $+Y = 0.55\,\text{m}$ (tabletop at $Z_{\text{world}} = 0.30\,\text{m}$).
  - Destination Place Table at $-Y = -0.55\,\text{m}$ (tabletop at $Z_{\text{world}} = 0.30\,\text{m}$).
  - Mounting Pedestal at $(0, 0)$ elevating robot base to $Z_{\text{world}} = 0.05\,\text{m}$.
  - Cylindrical Workpiece ($80\,\text{mm}$ height, $50\,\text{mm}$ diameter, mass $0.35\,\text{kg}$) with high-grip friction pads ($\mu=10.0$).
- **MoveIt 2 Planning Scene Synchronization**: Injects collision models dynamically, monitors obstacles, attaches workpiece to `tool0` upon grasp (`AttachedCollisionObject`), and detaches upon release.
- **Kinematically Aligned Vertical Descent**: Tool flange orientation remains strictly vertical down ($\hat{z}_{\text{tool0}} = [0, 0, -1]$) during approach, descent, grasp, lift, transit, and placement.
- **Simulation Clock Synchronization**: Employs `use_sim_time: true` and simulation-aware sleep intervals (`sleep_sim()`) to prevent timing desynchronization under fluctuating Real-Time Factors.
- **Gripper Stall & Contact Clamping**: Automatically detects physical workpiece contact via `stalled: true` reporting in `ParallelGripperCommand` action server.

### Launch Commands & Node Execution

#### Method 1: Single-Command Automated Demonstration (`pick_and_place_demo.launch.py`)
Launches the entire workcell, RViz2, MoveIt 2, and automatically triggers the pick-and-place sequence:
```bash
ros2 launch lexium_cobot_l18s_pick_and_place pick_and_place_demo.launch.py
```

#### Method 2: Two-Step Independent Execution (Recommended for Debugging)

**Step 1: Launch the Robotic Workcell**
```bash
# In Simulation (Gazebo Harmonic + MoveIt + RViz):
ros2 launch lexium_cobot_l18s_pick_and_place pick_and_place_cell.launch.py mode:=sim

# In Mock Hardware Mode (Offline fast execution):
ros2 launch lexium_cobot_l18s_pick_and_place pick_and_place_cell.launch.py mode:=mock
```

**Step 2: Run the Task Orchestrator Node**
```bash
# Run a single pick-and-place cycle:
ros2 run lexium_cobot_l18s_pick_and_place pick_and_place_node.py

# Run continuous automated pick-and-place loop:
ros2 run lexium_cobot_l18s_pick_and_place pick_and_place_node.py --continuous

# Run a specific number of cycles (e.g. 3 cycles):
ros2 run lexium_cobot_l18s_pick_and_place pick_and_place_node.py --cycles 3
```

---

### Pick & Place 13-Stage State Machine

The task orchestrator executes a deterministic 13-stage pick, transfer, and place cycle:

1. **Step 1: Open Gripper** ($40\,\text{mm}$ stroke) & Settle
2. **Step 2: Move Arm to Safe Ready** inspection posture
3. **Step 3: Move to Pick Table Approach** ($+Y=0.55\,\text{m}$, clearance height)
4. **Step 4: Lower Open Gripper** to grasp height ($Z_{\text{tool0}}=0.396\,\text{m}$, rubber pads at workpiece CoM)
5. **Step 5: Close Gripper on Workpiece** ($100\,\text{N}$ effort, physical stall contact detection)
6. **Step 6: Attach Workpiece** in MoveIt Planning Scene (`AttachedCollisionObject`)
7. **Step 7: Lift Workpiece** vertically to pick approach clearance height
8. **Step 8: Transit Workpiece** across workcell to Destination Table approach ($-Y=-0.55\,\text{m}$)
9. **Step 9: Lower Workpiece** onto destination table surface
10. **Step 10: Open Gripper** to release workpiece
11. **Step 11: Detach Workpiece** in Planning Scene and add as static obstacle at destination
12. **Step 12: Retract Arm** vertically to destination clearance height
13. **Step 13: Return Arm to Safe Ready** posture and close gripper for standby

*(For the complete interactive Mermaid sequence diagram, see [`lexium_cobot_l18s_pick_and_place/README.md`](src/lexium_cobot_l18s_pick_and_place/README.md#pick--place-state-machine-sequence))*

---

## Kinematic Calibration & TF Frame Reference

Because the robot base is mounted on a pedestal at $Z_{\text{world}} = 0.050\,\text{m}$ in Gazebo Harmonic, all coordinates in the robot planning frame (`l_Base_Link002`) are related to Gazebo world coordinates by:
$$Z_{\text{base}} = Z_{\text{world}} - 0.050\,\text{m}$$

### Verified Key Waypoints & Frame Coordinates

| Landmark / Waypoint | Coordinates in `l_Base_Link002` [X, Y, Z] | Coordinates in Gazebo World [X, Y, Z] | Physical Clearance / Purpose |
| :--- | :--- | :--- | :--- |
| **Tabletop Surface** | `[0.450, ±0.550, 0.250]` | `[0.450, ±0.550, 0.300]` | Workbench top surface |
| **Workpiece Center (CoM)**| `[0.450, 0.550, 0.290]` | `[0.450, 0.550, 0.340]` | Workpiece cylindrical geometric center |
| **Workpiece Top Rim** | `[0.450, 0.550, 0.330]` | `[0.450, 0.550, 0.380]` | Top boundary of 80mm cylinder |
| **Approach tool0 Flange** | `[0.450, ±0.550, 0.560]` | `[0.450, ±0.550, 0.610]` | Clearance height over workcell |
| **Grasp tool0 Flange** | `[0.450, 0.550, 0.396]` | `[0.450, 0.550, 0.446]` | **Rubber pads center exactly at Workpiece CoM** |
| **Grasp Gripper Fingertips**| `[0.450, 0.550, 0.272]` | `[0.450, 0.550, 0.322]` | **$+22.0\,\text{mm}$ safe clearance above table** |

### Verified Joint Postures (Radians)

```python
# Ready Inspection (Facing workspace center between both tables)
ready_posture         = [-1.5708,  0.4500, -1.0500,  0.6000,  1.5708,  0.0000]

# Pick Approach (+Y = 0.55m, tool pointing vertical down)
pick_approach_joints  = [-0.9042,  0.3632, -1.0971,  0.1105,  1.5708, -0.1013]

# Pick Grasp (+Y = 0.55m, tool0 at Z=0.396m)
pick_grasp_joints     = [-0.9042,  0.3932, -1.4672, -0.2896,  1.5708, -0.1012]

# Place Approach (-Y = -0.55m, tool pointing vertical down)
place_approach_joints = [ 0.9042, -0.3632,  1.0971, -0.1105, -1.5708,  0.1013]

# Place Release (-Y = -0.55m, tool0 at Z=0.396m)
place_release_joints  = [ 0.9042, -0.3932,  1.4672,  0.2896, -1.5708,  0.1012]
```

---

## Troubleshooting & Best Practices

1. **Simulation Real-Time Factor (RTF)**:
   - When running on complex worlds, Gazebo Harmonic RTF may drop to $\sim 0.35$.
   - **Always** ensure nodes using timers or delays specify `use_sim_time: true` to prevent premature timeout before the physical simulation steps advance.
2. **Gripper Stall Handling in ROS 2**:
   - `ParallelGripperCommand` completes with `stalled: true` and `reached_goal: false` when clamping onto an object. In robot controllers with `allow_stalling: true`, this is the standard success condition for force clamping.
3. **Resetting Workpiece in Gazebo World**:
   - To reset the workpiece back to the pick table without restarting Gazebo:
     ```bash
     gz service -s /world/pick_and_place_cell/set_pose \
       --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
       --req 'name: "target_workpiece", position: {x: 0.45, y: 0.55, z: 0.34}, orientation: {w: 1.0}'
     ```
4. **Inspecting Active Controllers**:
   ```bash
   ros2 control list_controllers
   ```
   Expected active controllers: `joint_state_broadcaster`, `arm_controller`, and `gripper_controller`.
