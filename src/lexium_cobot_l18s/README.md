# `lexium_cobot_l18s`

Robot description and base modeling assets for the **Schneider Electric Lexium Cobot L18S** collaborative robot in ROS 2 Jazzy.

---

## Overview

This package serves as the single source of truth for the physical, kinematic, visual, inertial, and sensor attributes of the Lexium Cobot L18S:
- **Kinematic Model**: Complete 6-DOF articulated serial arm definitions (`l_Base_Link002` to wrist flange `tool0`).
- **3D Meshes**: High-resolution visual OBJ models and optimized collision geometries.
- **Modular End-Effector Framework (`Lexium_Cobot_L18S002_tools.urdf.xacro`)**:
  - `tool_type:=gripper`: Parallel jaw gripper with polyurethane contact pads, ODE friction surfaces, and dynamic mimic joints.
  - `tool_type:=vacuum`: Pneumatic suction tool with mounting flange and elastomeric vacuum seal.
  - `tool_type:=none`: Bare ISO tool flange (`tool0` / `tcp_link`).
- **`ros2_control` Interface**: Hardware interface integration supporting mock hardware and Gazebo Harmonic (`gz_ros2_control`).

| Bare Flange (`tool_type:=none`) | Parallel Jaw Gripper (`tool_type:=gripper`) | Vacuum Suction Tool (`tool_type:=vacuum`) |
| :---: | :---: | :---: |
| <img width="260" alt="Gazebo Bare Flange" src="https://github.com/user-attachments/assets/e73106a6-f2a6-4d17-84d3-6606b19013d9" /> | <img width="260" alt="Gazebo Parallel Gripper" src="https://github.com/user-attachments/assets/a0408b53-bc13-463f-9bdf-4e55041a670c" /> | <img width="260" alt="Gazebo Vacuum Tool" src="https://github.com/user-attachments/assets/f6b58c3a-9539-42d7-84f6-bcc5c2502c0b" /> |

---

## Package Structure

```text
lexium_cobot_l18s/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── Lexium_Cobot_L18S002_controllers.yaml   # Joint Trajectory & Gripper controllers
│   └── gui.config                              # Gazebo Harmonic GUI layout
├── launch/
│   ├── description.launch.py                   # robot_state_publisher launcher
│   ├── display.launch.py                       # RViz2 with joint sliders
│   ├── gazebo.launch.py                        # Base Gazebo Harmonic spawn launcher
│   └── mock.launch.py                          # Base mock hardware launcher
├── meshes/                                     # Link OBJ visual and collision models
└── urdf/
    ├── Lexium_Cobot_L18S002.urdf               # Base robot URDF
    ├── Lexium_Cobot_L18S002_wrapper.urdf.xacro # Top-level Xacro wrapper
    ├── Lexium_Cobot_L18S002_tools.urdf.xacro   # Modular tool attachment macros
    ├── Lexium_Cobot_L18S002_ros2_control.urdf.xacro # ros2_control hardware tags
    ├── Lexium_Cobot_L18S002_sensors.urdf.xacro # Camera and sensor integration
    └── tools/
        ├── generic_parallel_gripper.urdf.xacro # 40mm parallel jaw gripper
        └── generic_vacuum_gripper.urdf.xacro   # Industrial vacuum cup
```

---

## Launch Commands

### 1. Visual Inspection in RViz with Joint Sliders

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

### 2. Publish Robot Description
```bash
ros2 launch lexium_cobot_l18s description.launch.py tool_type:=gripper
```

### 3. Base Gazebo Harmonic Simulation
```bash
ros2 launch lexium_cobot_l18s gazebo.launch.py tool_type:=gripper
```

### 4. Base Mock Hardware
```bash
ros2 launch lexium_cobot_l18s mock.launch.py tool_type:=gripper
```
