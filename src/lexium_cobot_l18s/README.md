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
