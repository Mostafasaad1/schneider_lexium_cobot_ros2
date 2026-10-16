# `lexium_cobot_l18s_moveit_config`

MoveIt 2 motion planning configuration package for the **Schneider Electric Lexium Cobot L18S** collaborative robot arm in ROS 2 Jazzy.

---

## Overview

This package configures the complete MoveIt 2 motion planning framework for the Lexium Cobot L18S:
- **Dual Motion Planning Pipelines**:
  - **OMPL**: Default RRTConnect, PRM, and KPIECE for collision-free global trajectory discovery.
  - **Pilz Industrial Motion Planner**: Deterministic industrial trajectories supporting **PTP** (Point-to-Point), **LIN** (Linear straight Cartesian motions), **CIRC** (Circular arcs), and **MoveGroupSequenceAction** trajectory blending.
- **Semantic Robot Description (SRDF)**: Complete allowed collision matrix (ACM), joint groups (`lexium_arm` and `gripper`), end-effector bindings, and named postures (`home`, `ready`, `open`, `close`).
- **Real-Time Jogging with MoveIt Servo**: Low-latency Cartesian Twist and Joint Jog streaming for joystick teleoperation.
- **Cartesian & Kinematic Limits**: Strict acceleration, jerk, and velocity limits configured for safe human-robot collaboration.

https://github.com/user-attachments/assets/6361c3aa-ce23-4cde-8aae-f646ef4732f5

> **Pilz Industrial Motion Execution**: Trajectory execution demonstrating deterministic linear Cartesian motions (LIN) and circular arcs (CIRC) with trajectory blending.

---

## Package Structure

```text
lexium_cobot_l18s_moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── Lexium_Cobot_L18S002.srdf.xacro         # Dynamic Semantic Robot Description (SRDF)
│   ├── kinematics.yaml                         # KDL / pick_ik kinematics solver parameters
│   ├── ompl_planning.yaml                      # OMPL algorithm parameters
│   ├── pilz_industrial_motion_planner.yaml     # Pilz motion planning configuration
│   ├── pilz_cartesian_limits.yaml              # Pilz linear & angular Cartesian limits
│   ├── joint_limits.yaml                       # Velocity, acceleration, effort limits
│   ├── moveit_controllers.yaml                 # Controller action bindings
│   └── moveit_servo.yaml                       # MoveIt Servo real-time streaming parameters
└── launch/
    ├── demo.launch.py                          # Interactive MoveIt demonstration
    ├── move_group.launch.py                    # MoveGroup planning server
    ├── moveit_rviz.launch.py                   # MoveIt RViz motion planning display
    ├── moveit_gazebo.launch.py                 # MoveIt integration with Gazebo Harmonic
    └── servo.launch.py                         # Standalone MoveIt Servo streaming node
```

---

## Launch Commands

### 1. Interactive MoveIt 2 Demonstration (`demo.launch.py`)
Launches MoveGroup with fake/mock controllers and the full RViz2 Motion Planning display with interactive 6-DOF Cartesian drag markers:

<p align="center">
  <img width="650" alt="MoveIt Trajectory Trail Visualization in RViz" src="https://github.com/user-attachments/assets/eb38aa3e-282e-45f6-8636-e1e4ed54a976" />
</p>

```bash
# Bare arm flange
ros2 launch lexium_cobot_l18s_moveit_config demo.launch.py tool_type:=none

# With Parallel Gripper attached
ros2 launch lexium_cobot_l18s_moveit_config demo.launch.py tool_type:=gripper

# With Vacuum Suction Cup attached
ros2 launch lexium_cobot_l18s_moveit_config demo.launch.py tool_type:=vacuum
```

### 2. Standalone MoveGroup Planning Server (`move_group.launch.py`)
Runs the MoveGroup planning node headless (no RViz):
```bash
ros2 launch lexium_cobot_l18s_moveit_config move_group.launch.py tool_type:=gripper
```

### 3. Connect RViz to Existing MoveGroup (`moveit_rviz.launch.py`)
```bash
ros2 launch lexium_cobot_l18s_moveit_config moveit_rviz.launch.py
```

### 4. MoveIt Connected to Gazebo Simulation (`moveit_gazebo.launch.py`)
```bash
ros2 launch lexium_cobot_l18s_moveit_config moveit_gazebo.launch.py tool_type:=gripper
```

### 5. Real-Time MoveIt Servo Node (`servo.launch.py`)
Initializes the `moveit_servo` node accepting Twist and Jog commands on `/servo_node/delta_twist_cmds` and `/servo_node/delta_joint_cmds`:
```bash
ros2 launch lexium_cobot_l18s_moveit_config servo.launch.py
```
