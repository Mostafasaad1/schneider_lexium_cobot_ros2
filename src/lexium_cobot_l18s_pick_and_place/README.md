# `lexium_cobot_l18s_pick_and_place`

Complete industrial Table-to-Table Pick and Place application for the **Schneider Electric Lexium Cobot L18S** robot arm in ROS 2 Jazzy.

---

## Overview

This package provides an end-to-end industrial pick-and-place workcell with:
- **Dual Table Workcell**: Symmetrical source (Pick Table at $+Y$) and destination (Place Table at $-Y$) workbenches with robot base mounting pedestal.
- **Gazebo Harmonic World**: High-fidelity physics simulation (`pick_and_place_cell.sdf`) featuring realistic lighting, workpiece inertial and high-friction contact mechanics.
- **MoveIt 2 Planning Scene Synchronization**: Automated injection of tables, obstacles, and dynamic workpiece attachment (`AttachedCollisionObject`) on grasp and detachment on release.
- **Robust Motion State Machine**: 13-stage deterministic pick, lift, transit, place, release, and safe return cycle with Pilz Industrial Motion Planner / OMPL.
- **Multi-Backend Execution**: Seamless execution in both **Mock Hardware** (`mode:=mock`) and **Gazebo Harmonic Simulation** (`mode:=sim`).

---

## Package Structure

```text
lexium_cobot_l18s_pick_and_place/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── scene_objects.yaml           # Planning scene geometries & coordinates
│   └── pick_and_place_params.yaml    # Speeds, offsets, tolerances, waypoints
├── worlds/
│   └── pick_and_place_cell.sdf      # Gazebo Harmonic dual-table workcell
├── launch/
│   ├── pick_and_place_cell.launch.py # Workcell bringup (MoveIt + RViz + Controllers)
│   └── pick_and_place_demo.launch.py # Single-command bringup + auto task execution
├── scripts/
│   └── pick_and_place_node.py       # Main MoveIt 2 pick & place orchestrator node
└── rviz/
    └── pick_and_place.rviz          # Tailored RViz2 layout with planning scene
```

---

## Quick Start & Usage

### 1. Single-Command Automated Demonstration (Mock Mode)

To launch the entire workcell (Robot + Parallel Gripper + MoveIt 2 + RViz) and automatically execute the pick-and-place cycle:

```bash
ros2 launch lexium_cobot_l18s_pick_and_place pick_and_place_demo.launch.py
```

### 2. Manual Two-Step Execution (Sim / Mock)

#### Step 1: Launch the Robotic Workcell
```bash
# Offline Mock Hardware + MoveIt 2 + RViz:
ros2 launch lexium_cobot_l18s_pick_and_place pick_and_place_cell.launch.py mode:=mock

# Gazebo Harmonic Physics Simulation + MoveIt 2 + RViz:
ros2 launch lexium_cobot_l18s_pick_and_place pick_and_place_cell.launch.py mode:=sim
```

#### Step 2: Run the Pick and Place Task Node
```bash
# Run a single pick-and-place cycle:
ros2 run lexium_cobot_l18s_pick_and_place pick_and_place_node.py

# Run continuous automated pick-and-place loop:
ros2 run lexium_cobot_l18s_pick_and_place pick_and_place_node.py --continuous

# Run a specific number of cycles (e.g., 5 cycles):
ros2 run lexium_cobot_l18s_pick_and_place pick_and_place_node.py --cycles 5
```

---

## Pick & Place State Machine Sequence

```text
[1] INIT & CONNECT      -> Connect to /move_action, /gripper_controller/gripper_cmd, /apply_planning_scene
[2] SETUP SCENE         -> Publish Pick Table, Place Table, Pedestal & Workpiece to MoveIt Planning Scene
[3] SAFE READY          -> Move arm to Ready inspection posture
[4] OPEN GRIPPER        -> Open parallel gripper (40mm stroke)
[5] PICK APPROACH       -> Move arm to clearance height above Pick Table (+Y=0.40m)
[6] PICK DESCEND        -> Lower gripper to workpiece grasp height
[7] GRASP WORKPIECE     -> Close gripper on workpiece (15mm stroke, 75N effort limit)
[8] ATTACH OBJECT       -> Dynamically attach workpiece in MoveIt Planning Scene (AttachedCollisionObject)
[9] PICK RETREAT        -> Lift workpiece off Pick Table to clearance height
[10] TRANSIT            -> Collision-free transit across workcell to Destination Table (-Y=-0.40m)
[11] PLACE DESCEND      -> Lower workpiece onto Destination Table surface
[12] RELEASE WORKPIECE  -> Open gripper to release workpiece
[13] DETACH OBJECT      -> Detach workpiece in Planning Scene at destination coordinates
[14] PLACE RETREAT      -> Retract arm to destination clearance height
[15] SAFE HOME/READY    -> Return arm to Safe Ready posture & close gripper
```

---

## Interfaces & Topics

- **Action Servers**:
  - `/move_action` (`moveit_msgs/action/MoveGroup`): Programmatic trajectory planning and execution.
  - `/gripper_controller/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`): Gripper actuation.
- **Services**:
  - `/apply_planning_scene` (`moveit_msgs/srv/ApplyPlanningScene`): Dynamic collision object & attachment management.
- **Topics**:
  - `/joint_states` (`sensor_msgs/msg/JointState`): Real-time joint feedback.
  - `/monitored_planning_scene` (`moveit_msgs/msg/PlanningScene`): Live planning scene state.
