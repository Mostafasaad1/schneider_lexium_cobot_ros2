#!/usr/bin/env python3
"""Interactive Keyboard Teleoperation for Schneider Electric Lexium Cobot L18S with MoveIt 2 Servo."""

import sys
import termios
import tty
import select
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.msg import JointJog
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions
from moveit_msgs.srv import ServoCommandType

HELP_MSG = """
======================================================
  Schneider Electric Lexium Cobot L18S - MoveIt Servo Teleop
======================================================
  Mode Switching:
    [1] : Joint Jogging Mode
    [2] : Cartesian (3D Twist) Mode

  Pose Presets:
    [0] : Move to Zero / Home Position (all joints 0.0)
    [9] : Move to Ready / Working Posture

  Joint Jog Mode (Press 1):
    [q / a] : Joint 1 (Base Rotation) +/-
    [w / s] : Joint 2 (Shoulder) +/-
    [e / d] : Joint 3 (Elbow) +/-
    [r / f] : Joint 4 (Wrist 1) +/-
    [t / g] : Joint 5 (Wrist 2) +/-
    [y / h] : Joint 6 (Wrist 3) +/-

  Cartesian Mode (Press 2):
    [Arrow Up / Down]         : Linear X (Forward / Backward)
    [Arrow Left / Right]      : Linear Y (Left / Right)
    [Shift + Up / Down]       : Linear Z (Up / Down)
    [w / s]                   : Linear Z (Up / Down) alternative
    [PageUp / PageDown]       : Linear Z (Up / Down) alternative

  Speed Control:
    [+] : Increase Speed
    [-] : Decrease Speed
    [Space] : Stop Motion
    [Ctrl+C] : Quit
======================================================
"""

KEY_MAPPINGS = {
    # Joint jog keys: (joint_index, direction)
    'q': (0, 1.0), 'a': (0, -1.0),
    'w': (1, 1.0), 's': (1, -1.0),
    'e': (2, 1.0), 'd': (2, -1.0),
    'r': (3, 1.0), 'f': (3, -1.0),
    't': (4, 1.0), 'g': (4, -1.0),
    'y': (5, 1.0), 'h': (5, -1.0),
}

JOINT_NAMES = [
    'l_Base_Link002__to__l_Link_012',
    'l_Link_012__to__l_Link_013',
    'l_Link_013__to__l_Link_014',
    'l_Link_014__to__l_Link_015',
    'l_Link_015__to__l_Link_016',
    'l_Link_016__to__l_Link_017',
]


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('lexium_servo_keyboard_teleop')
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.switch_cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.mode = 'joint'  # 'joint' or 'cartesian'
        self.joint_speed = 0.4
        self.linear_speed = 0.1
        self.joint_positions = {}

        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

    def js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def switch_mode(self, mode_int: int):
        if not self.switch_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /servo_node/switch_command_type not available!")
            return False
        req = ServoCommandType.Request()
        req.command_type = mode_int
        fut = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.result() and fut.result().success

    def publish_joint_jog(self, joint_idx: int, direction: float):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'l_Base_Link002'
        msg.joint_names = [JOINT_NAMES[joint_idx]]
        msg.velocities = [direction * self.joint_speed]
        self.joint_pub.publish(msg)

    def publish_twist(self, lx=0.0, ly=0.0, lz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'l_Base_Link002'
        msg.twist.linear.x = float(lx) * self.linear_speed
        msg.twist.linear.y = float(ly) * self.linear_speed
        msg.twist.linear.z = float(lz) * self.linear_speed
        self.twist_pub.publish(msg)

    def move_to_joint_target(self, target_positions, pose_name="Target"):
        print(f"\n>>> Moving to {pose_name} pose...")
        
        # 1. Try MoveGroup action first (with full collision-aware planning)
        if self.move_client.wait_for_server(timeout_sec=1.0):
            print(">>> Planning collision-aware trajectory via MoveGroup...")
            goal_msg = MoveGroup.Goal()
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.group_name = "lexium_arm"
            goal_msg.request.num_planning_attempts = 5
            goal_msg.request.allowed_planning_time = 5.0

            constraints = Constraints()
            for j_name, pos in zip(JOINT_NAMES, target_positions):
                jc = JointConstraint()
                jc.joint_name = j_name
                jc.position = float(pos)
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

            goal_msg.request.goal_constraints.append(constraints)
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False

            send_future = self.move_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            goal_handle = send_future.result()
            if goal_handle and goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
                result = result_future.result().result
                if result.error_code.val == 1:
                    print(f">>> Successfully reached {pose_name} pose!")
                    mode_int = ServoCommandType.Request.JOINT_JOG if self.mode == 'joint' else ServoCommandType.Request.TWIST
                    self.switch_mode(mode_int)
                    return True
                else:
                    print(f">>> MoveGroup planning failed (error code {result.error_code.val}), attempting direct controller trajectory...")

        # 2. Fallback to direct FollowJointTrajectory action
        if self.traj_client.wait_for_server(timeout_sec=2.0):
            print(">>> Sending trajectory directly to /arm_controller/follow_joint_trajectory...")
            traj_goal = FollowJointTrajectory.Goal()
            traj_goal.trajectory.joint_names = JOINT_NAMES
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in target_positions]
            pt.time_from_start = Duration(sec=4, nanosec=0)
            traj_goal.trajectory.points.append(pt)

            send_future = self.traj_client.send_goal_async(traj_goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            goal_handle = send_future.result()
            if goal_handle and goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=8.0)
                print(f">>> Successfully reached {pose_name} pose via controller!")
                mode_int = ServoCommandType.Request.JOINT_JOG if self.mode == 'joint' else ServoCommandType.Request.TWIST
                self.switch_mode(mode_int)
                return True
            else:
                print(">>> Trajectory goal rejected by controller.")
                return False

        print(">>> Error: Neither MoveGroup nor arm_controller action servers are reachable.")
        return False


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = os.read(sys.stdin.fileno(), 16).decode('utf-8', errors='ignore')
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()

    print(HELP_MSG)
    print("Initializing in Joint Jog mode...")
    node.switch_mode(ServoCommandType.Request.JOINT_JOG)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)

            if not key:
                continue

            if key == '\x03':  # Ctrl+C
                break
            elif key == '1':
                node.mode = 'joint'
                node.switch_mode(ServoCommandType.Request.JOINT_JOG)
                print("\n>>> Switched to [JOINT JOG] mode.")
            elif key == '2':
                node.mode = 'cartesian'
                node.switch_mode(ServoCommandType.Request.TWIST)
                print("\n>>> Switched to [CARTESIAN TWIST] mode.")
            elif key == '0':
                node.move_to_joint_target([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], pose_name="Zero / Home")
            elif key == '9':
                node.move_to_joint_target([0.0, -0.785, 1.57, -0.785, 1.57, 0.0], pose_name="Ready")
            elif key in ['+', '=']:
                node.joint_speed = min(1.5, node.joint_speed + 0.05)
                node.linear_speed = min(0.5, node.linear_speed + 0.02)
                print(f"\nSpeed increased -> Joint: {node.joint_speed:.2f} rad/s | Cartesian: {node.linear_speed:.2f} m/s")
            elif key in ['-', '_']:
                node.joint_speed = max(0.05, node.joint_speed - 0.05)
                node.linear_speed = max(0.01, node.linear_speed - 0.02)
                print(f"\nSpeed decreased -> Joint: {node.joint_speed:.2f} rad/s | Cartesian: {node.linear_speed:.2f} m/s")
            elif key == ' ':
                # Send zero command to halt
                if node.mode == 'joint':
                    node.publish_joint_jog(0, 0.0)
                else:
                    node.publish_twist(0, 0, 0)
                print("\n>>> STOP commanded.")

            if node.mode == 'joint' and key in KEY_MAPPINGS:
                idx, direction = KEY_MAPPINGS[key]
                node.publish_joint_jog(idx, direction)
                print(f"\rJogging {JOINT_NAMES[idx].split('__to__')[-1]}: {direction * node.joint_speed:+.2f} rad/s", end="", flush=True)

            elif node.mode == 'cartesian':
                if key in ['\x1b[A', '\x1bOA']:  # Up arrow
                    node.publish_twist(lx=1.0)
                    print(f"\rCartesian +X (Forward): +{node.linear_speed:.2f} m/s", end="", flush=True)
                elif key in ['\x1b[B', '\x1bOB']:  # Down arrow
                    node.publish_twist(lx=-1.0)
                    print(f"\rCartesian -X (Backward): -{node.linear_speed:.2f} m/s", end="", flush=True)
                elif key in ['\x1b[D', '\x1bOD']:  # Left arrow
                    node.publish_twist(ly=1.0)
                    print(f"\rCartesian +Y (Left): +{node.linear_speed:.2f} m/s", end="", flush=True)
                elif key in ['\x1b[C', '\x1bOC']:  # Right arrow
                    node.publish_twist(ly=-1.0)
                    print(f"\rCartesian -Y (Right): -{node.linear_speed:.2f} m/s", end="", flush=True)
                elif key in ['\x1b[1;2A', '\x1b[a', '\x1bO2A', 'w', 'W', '\x1b[5~']:  # Shift+Up arrow, 'w', or Page Up
                    node.publish_twist(lz=1.0)
                    print(f"\rCartesian +Z (Up): +{node.linear_speed:.2f} m/s", end="", flush=True)
                elif key in ['\x1b[1;2B', '\x1b[b', '\x1bO2B', 's', 'S', '\x1b[6~']:  # Shift+Down arrow, 's', or Page Down
                    node.publish_twist(lz=-1.0)
                    print(f"\rCartesian -Z (Down): -{node.linear_speed:.2f} m/s", end="", flush=True)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleoperation stopped.")


if __name__ == '__main__':
    main()
