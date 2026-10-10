#!/usr/bin/env python3
"""Interactive Keyboard Teleoperation for Schneider Electric Lexium Cobot L18S with MoveIt 2 Servo."""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ServoCommandType
import os

HELP_MSG = """
======================================================
  Schneider Electric Lexium Cobot L18S - MoveIt Servo Teleop
======================================================
  Mode Switching:
    [1] : Joint Jogging Mode
    [2] : Cartesian (3D Twist) Mode

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
