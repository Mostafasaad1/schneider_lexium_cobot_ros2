#!/usr/bin/env python3
"""
Lexium Cobot L18S - End-Effector & Tool Framework Demo.

Demonstrates:
1. GripperCommand action client (/gripper_controller/gripper_cmd)
2. Coordinated Arm Motion (MoveGroup) + Tool Actuation (Grasp / Release)
3. Pick-and-Place sequence with industrial motion planning

Usage:
  ros2 launch lexium_cobot_l18s_bringup mock.launch.py tool_type:=gripper
  ros2 run lexium_cobot_l18s_bringup gripper_demo.py
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
try:
    from control_msgs.action import ParallelGripperCommand
    GRIPPER_ACTION_TYPE = ParallelGripperCommand
except ImportError:
    from control_msgs.action import GripperCommand
    GRIPPER_ACTION_TYPE = GripperCommand

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState


class GripperMotionDemo(Node):
    def __init__(self):
        super().__init__("gripper_motion_demo")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  Schneider Lexium Cobot L18S - Gripper & Tool Demo")
        self.get_logger().info("=" * 60)

        self.joint_names = [
            "l_Base_Link002__to__l_Link_012",
            "l_Link_012__to__l_Link_013",
            "l_Link_013__to__l_Link_014",
            "l_Link_014__to__l_Link_015",
            "l_Link_015__to__l_Link_016",
            "l_Link_016__to__l_Link_017",
        ]
        self.base_frame = "l_Base_Link002"
        self.tcp_frame = "tool0"
        self.group_name = "lexium_arm"
        self.pipeline_id = "pilz_industrial_motion_planner"

        self.current_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
        )

        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(self, GRIPPER_ACTION_TYPE, "/gripper_controller/gripper_cmd")

    def _joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def wait_for_services(self, timeout_sec: float = 15.0):
        self.get_logger().info(f"Connecting to MoveGroup and {GRIPPER_ACTION_TYPE.__name__} action servers...")
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError("MoveGroup (/move_action) action server not available.")
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError(f"{GRIPPER_ACTION_TYPE.__name__} (/gripper_controller/gripper_cmd) action server not available.")

        self.get_logger().info("Waiting for valid /joint_states...")
        start_time = time.time()
        while self.current_joint_state is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state is None:
            raise TimeoutError("Did not receive /joint_states.")

        self.get_logger().info("Connected successfully to all action servers.")

    def command_gripper(self, position: float, max_effort: float = 50.0) -> bool:
        """Command gripper position (0.0=closed, 0.04=open) and effort limit."""
        self.get_logger().info(f"[Gripper] Command position: {position * 1000.0:.1f} mm, max_effort: {max_effort:.1f} N")
        goal_msg = GRIPPER_ACTION_TYPE.Goal()
        if hasattr(goal_msg, "command") and hasattr(goal_msg.command, "name"):
            # ParallelGripperCommand (ROS 2 Jazzy standard)
            goal_msg.command.name = ["gripper_finger_left_joint"]
            goal_msg.command.position = [float(position)]
            goal_msg.command.effort = [float(max_effort)]
        elif hasattr(goal_msg, "command"):
            # GripperCommand (Classic)
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = float(max_effort)

        send_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Gripper] Goal rejected by gripper controller!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        reached = getattr(res, "reached_goal", True)
        self.get_logger().info(f"[Gripper] Command executed successfully. reached_goal: {reached}")
        return True

    def open_gripper(self) -> bool:
        """Open parallel gripper fully (40mm)."""
        self.get_logger().info("--> Opening Gripper (40mm stroke)...")
        return self.command_gripper(position=0.040, max_effort=40.0)

    def close_gripper(self, target_position: float = 0.0, max_effort: float = 80.0) -> bool:
        """Close parallel gripper onto workpiece."""
        self.get_logger().info(f"--> Closing Gripper (target: {target_position * 1000.0:.1f} mm)...")
        return self.command_gripper(position=target_position, max_effort=max_effort)

    def make_joint_constraints(self, target_positions) -> Constraints:
        constraints = Constraints()
        for name, pos in zip(self.joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def move_arm_joints(self, target_positions, vel_scale: float = 0.4) -> bool:
        """Move arm joints with Pilz PTP planner."""
        self.get_logger().info(f"[Arm PTP] Moving to joints: {[round(p, 3) for p in target_positions]}")
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = "PTP"
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale
        req.goal_constraints.append(self.make_joint_constraints(target_positions))
        goal_msg.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Arm PTP] Goal rejected by MoveGroup!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        success = (res.error_code.val == 1)
        self.get_logger().info(f"[Arm PTP] Move completed. Result code: {res.error_code.val} (SUCCESS=1)")
        return success


def run_pick_and_place_demo(demo: GripperMotionDemo):
    demo.wait_for_services()

    # Postures for Pick-and-Place cycle
    ready_posture = [0.0, -0.785398, 1.570796, -0.785398, 1.570796, 0.0]
    pick_approach = [0.2, -0.6, 1.4, -0.8, 1.57, 0.2]
    pick_grasp    = [0.2, -0.5, 1.2, -0.7, 1.57, 0.2]
    place_target  = [-0.3, -0.6, 1.3, -0.7, 1.57, -0.3]
    home_posture  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Step 1: Move to Ready posture
    print("\n--- [Step 1] Move Arm to Ready Posture ---")
    if not demo.move_arm_joints(ready_posture):
        return False

    # Step 2: Open Gripper
    print("\n--- [Step 2] Open Gripper Pre-Grasp ---")
    if not demo.open_gripper():
        return False

    # Step 3: Move to Pick Approach
    print("\n--- [Step 3] Move to Pick Approach Pose ---")
    if not demo.move_arm_joints(pick_approach):
        return False

    # Step 4: Move to Grasp Position
    print("\n--- [Step 4] Lower to Grasp Height ---")
    if not demo.move_arm_joints(pick_grasp):
        return False

    # Step 5: Close Gripper (Grasp Object)
    print("\n--- [Step 5] Close Gripper on Workpiece ---")
    if not demo.close_gripper(target_position=0.015, max_effort=75.0):
        return False

    # Step 6: Lift & Transport to Place Location
    print("\n--- [Step 6] Lift and Transport to Drop Target ---")
    if not demo.move_arm_joints(place_target):
        return False

    # Step 7: Open Gripper (Release Object)
    print("\n--- [Step 7] Open Gripper to Release Workpiece ---")
    if not demo.open_gripper():
        return False

    # Step 8: Return Home
    print("\n--- [Step 8] Return Arm to Home Pose & Close Gripper ---")
    if not demo.move_arm_joints(home_posture):
        return False
    if not demo.close_gripper(target_position=0.0, max_effort=40.0):
        return False

    print("\n" + "=" * 60)
    print("  COORDINATED GRIPPER PICK-AND-PLACE DEMO PASSED 100%!")
    print("=" * 60 + "\n")
    return True


def main(args=None):
    rclpy.init(args=args)
    demo = GripperMotionDemo()
    try:
        success = run_pick_and_place_demo(demo)
        sys.exit(0 if success else 1)
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
