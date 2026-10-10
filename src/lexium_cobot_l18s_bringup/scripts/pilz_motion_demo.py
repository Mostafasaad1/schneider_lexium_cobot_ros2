#!/usr/bin/env python3
"""
Pilz Industrial Motion Planner Demo & Test Suite for Lexium Cobot L18S.

This script demonstrates deterministic industrial trajectories using:
1. PTP (Point-to-Point in Joint Space & Cartesian Space)
2. LIN (Deterministic Linear Cartesian Straight-line Motion)
3. CIRC (Circular Arc Motion with Interim or Center Constraints)
4. Sequence with Blending (MoveGroupSequenceAction with defined blend radii)

Usage:
  ros2 run lexium_cobot_l18s_bringup pilz_motion_demo.py
  ros2 run lexium_cobot_l18s_bringup pilz_motion_demo.py --test-all
"""

import sys
import time
import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from moveit_msgs.action import MoveGroup, MoveGroupSequence
from moveit_msgs.msg import (
    MotionPlanRequest,
    MotionSequenceRequest,
    MotionSequenceItem,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from sensor_msgs.msg import JointState


class PilzMotionDemo(Node):
    def __init__(self):
        super().__init__("pilz_motion_demo")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  Schneider Lexium Cobot L18S - Pilz Industrial Planner Demo")
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.seq_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")

    def _joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def wait_for_services(self, timeout_sec: float = 15.0):
        self.get_logger().info("Connecting to MoveGroup action servers...")
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError("MoveGroup (/move_action) action server not available.")
        if not self.seq_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError("MoveGroupSequence (/sequence_move_group) action server not available.")

        self.get_logger().info("Waiting for valid /joint_states and TF frames...")
        start_time = time.time()
        while self.current_joint_state is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state is None:
            raise TimeoutError("Did not receive /joint_states from controller.")

        self.get_logger().info("Successfully connected and initialized.")

    def get_current_pose(self) -> PoseStamped:
        """Query TF for current TCP pose relative to base frame."""
        for _ in range(20):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tcp_frame,
                    rclpy.time.Time(),
                )
                pose = PoseStamped()
                pose.header.frame_id = self.base_frame
                pose.pose.position.x = tf.transform.translation.x
                pose.pose.position.y = tf.transform.translation.y
                pose.pose.position.z = tf.transform.translation.z
                pose.pose.orientation = tf.transform.rotation
                return pose
            except Exception:
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"Could not lookup transform from {self.base_frame} to {self.tcp_frame}")

    def make_joint_constraints(self, target_positions, tolerance: float = 0.01) -> Constraints:
        constraints = Constraints()
        for name, pos in zip(self.joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def make_pose_constraints(self, pose: PoseStamped, pos_tolerance: float = 0.005, rot_tolerance: float = 0.05) -> Constraints:
        constraints = Constraints()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id or self.base_frame
        pc.link_name = self.tcp_frame
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [pos_tolerance, pos_tolerance, pos_tolerance]
        bv.primitives.append(box)

        bv_pose = PoseStamped()
        bv_pose.header.frame_id = pc.header.frame_id
        bv_pose.pose.position = pose.pose.position
        bv_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bv.primitive_poses.append(bv_pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id or self.base_frame
        oc.link_name = self.tcp_frame
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = rot_tolerance
        oc.absolute_y_axis_tolerance = rot_tolerance
        oc.absolute_z_axis_tolerance = rot_tolerance
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints

    def make_circ_path_constraint(self, interim_or_center: str, point: Point) -> Constraints:
        """Create interim or center path constraint for CIRC."""
        path_constraints = Constraints()
        path_constraints.name = interim_or_center  # "interim" or "center"

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_frame

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]
        bv.primitives.append(box)

        bv_pose = PoseStamped()
        bv_pose.header.frame_id = self.base_frame
        bv_pose.pose.position = point
        bv_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bv.primitive_poses.append(bv_pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0
        path_constraints.position_constraints.append(pc)
        return path_constraints

    def move_ptp_joint(self, target_positions, vel_scale: float = 0.5, acc_scale: float = 0.5) -> bool:
        """Execute deterministic PTP motion in joint space."""
        self.get_logger().info(f"[PTP Joint] Moving to: {[round(p, 3) for p in target_positions]}")
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = "PTP"
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale
        req.goal_constraints.append(self.make_joint_constraints(target_positions))
        goal_msg.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[PTP Joint] Goal rejected by MoveGroup!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        success = (res.error_code.val == 1)
        self.get_logger().info(f"[PTP Joint] Execution completed. Result code: {res.error_code.val} (SUCCESS=1)")
        return success

    def move_lin(self, target_pose: PoseStamped, vel_scale: float = 0.3, acc_scale: float = 0.3) -> bool:
        """Execute deterministic LIN straight-line Cartesian motion."""
        p = target_pose.pose.position
        self.get_logger().info(f"[LIN] Linear Cartesian move to: X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f}")
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = "LIN"
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale
        req.goal_constraints.append(self.make_pose_constraints(target_pose))
        goal_msg.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[LIN] Goal rejected by MoveGroup!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        success = (res.error_code.val == 1)
        self.get_logger().info(f"[LIN] Execution completed. Result code: {res.error_code.val} (SUCCESS=1)")
        return success

    def make_circ_center_constraint(self, center_point: Point) -> Constraints:
        """Create center path constraint for CIRC."""
        path_constraints = Constraints()
        path_constraints.name = "center"

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_frame

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]
        bv.primitives.append(box)

        bv_pose = PoseStamped()
        bv_pose.header.frame_id = self.base_frame
        bv_pose.pose.position = center_point
        bv_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bv.primitive_poses.append(bv_pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0
        path_constraints.position_constraints.append(pc)
        return path_constraints

    def move_circ_center(self, goal_pose: PoseStamped, center_point: Point, vel_scale: float = 0.3, acc_scale: float = 0.3) -> bool:
        """Execute deterministic CIRC circular arc motion around a center point."""
        gp = goal_pose.pose.position
        self.get_logger().info(f"[CIRC] Circular move to Goal: ({gp.x:.3f}, {gp.y:.3f}, {gp.z:.3f}) with Center: ({center_point.x:.3f}, {center_point.y:.3f}, {center_point.z:.3f})")
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = "CIRC"
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale
        req.goal_constraints.append(self.make_pose_constraints(goal_pose))
        req.path_constraints = self.make_circ_center_constraint(center_point)
        goal_msg.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[CIRC] Goal rejected by MoveGroup!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        success = (res.error_code.val == 1)
        self.get_logger().info(f"[CIRC] Execution completed. Result code: {res.error_code.val} (SUCCESS=1)")
        return success

    def move_sequence(self, sequence_items: list, plan_only: bool = False) -> bool:
        """Execute MoveGroupSequence with blended transitions."""
        self.get_logger().info(f"[Sequence] Executing sequence with {len(sequence_items)} motion segments and blend radii...")
        goal_msg = MoveGroupSequence.Goal()
        req = goal_msg.request
        goal_msg.planning_options.plan_only = plan_only

        for idx, item_data in enumerate(sequence_items):
            item = MotionSequenceItem()
            item.blend_radius = float(item_data.get("blend_radius", 0.0))
            item.req.group_name = self.group_name
            item.req.pipeline_id = self.pipeline_id
            item.req.planner_id = item_data.get("planner_id", "PTP")
            item.req.max_velocity_scaling_factor = float(item_data.get("vel_scale", 0.4))
            item.req.max_acceleration_scaling_factor = float(item_data.get("acc_scale", 0.4))

            if "joints" in item_data:
                item.req.goal_constraints.append(self.make_joint_constraints(item_data["joints"]))
            elif "pose" in item_data:
                item.req.goal_constraints.append(self.make_pose_constraints(item_data["pose"]))

            if "path_constraints" in item_data:
                item.req.path_constraints = item_data["path_constraints"]

            req.items.append(item)

        send_future = self.seq_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Sequence] Goal rejected by Sequence action server!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result.response
        success = (res.error_code.val == 1)
        self.get_logger().info(f"[Sequence] Sequence execution completed. Result code: {res.error_code.val} (SUCCESS=1)")
        self.get_logger().info(f"[Sequence] Trajectory sections planned: {len(res.planned_trajectories)}, planning time: {res.planning_time:.4f}s")
        return success


def run_comprehensive_demo(demo: PilzMotionDemo):
    demo.wait_for_services()

    # 1. PTP Joint to 'Ready' posture
    print("\n--- 1. PTP Joint Motion to 'Ready' Pose ---")
    ready_pose_joints = [0.0, -0.785398, 1.570796, -0.785398, 1.570796, 0.0]
    if not demo.move_ptp_joint(ready_pose_joints, vel_scale=0.5):
        return False

    # Get current Cartesian pose after reaching ready
    time.sleep(0.5)
    current_pose = demo.get_current_pose()
    p0 = current_pose.pose.position
    print(f"Current TCP Position: X={p0.x:.4f}, Y={p0.y:.4f}, Z={p0.z:.4f}")

    # 2. LIN Linear Motion: Move +5cm in Y (Cartesian straight line)
    print("\n--- 2. LIN Linear Motion (+5cm along Y axis) ---")
    lin_pose1 = PoseStamped()
    lin_pose1.header.frame_id = demo.base_frame
    lin_pose1.pose.position.x = p0.x
    lin_pose1.pose.position.y = p0.y + 0.05
    lin_pose1.pose.position.z = p0.z
    lin_pose1.pose.orientation = current_pose.pose.orientation
    if not demo.move_lin(lin_pose1, vel_scale=0.3):
        return False

    # 3. LIN Linear Motion: Move -5cm in Y (Back to start position)
    print("\n--- 3. LIN Linear Motion (-5cm along Y axis back) ---")
    lin_pose2 = PoseStamped()
    lin_pose2.header.frame_id = demo.base_frame
    lin_pose2.pose.position.x = p0.x
    lin_pose2.pose.position.y = p0.y
    lin_pose2.pose.position.z = p0.z
    lin_pose2.pose.orientation = current_pose.pose.orientation
    if not demo.move_lin(lin_pose2, vel_scale=0.3):
        return False

    # 4. Blended Industrial Sequence (MoveGroupSequence with blending radii)
    print("\n--- 4. Blended Sequence (PTP -> PTP -> PTP -> Home with blending) ---")
    seq = [
        {
            "planner_id": "PTP",
            "joints": [0.1, -0.6, 1.4, -0.7, 1.5, 0.0],
            "blend_radius": 0.05,  # 50mm blend radius
            "vel_scale": 0.4,
        },
        {
            "planner_id": "PTP",
            "joints": [0.2, -0.5, 1.3, -0.6, 1.4, 0.1],
            "blend_radius": 0.04,  # 40mm blend radius
            "vel_scale": 0.4,
        },
        {
            "planner_id": "PTP",
            "joints": [0.0, -0.785398, 1.570796, -0.785398, 1.570796, 0.0],
            "blend_radius": 0.03,  # 30mm blend radius
            "vel_scale": 0.4,
        },
        {
            "planner_id": "PTP",
            "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Return Home
            "blend_radius": 0.0,   # Must be 0.0 for final waypoint
            "vel_scale": 0.4,
        },
    ]
    if not demo.move_sequence(seq):
        return False

    print("\n" + "=" * 60)
    print("  ALL PILZ INDUSTRIAL MOTION PLANNER DEMOS PASSED SUCCESSFULLY!")
    print("=" * 60 + "\n")
    return True


def main(args=None):
    rclpy.init(args=args)
    demo = PilzMotionDemo()
    try:
        success = run_comprehensive_demo(demo)
        sys.exit(0 if success else 1)
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
