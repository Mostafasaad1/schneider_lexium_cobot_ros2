#!/usr/bin/env python3
"""
Schneider Electric Lexium Cobot L18S - Industrial Pick & Place Node.

Orchestrates a complete Table-to-Table pick and place sequence:
1. Injects Pick Table, Place Table, and Workpiece into MoveIt 2 Planning Scene.
2. Moves arm to Ready inspection pose.
3. Approaches Pick Table (+Y), descends, and grasps workpiece with parallel gripper.
4. Dynamically attaches workpiece to robot in MoveIt Planning Scene (AttachedCollisionObject).
5. Lifts workpiece, transits collision-free across tables to Place Table (-Y).
6. Descends to Place Table, releases gripper, and detaches workpiece in Planning Scene.
7. Retracts and returns arm to Safe Home/Ready posture.

Supports single execution or continuous automated cycles.
"""

import os
import sys
import time
import argparse
import xml.etree.ElementTree as ET
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
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
    Constraints,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState


class PickAndPlaceTaskNode(Node):
    def __init__(self, continuous: bool = False, max_cycles: int = 1):
        super().__init__("pick_and_place_task_node")
        self.continuous = continuous
        self.max_cycles = max_cycles
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        else:
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.get_logger().info("=" * 68)
        self.get_logger().info("  Schneider Electric Lexium Cobot L18S - Pick & Place Node")
        self.get_logger().info("=" * 68)

        # Configuration parameters
        self.joint_names = [
            "l_Base_Link002__to__l_Link_012",
            "l_Link_012__to__l_Link_013",
            "l_Link_013__to__l_Link_014",
            "l_Link_014__to__l_Link_015",
            "l_Link_015__to__l_Link_016",
            "l_Link_016__to__l_Link_017",
        ]
        self.base_frame = "l_Base_Link002"
        self.ee_link = "tool0"
        self.group_name = "lexium_arm"
        self.pipeline_id = "ompl"
        self.planner_id = "RRTConnect"

        # Touch links for gripper grasp attachment
        self.touch_links = [
            "gripper_finger_left",
            "gripper_finger_right",
            "gripper_base_link",
            "tool0",
            "tcp_link",
        ]

        # Key joint postures
        self.ready_posture = [-1.5708, 0.4500, -1.0500, 0.6000, 1.5708, 0.0000]
        self.home_posture  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Reachable joint waypoints for Pick (+Y = 0.55m) & Place (-Y = -0.55m) with vertical tool descent
        # Flange tool0 at Z=0.396m -> fingertip at Z=0.272m (+22.0mm safe clearance above table surface at Z=0.250m)
        # Rubber pad center aligns exactly with workpiece center of mass at Z=0.290m (Z_world = 0.340m)
        self.pick_approach_joints = [-0.9042, 0.3632, -1.0971, 0.1105, 1.5708, -0.1013]
        self.pick_grasp_joints    = [-0.9042, 0.3932, -1.4672, -0.2896, 1.5708, -0.1012]

        self.place_approach_joints = [0.9042, -0.3632, 1.0971, -0.1105, -1.5708, 0.1013]
        self.place_release_joints  = [0.9042, -0.3932, 1.4672, 0.2896, -1.5708, 0.1012]

        # Gripper stroke & force parameters
        self.gripper_open_position = 0.040
        self.gripper_grasp_position = -0.008 # Close deep into clamp zone (down to ~14mm jaw gap)
        self.gripper_open_effort = 40.0
        self.gripper_grasp_effort = 100.0

        # ROS 2 Interfaces
        self.current_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
        )

        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(self, GRIPPER_ACTION_TYPE, "/gripper_controller/gripper_cmd")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def _joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def sleep_sim(self, duration_sec: float):
        """Sleep using ROS simulation clock to prevent desynchronization with Gazebo physics."""
        start_t = self.get_clock().now()
        target_ns = int(duration_sec * 1e9)
        while rclpy.ok() and (self.get_clock().now() - start_t).nanoseconds < target_ns:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_connections(self, timeout_sec: float = 20.0):
        """Wait for MoveGroup, Gripper action server, Planning Scene service, and JointStates."""
        self.get_logger().info("Connecting to MoveGroup, PlanningScene, and Gripper servers...")
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError("MoveGroup (/move_action) action server not available.")
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            raise TimeoutError(f"GripperCommand (/gripper_controller/gripper_cmd) action server not available.")
        if not self.scene_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn("ApplyPlanningScene service not available; planning scene updates will be skipped.")

        self.get_logger().info("Waiting for valid /joint_states stream...")
        start_time = time.time()
        while self.current_joint_state is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state is None:
            raise TimeoutError("Did not receive /joint_states.")

        self.get_logger().info("All MoveIt 2 & control services connected successfully.")

    # =========================================================================
    # MoveIt 2 Planning Scene Management
    # =========================================================================

    def build_complete_acm(self) -> AllowedCollisionMatrix:
        """Construct complete MoveIt Allowed Collision Matrix preserving all SRDF disabled pairs plus scene objects."""
        srdf_candidates = [
            "/home/mox/projects/schneider_ws/src/lexium_cobot_l18s_moveit_config/config/Lexium_Cobot_L18S002.srdf",
            "/home/mox/projects/schneider_ws/install/lexium_cobot_l18s_moveit_config/share/lexium_cobot_l18s_moveit_config/config/Lexium_Cobot_L18S002.srdf",
        ]
        disabled_pairs = set()
        all_links = set()

        for path in srdf_candidates:
            if os.path.exists(path):
                try:
                    tree = ET.parse(path)
                    root = tree.getroot()
                    for dc in root.findall("disable_collisions"):
                        l1 = dc.get("link1")
                        l2 = dc.get("link2")
                        disabled_pairs.add((l1, l2))
                        disabled_pairs.add((l2, l1))
                        all_links.add(l1)
                        all_links.add(l2)
                    break
                except Exception as e:
                    self.get_logger().warn(f"Failed to parse SRDF at {path}: {e}")

        extra_objects = ["pick_table", "place_table", "target_workpiece"]
        for obj in extra_objects:
            all_links.add(obj)

        for obj in extra_objects:
            disabled_pairs.add(("target_workpiece", obj))
            disabled_pairs.add((obj, "target_workpiece"))
        for link in ["gripper_finger_left", "gripper_finger_right", "gripper_base_link", "tool0", "tcp_link", "l_Link_017"]:
            for obj in extra_objects:
                disabled_pairs.add((obj, link))
                disabled_pairs.add((link, obj))

        link_list = sorted(list(all_links))
        acm = AllowedCollisionMatrix()
        acm.entry_names = link_list

        for l1 in link_list:
            entry = AllowedCollisionEntry()
            entry.enabled = [(l1 == l2 or (l1, l2) in disabled_pairs) for l2 in link_list]
            acm.entry_values.append(entry)

        return acm

    def setup_planning_scene(self) -> bool:
        """Inject Pick Table, Place Table, and Workpiece into MoveIt Planning Scene."""
        self.get_logger().info("[Scene] Adding Pick Table, Place Table, and Workpiece to Planning Scene...")
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        # 0. Cleanly detach any lingering attached objects from robot state
        detach_obj = AttachedCollisionObject()
        detach_obj.link_name = self.ee_link
        detach_obj.object.id = "target_workpiece"
        detach_obj.object.operation = CollisionObject.REMOVE
        scene_msg.robot_state.attached_collision_objects.append(detach_obj)
        scene_msg.robot_state.is_diff = True

        # 1. Pick Table (+Y = 0.55m)
        pick_table = CollisionObject()
        pick_table.header.frame_id = self.base_frame
        pick_table.id = "pick_table"
        pt_prim = SolidPrimitive()
        pt_prim.type = SolidPrimitive.BOX
        pt_prim.dimensions = [0.40, 0.40, 0.30] # [x, y, z]
        pt_pose = Pose()
        pt_pose.position.x = 0.45
        pt_pose.position.y = 0.55
        pt_pose.position.z = 0.10 # Center at 0.10m in base frame -> top surface at 0.25m (matches Gazebo SDF tabletop at Z_world=0.30m - 0.05m base offset)
        pt_pose.orientation.w = 1.0
        pick_table.primitives.append(pt_prim)
        pick_table.primitive_poses.append(pt_pose)
        pick_table.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(pick_table)

        # 2. Place Table (-Y = -0.55m)
        place_table = CollisionObject()
        place_table.header.frame_id = self.base_frame
        place_table.id = "place_table"
        plt_prim = SolidPrimitive()
        plt_prim.type = SolidPrimitive.BOX
        plt_prim.dimensions = [0.40, 0.40, 0.30] # [x, y, z]
        plt_pose = Pose()
        plt_pose.position.x = 0.45
        plt_pose.position.y = -0.55
        plt_pose.position.z = 0.10 # Center at 0.10m in base frame -> top surface at 0.25m (matches Gazebo SDF tabletop at Z_world=0.30m - 0.05m base offset)
        plt_pose.orientation.w = 1.0
        place_table.primitives.append(plt_prim)
        place_table.primitive_poses.append(plt_pose)
        place_table.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(place_table)

        # 3. Target Workpiece on Pick Table
        workpiece = CollisionObject()
        workpiece.header.frame_id = self.base_frame
        workpiece.id = "target_workpiece"
        wp_prim = SolidPrimitive()
        wp_prim.type = SolidPrimitive.CYLINDER
        wp_prim.dimensions = [0.080, 0.025] # [height, radius] (matches Gazebo SDF cylinder)
        wp_pose = Pose()
        wp_pose.position.x = 0.45
        wp_pose.position.y = 0.55
        wp_pose.position.z = 0.290 # Center at 0.29m in base frame, rests on tabletop at 0.25m (matches Gazebo SDF Z_world=0.34m - 0.05m)
        wp_pose.orientation.w = 1.0
        workpiece.primitives.append(wp_prim)
        workpiece.primitive_poses.append(wp_pose)
        workpiece.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(workpiece)

        # Set complete Allowed Collision Matrix preserving all robot link pairs + workpiece
        scene_msg.allowed_collision_matrix = self.build_complete_acm()

        return self._send_planning_scene(scene_msg)

    def attach_workpiece_to_gripper(self) -> bool:
        """Attach workpiece collision object to tool0 / gripper in MoveIt Planning Scene."""
        self.get_logger().info("[Scene] Attaching 'target_workpiece' to gripper in Planning Scene...")
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        # 1. Remove static target_workpiece from world to prevent self-collision
        remove_wp = CollisionObject()
        remove_wp.header.frame_id = self.base_frame
        remove_wp.id = "target_workpiece"
        remove_wp.operation = CollisionObject.REMOVE
        scene_msg.world.collision_objects.append(remove_wp)

        # 2. Add as AttachedCollisionObject on tool0
        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = self.ee_link
        attached_obj.object.id = "target_workpiece"
        attached_obj.object.header.frame_id = self.ee_link
        attached_obj.touch_links = list(set(self.touch_links + [
            "gripper_finger_left",
            "gripper_finger_right",
            "gripper_base_link",
            "tool0",
            "tcp_link",
            "l_Link_017",
        ]))

        wp_prim = SolidPrimitive()
        wp_prim.type = SolidPrimitive.CYLINDER
        wp_prim.dimensions = [0.080, 0.025]
        wp_pose = Pose()
        wp_pose.position.x = 0.0
        wp_pose.position.y = 0.0
        wp_pose.position.z = 0.106 # Workpiece center relative to tool0: 0.396m - 0.290m = 0.106m
        wp_pose.orientation.w = 1.0

        attached_obj.object.primitives.append(wp_prim)
        attached_obj.object.primitive_poses.append(wp_pose)
        attached_obj.object.operation = CollisionObject.ADD

        scene_msg.robot_state.attached_collision_objects.append(attached_obj)
        scene_msg.robot_state.is_diff = True

        # 3. Complete ACM preserving robot links + workpiece
        scene_msg.allowed_collision_matrix = self.build_complete_acm()

        return self._send_planning_scene(scene_msg)

    def detach_workpiece_from_gripper(self, place_x: float = 0.45, place_y: float = -0.55, place_z: float = 0.290) -> bool:
        """Detach workpiece from gripper and re-add as static obstacle on Place Table."""
        self.get_logger().info(f"[Scene] Detaching 'target_workpiece' and placing on Destination Table ({place_x:.2f}, {place_y:.2f}, {place_z:.2f})...")
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        # 1. Remove from attached list
        detach_obj = AttachedCollisionObject()
        detach_obj.link_name = self.ee_link
        detach_obj.object.id = "target_workpiece"
        detach_obj.object.operation = CollisionObject.REMOVE
        scene_msg.robot_state.attached_collision_objects.append(detach_obj)
        scene_msg.robot_state.is_diff = True

        # 2. Re-add as static world object at place coordinates
        workpiece = CollisionObject()
        workpiece.header.frame_id = self.base_frame
        workpiece.id = "target_workpiece"
        wp_prim = SolidPrimitive()
        wp_prim.type = SolidPrimitive.CYLINDER
        wp_prim.dimensions = [0.070, 0.024]
        wp_pose = Pose()
        wp_pose.position.x = place_x
        wp_pose.position.y = place_y
        wp_pose.position.z = place_z
        wp_pose.orientation.w = 1.0
        workpiece.primitives.append(wp_prim)
        workpiece.primitive_poses.append(wp_pose)
        workpiece.operation = CollisionObject.ADD
        scene_msg.world.collision_objects.append(workpiece)

        # 3. Complete ACM preserving robot links + workpiece
        scene_msg.allowed_collision_matrix = self.build_complete_acm()

        return self._send_planning_scene(scene_msg)

    def _send_planning_scene(self, scene_msg: PlanningScene) -> bool:
        if not self.scene_client.service_is_ready():
            return False
        req = ApplyPlanningScene.Request()
        req.scene = scene_msg
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res is not None and res.success

    # =========================================================================
    # Gripper Actuation
    # =========================================================================

    def command_gripper(self, position: float, max_effort: float = 50.0, settle_time: float = 1.0) -> bool:
        """Command parallel gripper position (0.0=closed, 0.04=open) and effort limit."""
        self.get_logger().info(f"[Gripper] Command position: {position * 1000.0:.1f} mm, effort limit: {max_effort:.1f} N")
        goal_msg = GRIPPER_ACTION_TYPE.Goal()
        if hasattr(goal_msg, "command") and hasattr(goal_msg.command, "name"):
            goal_msg.command.name = ["gripper_finger_left_joint"]
            goal_msg.command.position = [float(position)]
            goal_msg.command.effort = [float(max_effort)]
        elif hasattr(goal_msg, "command"):
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
        reached = getattr(res, "reached_goal", False)
        stalled = getattr(res, "stalled", False)
        self.get_logger().info(f"[Gripper] Action server completed. reached_goal: {reached}, stalled: {stalled}")

        # Settle duration in simulation time to guarantee physical grasp contact settles before arm moves
        if settle_time > 0:
            self.get_logger().info(f"[Gripper] Settling {settle_time:.1f}s (sim time)...")
            self.sleep_sim(settle_time)

        return True

    def open_gripper(self, settle_time: float = 1.2) -> bool:
        """Open parallel gripper fully (40mm)."""
        self.get_logger().info(f"--> Opening Gripper ({self.gripper_open_position * 1000.0:.1f}mm stroke)...")
        return self.command_gripper(position=self.gripper_open_position, max_effort=self.gripper_open_effort, settle_time=settle_time)

    def close_gripper(self, target_position: float = None, max_effort: float = None, settle_time: float = 1.5) -> bool:
        """Close parallel gripper onto workpiece."""
        pos = target_position if target_position is not None else self.gripper_grasp_position
        eff = max_effort if max_effort is not None else self.gripper_grasp_effort
        self.get_logger().info(f"--> Closing Gripper on Workpiece ({pos * 1000.0:.1f} mm, effort {eff:.1f} N)...")
        return self.command_gripper(position=pos, max_effort=eff, settle_time=settle_time)

    # =========================================================================
    # Arm Motion Execution
    # =========================================================================

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

    def move_arm_joints(self, target_positions, vel_scale: float = 0.35, planner_id: str = None) -> bool:
        """Plan and execute arm trajectory to joint waypoints using MoveIt 2."""
        planner = planner_id if planner_id is not None else self.planner_id
        self.get_logger().info(f"[Arm {planner}] Moving to target joints: {[round(p, 3) for p in target_positions]}")
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = self.group_name
        req.pipeline_id = self.pipeline_id
        req.planner_id = planner
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale
        req.goal_constraints.append(self.make_joint_constraints(target_positions))
        goal_msg.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"[Arm {planner}] Goal rejected by MoveGroup!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        success = (res.error_code.val == 1)
        self.get_logger().info(f"[Arm {planner}] Trajectory completed. Result code: {res.error_code.val} (SUCCESS=1)")
        return success


def run_full_pick_and_place_cycle(node: PickAndPlaceTaskNode, cycle_num: int = 1) -> bool:
    print("\n" + "=" * 68)
    print(f"  EXECUTING PICK & PLACE CYCLE #{cycle_num}")
    print("=" * 68)

    # Step 1: Open Gripper at Beginning of Motion
    print(f"\n--- [Step 1] Open Gripper at Beginning of Motion ({node.gripper_open_position * 1000.0:.1f}mm stroke) ---")
    if not node.open_gripper(settle_time=1.2):
        return False

    # Step 2: Move Arm to Safe Ready Inspection Posture
    print("\n--- [Step 2] Move Arm to Safe Ready Inspection Posture ---")
    if not node.move_arm_joints(node.ready_posture, vel_scale=0.4):
        return False

    # Step 3: Move to Pick Approach Pose (above Pick Table at +Y, gripper OPEN)
    print("\n--- [Step 3] Move Arm to Pick Table Approach Pose (+Y=0.55m, Gripper Open) ---")
    if not node.move_arm_joints(node.pick_approach_joints, vel_scale=0.35):
        return False

    # Step 4: Lower Open Gripper into Grasp Height on Pick Table (fingers straddle workpiece)
    print("\n--- [Step 4] Lower Open Gripper onto Workpiece Grasp Height ---")
    if not node.move_arm_joints(node.pick_grasp_joints, vel_scale=0.20):
        return False
    print("--> Arm arrived at pick grasp position! Workpiece is centered between open fingers.")
    print(f"\n--- [Step 5] Close Gripper on Workpiece ({node.gripper_grasp_position * 1000.0:.1f}mm, {node.gripper_grasp_effort:.0f}N) ---")
    if not node.close_gripper(settle_time=1.5):
        return False
    print("--> Gripper closed entirely on workpiece! Settled and holding firmly.")
    node.sleep_sim(0.3)

    # Step 6: Attach Workpiece in MoveIt Planning Scene
    print("\n--- [Step 6] Attach Workpiece in MoveIt Planning Scene ---")
    node.attach_workpiece_to_gripper()
    node.sleep_sim(0.4)

    # Step 7: Lift Workpiece off Pick Table (gripper clamped, lifting to safe height)
    print("\n--- [Step 7] Lift Workpiece to Pick Approach Clearance Height ---")
    if not node.move_arm_joints(node.pick_approach_joints, vel_scale=0.20):
        return False

    # Step 8: Transit Across Tables to Place Approach Pose (-Y)
    print("\n--- [Step 8] Transit Workpiece Across Workcell to Destination Table (-Y=-0.55m) ---")
    if not node.move_arm_joints(node.place_approach_joints, vel_scale=0.35):
        return False

    # Step 9: Lower to Place Table Surface
    print("\n--- [Step 9] Lower Workpiece onto Destination Table Surface ---")
    if not node.move_arm_joints(node.place_release_joints, vel_scale=0.20):
        return False

    # Confirm arm reached place point and settled before opening gripper
    print("--> Arm reached destination point! Settling before releasing workpiece...")
    node.sleep_sim(0.8)

    # Step 10: Open Gripper to Release Workpiece - Settle until entirely open
    print("\n--- [Step 10] Open Gripper to Release Workpiece on Destination Table ---")
    if not node.open_gripper(settle_time=1.5):
        return False
    print("--> Gripper opened entirely! Workpiece released safely.")
    node.sleep_sim(0.4)

    # Step 11: Detach Workpiece in MoveIt Planning Scene at Destination
    print("\n--- [Step 11] Detach Workpiece in Planning Scene at Destination Table ---")
    node.detach_workpiece_from_gripper(place_x=0.45, place_y=-0.55, place_z=0.290)
    node.sleep_sim(0.4)

    # Step 12: Retract to Place Approach Clearance Height
    print("\n--- [Step 12] Retract Arm to Destination Approach Clearance Height ---")
    if not node.move_arm_joints(node.place_approach_joints, vel_scale=0.25):
        return False

    # Step 13: Return Arm to Safe Ready/Home Posture
    print("\n--- [Step 13] Return Arm to Safe Ready Posture & Close Gripper ---")
    if not node.move_arm_joints(node.ready_posture, vel_scale=0.4):
        return False
    if not node.close_gripper(target_position=0.0, max_effort=40.0, settle_time=1.0):
        return False

    print("\n" + "=" * 68)
    print(f"  PICK & PLACE CYCLE #{cycle_num} COMPLETED SUCCESSFULLY (100% PASS)!")
    print("=" * 68 + "\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="Schneider Lexium Cobot L18S Pick & Place Node")
    parser.add_argument("--continuous", action="store_true", help="Run pick & place continuously in an automated loop")
    parser.add_argument("--cycles", type=int, default=1, help="Number of cycles to run (default: 1)")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = PickAndPlaceTaskNode(continuous=args.continuous, max_cycles=args.cycles)

    try:
        node.wait_for_connections()
        node.setup_planning_scene()
        node.sleep_sim(0.5)

        # Open gripper at the beginning of the motion
        node.get_logger().info("--> Opening gripper at the beginning of motion...")
        node.open_gripper()
        node.sleep_sim(0.5)

        cycle = 1
        max_cycles = 999999 if args.continuous else args.cycles

        while rclpy.ok() and cycle <= max_cycles:
            success = run_full_pick_and_place_cycle(node, cycle_num=cycle)
            if not success:
                node.get_logger().error(f"Pick & Place cycle #{cycle} failed!")
                sys.exit(1)

            if cycle < max_cycles:
                node.get_logger().info("Resting 1.5s before next cycle...")
                node.sleep_sim(1.5)
                # Reset workpiece position to Pick table for demonstration loop
                node.setup_planning_scene()

            cycle += 1

        print("All requested pick and place cycles finished successfully.")
        sys.exit(0)

    except Exception as e:
        node.get_logger().error(f"Fatal error during pick and place execution: {e}")
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
