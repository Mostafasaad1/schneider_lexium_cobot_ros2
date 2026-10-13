import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
import time

rclpy.init()
node = Node("step_logger")
node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

gripper_client = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
if not gripper_client.wait_for_server(timeout_sec=2.0):
    gripper_client = ActionClient(node, GripperCommand, "/gripper_controller/gripper_cmd")
    gripper_client.wait_for_server(timeout_sec=2.0)

move_client = ActionClient(node, MoveGroup, "/move_action")
move_client.wait_for_server(timeout_sec=5.0)

current_joints = {}
def js_cb(msg):
    for n, p in zip(msg.name, msg.position):
        current_joints[n] = p
node.create_subscription(JointState, "/joint_states", js_cb, 10)

def spin_some(dur=0.2):
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)

joint_names = [
    "l_Base_Link002__to__l_Link_012",
    "l_Link_012__to__l_Link_013",
    "l_Link_013__to__l_Link_014",
    "l_Link_014__to__l_Link_015",
    "l_Link_015__to__l_Link_016",
    "l_Link_016__to__l_Link_017",
]

def move_arm(target, name):
    print(f"\n--- Starting Arm Move: {name} ---")
    goal = MoveGroup.Goal()
    goal.request.group_name = "lexium_arm"
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"
    goal.request.max_velocity_scaling_factor = 0.35
    goal.request.max_acceleration_scaling_factor = 0.35
    c = Constraints()
    for jn, jv in zip(joint_names, target):
        jc = JointConstraint()
        jc.joint_name = jn
        jc.position = float(jv)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    goal.request.goal_constraints.append(c)
    f = move_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f)
    h = f.result()
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    spin_some(0.3)
    g_left = current_joints.get("gripper_finger_left_joint", 0.0)
    print(f"--- Completed Arm Move: {name}. Gripper finger left pos: {g_left*1000:.2f} mm ---")

def cmd_gripper(pos, effort, name):
    print(f"\n--- Starting Gripper: {name} ({pos*1000:.1f} mm) ---")
    g = ParallelGripperCommand.Goal()
    g.command.name = ["gripper_finger_left_joint"]
    g.command.position = [float(pos)]
    g.command.effort = [float(effort)]
    f = gripper_client.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f)
    h = f.result()
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    res = rf.result().result
    spin_some(0.5)
    g_left = current_joints.get("gripper_finger_left_joint", 0.0)
    print(f"--- Completed Gripper: {name}. reached_goal: {res.reached_goal}. Finger pos: {g_left*1000:.2f} mm ---")

ready = [-1.5708, 0.4500, -1.0500, 0.6000, 1.5708, 0.0000]
pick_app = [-0.9042, 0.3632, -1.0971, 0.1105, 1.5708, -0.1013]
pick_grasp = [-0.9041, 0.3760, -1.3997, -0.2045, 1.5708, -0.1012]

spin_some(0.5)
cmd_gripper(0.040, 40.0, "Step 1: Open")
move_arm(ready, "Step 2: Ready")
move_arm(pick_app, "Step 3: Pick Approach")
move_arm(pick_grasp, "Step 4: Pick Grasp")
cmd_gripper(-0.008, 100.0, "Step 5: Close")
move_arm(pick_app, "Step 7: Lift")

node.destroy_node()
rclpy.shutdown()
