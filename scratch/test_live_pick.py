import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
import time
import subprocess

rclpy.init()
node = Node("grasp_tester")
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

def sleep_sim(dur):
    start = node.get_clock().now()
    ns = int(dur * 1e9)
    while (node.get_clock().now() - start).nanoseconds < ns:
        rclpy.spin_once(node, timeout_sec=0.02)

joint_names = [
    "l_Base_Link002__to__l_Link_012",
    "l_Link_012__to__l_Link_013",
    "l_Link_013__to__l_Link_014",
    "l_Link_014__to__l_Link_015",
    "l_Link_015__to__l_Link_016",
    "l_Link_016__to__l_Link_017",
]

def move_arm(target, name):
    print(f"\n[Arm] Moving to {name}: {target}")
    goal = MoveGroup.Goal()
    goal.request.group_name = "lexium_arm"
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"
    goal.request.max_velocity_scaling_factor = 0.25
    goal.request.max_acceleration_scaling_factor = 0.25
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
    print(f"[Arm] {name} result: {rf.result().result.error_code.val}")

def cmd_gripper(pos, effort, name):
    print(f"\n[Gripper] {name} target: {pos*1000:.1f} mm, effort: {effort} N")
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
    print(f"[Gripper] {name} completed, reached_goal: {res.reached_goal}")
    sleep_sim(1.0)
    print(f"[Gripper] Current finger pos: {current_joints.get('gripper_finger_left_joint', 0.0)*1000:.2f} mm")

# 1. Open gripper
cmd_gripper(0.040, 40.0, "Open")

# 2. Pick approach
pick_app = [-0.9042, 0.3632, -1.0971, 0.1105, 1.5708, -0.1013]
move_arm(pick_app, "Pick Approach")

# 3. Pick grasp (new calculated height: tool0 at Z=0.396m)
pick_grasp = [-0.9042, 0.3932, -1.4672, -0.2896, 1.5708, -0.1012]
move_arm(pick_grasp, "Pick Grasp")

sleep_sim(0.5)

# Inspect Gazebo poses at grasp
print("\n--- Gazebo Link Poses at Grasp ---")
res = subprocess.run(["gz", "model", "-m", "Lexium_Cobot_L18S002"], capture_output=True, text=True)
for line in res.stdout.splitlines():
    if "gripper_finger_left" in line or "l_Link_017" in line or "Pose [ XYZ" in line:
        print(line)

res_wp = subprocess.run(["gz", "model", "-m", "target_workpiece"], capture_output=True, text=True)
for line in res_wp.stdout.splitlines():
    if "Pose [ XYZ" in line:
        print("Workpiece:", line)

# 4. Close gripper
cmd_gripper(-0.008, 100.0, "Close on Workpiece")

# 5. Lift arm
move_arm(pick_app, "Lift Workpiece")
sleep_sim(1.0)

# Check workpiece pose after lift!
print("\n--- Workpiece Pose After Lift ---")
res_wp = subprocess.run(["gz", "model", "-m", "target_workpiece"], capture_output=True, text=True)
for line in res_wp.stdout.splitlines():
    if "Pose [ XYZ" in line:
        print("Workpiece:", line)

node.destroy_node()
rclpy.shutdown()
