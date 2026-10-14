#!/usr/bin/env python3
"""Read the UR joint states once, then rotate wrist_3 by a small amount."""
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# --- Settings ---
DELTA = 0.2        # rad, change applied to wrist_3 (negative rotates the other way)
DURATION = 6.0     # s, time the move takes
MAX_DELTA = 0.5    # rad, safety limit for this demo
ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def main():
    if abs(DELTA) > MAX_DELTA:
        raise SystemExit(f"DELTA must be within +/-{MAX_DELTA} rad")

    rclpy.init()
    node = Node("wrist_nudge")

    # 1. Read joint states once.
    latest = []
    node.create_subscription(JointState, "/joint_states", latest.append,
                             qos_profile_sensor_data)
    while rclpy.ok() and not latest:
        rclpy.spin_once(node, timeout_sec=0.5)
    current = dict(zip(latest[0].name, latest[0].position))
    positions = [current[j] for j in JOINTS]
    print("current:", [round(p, 4) for p in positions])

    # 2. Same pose, but wrist_3 changed by DELTA.
    target = list(positions)
    target[-1] += DELTA
    print("target: ", [round(p, 4) for p in target])
    input("Slider low, area clear, hand near e-stop. Enter to move, Ctrl-C to cancel: ")

    # 3. Send the trajectory and wait for it to finish.
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=5.0):
        raise SystemExit("Action server not found. Is the driver running?")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = target
    point.time_from_start = Duration(sec=int(DURATION),
                                     nanosec=int(DURATION % 1 * 1e9))
    goal.trajectory.points = [point]

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        raise SystemExit("Goal rejected. Is the External Control program running on the pendant?")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    code = result_future.result().result.error_code
    print("done" if code == FollowJointTrajectory.Result.SUCCESSFUL else f"failed, error_code={code}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()