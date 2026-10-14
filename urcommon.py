"""Shared helpers for the impact-testing scripts.

Arm motion is done the same way as wrist_nudge.py: read /joint_states, send a
FollowJointTrajectory goal. Cartesian moves use the ikfast solver from
the local myur/ folder. Positions are in meters and joint angles in radians.

The hammer/probe tool is still run by the Robot Control Panel web page; the
Channel and Tool classes talk to it over the same websocket channel the
Jupyter code used.
"""
import asyncio
import json
import os
import queue
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from myur.ik_solver.ur_kinematics import URKinematics

HERE = os.path.dirname(os.path.abspath(__file__))

# --- Settings ---
ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
SPEED = 0.02            # m/s, tool speed used to pick move durations
MIN_MOVE_TIME = 0.5     # s, shortest move duration
MAX_JOG = 0.04          # m, largest single jog (same 4 cm limit as the panel)
MAX_JOINT_STEP = 1.0    # rad, refuse moves where any joint turns more (catches IK flips)
LIFT = 0.010            # m, clearance above the plate when moving between points
HAMMER_OFFSET = (-0.015, 0.012)  # m, x/y shift from probe to hammer
HIT_Z_OFFSET = 0.0005   # m, hammer height above the probed surface
GRIPPER_OPEN = 58       # %, sized for the impact-testing tool
GRIPPER_CLOSE = 60      # %
CHANNEL_URI = "wss://chrisrogers.pyscriptapps.com/talking-on-a-channel/api/channels/hackathon"
TOOL_REQUEST = "/modal/tool/request"
STATE_FILE = os.path.join(HERE, "state.json")


# ---------------------------------------------------------------- state file

def load_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def save_state(state):
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, indent=2)
    os.replace(tmp, STATE_FILE)


def fmt_mm(xyz):
    return "[" + ", ".join(f"{v * 1000:8.2f}" for v in xyz) + "] mm"


# ---------------------------------------------------------------------- arm

class Arm:
    """Minimal UR3e client: joint reads, FK/IK, blocking moves, gripper."""

    def __init__(self, name="impact_testing"):
        rclpy.init()
        self.node = Node(name)
        self.ik = URKinematics("ur3e")
        self._latest = None
        self.node.create_subscription(JointState, "/joint_states", self._on_joints,
                                      qos_profile_sensor_data)
        self._client = ActionClient(self.node, FollowJointTrajectory, ACTION)
        self._gripper_pub = self.node.create_publisher(Int32MultiArray, "/gripper/control", 10)
        self._gripper_state = None
        self.node.create_subscription(Int32MultiArray, "/gripper/state", self._on_gripper, 10)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _on_joints(self, msg):
        self._latest = msg

    def _on_gripper(self, msg):
        self._gripper_state = list(msg.data)

    # --- reading

    def joints(self):
        """Fresh joint positions in JOINTS order (radians)."""
        self._latest = None
        start = time.monotonic()
        while self._latest is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start > 5.0:
                raise SystemExit("No /joint_states received. Is the driver running?")
        current = dict(zip(self._latest.name, self._latest.position))
        return [current[j] for j in JOINTS]

    def pose(self, joints=None):
        """Tool pose as a 3x4 [rotation | translation] matrix."""
        if joints is None:
            joints = self.joints()
        return np.asarray(self.ik.forward(joints, rotation_type="matrix"))

    def xyz(self):
        return self.pose()[:, 3].tolist()

    # --- moving

    def move_joints(self, target, duration):
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise SystemExit("Action server not found. Is the driver running?")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in target]
        point.time_from_start = Duration(sec=int(duration),
                                         nanosec=int(duration % 1 * 1e9))
        goal.trajectory.points = [point]

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if not handle.accepted:
            raise SystemExit("Goal rejected. Is the External Control program running on the pendant?")

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self.node, result_future)
        except KeyboardInterrupt:
            # Stop the arm instead of letting the trajectory finish.
            print("\nCancelling move...")
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=2.0)
            raise
        code = result_future.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise SystemExit(f"Move failed, error_code={code}")

    def move_to(self, xyz, rot=None, duration=None):
        """Move the tool in a joint-space move to xyz, keeping rot (3x3) or the current rotation."""
        q_now = self.joints()
        pose_now = self.pose(q_now)
        target = pose_now.copy()
        if rot is not None:
            target[:, :3] = np.asarray(rot)
        target[:, 3] = xyz

        q = self.ik.inverse(target.reshape(-1).tolist(), q_guess=q_now)
        if q is None:
            raise SystemExit(f"No IK solution for {fmt_mm(xyz)}")
        step = max(abs(a - b) for a, b in zip(q, q_now))
        if step > MAX_JOINT_STEP:
            raise SystemExit(f"Refusing move: a joint would turn {step:.2f} rad "
                             f"(limit {MAX_JOINT_STEP}). Check the target.")
        if duration is None:
            dist = float(np.linalg.norm(np.asarray(xyz) - pose_now[:, 3]))
            duration = max(MIN_MOVE_TIME, dist / SPEED)
        self.move_joints(q, duration)

    def move_by(self, dx=0.0, dy=0.0, dz=0.0, duration=None):
        x, y, z = self.xyz()
        self.move_to([x + dx, y + dy, z + dz], duration=duration)

    def travel(self, xyz, rot=None):
        """Lift, move over xyz, then lower onto it, so the tool never drags across the plate."""
        x, y, z = self.xyz()
        safe_z = max(z, xyz[2]) + LIFT
        self.move_to([x, y, safe_z], rot=rot)
        self.move_to([xyz[0], xyz[1], safe_z], rot=rot)
        self.move_to(xyz, rot=rot)

    # --- gripper (needs Gripper_Node.py running)

    def gripper(self, percent, speed=50, force=50, timeout=5.0):
        start = time.monotonic()
        while self._gripper_pub.get_subscription_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start > 2.0:
                raise SystemExit("Nothing is listening on /gripper/control. Start Gripper_Node.py first.")
        msg = Int32MultiArray()
        msg.data = [int(255 * v / 100) for v in (percent, speed, force)]
        self._gripper_pub.publish(msg)

        # /gripper/state = [pos, speed, force, target, PRE, OBJ]; done when PRE matches
        # the requested target and OBJ is non-zero (stopped moving).
        time.sleep(0.1)
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            s = self._gripper_state
            if s and len(s) >= 6 and s[4] == s[3] and s[5] != 0:
                return
        print("Gripper command sent (no confirmation from /gripper/state).")


# ------------------------------------------------------------------ channel

class Channel:
    """Websocket connection to the channel the Robot Control Panel uses.

    Runs in a background thread; incoming (topic, value) pairs land in a queue.
    """

    def __init__(self, uri=CHANNEL_URI, timeout=10.0):
        import websockets  # pip install --user websockets
        self._websockets = websockets
        self.uri = uri
        self.inbox = queue.Queue()
        self._ws = None
        self._ready = threading.Event()
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_until_complete, args=(self._run(),),
                         daemon=True).start()
        if not self._ready.wait(timeout):
            raise SystemExit(f"Could not connect to channel {uri}")

    async def _run(self):
        while True:
            try:
                async with self._websockets.connect(self.uri) as ws:
                    self._ws = ws
                    self._ready.set()
                    async for raw in ws:
                        self._handle(raw)
            except Exception as e:
                print(f"[channel] {e}; reconnecting")
            self._ws = None
            await asyncio.sleep(2)

    def _handle(self, raw):
        try:
            msg = json.loads(raw)
            if msg.get("type") == "data" and msg.get("payload"):
                payload = json.loads(msg["payload"])
                self.inbox.put((payload.get("topic", ""), payload.get("value")))
        except (ValueError, TypeError, AttributeError):
            pass

    def send(self, topic, value):
        if self._ws is None:
            raise SystemExit("Channel is not connected.")
        data = json.dumps({"topic": topic, "value": value})
        asyncio.run_coroutine_threadsafe(self._ws.send(data), self._loop).result(5)

    def get(self, timeout=0.1):
        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty:
            return None

    def clear(self):
        while self.get(timeout=0) is not None:
            pass


class Tool:
    """Hammer and probe, run through the Robot Control Panel page.

    The panel replies on /modal/URarm/tool/response (BLE mode) or
    /modal/tool/request/response (serial mode) with
    {"motor_status": "running"|"not_running", "sensor_status": "sensing"|"sensed"}.
    """

    def __init__(self, channel=None):
        self.channel = channel or Channel()

    @staticmethod
    def parse_status(topic, value):
        if "/tool/" not in topic or not topic.endswith("response"):
            return None
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except ValueError:
                return None
        return value if isinstance(value, dict) else None

    def send(self, command):
        self.channel.clear()
        self.channel.send(TOOL_REQUEST, command)

    def wait_for(self, key, wanted, timeout):
        """Wait until the tool reports status[key] == wanted. Returns True if seen."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            item = self.channel.get(timeout=0.05)
            if item:
                status = self.parse_status(*item)
                if status and status.get(key) == wanted:
                    return True
        return False

    def hit(self, timeout=15.0):
        """Fire the hammer once and wait for it to finish."""
        self.send("hit")
        if not self.wait_for("motor_status", "running", 5.0):
            print("  (no 'running' reply from tool - is the control panel connected?)")
            return False
        if not self.wait_for("motor_status", "not_running", timeout):
            print("  (hammer did not report done)")
            return False
        return True
