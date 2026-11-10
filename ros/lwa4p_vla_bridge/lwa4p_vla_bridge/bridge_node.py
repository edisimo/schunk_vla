"""TCP bridge that lets a LeRobot process (own venv, any Python) drive the LWA 4P over ROS 2.

One JSON object per line, one reply per request:
  {"op": "state"}                         -> {"q": [6], "gripper": m, "stamp": s, "error": str}
  {"op": "act", "q": [6], "gripper": m}   -> {"ok": bool, "q": [6] sent, "error": str}
  {"op": "hold"}                          -> {"ok": bool}
  {"op": "urdf"}                          -> {"urdf": str}
Every arm target is velocity-clamped, collision-checked by MoveIt and sent as a short
FollowJointTrajectory goal that preempts the previous one, so the arm stops on its own
within max_segment_s when the client goes quiet.
"""

import json
import socketserver
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

GRIPPER_JOINT = "gripper_joint"


class VlaBridge(Node):
    """ROS side of the bridge: state cache, safety checks and goal streaming."""

    def __init__(self):
        super().__init__("lwa4p_vla_bridge")

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.host, self.port = p("host", "127.0.0.1"), p("port", 5555)
        self.joints = list(p("joint_names", [f"arm_{i}_joint" for i in range(1, 7)]))
        self.max_vel = p("max_velocity_rad_s", 0.45)
        self.max_seg = p("max_segment_s", 0.5)
        self.min_seg = p("min_segment_s", 0.1)
        # the driver's look-ahead: moving targets are extended this far along their velocity
        self.lookahead = p("lookahead_s", 0.3)
        self.check = p("check_collisions", True)
        self.group = p("planning_group", "arm")
        self.grip_thr = p("gripper_threshold_m", 0.01)
        self.grip_force = p("gripper_force_n", 35.0)

        self.lock = threading.Lock()
        self.pos, self.stamp, self.urdf, self.error = {}, 0.0, "", ""
        self.grip_sent, self.grip_busy = None, False
        self.last_ok = self.sent = None
        self.t_sent, self.sent_moving = 0.0, False
        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, "joint_states", self.on_js, 50, callback_group=cb)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "robot_description", self.on_urdf, latched)
        self.arm = ActionClient(
            self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory", callback_group=cb
        )
        self.grip = ActionClient(
            self, GripperCommand, "gripper_controller/gripper_cmd", callback_group=cb
        )
        self.validity = self.create_client(
            GetStateValidity, "check_state_validity", callback_group=cb
        )

    def on_js(self, msg):
        with self.lock:
            self.pos.update(zip(msg.name, msg.position))
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_urdf(self, msg):
        self.urdf = msg.data

    def dispatch(self, req):
        op = req.get("op")
        if op == "state":
            return self.state()
        if op == "act":
            return self.act([float(v) for v in req["q"]], req.get("gripper"), req.get("ahead"))
        if op == "hold":
            q = self.arm_q()
            if q is not None:
                self.send_arm([(q, self.min_seg)])
            return {"ok": q is not None}
        if op == "urdf":
            return {"urdf": self.urdf}
        return {"ok": False, "error": f"unknown op {op!r}"}

    def arm_q(self):
        with self.lock:
            q = [self.pos.get(n) for n in self.joints]
        return None if None in q else q

    def state(self):
        with self.lock:
            grip, stamp = self.pos.get(GRIPPER_JOINT), self.stamp
        return {
            "q": self.arm_q(),
            "q_cmd": self.sent,  # the commanded joint target the drives are heading for
            "gripper": grip,
            "stamp": stamp,
            "error": self.error,
        }

    def act(self, q_cmd, grip, ahead=None):
        q = self.arm_q()
        if q is None:
            return {"ok": False, "error": "no joint_states yet"}
        # joint speed limit against the previous target, so it holds at any command rate
        since = time.monotonic() - self.t_sent
        recent = self.sent is not None and since < 0.5
        ref = self.sent if recent else q
        target = _toward(ref, q_cmd, self.max_vel * (min(max(since, 0.02), 0.2) if recent else 0.2))
        # and never more than one segment ahead of the arm
        target = _toward(q, target, self.max_vel * self.max_seg)
        if ahead and max(abs(a - b) for a, b in zip(target, q_cmd)) > 1e-9:
            ahead = None  # the client's plan no longer starts where the arm is sent
        far = max(abs(a - b) for a, b in zip(target, q))
        ok, why = self.valid(target, q)
        if not ok:
            # stop at the last accepted (collision-free) target, not where the arm is coasting to
            last = self.last_ok
            near = (
                last is not None
                and max(abs(a - b) for a, b in zip(last, q)) < self.max_vel * self.max_seg
            )
            hold = last if near else q
            self.send_arm([(hold, self.min_seg)])
            self.error = why
            return {"ok": False, "q": hold, "error": why}
        self.last_ok = target
        t1 = max(self.min_seg, far / self.max_vel)
        if ahead:
            # the client's plan: the driver looks into it (velocity feed-forward); shifted later
            # when the arm lags, so the first segment stays within the driver's speed check
            shift = t1 - ahead[0][0]
            points = [(qk, max(t, 0.0) + max(shift, 0.0)) for t, qk in ahead]
            if not self.valid(points[-1][0], q)[0]:
                points = [(target, t1)]
        else:
            # extend a moving target along its velocity by the driver's look-ahead, likewise
            dt = min(max(since, 0.02), 0.2)
            vel = [(a - b) / dt for a, b in zip(target, self.sent)] if recent else [0.0] * len(q)
            points = [(target, t1)]
            if max(abs(v) for v in vel) > 1e-3:
                ext = [a + v * self.lookahead for a, v in zip(target, vel)]
                if self.valid(ext, q)[0]:
                    points.append((ext, t1 + self.lookahead))
        moving = any(max(abs(a - b) for a, b in zip(qk, target)) > 1e-3 for qk, _ in points)
        # an unchanged target would only restart the drive profile (that made the arm shake);
        # after a move, send it once more without the extension so the arm stops on it
        changed = self.sent is None or max(abs(a - b) for a, b in zip(target, self.sent)) > 1e-3
        if changed or self.sent_moving or moving:
            self.send_arm(points if moving else [(target, t1)])
        if grip is not None:
            self.send_gripper(float(grip))
        return {"ok": True, "q": target, "error": self.error}

    def valid(self, target, current):
        if not self.check:
            return True, ""
        res = self.check_state(target)
        if res is None:
            return False, "MoveIt collision check unavailable or timed out (move_group running?)"
        if res.valid:
            return True, ""
        here = self.check_state(current)
        # already in contact (real or in the model): accept moves that don't deepen it, so the
        # arm can slide out of a sideways overlap (fingers around the holder) by going up
        if here is not None and not here.valid and _depth(res) <= _depth(here) + 1e-4:
            return True, ""
        c = res.contacts[0] if res.contacts else None
        return False, "collision" + (f" {c.contact_body_1} - {c.contact_body_2}" if c else "")

    def check_state(self, q):
        """MoveIt's validity result for the arm at `q`, or None when MoveIt doesn't answer."""
        if not self.validity.service_is_ready():
            return None
        req = GetStateValidity.Request()
        req.group_name = self.group
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = self.joints
        req.robot_state.joint_state.position = q
        done = threading.Event()
        fut = self.validity.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        return fut.result() if done.wait(0.2) else None

    def send_arm(self, points):
        """Send [(positions, time_from_start), ...]; the first point is the new target."""
        self.sent, self.t_sent, self.sent_moving = points[0][0], time.monotonic(), len(points) > 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        goal.trajectory.points = [_point(qk, t) for qk, t in points]
        self.arm.send_goal_async(goal).add_done_callback(self.on_arm_goal)

    def on_arm_goal(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.error = "arm goal rejected (drives enabled? see the driver log)"
            return
        gh.get_result_async().add_done_callback(self.on_arm_result)

    def on_arm_result(self, fut):
        res = fut.result().result
        self.error = res.error_string if res.error_code != res.SUCCESSFUL else ""

    def send_gripper(self, gap):
        with self.lock:
            current = self.pos.get(GRIPPER_JOINT)
        if self.grip_sent is None:
            self.grip_sent = current
        if self.grip_busy or self.grip_sent is None or abs(gap - self.grip_sent) < self.grip_thr:
            return
        if not self.grip.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = max(0.0, gap)
        goal.command.max_effort = self.grip_force
        self.grip_busy, self.grip_sent = True, gap
        self.grip.send_goal_async(goal).add_done_callback(self.on_grip_goal)

    def on_grip_goal(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.grip_busy = False
            return
        gh.get_result_async().add_done_callback(lambda _: setattr(self, "grip_busy", False))


def _point(q, t):
    point = JointTrajectoryPoint(positions=list(q))
    point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
    return point


def _toward(start, goal, limit):
    """`goal` pulled back on the straight line from `start` so no joint moves more than `limit`."""
    far = max(abs(g - s) for g, s in zip(goal, start))
    k = min(1.0, limit / far) if far > 1e-9 else 1.0
    return [s + (g - s) * k for g, s in zip(goal, start)]


def _depth(res):
    return sum(c.depth for c in res.contacts)


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        bridge = self.server.bridge
        bridge.get_logger().info(f"client {self.client_address} connected")
        for line in self.rfile:
            try:
                reply = bridge.dispatch(json.loads(line))
            except Exception as e:  # malformed request: answer, keep the connection
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}
            self.wfile.write((json.dumps(reply) + "\n").encode())
        bridge.get_logger().info(f"client {self.client_address} disconnected")


def main():
    rclpy.init()
    node = VlaBridge()
    server = _Server((node.host, node.port), _Handler)
    server.bridge = node
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"listening on {node.host}:{node.port}")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
