"""TCP bridge that lets a LeRobot process (own venv, any Python) drive the LWA 4P over ROS 2.

One JSON object per line, one reply per request:
  {"op": "state"}                         -> {"q": [6], "gripper": m, "stamp": s, "error": str}
  {"op": "act", "q": [6], "gripper": m}   -> {"ok": bool, "q": [6] sent, "error": str}
  {"op": "hold"}                          -> {"ok": bool}
  {"op": "urdf"}                          -> {"urdf": str}
IP commands retain complete constant-jerk trajectories ending at rest, bounded by
max_smooth_duration_s and the original maximum joint-position lead. Refused IP commands leave
the previous validated stop in place. The Python driver keeps its short position-only goals.
"""

import json
import math
import socketserver
import threading
import time
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
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
        self.ip = p("interpolated_position", False)
        self.max_smooth_duration = p("max_smooth_duration_s", 2.0)
        self.max_acc = p("max_acceleration_rad_s2", 0.4)
        self.max_jerk = p("max_jerk_rad_s3", 4.0)
        self.validation_timeout = p("validation_timeout_s", 0.035)
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
        self.fault, self.bounds = "", None
        self.command_lock = threading.Lock()
        self.command_plan, self.command_stamp = [], 0.0
        self.goal_sequence = 0
        self.trace = deque(maxlen=200)
        self.controller_state = None
        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, "joint_states", self.on_js, 50, callback_group=cb)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "robot_description", self.on_urdf, latched)
        self.create_subscription(String, "lwa4p_driver/fault", self.on_fault, latched)
        self.create_subscription(Float64MultiArray, "lwa4p_driver/limits", self.on_limits, latched)
        self.create_subscription(
            JointTrajectoryControllerState,
            "arm_controller/controller_state",
            self.on_controller_state,
            10,
            callback_group=cb,
        )
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

    def on_fault(self, msg):
        """Latch drive faults; an explicit successful enable is the recovery boundary."""
        with self.lock:
            self.fault = msg.data
            if not msg.data:
                self.goal_sequence += 1
                self.command_plan, self.command_stamp = [], 0.0
                self.sent, self.last_ok, self.error = None, None, ""

    def on_limits(self, msg):
        """Use the same signed, offset and margin-adjusted bounds as the actual hardware."""
        values = list(msg.data)
        if len(values) == 2 * len(self.joints) and all(math.isfinite(v) for v in values):
            bounds = list(zip(values[::2], values[1::2]))
            if all(lo < hi for lo, hi in bounds):
                self.bounds = bounds

    def on_controller_state(self, msg):
        """Retain controller demand and tracking error for the passive capture command."""
        self.controller_state = {
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "desired": {
                name: list(getattr(msg.desired, name))
                for name in ("positions", "velocities", "accelerations")
            },
            "actual": {
                name: list(getattr(msg.actual, name)) for name in ("positions", "velocities")
            },
            "error": list(msg.error.positions),
        }

    def dispatch(self, req):
        op = req.get("op")
        if op == "state":
            return self.state()
        if op == "act":
            return self.act(
                [float(v) for v in req["q"]],
                req.get("gripper"),
                req.get("ahead"),
                req.get("smooth"),
                req.get("stamp"),
            )
        if op == "hold":
            q = self.arm_q()
            if q is not None and not self.ip:
                self.send_arm([(q, self.min_seg)])
            return {"ok": q is not None, "error": self.fault}
        if op == "trace":
            with self.lock:
                return {"events": list(self.trace)}
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
            "fault": self.fault,
            "interpolated_position": self.ip,
            "limits": self.bounds,
            "command_state": _sample_plan(self.command_plan, time.time() - self.command_stamp),
            "controller": self.controller_state,
        }

    def act(self, q_cmd, grip, ahead=None, smooth=None, stamp=None):
        if self.ip:
            with self.command_lock:
                return self.act_ip(q_cmd, grip, smooth, stamp)
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
            smooth = None
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

    def act_ip(self, target, grip, smooth, stamp):
        """Accept a complete continuous stop plan, or retain the last validated trajectory."""
        q = self.arm_q()
        why = self.fault
        points = []
        started = time.monotonic()
        try:
            if why:
                raise ValueError(why)
            if q is None or self.bounds is None:
                raise ValueError("drive feedback or limits not available")
            points = _smooth_points(
                smooth,
                stamp,
                len(q),
                self.max_smooth_duration,
                self.max_vel,
                self.max_acc,
                self.max_jerk,
            )
            _check_path_bounds(points, q, self.bounds, self.max_vel * self.max_seg)
            previous = _sample_plan(self.command_plan, stamp - self.command_stamp)
            if previous is not None:
                for a, b, tolerance in zip(
                    (points[0][0], points[0][2]), previous[:2], (1e-4, 1e-3)
                ):
                    if max(abs(x - y) for x, y in zip(a, b)) > tolerance:
                        raise ValueError("preview does not continue the accepted command state")
                if max(abs(a - b) for a, b in zip(points[0][3], previous[2])) > 1e-3:
                    raise ValueError("preview changes acceleration at replacement")
            # The reference remains the 100 ms target, independent of phase sample spacing.
            reference = _sample_plan(points, 0.1)[0]
            if (
                len(target) != len(q)
                or not all(math.isfinite(v) for v in target)
                or max(abs(a - b) for a, b in zip(reference, target)) > 1e-5
            ):
                raise ValueError("target does not match the smooth preview")
            ok, why = self.valid_many(_collision_samples(points), q)
            if not ok:
                raise ValueError(why)
            if time.time() - stamp > 0.1:
                raise ValueError("preview expired during validation")
            if self.fault:
                raise ValueError(self.fault)
            self.send_arm(points, stamp, reference)
            self.last_ok, self.error = reference, ""
            if grip is not None:
                self.send_gripper(float(grip))
        except (ValueError, TypeError, IndexError) as error:
            why = str(error)
            self.error = why
        event = {
            "stamp": stamp,
            "received": time.time(),
            "validation_ms": (time.monotonic() - started) * 1000.0,
            "accepted": not bool(why),
            "error": why,
            "points": points,
        }
        with self.lock:
            self.trace.append(event)
        return {"ok": not bool(why), "q": self.sent or q, "error": why, "fault": self.fault}

    def valid_many(self, targets, current):
        """Check unique states concurrently with one bounded deadline and no stale cache."""
        if not self.check:
            return True, ""
        if not self.validity.service_is_ready():
            return False, "MoveIt collision check unavailable"
        states = list(dict.fromkeys(tuple(q) for q in [current, *targets]))
        deadline = time.monotonic() + self.validation_timeout
        pending = []
        done = threading.Event()
        completed = set()
        completion_lock = threading.Lock()

        def completed_one(future):
            """Wake the command thread only when every state has a result."""
            with completion_lock:
                completed.add(future)
                if len(completed) == len(states):
                    done.set()

        try:
            for values in states:
                if time.monotonic() >= deadline:
                    return False, "MoveIt collision validation deadline exceeded"
                request = GetStateValidity.Request()
                request.group_name = self.group
                request.robot_state.is_diff = True
                request.robot_state.joint_state.name = self.joints
                request.robot_state.joint_state.position = list(values)
                future = self.validity.call_async(request)
                pending.append(future)
                future.add_done_callback(completed_one)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not done.wait(remaining):
                return False, "MoveIt collision validation deadline exceeded"
            results = [future.result() for future in pending]
            if any(result is None for result in results):
                return False, "MoveIt collision result unavailable"
            here = results[0]
            for result in results[1:]:
                if result.valid or (not here.valid and _depth(result) <= _depth(here) + 1e-4):
                    continue
                contact = result.contacts[0] if result.contacts else None
                return False, "collision" + (
                    f" {contact.contact_body_1} - {contact.contact_body_2}" if contact else ""
                )
            return True, ""
        except Exception as error:
            return False, f"MoveIt collision check failed: {error}"
        finally:
            for future in pending:
                if not future.done():
                    self.validity.remove_pending_request(future)
                    future.cancel()

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

    def send_arm(self, points, stamp=None, reference=None):
        """Send [(positions, time_from_start), ...]; the first point is the new target."""
        self.sent = reference if reference is not None else points[0][0]
        self.t_sent, self.sent_moving = time.monotonic(), len(points) > 1
        if self.ip:
            self.command_plan, self.command_stamp = points, stamp
        self.goal_sequence += 1
        sequence = self.goal_sequence
        goal = FollowJointTrajectory.Goal()
        if stamp is not None:
            goal.trajectory.header.stamp.sec = int(stamp)
            goal.trajectory.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
        goal.trajectory.joint_names = self.joints
        goal.trajectory.points = [_point(*point) for point in points]
        self.arm.send_goal_async(goal).add_done_callback(
            lambda future: self.on_arm_goal(future, sequence)
        )

    def on_arm_goal(self, fut, sequence):
        """Latch rejection of the latest submission without reviving older callbacks."""
        if sequence != self.goal_sequence:
            return
        try:
            gh = fut.result()
        except Exception as error:
            self.fault = f"arm goal transport failed: {error}"
            return
        if not gh.accepted:
            self.error = "arm goal rejected (drives enabled? see the driver log)"
            if self.ip and not self.fault:
                self.fault = self.error
            return
        gh.get_result_async().add_done_callback(lambda future: self.on_arm_result(future, sequence))

    def on_arm_result(self, fut, sequence):
        """Preserve the latest controller failure until explicit drive recovery."""
        if sequence != self.goal_sequence:
            return
        try:
            res = fut.result().result
        except Exception as error:
            self.fault = f"arm result transport failed: {error}"
            return
        self.error = res.error_string if res.error_code != res.SUCCESSFUL else ""
        if self.ip and res.error_code != res.SUCCESSFUL and not self.fault:
            self.fault = self.error or "arm trajectory failed"

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


def _point(q, t, velocity=None, acceleration=None):
    """Preserve optional planner derivatives in a ROS trajectory waypoint."""
    point = JointTrajectoryPoint(positions=list(q))
    if velocity is not None:
        point.velocities = list(velocity)
    if acceleration is not None:
        point.accelerations = list(acceleration)
    point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
    return point


def _smooth_points(
    samples, stamp, joint_count, max_duration, max_velocity, max_acceleration=0.4, max_jerk=4.0
):
    """Require bounded constant-jerk segments and a stationary final state."""
    if stamp is None or not math.isfinite(stamp) or not 0.0 <= time.time() - stamp <= 0.1:
        raise ValueError("smooth preview timestamp is stale or invalid")
    if not isinstance(samples, list) or not 2 <= len(samples) <= 256:
        raise ValueError("smooth preview must contain 2 to 256 samples")
    points = []
    previous = -1.0
    for t, position, velocity, acceleration in samples:
        if not math.isfinite(t) or not previous < t <= max_duration:
            raise ValueError("smooth preview times must increase within max_smooth_duration_s")
        for values in (position, velocity, acceleration):
            if len(values) != joint_count or not all(math.isfinite(v) for v in values):
                raise ValueError("smooth preview must contain finite states for every joint")
        if any(abs(v) > max_velocity + 1e-6 for v in velocity):
            raise ValueError("smooth preview exceeds joint velocity limit")
        if any(abs(a) > max_acceleration + 1e-6 for a in acceleration):
            raise ValueError("smooth preview exceeds joint acceleration limit")
        points.append((position, t, velocity, acceleration))
        previous = t
    if len(points) < 2 or points[0][1] != 0.0:
        raise ValueError("smooth preview must start at zero and include future samples")
    if any(abs(v) > 1e-6 for values in points[-1][2:] for v in values):
        raise ValueError("smooth preview must finish at rest")
    for start, end in zip(points, points[1:]):
        dt = end[1] - start[1]
        if dt < 1e-7:
            raise ValueError("smooth preview contains unrepresentable time intervals")
        for p0, v0, a0, p1, v1, a1 in zip(start[0], start[2], start[3], end[0], end[2], end[3]):
            jerk = (a1 - a0) / dt
            if abs(jerk) > max_jerk + 1e-5:
                raise ValueError("smooth preview exceeds joint jerk limit")
            expected_p = p0 + v0 * dt + a0 * dt * dt / 2 + jerk * dt * dt * dt / 6
            expected_v = v0 + a0 * dt + jerk * dt * dt / 2
            if abs(expected_p - p1) > 1e-7 or abs(expected_v - v1) > 1e-6:
                raise ValueError("smooth preview omitted a jerk phase boundary")
            if abs(jerk) > 1e-12 and 0.0 < -a0 / jerk < dt:
                t = -a0 / jerk
                if abs(v0 + a0 * t + jerk * t * t / 2) > max_velocity + 1e-6:
                    raise ValueError("smooth preview exceeds velocity between samples")
    return points


def _sample_plan(points, t):
    """Evaluate the accepted constant-jerk plan, holding its terminal position after expiry."""
    if not points:
        return None
    if t <= 0.0:
        return [points[0][0], points[0][2], points[0][3]]
    for start, end in zip(points, points[1:]):
        if t < end[1]:
            dt, elapsed = end[1] - start[1], t - start[1]
            jerk = [(b - a) / dt for a, b in zip(start[3], end[3])]
            return [
                [
                    p + v * elapsed + a * elapsed**2 / 2 + j * elapsed**3 / 6
                    for p, v, a, j in zip(start[0], start[2], start[3], jerk)
                ],
                [v + a * elapsed + j * elapsed**2 / 2 for v, a, j in zip(start[2], start[3], jerk)],
                [a + j * elapsed for a, j in zip(start[3], jerk)],
            ]
    return [points[-1][0], points[-1][2], points[-1][3]]


def _collision_samples(points):
    """Keep the 100 ms collision-check spacing independent of dense spline samples."""
    count = math.ceil(points[-1][1] / 0.1)
    return [_sample_plan(points, min(k * 0.1, points[-1][1]))[0] for k in range(count + 1)]


def _check_path_bounds(points, current, bounds, max_lead):
    """Check actual drive limits and position lead, including extrema between samples."""
    for start, end in zip(points, points[1:]):
        dt = end[1] - start[1]
        for p0, v0, a0, a1, actual, (lo, hi) in zip(
            start[0], start[2], start[3], end[3], current, bounds
        ):
            jerk = (a1 - a0) / dt
            times = [0.0, dt]
            if abs(jerk) < 1e-12:
                if abs(a0) > 1e-12:
                    times.append(-v0 / a0)
            else:
                discriminant = a0 * a0 - 2 * jerk * v0
                if discriminant >= 0:
                    times.extend(
                        [
                            (-a0 - math.sqrt(discriminant)) / jerk,
                            (-a0 + math.sqrt(discriminant)) / jerk,
                        ]
                    )
            for t in times:
                if not 0.0 <= t <= dt:
                    continue
                position = p0 + v0 * t + a0 * t * t / 2 + jerk * t * t * t / 6
                if not lo <= position <= hi:
                    raise ValueError("smooth preview exceeds actual drive limits")
                if abs(position - actual) > max_lead + 1e-6:
                    raise ValueError("smooth preview exceeds maximum position lead")


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
