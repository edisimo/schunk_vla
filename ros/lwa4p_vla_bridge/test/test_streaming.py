# Copyright (c) 2026 Airvolute s.r.o.
# Licensed under the Airvolute DroneCore Platform License (ADPL), Version 1.0.
# Use permitted only within Airvolute Hardware, the DroneCore Platform,
# or Derived Systems. IP remains with Airvolute s.r.o.
# Contact: info@airvolute.com

"""IP bridge behavior with real ROS messages and fake services; no ROS graph or robot."""

import threading
import time
from concurrent.futures import Future
from types import MethodType, SimpleNamespace

import pytest
from lwa4p_vla_bridge.bridge_node import (
    VlaBridge,
    _check_path_bounds,
    _collision_samples,
    _point,
    _sample_plan,
    _smooth_points,
)


def preview():
    """Integrate four jerk phases into a small, complete move from rest to rest."""
    p = v = a = t = 0.0
    samples = [[t, [p] * 6, [v] * 6, [a] * 6]]
    for jerk in (1.0, -1.0, -1.0, 1.0):
        h = 0.2
        p, v, a = (
            p + v * h + a * h * h / 2 + jerk * h**3 / 6,
            v + a * h + jerk * h * h / 2,
            a + jerk * h,
        )
        t += h
        samples.append([t, [p] * 6, [v] * 6, [a] * 6])
    return samples


@pytest.fixture
def bridge():
    """Build a bridge command path without initializing a ROS node or executor."""
    instance = SimpleNamespace(
        ip=True,
        fault="",
        bounds=[(-1.0, 1.0)] * 6,
        max_vel=0.45,
        max_seg=0.5,
        max_smooth_duration=2.0,
        max_acc=0.4,
        max_jerk=4.0,
        min_seg=0.1,
        max_lead=0.225,
        sent=None,
        t_sent=0.0,
        sent_moving=False,
        error="",
        last_ok=None,
        command_plan=[],
        command_stamp=0.0,
        trace=[],
        lock=threading.Lock(),
        goal_sequence=0,
        joints=[f"arm_{i}_joint" for i in range(1, 7)],
        arm_q=lambda: [0.0] * 6,
        valid_many=lambda targets, current: (True, ""),
    )
    sent = []

    def send(points, stamp=None, reference=None):
        """Record the exact command timeline that a real controller would receive."""
        sent.append((points, stamp))
        instance.command_plan, instance.command_stamp = points, stamp
        instance.sent = reference if reference is not None else points[0][0]

    instance.send_arm = send
    instance.act_ip = MethodType(VlaBridge.act_ip, instance)
    return instance, sent


def test_complete_preview_preserves_dynamics(bridge):
    """A valid move retains all derivatives, timestamp and terminal rest."""
    instance, sent = bridge
    samples = preview()
    stamp = time.time()
    reference = _sample_plan(_smooth_points(samples, stamp, 6, 2.0, 0.45), 0.1)[0]
    result = instance.act_ip(reference, None, samples, stamp)
    assert result["ok"]
    points, actual_stamp = sent[-1]
    assert actual_stamp == stamp
    assert points[-1][1] > 0.4
    assert list(_point(*points[1]).accelerations) == samples[1][3]
    assert points[-1][2] == pytest.approx([0.0] * 6)


@pytest.mark.parametrize("reason", ["collision", "deadline", "limits", "discontinuity", "missing"])
def test_refusal_keeps_accepted_stop(bridge, reason):
    """Every refusal leaves the old finite plan intact, with no position-only hold."""
    instance, sent = bridge
    old = _smooth_points(preview(), time.time(), 6, 2.0, 0.45)
    old_stamp = time.time() - 0.05
    instance.command_plan, instance.command_stamp = old, old_stamp
    current = _sample_plan(old, 0.05)
    samples = [[0.0, *current]] + [[p[1] - 0.05, p[0], p[2], p[3]] for p in old if p[1] > 0.05]
    if reason == "collision":
        instance.valid_many = lambda targets, current: (False, "collision")
    elif reason == "deadline":
        instance.valid_many = lambda targets, current: (
            False,
            "MoveIt collision validation deadline exceeded",
        )
    elif reason == "limits":
        instance.bounds = [(-0.001, 0.001)] * 6
    elif reason == "discontinuity":
        for point in samples:
            point[1] = [p + 0.01 for p in point[1]]
    else:
        samples = None
    reference = _sample_plan(old, 0.15)[0]
    result = instance.act_ip(reference, None, samples, old_stamp + 0.05)
    assert not result["ok"]
    assert not sent
    assert instance.command_plan is old
    assert all(abs(v) < 1e-9 for values in _sample_plan(old, 5.0)[1:] for v in values)


def test_replacement_preserves_velocity_and_acceleration(bridge):
    """Replacement starts at the old trajectory's state at the original planner timestamp."""
    instance, sent = bridge
    old_stamp = time.time() - 0.05
    old = _smooth_points(preview(), old_stamp, 6, 2.0, 0.45)
    instance.command_plan, instance.command_stamp = old, old_stamp
    current = _sample_plan(old, 0.05)
    samples = [[0.0, *current]] + [[p[1] - 0.05, p[0], p[2], p[3]] for p in old if p[1] > 0.05]
    result = instance.act_ip(_sample_plan(old, 0.15)[0], None, samples, old_stamp + 0.05)
    assert result["ok"], result
    assert sent[-1][0][0][2] == pytest.approx(current[1])
    assert sent[-1][0][0][3] == pytest.approx(current[2])


def test_no_measured_position_rewind_on_hold(bridge):
    """Disconnect/hold keeps the validated braking tail instead of inserting a feedback jump."""
    instance, sent = bridge
    result = VlaBridge.dispatch(instance, {"op": "hold"})
    assert result["ok"]
    assert not sent


@pytest.mark.parametrize(
    "bad", ["stale", "nonfinite", "velocity", "acceleration", "jerk", "phase", "terminal", "count"]
)
def test_invalid_motion_never_reaches_controller(bad):
    """Reject malformed dynamics before they can be turned into splines."""
    samples = preview()
    stamp = time.time()
    if bad == "stale":
        stamp -= 1
    elif bad == "nonfinite":
        samples[1][1][0] = float("nan")
    elif bad == "velocity":
        samples[1][2][0] = 1.0
    elif bad == "acceleration":
        samples[1][3][0] = 1.0
    elif bad == "jerk":
        samples[1][0] = 0.01
    elif bad == "phase":
        samples = [samples[0], samples[-1]]
    elif bad == "terminal":
        samples = samples[:-1]
    elif bad == "count":
        samples = []
    with pytest.raises(ValueError):
        _smooth_points(samples, stamp, 6, 2.0, 0.45)


def test_position_extrema_cannot_escape_limits_between_samples():
    """Endpoint checks alone miss a parabola that crosses the upper drive limit."""
    points = [([0.0], 0.0, [0.1], [-0.4]), ([0.0], 0.5, [-0.1], [-0.4])]
    with pytest.raises(ValueError, match="actual drive limits"):
        _check_path_bounds(points, [0.0], [(-0.1, 0.01)], 0.2)


def test_dense_waypoints_do_not_multiply_collision_requests():
    """Collision sampling follows time, rather than every Ruckig phase or interpolation point."""
    points = _smooth_points(preview(), time.time(), 6, 2.0, 0.45)
    dense = [
        (_sample_plan(points, k * 0.01)[0], k * 0.01, *_sample_plan(points, k * 0.01)[1:])
        for k in range(81)
    ]
    assert len(_collision_samples(dense)) == 9


def test_collision_calls_share_one_deadline_and_deduplicate(bridge):
    """All independent checks are sent before waiting, avoiding serialized transport latency."""
    instance, _ = bridge
    futures = []

    class Client:
        """Complete a batch together after its final request arrives."""

        def service_is_ready(self):
            """Expose a ready fake MoveIt server."""
            return True

        def call_async(self, request):
            """Only complete requests once the entire unique batch has been submitted."""
            future = Future()
            futures.append(future)
            if len(futures) == 3:
                for pending in futures:
                    pending.set_result(SimpleNamespace(valid=True, contacts=[]))
            return future

        def remove_pending_request(self, future):
            """Mirror rclpy cleanup on a timed-out request."""
            pass

    instance.check = True
    instance.group = "arm"
    instance.validation_timeout = 0.5
    instance.validity = Client()
    result = VlaBridge.valid_many(instance, [[0.01] * 6, [0.02] * 6, [0.01] * 6], [0.0] * 6)
    assert result == (True, "")
    assert len(futures) == 3


def test_timeout_cleans_up_pending_collision_requests(bridge):
    """A missing collision reply must not leak requests or allow a new goal."""
    instance, _ = bridge
    pending = []
    removed = []

    def request(_):
        """Leave one fake service request unanswered."""
        future = Future()
        pending.append(future)
        return future

    instance.check = True
    instance.group = "arm"
    instance.validation_timeout = 0.001
    instance.validity = SimpleNamespace(
        service_is_ready=lambda: True, call_async=request, remove_pending_request=removed.append
    )
    result = VlaBridge.valid_many(instance, [[0.01] * 6], [0.0] * 6)
    assert not result[0]
    assert removed == pending
    assert all(f.cancelled() for f in pending)


def test_stale_action_result_cannot_clear_new_fault(bridge):
    """Preempted goal callbacks must not clear a later undervoltage reason."""
    instance, _ = bridge
    instance.goal_sequence = 2
    instance.fault = "arm_5_joint emergency 0x3222"
    instance.error = instance.fault
    future = Future()
    future.set_result(
        SimpleNamespace(result=SimpleNamespace(error_code=0, SUCCESSFUL=0, error_string=""))
    )
    VlaBridge.on_arm_result(instance, future, 1)
    assert "0x3222" in instance.fault
    assert "0x3222" in instance.error


def test_collision_deadline_includes_request_submission(bridge):
    """Slow service submission must consume the same budget as waiting for replies."""
    instance, _ = bridge
    pending = []
    removed = []

    def delayed_request(_):
        """Simulate transport serialization taking most of the validation budget."""
        time.sleep(0.003)
        future = Future()
        pending.append(future)
        return future

    instance.check = True
    instance.group = "arm"
    instance.validation_timeout = 0.004
    instance.validity = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=delayed_request,
        remove_pending_request=removed.append,
    )
    result = VlaBridge.valid_many(instance, [[0.01] * 6, [0.02] * 6, [0.03] * 6], [0.0] * 6)
    assert not result[0]
    assert len(pending) <= 2
    assert removed == pending
