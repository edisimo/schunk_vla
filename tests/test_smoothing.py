"""Smoother: straight (phase-synchronized) joint motion, exact stops, no overshoot."""

import numpy as np

from lerobot_robot_lwa4p.smoothing import JointSmoother


def _run(smoother, goals, dt=0.05):
    """Feed one goal per step; return the commanded positions."""
    out, now = [], 0.0
    for g in goals:
        smoother.step(g, now)
        now += dt
        out.append(smoother.position(now))
    return np.array(out)


def test_straight_line_from_rest_and_exact_stop():
    s = JointSmoother(2, 0.45, 0.4, 2.0)
    s.reset([0.0, 0.0], 0.0)
    path = _run(s, [[0.1, 0.05]] * 60)
    moving = path[(path[:, 0] > 1e-4) & (path[:, 0] < 0.1 - 1e-4)]
    assert np.allclose(moving[:, 1] / moving[:, 0], 0.5, atol=1e-6)
    assert np.allclose(path[-1], [0.1, 0.05], atol=1e-9)


def test_no_overshoot_when_streamed_goal_stops():
    s = JointSmoother(1, 0.45, 0.4, 2.0)
    s.reset([0.0], 0.0)
    goals = [[0.004 * k] for k in range(1, 26)] + [[0.1]] * 60  # 0.08 rad/s, then stop at 0.1
    path = _run(s, goals)
    assert path.max() <= 0.1 + 1e-9 and abs(path[-1, 0] - 0.1) < 1e-9


def test_limits_hold():
    s = JointSmoother(1, 0.45, 0.4, 2.0)
    s.reset([0.0], 0.0)
    path = _run(s, [[1.0]] * 200)[:, 0]
    vel = np.diff(path) / 0.05
    assert vel.max() <= 0.45 + 1e-6 and np.abs(np.diff(vel) / 0.05).max() <= 0.4 + 1e-3
