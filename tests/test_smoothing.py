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


def test_preview_preserves_derivatives_at_replanning_and_at_rest():
    """A replacement shares the old plan's state at its timestamp, including derivatives."""
    smoother = JointSmoother(2, 0.45, 0.4, 4.0)
    smoother.reset([0.0, 0.0], 0.0)
    smoother.step([0.1, -0.05], 0.0)
    before = smoother.traj.at_time(0.05)
    smoother.step([0.12, -0.06], 0.05)
    preview = smoother.preview()
    assert preview[0][0] == 0.0
    np.testing.assert_allclose(preview[0][1:], before, atol=1e-12)
    assert np.any(np.abs(preview[0][2]) > 0.0)
    assert np.any(np.abs(preview[0][3]) > 0.0)
    for t, position, velocity, acceleration in preview:
        np.testing.assert_allclose(
            [position, velocity, acceleration],
            smoother.traj.at_time(min(t, smoother.traj.duration)),
            atol=1e-12,
        )
    smoother.step([0.12, -0.06], 10.0)
    for _, position, velocity, acceleration in smoother.preview():
        np.testing.assert_allclose(position, [0.12, -0.06], atol=1e-12)
        np.testing.assert_allclose([velocity, acceleration], 0.0, atol=1e-12)


def test_refused_replan_resumes_accepted_timeline():
    """A rejected target must not erase velocity or rewind to lagging measurements."""
    smoother = JointSmoother(1, 0.45, 0.3, 2.0)
    smoother.reset([0.0], 0.0)
    smoother.step([0.15], 0.0)
    accepted = smoother.snapshot()
    expected = smoother.traj.at_time(0.13)
    smoother.step([-0.15], 0.05)
    smoother.restore(accepted)
    np.testing.assert_allclose(smoother.position(0.13), expected[0], atol=1e-12)
    smoother.step([0.16], 0.13)
    np.testing.assert_allclose(smoother.preview()[0][1:], expected, atol=1e-12)


def test_full_preview_preserves_jerk_and_dropouts_end_at_rest():
    """Represent every jerk change exactly, including after reversal and delayed updates."""
    smoother = JointSmoother(6, 0.45, 0.3, 2.0)
    smoother.reset(np.zeros(6), 0.0)
    now = 0.0
    for k in range(80):
        now += (0.05, 0.08, 0.11, 0.03)[k % 4]
        smoother.step(0.15 * np.sin(now + np.arange(6) * 0.4), now, max_step=0.2)
        preview = smoother.preview()
        assert preview[-1][0] >= smoother.traj.duration
        assert max(abs(v) for state in preview[-1][2:] for v in state) < 1e-6
        for start, end in zip(preview, preview[1:]):
            dt = end[0] - start[0]
            p, v, a = map(np.array, start[1:])
            pn, vn, an = map(np.array, end[1:])
            jerk = (an - a) / dt
            assert np.max(abs(jerk)) <= 2.0 + 1e-5
            np.testing.assert_allclose(p + v * dt + a * dt**2 / 2 + jerk * dt**3 / 6, pn, atol=1e-7)
            np.testing.assert_allclose(v + a * dt + jerk * dt**2 / 2, vn, atol=1e-6)
            # A quintic through these states reduces to the same cubic within each phase.
            sample = dt * 0.37
            exact = smoother.traj.at_time(min(start[0] + sample, smoother.traj.duration))
            np.testing.assert_allclose(
                p + v * sample + a * sample**2 / 2 + jerk * sample**3 / 6, exact[0], atol=1e-7
            )
        assert np.max(abs(np.array(preview[-1][1]) - preview[0][1])) <= 0.2 + 1e-6
