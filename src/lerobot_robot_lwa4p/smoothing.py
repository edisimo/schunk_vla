"""Jerk-limited, phase-synchronized joint streaming (Ruckig): all joints start and stop together."""

import numpy as np
from ruckig import InputParameter, Result, Ruckig, RuckigError, Synchronization, Trajectory

# straight line (proportional joints) first; Ruckig can fail it numerically while streaming
_SYNC = (Synchronization.Phase, Synchronization.Time, Synchronization.No)


def _calculate(otg, inp, n):
    """Ruckig plan, straight (phase) first; it can fail numerically, so fall back."""
    traj = Trajectory(n)
    for sync in _SYNC:
        inp.synchronization = sync
        try:
            if otg.calculate(inp, traj) in (Result.Working, Result.Finished):
                return traj, sync
        except RuckigError:
            pass
    raise RuntimeError("no smooth trajectory")


class JointSmoother:
    """Re-plans one smooth trajectory to every new joint target.

    Phase synchronization moves the joints proportionally, so the arm travels on a straight line
    in joint space; planning to stop at each target means it never overshoots when targets stop.
    The command trails a moving target by about the braking distance, and across that distance
    it cuts the target's curves, so the path error grows with speed squared. A target stream
    that pauses and jumps bends the plans further, so callers should not throttle targets.
    """

    def __init__(self, n, max_vel, max_acc, max_jerk, horizon=(0.1, 0.2, 0.3, 0.4)):
        self.n, self.horizon = n, horizon
        self.otg, self.inp = Ruckig(n), InputParameter(n)
        self.inp.max_velocity = [max_vel] * n
        self.inp.max_acceleration = [max_acc] * n
        self.inp.max_jerk = [max_jerk] * n
        self.traj, self.state, self.t_plan = None, None, 0.0
        self.fallbacks = {sync: 0 for sync in _SYNC}

    def reset(self, q, now):
        """Start from the arm at rest at `q`."""
        self.state = (np.asarray(q, dtype=float), np.zeros(self.n), np.zeros(self.n))
        self.traj, self.t_plan = None, now

    def position(self, now):
        """Commanded position at time `now`."""
        return self._state_at(now)[0]

    def preview(self):
        """Export every jerk boundary and a stationary endpoint for exact interpolation."""
        end = max(0.1, self.traj.duration)
        times = {0.0, end}
        times.update(float(t) for t in np.arange(0.02, end, 0.02))
        offset = 0.0
        for section, profiles in enumerate(self.traj.profiles):
            for profile in profiles:
                elapsed = offset
                for duration in (*profile.brake.t, *profile.t):
                    elapsed += duration
                    if 0.0 < elapsed < end:
                        times.add(elapsed)
            offset = self.traj.intermediate_durations[section]
        # Sub-nanosecond gaps cannot be represented by ROS Duration and destabilize splines.
        ordered = []
        for t in sorted(times):
            if ordered and t - ordered[-1] < 1e-7:
                if len(ordered) > 1:
                    ordered[-1] = t
            else:
                ordered.append(t)
        return [
            [t, *(values.tolist() for values in self._state_at(self.t_plan + t))] for t in ordered
        ]

    def snapshot(self):
        """Retain the accepted plan without copying or advancing its time origin."""
        return self.traj, self.state, self.t_plan

    def restore(self, snapshot):
        """Resume the previously accepted plan at the current clock time after a refusal."""
        self.traj, self.state, self.t_plan = snapshot

    def step(self, goal, now, max_step=None):
        """Re-plan to `goal`; return the planned positions at the horizon times after `now`."""
        p, v, a = self._state_at(now)
        self.inp.current_position, self.inp.current_velocity = p, v
        self.inp.current_acceleration = a
        delta = np.asarray(goal, dtype=float) - p
        if not np.all(np.isfinite(delta)):
            raise ValueError("joint target must be finite")
        scale = min(1.0, max_step / max(abs(delta))) if max_step and np.any(delta) else 1.0
        self.inp.target_position = p + delta * scale
        traj, sync = _calculate(self.otg, self.inp, self.n)
        self.fallbacks[sync] += 1
        self.traj, self.t_plan = traj, now
        return [(t, np.asarray(traj.at_time(min(t, traj.duration))[0])) for t in self.horizon]

    def _state_at(self, now):
        if self.traj is None:
            return self.state
        t = min(max(now - self.t_plan, 0.0), self.traj.duration)
        return tuple(np.asarray(x) for x in self.traj.at_time(t))
