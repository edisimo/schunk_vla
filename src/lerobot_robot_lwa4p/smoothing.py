"""Jerk-limited, phase-synchronized joint streaming (Ruckig): all joints start and stop together."""

import numpy as np
from ruckig import InputParameter, Result, Ruckig, Synchronization, Trajectory


class JointSmoother:
    """Re-plans one smooth trajectory to every new joint target.

    Phase synchronization moves the joints proportionally, so the arm travels on a straight line
    in joint space; planning to stop at each target means it never overshoots when targets stop.
    """

    def __init__(self, n, max_vel, max_acc, max_jerk, horizon=(0.1, 0.2, 0.3, 0.4)):
        self.n, self.horizon = n, horizon
        self.otg, self.inp = Ruckig(n), InputParameter(n)
        self.inp.max_velocity = [max_vel] * n
        self.inp.max_acceleration = [max_acc] * n
        self.inp.max_jerk = [max_jerk] * n
        self.inp.synchronization = Synchronization.Phase
        self.traj, self.state, self.t_plan = None, None, 0.0

    def reset(self, q, now):
        """Start from the arm at rest at `q`."""
        self.state = (np.asarray(q, dtype=float), np.zeros(self.n), np.zeros(self.n))
        self.traj, self.t_plan = None, now

    def position(self, now):
        """Commanded position at time `now`."""
        return self._state_at(now)[0]

    def step(self, goal, now):
        """Re-plan to `goal`; return the planned positions at the horizon times after `now`."""
        p, v, a = self._state_at(now)
        self.inp.current_position, self.inp.current_velocity = p, v
        self.inp.current_acceleration = a
        self.inp.target_position = np.asarray(goal, dtype=float)
        traj = Trajectory(self.n)
        if self.otg.calculate(self.inp, traj) not in (Result.Working, Result.Finished):
            raise RuntimeError(f"no smooth trajectory from {p} to {goal}")
        self.traj, self.t_plan = traj, now
        return [(t, np.asarray(traj.at_time(min(t, traj.duration))[0])) for t in self.horizon]

    def _state_at(self, now):
        if self.traj is None:
            return self.state
        t = min(now - self.t_plan, self.traj.duration)
        return tuple(np.asarray(x) for x in self.traj.at_time(t))
