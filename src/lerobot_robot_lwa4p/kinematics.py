"""Serial-chain kinematics from a URDF string: forward kinematics, Jacobian, differential IK."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_angle(axis, angle):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


@dataclass
class _Joint:
    name: str
    movable: bool
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class Chain:
    """Kinematic chain from the URDF root link to `tip`, over the given movable joints."""

    def __init__(self, urdf: str, joint_names: list[str], tip: str):
        joints = {}
        for j in ET.fromstring(urdf).findall("joint"):  # top level only, not ros2_control tags
            joints[j.find("child").get("link")] = (j, j.find("parent").get("link"))
        path, link = [], tip
        while link in joints:
            j, link = joints[link]
            path.append(j)
        self.joints = [self._parse(j) for j in reversed(path)]
        movable = [j.name for j in self.joints if j.movable]
        if movable != list(joint_names):
            raise ValueError(f"chain to {tip} has joints {movable}, expected {joint_names}")
        self.lower = np.array([j.lower for j in self.joints if j.movable])
        self.upper = np.array([j.upper for j in self.joints if j.movable])

    @staticmethod
    def _parse(j):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        origin = np.eye(4)
        origin[:3, :3], origin[:3, 3] = _rpy_matrix(*rpy), xyz
        a = j.find("axis")
        axis = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()])
        lim = j.find("limit")
        lo, hi = (
            (float(lim.get("lower", -np.pi)), float(lim.get("upper", np.pi)))
            if lim is not None
            else (-np.pi, np.pi)
        )
        movable = j.get("type") in ("revolute", "continuous")
        return _Joint(j.get("name"), movable, origin, axis / np.linalg.norm(axis), lo, hi)

    def fk(self, q):
        """Tip pose (4x4) and, per movable joint, its axis and origin in the root frame."""
        t, axes, points, k = np.eye(4), [], [], 0
        for j in self.joints:
            t = t @ j.origin
            if j.movable:
                axes.append(t[:3, :3] @ j.axis)
                points.append(t[:3, 3].copy())
                rot = np.eye(4)
                rot[:3, :3] = _axis_angle(j.axis, q[k])
                t = t @ rot
                k += 1
        return t, axes, points

    def jacobian(self, q):
        """Geometric 6xN Jacobian [linear; angular] of the tip in the root frame."""
        tip, axes, points = self.fk(q)
        p = tip[:3, 3]
        return np.vstack(
            [
                np.column_stack([np.cross(a, p - o) for a, o in zip(axes, points)]),
                np.column_stack(axes),
            ]
        )

    def manipulability(self, q):
        """Yoshikawa measure sqrt(det(J J^T)): 0 at a singular pose."""
        jac = self.jacobian(q)
        return float(np.sqrt(max(np.linalg.det(jac @ jac.T), 0.0)))

    def ik(self, target, q, iters=5, damping=0.01, margin=0.05, w0=0.01, max_damping=0.08):
        """Joint positions near `q` that put the tip at the 4x4 `target` pose (root frame).

        Damping grows near singular poses (manipulability below `w0`), so the solution stays
        bounded there instead of exploding.
        """
        q = np.asarray(q, dtype=float)
        for _ in range(iters):
            tip = self.fk(q)[0]
            err = np.concatenate([target[:3, 3] - tip[:3, 3], rotation_error(target, tip)])
            if np.linalg.norm(err) < 1e-7:
                break
            jac = self.jacobian(q)
            w = np.sqrt(max(np.linalg.det(jac @ jac.T), 0.0))
            lam = damping + (max_damping * (1.0 - w / w0) ** 2 if w < w0 else 0.0)
            dq = jac.T @ np.linalg.solve(jac @ jac.T + lam**2 * np.eye(6), err)
            q = np.clip(q + dq, self.lower + margin, self.upper - margin)
        return q


def rotation_error(target, pose):
    """Rotation vector (root frame) that turns `pose` into `target` (both 4x4)."""
    r = target[:3, :3] @ pose[:3, :3].T
    angle = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle > np.pi - 1e-6:  # axis from the symmetric part when sin(angle) vanishes
        axis = np.sqrt(np.clip((np.diag(r) + 1.0) / 2.0, 0.0, None))
        return angle * axis / np.linalg.norm(axis)
    return angle * axis / (2.0 * np.sin(angle))


def rotate(rotvec, pose):
    """`pose` (4x4) rotated by `rotvec` (root frame) about its own origin."""
    angle = np.linalg.norm(rotvec)
    out = pose.copy()
    if angle > 1e-12:
        out[:3, :3] = _axis_angle(np.asarray(rotvec) / angle, angle) @ pose[:3, :3]
    return out
