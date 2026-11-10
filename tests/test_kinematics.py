"""Kinematics on a planar two-link arm with known closed-form answers."""

import numpy as np
import pytest

from lerobot_robot_lwa4p.kinematics import Chain

URDF = """<robot name="planar">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tip"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="fix" type="fixed"><parent link="l2"/><child link="tip"/><origin xyz="1 0 0"/></joint>
  <ros2_control name="x"><joint name="j1"/></ros2_control>
</robot>"""


@pytest.fixture
def chain():
    return Chain(URDF, ["j1", "j2"], "tip")


def test_fk_matches_closed_form(chain):
    q = [0.3, 0.5]
    tip, _, _ = chain.fk(q)
    expected = [np.cos(0.3) + np.cos(0.8), np.sin(0.3) + np.sin(0.8), 0.0]
    assert np.allclose(tip[:3, 3], expected)


def test_ik_step_follows_twist(chain):
    q = np.array([0.3, 0.5])
    twist = chain.jacobian(q) @ np.array([0.2, -0.1])  # reachable with two joints
    tip0, _, _ = chain.fk(q)
    tip1, _, _ = chain.fk(chain.ik_step(q, twist, 0.01, damping=1e-4))
    assert np.allclose((tip1[:3, 3] - tip0[:3, 3]) / 0.01, twist[:3], atol=2e-3)


def test_ik_step_respects_limits(chain):
    q = chain.ik_step([2.9, 0.5], [0, 0, 0, 0, 0, 50.0], 1.0)
    assert np.all(q <= chain.upper - 0.05 + 1e-9)


def test_wrong_joint_names_rejected():
    with pytest.raises(ValueError):
        Chain(URDF, ["j1"], "tip")
