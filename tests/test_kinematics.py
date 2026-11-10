"""Kinematics on a planar two-link arm with known closed-form answers."""

import numpy as np
import pytest

from lerobot_robot_lwa4p.kinematics import Chain, rotate, rotation_error

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


def test_ik_reaches_reachable_pose(chain):
    target, _, _ = chain.fk([0.4, 0.9])
    q = chain.ik(target, [0.3, 0.5])
    tip, _, _ = chain.fk(q)
    assert np.allclose(tip, target, atol=1e-6)


def test_ik_respects_limits(chain):
    target, _, _ = chain.fk([0.0, 0.0])
    q = chain.ik(target, [2.9, 0.5])
    assert np.all(q <= chain.upper - 0.05 + 1e-9) and np.all(q >= chain.lower + 0.05 - 1e-9)


def test_rotate_and_rotation_error_are_inverse():
    pose = np.eye(4)
    rv = np.array([0.1, -0.2, 0.3])
    assert np.allclose(rotation_error(rotate(rv, pose), pose), rv)


def test_wrong_joint_names_rejected():
    with pytest.raises(ValueError):
        Chain(URDF, ["j1"], "tip")
