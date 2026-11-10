"""Stick shaping: deadzone, snapping, and no z/yaw cross-talk."""

import numpy as np

from lerobot_robot_lwa4p.teleop import stick_twist

SNAP = np.tan(np.radians(20.0))


def test_deadzone_and_rescale():
    assert not np.any(stick_twist(0.1, 0.05, -0.1, 0.0, 0.15, SNAP))
    assert np.isclose(stick_twist(0.0, -1.0, 0.0, 0.0, 0.15, SNAP)[0], 1.0)


def test_slanted_thumb_on_right_stick_gives_pure_z():
    tw = stick_twist(0.0, 0.0, -0.27, 0.96, 0.15, SNAP)  # pushing down, 15 deg off-axis
    assert tw[2] < 0 and tw[5] == 0.0


def test_left_stick_snaps_near_axis_keeps_diagonals():
    near_x = stick_twist(0.0, -0.95, 0.0, 0.0, 0.15, SNAP)
    assert near_x[1] == 0.0
    slight = stick_twist(0.2, -0.95, 0.0, 0.0, 0.15, SNAP)  # 12 deg off the x axis
    assert slight[1] == 0.0 and slight[0] > 0
    diagonal = stick_twist(0.7, -0.7, 0.0, 0.0, 0.15, SNAP)
    assert diagonal[0] > 0 and diagonal[1] < 0
