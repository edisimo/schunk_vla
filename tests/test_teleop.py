"""Stick shaping: deadzone, snapping, and no z/yaw cross-talk."""

import numpy as np

from lerobot_robot_lwa4p.teleop import Lwa4pGamepad, Lwa4pGamepadConfig, stick_twist

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


def test_ik_uses_actual_drive_limits_without_opening_gamepad():
    """A narrow drive limit must constrain IK before hardware clipping can occur."""
    from types import SimpleNamespace

    joints = []
    for i in range(1, 7):
        joints.append(
            f'<joint name="arm_{i}_joint" type="revolute"><parent link="l{i-1}"/>'
            f'<child link="l{i}"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>'
        )
    urdf = '<robot name="test">' + "".join(joints) + "</robot>"
    teleop = Lwa4pGamepad(Lwa4pGamepadConfig(tip_link="l6"))
    teleop.pad = SimpleNamespace(open=lambda: "fake gamepad")
    teleop.bridge = SimpleNamespace(
        connect=lambda: None,
        call=lambda op: (
            {"urdf": urdf}
            if op == "urdf"
            else {"interpolated_position": True, "limits": [[-2.0, 2.0]] * 6}
        ),
    )
    teleop.connect()
    assert np.all(teleop.chain.lower == -2.0)
    assert np.all(teleop.chain.upper == 2.0)
    assert teleop.margin == 0.005


def test_precision_step_can_escape_a_near_limit_pose():
    """Small moves away from a bound remain available even inside the proximity buffer."""
    from types import SimpleNamespace

    teleop = Lwa4pGamepad(Lwa4pGamepadConfig())
    teleop.margin = 0.005
    teleop.q = np.zeros(6)
    teleop.q[0] = -0.995
    teleop.chain = SimpleNamespace(
        lower=-np.ones(6),
        upper=np.ones(6),
        fk=lambda q: (np.eye(4), None, None),
        manipulability=lambda q: 0.1,
    )
    away = teleop.q.copy()
    away[0] += 0.0001
    inward = teleop.q.copy()
    inward[0] -= 0.0001
    assert teleop._unsafe(away, np.eye(4)) == ""
    assert "limit" in teleop._unsafe(inward, np.eye(4))
