"""Gamepad teleoperation of the LWA 4P: 6-DoF TCP velocity -> differential IK -> joint targets."""

import os
import time
from dataclasses import dataclass
from functools import cached_property

import numpy as np
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig

from .bridge_client import BridgeClient
from .kinematics import Chain, rotate, rotation_error
from .robot import ARM_JOINTS, MOTORS

_HELP = """LWA 4P gamepad: hold LB to move (release = stop), also hold RB for precision speed
  left stick      TCP x / y          right stick   z (vertical), yaw (horizontal)
  d-pad           roll / pitch       A / B         close / open gripper"""


@TeleoperatorConfig.register_subclass("lwa4p_gamepad")
@dataclass(kw_only=True)
class Lwa4pGamepadConfig(TeleoperatorConfig):
    """Bridge address and speeds; buttons follow SDL's standard (Xbox-style) layout."""

    host: str = "127.0.0.1"
    port: int = 5555
    tip_link: str = "gripper_tcp"
    # path error grows with speed squared (planning lag at the supply's 0.4 rad/s^2): ~2 mm off
    # a straight line at 3 cm/s, ~5 mm at 5 cm/s, sub-mm in precision mode
    linear_speed_m_s: float = 0.03
    angular_speed_rad_s: float = 0.3
    precision_scale: float = 0.25
    # safety bound on how far the desired pose may run ahead of the commanded one (drives
    # disabled, arm blocked); normal motion stays below it, since pausing bends the path
    max_lead_m: float = 0.05
    max_lead_rad: float = 0.3
    # refuse a step (stop) rather than leave the line near singular poses or joint limits
    min_manipulability: float = 0.002
    max_ik_error_m: float = 5e-4
    # near singular poses a small TCP step needs big joint moves: slow down instead
    max_joint_speed_rad_s: float = 0.3
    deadzone: float = 0.15
    # left stick snaps to pure x or y within this angle of an axis (diagonals stay possible)
    snap_deg: float = 20.0
    gripper_open_m: float = 0.085
    # A closes to this gap: set it a few mm below the object's width, so the WSG recognises
    # the part (HOLDING, force kept) instead of reporting it as blocked
    grasp_gap_m: float = 0.0


def _shape(x, y, deadzone):
    """Radial deadzone, rescaled so motion starts from zero at its edge."""
    r = np.hypot(x, y)
    if r < deadzone:
        return 0.0, 0.0
    k = min(1.0, (r - deadzone) / (1.0 - deadzone)) / r
    return x * k, y * k


def stick_twist(lx, ly, rx, ry, deadzone, snap):
    """Unit twist [x, y, z, roll, pitch, yaw] from raw stick axes (SDL: +x right, +y down)."""
    lx, ly = _shape(lx, ly, deadzone)
    rx, ry = _shape(rx, ry, deadzone)
    # z and yaw share the right stick: only the dominant one, so a slanted thumb won't twist
    rx, ry = (0.0, ry) if abs(ry) >= abs(rx) else (rx, 0.0)
    # the left stick snaps to pure x or y near an axis; diagonals in between stay possible
    if abs(lx) < snap * abs(ly):
        lx = 0.0
    elif abs(ly) < snap * abs(lx):
        ly = 0.0
    return np.array([-ly, -lx, -ry, 0.0, 0.0, -rx])


class GamepadReader:
    """First gamepad through SDL's GameController mapping, so any known pad reads the same."""

    def __init__(self, deadzone: float = 0.15, snap_deg: float = 20.0):
        self.deadzone, self.snap = deadzone, np.tan(np.radians(snap_deg))
        self.pad = self._pg = None

    def open(self) -> str:
        """Open the first gamepad and return its name."""
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        import pygame  # lazy: every LeRobot CLI imports this plugin, only teleop needs pygame
        from pygame._sdl2 import controller

        pygame.init()
        controller.init()
        if controller.get_count() == 0 or not controller.is_controller(0):
            raise ConnectionError("no known gamepad found (plugged in? see 'ls /dev/input/js*')")
        self._pg, self.pad = pygame, controller.Controller(0)
        return self.pad.name

    def close(self) -> None:
        if self._pg is not None:
            self._pg.quit()
        self.pad = self._pg = None

    def read(self) -> dict:
        """Deadman, precision, close, open, and the unit twist [x, y, z, roll, pitch, yaw]."""
        pg, pad = self._pg, self.pad
        pg.event.pump()

        def axis(a):
            return pad.get_axis(a) / 32767.0

        def button(b):
            return int(pad.get_button(b))

        twist = stick_twist(
            axis(pg.CONTROLLER_AXIS_LEFTX),
            axis(pg.CONTROLLER_AXIS_LEFTY),
            axis(pg.CONTROLLER_AXIS_RIGHTX),
            axis(pg.CONTROLLER_AXIS_RIGHTY),
            self.deadzone,
            self.snap,
        )
        twist[3] = button(pg.CONTROLLER_BUTTON_DPAD_RIGHT) - button(pg.CONTROLLER_BUTTON_DPAD_LEFT)
        twist[4] = button(pg.CONTROLLER_BUTTON_DPAD_UP) - button(pg.CONTROLLER_BUTTON_DPAD_DOWN)
        return {
            "deadman": bool(button(pg.CONTROLLER_BUTTON_LEFTSHOULDER)),
            "precision": bool(button(pg.CONTROLLER_BUTTON_RIGHTSHOULDER)),
            "close": bool(button(pg.CONTROLLER_BUTTON_A)),
            "open": bool(button(pg.CONTROLLER_BUTTON_B)),
            "twist": twist,
        }


class Lwa4pGamepad(Teleoperator):
    """Pose servo: the pad moves a desired TCP pose and IK tracks it exactly (axes independent)."""

    config_class = Lwa4pGamepadConfig
    name = "lwa4p_gamepad"

    def __init__(self, config: Lwa4pGamepadConfig):
        super().__init__(config)
        self.config = config
        self.bridge = BridgeClient(config.host, config.port)
        self.pad = GamepadReader(config.deadzone, config.snap_deg)
        self._last_warn = 0.0
        self.chain = None
        self.q = self.target = self.gap = None
        self.t_prev = 0.0

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in MOTORS}

    @cached_property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.pad.pad is not None and self.bridge.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        """Nothing to calibrate."""

    def configure(self) -> None:
        """Nothing to configure."""

    def connect(self, calibrate: bool = True) -> None:
        name = self.pad.open()
        self.bridge.connect()
        urdf = self.bridge.call("urdf")["urdf"]
        if not urdf:
            raise ConnectionError("bridge has no robot_description yet")
        self.chain = Chain(urdf, [f"{j}_joint" for j in ARM_JOINTS], self.config.tip_link)
        print(f"{name}\n{_HELP}")

    def get_action(self) -> dict:
        c, now = self.config, time.monotonic()
        dt, self.t_prev = min(now - self.t_prev, 0.2), now
        pad = self.pad.read()
        state = self.bridge.call("state")
        measured = np.array(state["q"])
        # lead is measured against where the drives are commanded to, not where they lag behind
        ref = np.array(state["q_cmd"]) if state.get("q_cmd") is not None else measured
        tip = self.chain.fk(ref)[0]
        # anchor only at start and after a refused target: keeping the commanded pose otherwise
        # stops the drives' gravity sag from ratcheting the target down on every LB press
        if self.target is None or state["error"].startswith("collision"):
            self.q, self.target = measured, tip
        if pad["deadman"] and np.any(pad["twist"]):
            scale = c.precision_scale if pad["precision"] else 1.0
            step = pad["twist"] * np.repeat([c.linear_speed_m_s, c.angular_speed_rad_s], 3)
            step *= scale * dt
            nxt = rotate(step[3:], self.target)
            nxt[:3, 3] += step[:3]
            lead = np.linalg.norm(nxt[:3, 3] - tip[:3, 3]), np.linalg.norm(rotation_error(nxt, tip))
            # the arm isn't following (drives disabled, blocked): don't wind up
            if lead[0] <= c.max_lead_m and lead[1] <= c.max_lead_rad:
                q = self.chain.ik(nxt, self.q)
                fastest = np.abs(q - self.q).max() / max(dt, 1e-3)
                if fastest > c.max_joint_speed_rad_s:
                    k = c.max_joint_speed_rad_s / fastest
                    nxt = rotate(step[3:] * k, self.target)
                    nxt[:3, 3] += step[:3] * k
                    q = self.chain.ik(nxt, self.q)
                why = self._unsafe(q, nxt)
                if why:
                    if now - self._last_warn > 2.0:
                        print(f"teleop: stopped, {why}", flush=True)
                        self._last_warn = now
                else:
                    self.target, self.q = nxt, q
        if self.gap is None:  # snap to open/grasp: a measured width would be sent as a plain move
            opened = state["gripper"] > c.gripper_open_m - 0.01
            self.gap = c.gripper_open_m if opened else c.grasp_gap_m
        if pad["close"]:
            self.gap = c.grasp_gap_m
        elif pad["open"]:
            self.gap = c.gripper_open_m
        action = {f"{j}.pos": float(v) for j, v in zip(ARM_JOINTS, self.q)}
        action["gripper.pos"] = float(self.gap)
        return action

    def _unsafe(self, q, target):
        """Why the IK solution `q` would leave the requested line, or '' if it doesn't."""
        c = self.config
        if np.linalg.norm(self.chain.fk(q)[0][:3, 3] - target[:3, 3]) > c.max_ik_error_m:
            return "the pose is out of reach or at a joint limit"
        if np.any(q <= self.chain.lower + 0.051) or np.any(q >= self.chain.upper - 0.051):
            return "a joint is at its limit"
        if self.chain.manipulability(q) < c.min_manipulability:
            return "too close to a singular pose (move back or rotate the wrist)"
        return ""

    def send_feedback(self, feedback: dict) -> None:
        """No force feedback on a gamepad."""

    def disconnect(self) -> None:
        self.bridge.close()
        self.pad.close()
