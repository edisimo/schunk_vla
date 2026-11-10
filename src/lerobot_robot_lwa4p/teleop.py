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
    linear_speed_m_s: float = 0.05
    angular_speed_rad_s: float = 0.3
    precision_scale: float = 0.25
    max_lead_m: float = 0.03
    max_lead_rad: float = 0.15
    deadzone: float = 0.15
    gripper_open_m: float = 0.085
    # A closes to this gap: set it a few mm below the object's width, so the WSG recognises
    # the part (HOLDING, force kept) instead of reporting it as blocked
    grasp_gap_m: float = 0.0


class GamepadReader:
    """First gamepad through SDL's GameController mapping, so any known pad reads the same."""

    def __init__(self, deadzone: float = 0.15):
        self.deadzone = deadzone
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
            v = pad.get_axis(a) / 32767.0
            return 0.0 if abs(v) < self.deadzone else v

        def button(b):
            return int(pad.get_button(b))

        twist = np.array(
            [
                -axis(pg.CONTROLLER_AXIS_LEFTY),
                -axis(pg.CONTROLLER_AXIS_LEFTX),
                -axis(pg.CONTROLLER_AXIS_RIGHTY),
                button(pg.CONTROLLER_BUTTON_DPAD_RIGHT) - button(pg.CONTROLLER_BUTTON_DPAD_LEFT),
                button(pg.CONTROLLER_BUTTON_DPAD_UP) - button(pg.CONTROLLER_BUTTON_DPAD_DOWN),
                -axis(pg.CONTROLLER_AXIS_RIGHTX),
            ]
        )
        return {
            "deadman": bool(button(pg.CONTROLLER_BUTTON_LEFTSHOULDER)),
            "precision": bool(button(pg.CONTROLLER_BUTTON_RIGHTSHOULDER)),
            "close": bool(button(pg.CONTROLLER_BUTTON_A)),
            "open": bool(button(pg.CONTROLLER_BUTTON_B)),
            "twist": twist,
        }


class Lwa4pGamepad(Teleoperator):
    """Pose servo: the pad moves a desired TCP pose, IK tracks it exactly (no drift between axes)."""

    config_class = Lwa4pGamepadConfig
    name = "lwa4p_gamepad"

    def __init__(self, config: Lwa4pGamepadConfig):
        super().__init__(config)
        self.config = config
        self.bridge = BridgeClient(config.host, config.port)
        self.pad = GamepadReader(config.deadzone)
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
        tip = self.chain.fk(measured)[0]
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
            # wait for an arm that can't keep up (velocity limits, drive lag)
            lead = np.linalg.norm(nxt[:3, 3] - tip[:3, 3]), np.linalg.norm(rotation_error(nxt, tip))
            if lead[0] <= c.max_lead_m and lead[1] <= c.max_lead_rad:
                self.target = nxt
                self.q = self.chain.ik(self.target, self.q)
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

    def send_feedback(self, feedback: dict) -> None:
        """No force feedback on a gamepad."""

    def disconnect(self) -> None:
        self.bridge.close()
        self.pad.close()
