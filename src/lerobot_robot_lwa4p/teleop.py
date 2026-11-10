"""Gamepad teleoperation of the LWA 4P: 6-DoF TCP velocity -> differential IK -> joint targets."""

import os
import time
from dataclasses import dataclass
from functools import cached_property

import numpy as np
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig

from .bridge_client import BridgeClient
from .kinematics import Chain
from .robot import ARM_JOINTS, MOTORS

# Default indices are SDL's for Xbox-style pads on Linux; override them for other layouts.
_HELP = """LWA 4P gamepad: hold LB to move (release = stop), also hold RB for precision speed
  left stick      TCP x / y          right stick   z (vertical), yaw (horizontal)
  d-pad           roll / pitch       A / B         close / open gripper"""


@TeleoperatorConfig.register_subclass("lwa4p_gamepad")
@dataclass(kw_only=True)
class Lwa4pGamepadConfig(TeleoperatorConfig):
    """Bridge address, speeds and the gamepad layout."""

    host: str = "127.0.0.1"
    port: int = 5555
    tip_link: str = "gripper_tcp"
    linear_speed_m_s: float = 0.05
    angular_speed_rad_s: float = 0.3
    precision_scale: float = 0.25
    deadzone: float = 0.15
    gripper_open_m: float = 0.085
    axis_left_x: int = 0
    axis_left_y: int = 1
    axis_right_x: int = 3
    axis_right_y: int = 4
    button_deadman: int = 4
    button_precision: int = 5
    button_close: int = 0
    button_open: int = 1


class Lwa4pGamepad(Teleoperator):
    """Integrates gamepad twists from the last commanded joint target, re-anchored on release."""

    config_class = Lwa4pGamepadConfig
    name = "lwa4p_gamepad"

    def __init__(self, config: Lwa4pGamepadConfig):
        super().__init__(config)
        self.config = config
        self.bridge = BridgeClient(config.host, config.port)
        self.chain = self.pad = None
        self.q = self.gap = None
        self.t_prev = 0.0

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in MOTORS}

    @cached_property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.pad is not None and self.bridge.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        """Nothing to calibrate."""

    def configure(self) -> None:
        """Nothing to configure."""

    def connect(self, calibrate: bool = True) -> None:
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        import pygame  # lazy: every LeRobot CLI imports this plugin, only teleop needs pygame

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise ConnectionError("no gamepad found (plugged in? see 'ls /dev/input/js*')")
        self.pad = pygame.joystick.Joystick(0)
        self.bridge.connect()
        urdf = self.bridge.call("urdf")["urdf"]
        if not urdf:
            raise ConnectionError("bridge has no robot_description yet")
        self.chain = Chain(urdf, [f"{j}_joint" for j in ARM_JOINTS], self.config.tip_link)
        print(f"{self.pad.get_name()}\n{_HELP}")

    def _axis(self, i):
        v = self.pad.get_axis(i)
        return 0.0 if abs(v) < self.config.deadzone else v

    def get_action(self) -> dict:
        import pygame

        pygame.event.pump()
        c, now = self.config, time.monotonic()
        dt, self.t_prev = min(now - self.t_prev, 0.2), now
        state = self.bridge.call("state")
        if self.q is None or not self.pad.get_button(c.button_deadman):
            self.q = np.array(state["q"])
        else:
            hat_x, hat_y = self.pad.get_hat(0) if self.pad.get_numhats() else (0, 0)
            scale = c.precision_scale if self.pad.get_button(c.button_precision) else 1.0
            twist = scale * np.array(
                [
                    -self._axis(c.axis_left_y) * c.linear_speed_m_s,
                    -self._axis(c.axis_left_x) * c.linear_speed_m_s,
                    -self._axis(c.axis_right_y) * c.linear_speed_m_s,
                    hat_x * c.angular_speed_rad_s,
                    hat_y * c.angular_speed_rad_s,
                    -self._axis(c.axis_right_x) * c.angular_speed_rad_s,
                ]
            )
            if np.any(twist):
                self.q = self.chain.ik_step(self.q, twist, dt)
        if self.gap is None:
            self.gap = state["gripper"]
        if self.pad.get_button(c.button_close):
            self.gap = 0.0
        elif self.pad.get_button(c.button_open):
            self.gap = c.gripper_open_m
        action = {f"{j}.pos": float(v) for j, v in zip(ARM_JOINTS, self.q)}
        action["gripper.pos"] = float(self.gap)
        return action

    def send_feedback(self, feedback: dict) -> None:
        """No force feedback on a gamepad."""

    def disconnect(self) -> None:
        import pygame

        self.bridge.close()
        self.pad = None
        pygame.quit()
