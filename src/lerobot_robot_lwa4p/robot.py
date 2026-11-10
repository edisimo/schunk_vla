"""LeRobot robot for the Schunk LWA 4P + WSG 50, driven through the lwa4p_vla_bridge."""

import logging
import time
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
from lerobot.cameras import CameraConfig, DepthCamera, make_cameras_from_configs
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots import Robot, RobotConfig

from .bridge_client import ArmFault, BridgeClient
from .smoothing import JointSmoother

logger = logging.getLogger(__name__)

ARM_JOINTS = [f"arm_{i}" for i in range(1, 7)]
MOTORS = [*ARM_JOINTS, "gripper"]


def depth_to_rgb(depth_mm: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    """Depth (H, W, 1) in mm as a 3-channel uint8 image: near = bright, no reading = black.

    VLA vision encoders take RGB only; this lets SmolVLA and pi0.5 use depth unchanged.
    """
    d = depth_mm[..., 0].astype(np.float32)
    v = np.clip((far_m * 1000.0 - d) / ((far_m - near_m) * 1000.0), 0.0, 1.0) * 255.0
    v[d <= 0] = 0.0
    return np.repeat(v.astype(np.uint8)[..., None], 3, axis=2)


def _wrist_camera() -> dict[str, CameraConfig]:
    return {
        "wrist": RealSenseCameraConfig(
            serial_number_or_name="Intel RealSense D435",
            fps=30,
            width=640,
            height=480,
            use_depth=True,
        )
    }


@RobotConfig.register_subclass("lwa4p")
@dataclass(kw_only=True)
class Lwa4pConfig(RobotConfig):
    """Bridge address, cameras (wrist D435 with depth by default), depth range, gripper gap,
    and conservative joint-stream limits, which still require validation on the real supply."""

    host: str = "127.0.0.1"
    port: int = 5555
    cameras: dict[str, CameraConfig] = field(default_factory=_wrist_camera)
    gripper_open_m: float = 0.085
    depth_near_m: float = 0.1
    depth_far_m: float = 1.0
    max_velocity_rad_s: float = 0.45
    max_acceleration_rad_s2: float = 0.3
    max_jerk_rad_s3: float = 2.0
    max_target_step_rad: float = 0.2


class Lwa4p(Robot):
    """Joint-space robot: state and actions are 6 joint angles (rad) and the finger gap (m)."""

    config_class = Lwa4pConfig
    name = "lwa4p"
    supports_command_hold = True

    def __init__(self, config: Lwa4pConfig):
        super().__init__(config)
        self.config = config
        self.bridge = BridgeClient(config.host, config.port)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.smoother = JointSmoother(
            len(ARM_JOINTS),
            config.max_velocity_rad_s,
            config.max_acceleration_rad_s2,
            config.max_jerk_rad_s3,
        )
        self._last_warn = 0.0
        self._faulted = False
        self._ip = False

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        cams = {}
        for key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                cams[key] = (cam.height, cam.width, 3)
            if isinstance(cam, DepthCamera) and cam.use_depth:
                cams[f"{key}_depth"] = (cam.height, cam.width, 3)
        return {**self.action_features, **cams}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in MOTORS}

    @property
    def is_connected(self) -> bool:
        return self.bridge.is_connected and all(c.is_connected for c in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        """Nothing to calibrate: the drives report absolute positions."""

    def configure(self) -> None:
        """Nothing to configure: limits and safety live in the driver and the bridge."""

    def connect(self, calibrate: bool = True) -> None:
        self.bridge.connect()
        state = self._state()
        self._ip = state.get("interpolated_position", False)
        self._faulted = False
        if state["error"]:
            logger.warning(f"bridge reports: {state['error']}")
        self.smoother.reset(state["q"], time.monotonic())
        for cam in self.cameras.values():
            cam.connect()
        logger.info(f"{self} connected")

    def _state(self) -> dict:
        state = self.bridge.call("state")
        if state["q"] is None or state["gripper"] is None:
            raise RuntimeError("bridge has no joint_states yet (driver and gripper running?)")
        return state

    def get_observation(self) -> dict:
        state = self._state()
        obs = {f"{j}.pos": q for j, q in zip(ARM_JOINTS, state["q"])}
        obs["gripper.pos"] = state["gripper"]
        for key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                obs[key] = cam.read_latest()
            if isinstance(cam, DepthCamera) and cam.use_depth:
                obs[f"{key}_depth"] = depth_to_rgb(
                    cam.read_latest_depth(), self.config.depth_near_m, self.config.depth_far_m
                )
        return obs

    def send_action(self, action: dict) -> dict:
        """Send a continuous plan, retaining the accepted trajectory when a target is refused."""
        if self._faulted:
            raise ArmFault("arm remains stopped; recover the drive before restarting teleop")
        goal = [float(action[f"{j}.pos"]) for j in ARM_JOINTS]
        gap = min(max(float(action["gripper.pos"]), 0.0), self.config.gripper_open_m)
        now = time.monotonic()
        stamp = time.time()
        accepted = self.smoother.snapshot()
        plan = self.smoother.step(
            goal, now, max_step=self.config.max_target_step_rad if self._ip else None
        )
        q = plan[0][1].tolist()
        ahead = [[t, p.tolist()] for t, p in plan]
        extra = {"smooth": self.smoother.preview(), "stamp": stamp} if self._ip else {}
        reply = self.bridge.call("act", q=q, gripper=gap, ahead=ahead, **extra)
        if reply.get("fault") or reply.get("error", "").startswith("arm goal rejected"):
            self._faulted = True
            raise ArmFault(reply.get("fault") or reply["error"])
        if not reply["ok"] or np.abs(np.asarray(reply["q"]) - plan[0][1]).max() > 1e-6:
            if self._ip:
                self.smoother.restore(accepted)
            else:
                self.smoother.reset(self._state()["q"], now)
            if not reply["ok"] and now - self._last_warn > 1.0:
                logger.warning(f"bridge refused the action: {reply['error']}")
                self._last_warn = now
        sent = {f"{j}.pos": v for j, v in zip(ARM_JOINTS, reply.get("q") or q)}
        sent["gripper.pos"] = gap
        return sent

    def hold(self) -> None:
        """Let an IP trajectory finish its validated stop without rewinding to measured feedback."""
        self.bridge.call("hold")
        if not self._ip:
            self.smoother.reset(self._state()["q"], time.monotonic())

    def disconnect(self) -> None:
        """Release clients and cameras without sending goals into a faulted controller."""
        if self.bridge.is_connected:
            try:
                if not self._faulted:
                    self.hold()
            finally:
                self.bridge.close()
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()
        logger.info(f"{self} disconnected")
