"""LeRobot plugin for the Schunk LWA 4P; importing it registers `lwa4p` and `lwa4p_gamepad`."""

from .robot import Lwa4p, Lwa4pConfig
from .teleop import Lwa4pGamepad, Lwa4pGamepadConfig

__all__ = ["Lwa4p", "Lwa4pConfig", "Lwa4pGamepad", "Lwa4pGamepadConfig"]
