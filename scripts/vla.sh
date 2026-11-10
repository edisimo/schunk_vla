#!/usr/bin/env bash
# Run a command in the project venv (Python 3.12, LeRobot) without the ROS 2 Humble Python 3.10
# paths that a sourced ROS environment puts on PYTHONPATH.
#   scripts/vla.sh lerobot-record --robot.type=lwa4p ...
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH uv run --project "$ROOT" "$@"
