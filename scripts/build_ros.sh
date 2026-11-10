#!/usr/bin/env bash
# Apply our patches to the lwa4p_driver submodule (idempotent) and build the ROS 2 workspace
# (driver + bridge) into build/ and install/ at the repo root.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DRV="$ROOT/third_party/lwa4p_driver"
git -C "$ROOT" submodule update --init
for p in "$ROOT"/patches/lwa4p_driver/*.patch; do
  if git -C "$DRV" apply --reverse --check "$p" 2>/dev/null; then
    echo "already applied: $(basename "$p")"
  else
    git -C "$DRV" apply "$p" && echo "applied: $(basename "$p")"
  fi
done
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$ROOT"
colcon build --symlink-install --base-paths third_party/lwa4p_driver/src ros
