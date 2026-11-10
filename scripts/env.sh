# Source in every terminal that talks to the robot: . scripts/env.sh
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# comma-decimal locales (e.g. sk_SK) make ROS 2 read YAML doubles as strings
export LC_NUMERIC=en_US.UTF-8
unset _ROOT
