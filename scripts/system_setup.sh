#!/usr/bin/env bash
# System changes for the real robot, and their undo: CAN udev rule + can0/can1 up at 500 kbit/s,
# RealSense udev rule, and two apt packages. Touches only the can0/can1 SocketCAN links;
# Ethernet, Wi-Fi, IPs and routes are untouched.
#   scripts/system_setup.sh up       # install everything, bring can0/can1 up now
#   scripts/system_setup.sh down     # revert all of it
#   scripts/system_setup.sh status
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CAN_RULE=/etc/udev/rules.d/80-can.rules
RS_RULE=/etc/udev/rules.d/99-realsense-d435.rules
IFACES="can0 can1"
PKGS="ros-humble-moveit-ros-perception can-utils"

reload_udev() {
  sudo udevadm control --reload
  sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086
}

case "$1" in
  up)
    # shellcheck disable=SC2086
    sudo apt-get install -y $PKGS
    sudo cp "$ROOT/third_party/lwa4p_driver/80-can.rules" "$CAN_RULE"
    sudo cp "$ROOT/config/99-realsense-d435.rules" "$RS_RULE"
    reload_udev
    for i in $IFACES; do
      sudo ip link set "$i" down
      sudo ip link set "$i" type can bitrate 500000
      sudo ip link set "$i" txqueuelen 20
      sudo ip link set "$i" up
    done ;;
  down)
    for i in $IFACES; do
      sudo ip link set "$i" down || true
      sudo ip link set "$i" txqueuelen 10 || true
    done
    sudo rm -f "$CAN_RULE" "$RS_RULE"
    reload_udev
    # shellcheck disable=SC2086
    sudo apt-get remove -y $PKGS ;;
  status) ;;
  *) echo "usage: $0 up|down|status" >&2; exit 1 ;;
esac
for r in "$CAN_RULE" "$RS_RULE"; do ls -l "$r" 2>/dev/null || echo "$r: not installed"; done
ip -br link | grep can || echo "no CAN interfaces"
# shellcheck disable=SC2086
dpkg-query -W -f='${Package} ${db:Status-Status}\n' $PKGS 2>/dev/null || true
