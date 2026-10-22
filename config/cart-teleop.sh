#!/usr/bin/env bash
# Run ON THE CART (elroy, the Orin). Drives the robot from the leaders at
# the operator station, recording nothing.
#
#   ./config/cart-teleop.sh                 the full robot, video to the Mac's
#                                           rerun viewer IF it is running
#   ./config/cart-teleop.sh --no-watch      no video
#   ./config/cart-teleop.sh --watch         video, and warn if no viewer answers
#   ./config/cart-teleop.sh --watch=web     video as Foxglove, any browser
#   BASE=none ./config/cart-teleop.sh       arms and head only (no wheels)
#   BASE=omniwheels ...                     the three servo wheels on bus 2
#   BASE=odrive ...                         the two brushless wheels (default)

#
# Start ./config/operator-leader-host.sh on the Pi FIRST.
#
# Everything the robot has is local to this machine. Only the leaders are
# remote, and only their actions cross the wifi.

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
source "$(dirname "${BASH_SOURCE[0]}")/_video.sh"

warn_if_cpu_torch

CONFIG="$REPO/config/cart-remote.yaml"
CONFIG="${CONFIG_OVERRIDE:-$CONFIG}"

require_dev "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058116-if00" "left follower (bus1)"
require_dev "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A68009991-if00" "right follower (bus2)"

OPERATOR="$(awk -F'[ #]+' '/remote_ip:/ {print $3; exit}' "$CONFIG")"
echo "config  : $CONFIG"
base_report "$CONFIG"
echo "leaders : $OPERATOR:5557"
if ! ping -c1 -W2 "$OPERATOR" >/dev/null 2>&1; then
  echo "warning: $OPERATOR does not answer ping - is operator-leader-host.sh running?" >&2
fi

# Video is on by default here, but only if the viewer answers: rerun
# pointed at nothing back-pressures into the control loop. See _video.sh.
watch_by_default "$@"
set -- "${REMAINING[@]+"${REMAINING[@]}"}"
echo

exec "${RUN[@]}" lerobot-teleoperate --config_path="$CONFIG" \
  "${BASE_ARGS[@]+"${BASE_ARGS[@]}"}" \
  "${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"}" "$@"
