#!/usr/bin/env bash
# The ODESC on the bench: report, apply the bus limits, or watch it through
# a power cut, and bring up the motors. See config/odrive_check.py.
#
#   ./config/odrive-check.sh            report config vs the cart's limits
#   ./config/odrive-check.sh --apply    write them, save, reboot, verify
#   ./config/odrive-check.sh --watch    the XH-M609 relay-drop test
#
# Motor bring-up, in this order (see odrive_check.py):
#   --halls                       pole pairs, by turning each wheel once by hand
#   --setup-motors --pole-pairs N hub-motor config, saved
#   --calibrate [--axis 0|1]      DRILL PACK, WHEELS UP: beep, then a slow turn
#   --spin --axis 0|1             closed-loop velocity check, one wheel

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

if ! lsusb 2>/dev/null | grep -q "1209:0d3"; then
  echo "error: no ODrive on USB (lsusb shows no 1209:0d3x)." >&2
  echo "       The board's USB only works with DC power on its terminals." >&2
  echo "       0d31 would be DFU mode - also wrong." >&2
  exit 1
fi

exec "${RUN[@]}" python "$REPO/config/odrive_check.py" "$@"
