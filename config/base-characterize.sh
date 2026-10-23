#!/usr/bin/env bash
# Run ON THE CART (elroy). First-principles characterization and tuning of the
# two-wheel ODrive base - see config/base_characterize.py for the stages:
#
#   ./config/base-characterize.sh blocks            wheels OFF the ground
#   ./config/base-characterize.sh floor             on the floor, ~2 m clear both ways
#   ./config/base-characterize.sh design            no motion: the settings, with reasons
#   ./config/base-characterize.sh step --candidates on the floor: validate and pick
#
# On the drill pack. The model and its reasoning: claude/xlerobot-base-performance.md
# in the XLeRobot project.

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

if [[ "${1:-}" != "design" ]] && ! lsusb 2>/dev/null | grep -q "1209:0d3"; then
  echo "error: no ODrive on USB (lsusb shows no 1209:0d3x). Pack on, board powered?" >&2
  exit 1
fi

exec "${RUN[@]}" python "$REPO/config/base_characterize.py" "$@"
