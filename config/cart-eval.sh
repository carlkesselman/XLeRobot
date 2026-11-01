#!/usr/bin/env bash
# Run ON THE CART (elroy, the Orin). Runs a trained policy on the robot.
#
#   ./config/cart-eval.sh <policy> "<task description>"
#   BASE=none ./config/cart-eval.sh carlkesselman/act_xlerobot-pick "Pick the block, drop it in the box"
#   ./config/cart-eval.sh --record me/act_ds "task"     scored episodes, saved as a dataset
#   ./config/cart-eval.sh --leaders --record me/act_ds "task"
#   ./config/cart-eval.sh --watch me/act_ds "task"      video to the rerun viewer
#
# <policy> is a Hub repo id or a local pretrained_model directory.
# Flags go before the policy; everything after the task passes through to
# lerobot-rollout.
#
# THE POLICY DRIVES THE ARMS. Clear the workspace, keep a hand near the
# power switch, and run with the cart on blocks or BASE=none.
#
# MATCH THE RECORDING
#
# The robot must have the features the policy was trained on: the same
# BASE= as the dataset (a policy trained with BASE=none expects no .vel
# columns), the same cameras by name, the same resolution. lerobot-rollout
# checks the names; nothing checks that the cameras still point where they
# did. The task text should be the one recorded.
#
# TWO MODES
#
#   (default)   strategy base: the policy runs for DURATION seconds
#               (default 60; 0 = until Ctrl-C), records nothing. For a
#               first look.
#   --record    strategy episodic: NUM_EPISODES (10) episodes of
#               EPISODE_TIME_S (30), RESET_TIME_S (15) between, saved to
#               <user>/eval_<policy name> - lerobot stamps the date on, as
#               with recordings. Right arrow = episode done, left = redo,
#               escape = stop and save. Count the successes afterwards;
#               the dataset is the evidence.
#
# THE LEADERS
#
# Without --leaders the teleop block is dropped from the config, so rosie
# need not be running: between episodes the arms return to the pose they
# had at startup, and you reset the scene by hand.
#
# With --leaders the cart connects to rosie as cart-teleop.sh does (start
# operator-leader-host.sh FIRST), and between episodes the followers move
# to the leaders - so the operator can put the arms anywhere for the next
# start. Hold the leaders near the followers when a reset begins.
#
# DEFAULTS THIS SCRIPT SETS (each overridable by passing it again)
#   --dataset.push_to_hub=false        PUSH_TO_HUB=true to upload
#   --dataset.rgb_encoder.vcodec=h264  as cart-record.sh, for the same reason
#   --dataset.num_image_writer_processes=2
#   --play_sounds=false

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
source "$(dirname "${BASH_SOURCE[0]}")/_video.sh"

warn_if_cpu_torch

CONFIG="$REPO/config/cart-remote.yaml"
parse_watch_all "$@" || :
set -- "${REMAINING[@]+"${REMAINING[@]}"}"
CONFIG="${CONFIG_OVERRIDE:-$CONFIG}"

RECORD=false
LEADERS=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --record)  RECORD=true; shift ;;
    --leaders) LEADERS=true; shift ;;
    *) break ;;
  esac
done

if [[ $# -lt 2 || "$1" == -* ]]; then
  sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
  exit 1
fi

POLICY_PATH="$1"; shift
TASK="$1"; shift

# A local directory is used as given; anything else must be a Hub repo id.
if [[ ! -d "$POLICY_PATH" && "$POLICY_PATH" != */* ]]; then
  echo "error: policy must be a <user>/<name> repo id or a local directory, got '$POLICY_PATH'" >&2
  exit 2
fi

# The shared yaml is a teleoperate config. Its top-level fps: is valid here
# too (RolloutConfig has one), but its teleop: block is only wanted with
# --leaders - without it, rollout would wait 10 s for rosie and give up.
EVAL_CONFIG="$(mktemp --suffix=.yaml)"
trap 'rm -f "$EVAL_CONFIG"' EXIT
if $LEADERS; then
  cp "$CONFIG" "$EVAL_CONFIG"
else
  awk '/^teleop:/ {skip=1; next} /^[a-z_]+:/ {skip=0} !skip' "$CONFIG" > "$EVAL_CONFIG"
fi

require_dev "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058116-if00" "left follower (bus1)"
require_dev "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B91028673-if00" "right follower (bus2)"

echo "config  : $CONFIG"
base_report "$CONFIG"
echo "policy  : $POLICY_PATH"
echo "task    : $TASK"
if $LEADERS; then
  OPERATOR="$(awk -F'[ #]+' '/remote_ip:/ {print $3; exit}' "$CONFIG")"
  echo "leaders : $OPERATOR:5557"
  if ! ping -c1 -W2 "$OPERATOR" >/dev/null 2>&1; then
    echo "warning: $OPERATOR does not answer ping - is operator-leader-host.sh running?" >&2
  fi
else
  echo "leaders : none (--leaders to reset through them)"
fi

MODE_ARGS=()
if $RECORD; then
  NUM_EPISODES="${NUM_EPISODES:-10}"
  EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
  RESET_TIME_S="${RESET_TIME_S:-15}"
  # Named after the policy repo. A local directory is usually
  # .../checkpoints/last/pretrained_model, which names nothing - say it.
  if [[ -z "${EVAL_REPO:-}" ]]; then
    if [[ -d "$POLICY_PATH" ]]; then
      echo "error: with a local policy directory, set EVAL_REPO=<user>/eval_<name>" >&2
      exit 2
    fi
    EVAL_REPO="${POLICY_PATH%%/*}/eval_${POLICY_PATH#*/}"
  fi
  DATA_ROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}"
  echo "mode    : record, $NUM_EPISODES x ${EPISODE_TIME_S}s (reset ${RESET_TIME_S}s)"
  echo "saved to: $DATA_ROOT/${EVAL_REPO}_<date>_<time>"
  MODE_ARGS=(
    --strategy.type=episodic
    --dataset.repo_id="$EVAL_REPO"
    --dataset.single_task="$TASK"
    --dataset.fps=30
    --dataset.num_episodes="$NUM_EPISODES"
    --dataset.episode_time_s="$EPISODE_TIME_S"
    --dataset.reset_time_s="$RESET_TIME_S"
    --dataset.push_to_hub="${PUSH_TO_HUB:-false}"
    --dataset.rgb_encoder.vcodec="${VCODEC:-h264}"
    --dataset.num_image_writer_processes=2
  )
else
  DURATION="${DURATION:-60}"
  echo "mode    : run ${DURATION}s, record nothing (--record to score episodes)"
  MODE_ARGS=(--strategy.type=base --duration="$DURATION")
fi
echo

rc=0
"${RUN[@]}" lerobot-rollout \
  --config_path="$EVAL_CONFIG" \
  "${BASE_ARGS[@]+"${BASE_ARGS[@]}"}" \
  --policy.path="$POLICY_PATH" \
  --task="$TASK" \
  --play_sounds=false \
  "${MODE_ARGS[@]}" \
  "${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"}" \
  "$@" || rc=$?

if $RECORD; then
  newest="$(ls -dt "$DATA_ROOT/${EVAL_REPO}"_* 2>/dev/null | head -1 || true)"
  echo
  if [[ -n "$newest" ]]; then
    echo "dataset : $newest"
    echo "check it: ./config/dataset-check.sh $EVAL_REPO"
  else
    echo "no dataset found under $DATA_ROOT/${EVAL_REPO}_*"
  fi
fi
exit $rc
