#!/usr/bin/env bash
# Run ON THE CART (elroy), where the recordings are. Trains a policy on a
# recorded dataset - on Hugging Face Jobs, not on this machine.
#
#   ./config/cart-train.sh <repo_id>
#   ./config/cart-train.sh carlkesselman/xlerobot-pick          newest recording of that name
#   ./config/cart-train.sh carlkesselman/xlerobot-pick_20261026_141500
#   FLAVOR=a100-large STEPS=50000 ./config/cart-train.sh me/ds
#   FLAVOR=local ./config/cart-train.sh me/ds                    train here (slowly)
#
# Everything after the repo id passes through to lerobot-train:
#   ./config/cart-train.sh me/ds --policy.chunk_size=50
#
# WHY FROM THE CART
#
# The dataset is here, on elroy's disk - cart-record.sh keeps it local by
# default. lerobot-train with --job.target pushes a local-only dataset to a
# PRIVATE Hub repo before it submits, so recording with PUSH_TO_HUB=true is
# not needed first, and nothing has to be copied to another machine. The
# job itself runs on HF's GPU; this machine only uploads and streams logs.
#
# Needs `hf auth login` here once, and an HF account that can run Jobs
# (paid, with credit). `hf jobs hardware` lists flavors and prices.
#
# WHICH LEROBOT TRAINS
#
# The job runs in HF's huggingface/lerobot-gpu image - upstream lerobot,
# not this fork. That is fine for training: the fork changes cameras,
# display and shutdown, none of which training touches. If the job fails
# reading the dataset, a format version moved between the fork and the
# image: JOB_IMAGE=huggingface/lerobot-gpu:<tag> pins an older one.
#
# DEFAULTS THIS SCRIPT SETS (each overridable by passing it again)
#   POLICY=act          ACT: single task, small, and runs on the Orin.
#   FLAVOR=a10g-small   one 24 GB GPU; ACT at batch 8 needs ~1 GB of it.
#   STEPS=100000        lerobot's ACT default.
#   BATCH_SIZE=8
#   --policy.repo_id    <user>/<policy>_<dataset name, unstamped>. The
#                       trained model is pushed there when the job ends.
#   DETACH=false        stream the job's log here; true submits and returns.
#                       Inside tmux either way: an SSH drop kills the
#                       stream, though not the job.
#
# Then, on this machine:
#   ./config/cart-eval.sh <policy repo> "<the same task text>"

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

if [[ $# -lt 1 || "$1" == -* ]]; then
  sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
  exit 1
fi

REPO_ID="$1"; shift
if [[ "$REPO_ID" != */* ]]; then
  echo "error: repo id must look like <user>/<name>, got '$REPO_ID'" >&2
  exit 2
fi

POLICY="${POLICY:-act}"
FLAVOR="${FLAVOR:-a10g-small}"
STEPS="${STEPS:-100000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
DATA_ROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}"

# lerobot-record stamps every recording with _YYYYMMDD_HHMMSS. A plain name
# means the newest local one; an exact name, or one only on the Hub, is
# used as given.
if [[ ! -f "$DATA_ROOT/$REPO_ID/meta/info.json" ]]; then
  newest="$(ls -dt "$DATA_ROOT/${REPO_ID}"_* 2>/dev/null | head -1 || true)"
  if [[ -n "$newest" ]]; then
    REPO_ID="${REPO_ID%%/*}/$(basename "$newest")"
  fi
fi

# The model is named after the dataset without its stamp, so a retrain on
# a newer recording of the same task lands in the same model repo.
DATASET_NAME="${REPO_ID#*/}"
DATASET_NAME="$(sed -E 's/_[0-9]{8}_[0-9]{6}$//' <<<"$DATASET_NAME")"
POLICY_REPO="${POLICY_REPO:-${REPO_ID%%/*}/${POLICY}_${DATASET_NAME}}"
JOB_NAME="${POLICY}_${DATASET_NAME}"

if [[ -f "$DATA_ROOT/$REPO_ID/meta/info.json" ]]; then
  where="local, $DATA_ROOT/$REPO_ID (pushed PRIVATE if not on the Hub yet)"
else
  where="not local - must already be on the Hub"
fi

echo "dataset : $REPO_ID"
echo "          $where"
echo "policy  : $POLICY -> $POLICY_REPO"
echo "train on: $FLAVOR   steps $STEPS   batch $BATCH_SIZE"
echo

JOB_ARGS=()
if [[ "$FLAVOR" == local ]]; then
  warn_if_cpu_torch
  OUT="$REPO/outputs/train/$JOB_NAME"
  JOB_ARGS=(--output_dir="$OUT")
  echo "output  : $OUT"
else
  JOB_ARGS=(--job.target="$FLAVOR" --job.detach="${DETACH:-false}")
  [[ -n "${JOB_IMAGE:-}" ]] && JOB_ARGS+=(--job.image="$JOB_IMAGE")
fi

exec "${RUN[@]}" lerobot-train \
  --dataset.repo_id="$REPO_ID" \
  --policy.type="$POLICY" \
  --policy.repo_id="$POLICY_REPO" \
  --policy.device=cuda \
  --job_name="$JOB_NAME" \
  --steps="$STEPS" \
  --batch_size="$BATCH_SIZE" \
  "${JOB_ARGS[@]}" \
  "$@"
