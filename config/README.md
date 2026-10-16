# Hardware configs for Carl's XLeRobot

| doc | when |
|---|---|
| **`RUNNING.md`** | **every session — what to start, on which machine, in what order** |
| `ORIN-SETUP.md` | one-time bring-up of a cart machine |
| this file | what is here, and how the two repos fit together |
| `../lerobot/FORK.md` | what the lerobot fork changes and why |

Run everything from the XLeRobot checkout (`~/GitHub/XLeRobot`, the Mac's
`~/GitHub/Robots/XLeRobot`). Every script runs through `uv run --project` on
this workspace (see `_common.sh`), so no virtualenv needs activating.

## What is in this directory

**Run every session** (order and details in `RUNNING.md`):

| file | where | what |
|---|---|---|
| `operator-leader-host.sh` + `.yaml` | rosie | serves the leader arms and gamepad |
| `viewer.sh` | Mac | the rerun viewer, at the version the cart sends |
| `cart-teleop.sh` | elroy | drive the robot, record nothing |
| `cart-record.sh` | elroy | record a dataset |
| `cart-remote.yaml` | elroy | **the** robot: ports, cameras, base, plus the leader link. Everything on the cart reads its `robot:` block; `BASE=none\|omniwheels\|odrive` overrides the base per run |

**Check and diagnose** (all on elroy unless noted):

| file | what |
|---|---|
| `cart-selftest.sh` | the cart alone: rate, joints, a frame per camera; `--head --arms --base` to move things |
| `servo.sh scan\|power\|regs\|set-id\|recal` | the servo buses: who answers, voltage under load, every register, replace a servo, recalibrate one joint. Ports and ids in `servo_tools/hw.py` |
| `video.sh probe\|view\|synthetic` | a camera's buffer depth; the real cameras to the viewer; a synthetic feed for the network alone |
| `leader-link.sh tap\|soak` | the leader stream with no robot: which keys arrive; a graded N-minute link test |
| `dataset-check.sh` | read a recording back |
| `odrive-check.sh` | the ODESC: config, apply limits, watch through a power cut |
| `ms_clock.py` | **Mac**: millisecond clock for the glass-to-glass lag test |

**Setup and plumbing:** `sync.sh` (the right `uv sync` for this machine),
`_common.sh` / `_video.sh` (sourced by the scripts), `99-cameras-elroy.rules`
and `99-odrive-elroy.rules` (udev, see `ORIN-SETUP.md`).

## Which machine runs what

```
         CART                                 OPERATOR STATION
  elroy — Jetson Orin Nano                 rosie — Raspberry Pi 5
  ──────────────────────               ─────────────────────
  follower arms                           leader arms
  wrist cams + head RealSense             gamepad
  the base                                the rerun viewer
  the dataset                     actions
  the policy, later               <──────
                                  ──────>
                                   video
```

The GPU goes where the sensors and motors are, because that is where a
policy has to run: at full camera rate, with no network between seeing and
acting. So the Orin owns the robot, the cameras and the dataset, and the
only thing it does not have is the operator.

That makes the **leaders** the remote device, not the robot — the mirror
image of lekiwi. Two things cross the wifi and both tolerate loss: leader
actions one way, the operator's view the other. Camera frames go from USB
straight to disk without touching the network, so nothing can conflate or
drop them.

### The normal path

```
rosie   rerun --port 9876                        # viewer, first
rosie   ./config/operator-leader-host.sh         # leaders + gamepad
elroy   ./config/cart-teleop.sh --watch          # drive, record nothing
elroy   ./config/cart-record.sh --watch me/ds "task"
```

Run `cart-record.sh` from an SSH session opened *from the Pi*: lerobot's
episode keys (right arrow = end episode, left = re-record, escape = stop)
are read by the process on the cart, and it falls back to a terminal
listener when pynput cannot capture, so an SSH TTY puts them under the
operator's hands.

### The two settings that fail silently

The teleoperator and the robot must agree on arm key naming and on which
joints exist. Nothing checks this at startup and a mismatch does not
raise — the link comes up, the loop runs at the right rate, and the arms do
not move.

`operator-leader-host.yaml` must have `remap_arm_prefix: true` (XLerobot
filters with `startswith("left_arm_")`) and `emit_head`/`emit_base` true.
`operator-leader-host.sh` echoes all four on startup for this reason, and
the leader host reports **17 action keys** when they are right — 12 arm,
2 head, 3 base.

### Prerequisites

- **udev rules and calibration live on the Orin now.** `/dev/cam_left` and
  `/dev/cam_right` are keyed on `ENV{ID_PATH}`, and the port paths differ on
  the Orin — read the real ones with
  `udevadm info -q property /dev/video0 | grep ID_PATH`. Copy
  `~/.cache/huggingface/lerobot/calibration/` across as well; the leaders'
  half has to be on the Pi and the followers' half on the Orin.
- **JetPack torch on the Orin.** The fork routes ARM to PyPI's CPU wheels,
  which is wrong for the machine that will run the policy.
- **The rerun viewer on the Pi** for `--watch`. Check `rerun --help` for
  your version's listen flag.

## Layout: two sibling repos

```
~/GitHub/                       (the Mac: ~/GitHub/Robots/)
  XLeRobot/   branch rosie-config - workspace root, plugins, and this config/
  lerobot/    branch feature/xlerobot - the fork; the workspace points at ../lerobot
```

`config/` lived in the lerobot fork until 2026-10-15. It moved here because
everything in it is about this robot - its ports, ids, cameras, IPs - and
this repo is already the workspace every script runs through. The fork now
carries only its library changes, listed in `lerobot/FORK.md`, which keeps
upstream merges small. (Git history for these files before the move is in
the lerobot repo.)

lerobot auto-imports any installed package named `lerobot_robot_*`,
`lerobot_teleoperator_*`, etc. (`register_third_party_plugins()` in
`lerobot.utils.import_utils`), so an installed plugin makes its
`--robot.type=...` available on every standard CLI.

**Sync from here, never from lerobot**, and with `./config/sync.sh` rather
than a bare `uv sync` (it adds the Jetson's extras). The workspace maps
`lerobot` to `../lerobot` for every member. Syncing from the lerobot repo
instead prunes the plugins, and `--robot.type=xlerobot` stops resolving.

Why that direction: the plugins all depend on `lerobot`, so declaring them
in the fork would invert the dependency. Worse, the fork is itself named
`lerobot`, so uv would resolve the plugins' `lerobot` from PyPI and install
upstream beside the fork, both providing a `lerobot` module. Mapping it at
the workspace root removes the ambiguity by construction.

Examples live under `software/examples/`.

## Serial ports that exist but will not open

`lerobot-find-port` says the device is there; everything else says
"Could not connect on port ... Make sure you are using the correct port."
The port is almost always correct. Two things cause this on Ubuntu, and a
fresh machine usually has both:

**1. You are not in `dialout`.** `/dev/ttyACM*` is `root:dialout` mode 660.

```bash
id -nG | tr ' ' '\n' | grep -x dialout || sudo usermod -aG dialout $USER
```

Then **log out and back in** — group membership is established at login, so
a new shell in the same session still will not have it. `newgrp dialout`
works for one shell.

**2. ModemManager is probing the adapters.** These are CDC-ACM devices;
ModemManager opens every new one and talks AT commands at it for ~20
seconds. While it holds the port, your open fails in a way that is
indistinguishable from a permissions error — and because it lets go
eventually, a retry a minute later can succeed, which makes it look
intermittent and unexplainable.

```bash
sudo tee /etc/udev/rules.d/98-feetech-no-mm.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
```

`--action=add` matters; see `ORIN-SETUP.md`. If the machine has no cellular
modem at all, `sudo systemctl disable --now ModemManager` is simpler.

`./config/servo.sh scan` diagnoses both of these for you now.

## Python version

`pyrealsense2` publishes aarch64 wheels for cp39/cp310/cp312 only, and no
sdist. lerobot requires >=3.12. So on ARM, **Python 3.12 exactly**:

```bash
uv python pin 3.12
uv sync --extra feetech --extra intelrealsense --extra viz
```

Leave out `--extra dataset` until you need to record; it pulls `torchcodec`,
which on aarch64 needs >=0.11.0 and torch>=2.11.
