#!/usr/bin/env python

"""Serve the leader arms and gamepad to a remote recorder.

Runs at the OPERATOR STATION. Reads the local leader arms and gamepad through
the ordinary xlerobot_leader_gamepad teleoperator and publishes each action
over ZMQ for `xlerobot_leader_remote` on the cart to pick up.

    operator station                    cart
    ----------------                    ----
    leader_host.py            --->      lerobot-record
      xlerobot_leader_gamepad             robot  = xlerobot (local)
                                          teleop = xlerobot_leader_remote

Usage:

    python -m lerobot_teleoperator_xlerobot_leader_remote.leader_host \
        --config_path=config/operator-leader-host.yaml

The YAML is an XLerobotLeaderGamepadConfig at the TOP LEVEL - no `teleop:`
key to nest under - because that is what draccus parses here.
"""

import argparse
import json
import logging
import time

import draccus
import zmq

from lerobot_teleoperator_xlerobot_leader_gamepad import (
    XLerobotLeaderGamepad,
    XLerobotLeaderGamepadConfig,
)

logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="XLerobot remote leader host")
    parser.add_argument(
        "--config_path",
        type=str,
        default=None,
        help=(
            "YAML with the full XLerobotLeaderGamepadConfig (leader ports, "
            "gamepad axes, ...). Fields are at the top level of the file."
        ),
    )
    parser.add_argument("--teleop.id", type=str, default="xlerobot_leaders")
    parser.add_argument("--teleop.left_arm_config.port", type=str, default=None)
    parser.add_argument("--teleop.right_arm_config.port", type=str, default=None)
    parser.add_argument("--host.port_zmq_actions", type=int, default=5557)
    parser.add_argument(
        "--host.rate_hz",
        type=float,
        default=60.0,
        help=(
            "How often to read the leaders and publish. Run ABOVE the "
            "recording fps: the socket conflates, so extra actions cost "
            "nothing and the recorder always finds a fresh one waiting "
            "rather than one up to a full tick old."
        ),
    )
    parser.add_argument(
        "--host.no_calibrate",
        action="store_true",
        help="Skip the leaders' calibration check on connect.",
    )
    args = parser.parse_args()

    # NOT basicConfig alone: it is a no-op once the root logger has
    # handlers, and importing lerobot installs them. Every status line here
    # then vanishes - including "Publishing actions on ...", so a host that
    # is working looks identical to one that hung during startup.
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    logging.getLogger().setLevel(logging.INFO)

    if args.config_path:
        config = draccus.parse(
            config_class=XLerobotLeaderGamepadConfig, args=["--config_path", args.config_path]
        )
        logger.info("Loaded teleoperator config from %s", args.config_path)
    else:
        left = getattr(args, "teleop.left_arm_config.port")
        right = getattr(args, "teleop.right_arm_config.port")
        if not (left and right):
            parser.error(
                "Without --config_path you must give both "
                "--teleop.left_arm_config.port and --teleop.right_arm_config.port"
            )
        from lerobot.teleoperators.so_leader import SOLeaderConfig

        config = XLerobotLeaderGamepadConfig(
            id=getattr(args, "teleop.id"),
            left_arm_config=SOLeaderConfig(port=left),
            right_arm_config=SOLeaderConfig(port=right),
        )

    port = getattr(args, "host.port_zmq_actions")
    rate_hz = getattr(args, "host.rate_hz")
    period = 1.0 / rate_hz

    teleop = XLerobotLeaderGamepad(config)
    print("Connecting the leader arms and gamepad ...", flush=True)
    teleop.connect(calibrate=not getattr(args, "host.no_calibrate"))

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    # CONFLATE must be set before bind. It keeps only the newest action
    # queued, so a slow or absent reader can never make us block or build a
    # backlog of stale poses.
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(f"tcp://*:{port}")

    feats = sorted(teleop.action_features)
    print(f"\nPublishing on tcp://*:{port} at {rate_hz:.0f} Hz")
    print(f"{len(feats)} action keys:")
    for k in feats:
        print(f"  {k}")
    print("\nStart the cart now. Ctrl-C here to stop.\n", flush=True)

    sent = 0
    dropped = 0
    last_report = time.perf_counter()

    try:
        while True:
            loop_start = time.perf_counter()

            action = teleop.get_action()
            try:
                sock.send_string(json.dumps(action), flags=zmq.NOBLOCK)
                sent += 1
            except zmq.Again:
                # No peer attached yet, or its queue is full. Dropping is the
                # right answer: the next action supersedes this one anyway.
                dropped += 1

            now = time.perf_counter()
            if now - last_report >= 5.0:
                print(f"  {sent} sent, {dropped} dropped (no reader) "
                      f"in {now - last_report:.0f}s", flush=True)
                sent = dropped = 0
                last_report = now

            time.sleep(max(period - (time.perf_counter() - loop_start), 0.0))

    except KeyboardInterrupt:
        logger.info("Stopping.")
    finally:
        sock.close(linger=0)
        ctx.term()
        teleop.disconnect()
        logger.info("Leader host shut down cleanly.")


if __name__ == "__main__":
    main()
