#!/usr/bin/env python
"""Watch the leader stream without a robot attached.

Everything between the leader arms and the cart, exercised on its own:
the ZMQ link, XLerobotLeaderRemote itself, the action keys, and whether
moving a leader actually changes what arrives. The only thing missing is
the robot, which is exactly what you want when the robot is the expensive
part to get wrong.

    # on the cart, against the operator station
    python -m lerobot_teleoperator_xlerobot_leader_remote.tap --ip 192.168.1.100

    # on the operator station itself, to take the network out of it
    python -m lerobot_teleoperator_xlerobot_leader_remote.tap --ip 127.0.0.1

Run the leader host first. Move one leader joint at a time, then the
gamepad; the summary on Ctrl-C says which keys ever changed, which is the
question that matters - a key that never moves is a control that is not
reaching the cart.
"""

import argparse
import sys
import time

from .config_xlerobot_leader_remote import XLerobotLeaderRemoteConfig
from .xlerobot_leader_remote import XLerobotLeaderRemote


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.100", help="where the leader host is")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--hz", type=float, default=30.0, help="read rate, like record_loop's")
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    teleop = XLerobotLeaderRemote(XLerobotLeaderRemoteConfig(
        remote_ip=args.ip, port_zmq_actions=args.port, connect_timeout_s=args.timeout,
    ))

    print(f"connecting to tcp://{args.ip}:{args.port} ...")
    try:
        teleop.connect()
    except Exception as e:
        print(f"\n{e}", file=sys.stderr)
        return 1

    keys = sorted(teleop.action_features)
    print(f"connected. {len(keys)} action keys:\n")
    for k in keys:
        print(f"  {k}")
    print("\nMove one leader joint at a time, then the gamepad. Ctrl-C for the summary.\n")

    first: dict[str, float] = {}
    lo: dict[str, float] = {}
    hi: dict[str, float] = {}
    n = 0
    worst_ms = 0.0
    t0 = time.perf_counter()

    try:
        while True:
            loop = time.perf_counter()
            t = time.perf_counter()
            a = teleop.get_action()
            worst_ms = max(worst_ms, (time.perf_counter() - t) * 1000)
            n += 1

            for k, v in a.items():
                first.setdefault(k, v)
                lo[k] = min(lo.get(k, v), v)
                hi[k] = max(hi.get(k, v), v)

            # Show whatever is furthest from where it started - that is the
            # thing the operator is currently moving.
            moving = sorted(a, key=lambda k: -abs(a[k] - first[k]))[:3]
            shown = "  ".join(f"{k.replace('_arm_','.').replace('.pos',''):>22}={a[k]:+7.2f}"
                              for k in moving if abs(a[k] - first[k]) > 0.5)
            print(f"\r{n:6d} msgs  {shown or '(nothing moving yet)':<80}", end="", flush=True)

            time.sleep(max(1.0 / args.hz - (time.perf_counter() - loop), 0.0))
    except KeyboardInterrupt:
        print("\n")

    dur = time.perf_counter() - t0
    print("=" * 66)
    print(f"{n} reads in {dur:.0f}s ({n/dur:.1f} Hz), worst get_action {worst_ms:.2f} ms")
    print("=" * 66)
    changed = [k for k in keys if hi[k] - lo[k] > 0.5]
    still = [k for k in keys if k not in changed]

    print(f"\nCHANGED ({len(changed)}):")
    for k in changed:
        print(f"  {k:<34} {lo[k]:+8.2f} .. {hi[k]:+8.2f}")
    print(f"\nNEVER MOVED ({len(still)}):")
    for k in still:
        print(f"  {k:<34} {lo[k]:+8.2f}")
    if still:
        print("\n  A key that never moved is a control not reaching the cart -")
        print("  unless you simply did not touch it. Check against the list.")

    teleop.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
