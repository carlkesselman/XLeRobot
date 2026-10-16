#!/usr/bin/env python
"""Soak the leader link: is it a command channel or a best-effort one?

Sits on the cart and receives the leader host's stream for a fixed time,
timing every message that arrives. No robot, no teleoperator class - a raw
PULL with the same tuning the real client uses, so what is measured is the
network and the two kernels, nothing else.

    # on the cart, leader host running on the operator station
    python -m lerobot_teleoperator_xlerobot_leader_remote.soak --ip 192.168.1.100 --minutes 10

The question it answers is the one the 2026-10-11 teleop run raised: with
the cart's uplink full of video, does the action stream still arrive every
~17 ms, or does it stop for ~300 ms once a second? So run it twice, back to
back, from the same shell:

    1. quiet                - nothing else on the wifi
    2. under video load     - ./config/video.sh synthetic <mac-ip> running in
                              another terminal on the cart for the whole
                              soak; that is the traffic that caused the
                              stalls

Pass: no inter-arrival gap above --stall (the client's stale_after_ms, 300)
in either run, and the under-load run's gap histogram looks like the quiet
one. A stall count that is zero quiet and nonzero under load is the TOS fix
not taking effect at some hop - check `tc -s qdisc` on the cart and whether
the access point honours WMM.

One-way delay is send-time to arrival on two different clocks. Its absolute
value includes the clock offset and means nothing; its spread (p99 minus
median) is real and is what a stall looks like from the receiver.
"""

import argparse
import json
import statistics
import sys
import time

import zmq

from .link import SEQ_KEY, TIME_KEY, tune_socket

# Inter-arrival histogram edges, ms. The host sends at 60 Hz, so a healthy
# link lives entirely in the first two buckets.
_EDGES = (20, 35, 50, 100, 150, 300, 500, 1000)


def _bucket(ms: float) -> int:
    for i, e in enumerate(_EDGES):
        if ms < e:
            return i
    return len(_EDGES)


def _label(i: int) -> str:
    lo = 0 if i == 0 else _EDGES[i - 1]
    return f"{lo:>5}-{_EDGES[i]:<5}" if i < len(_EDGES) else f"{_EDGES[-1]:>5}+     "


def _pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(p * len(s)))]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.100", help="where the leader host is")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--minutes", type=float, default=5.0)
    ap.add_argument("--stall", type=float, default=300.0,
                    help="ms; an inter-arrival gap at or above this is a stall "
                         "(the client's stale_after_ms)")
    ap.add_argument("--report", type=float, default=5.0, help="seconds between progress lines")
    ap.add_argument("--conflate", action="store_true",
                    help="receive like the real client (newest only). Default is every "
                         "message, so sequence gaps mean loss on the wire, not here.")
    args = ap.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PULL)
    if args.conflate:
        sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 100)
    tune_socket(sock)
    sock.connect(f"tcp://{args.ip}:{args.port}")

    print(f"waiting for the first message from tcp://{args.ip}:{args.port} ...", flush=True)
    t_wait = time.perf_counter()
    while True:
        try:
            sock.recv_string()
            break
        except zmq.Again:
            if time.perf_counter() - t_wait > 10:
                print("nothing in 10 s - is the leader host running, and is the port open?",
                      file=sys.stderr)
                return 1

    print(f"soaking for {args.minutes:g} min; a gap >= {args.stall:.0f} ms counts as a stall. "
          f"Ctrl-C ends early.\n", flush=True)

    t0 = time.perf_counter()
    end = t0 + args.minutes * 60
    last_arrival = t0
    last_report = t0
    last_seq: int | None = None

    n = 0
    gaps: list[float] = []
    hist = [0] * (len(_EDGES) + 1)
    stalls: list[tuple[float, float]] = []     # (t since start, gap ms)
    seq_gaps = 0
    delays: list[float] = []
    worst_gap = 0.0
    n_report = 0
    worst_report = 0.0

    try:
        while time.perf_counter() < end:
            try:
                msg = sock.recv_string()
            except zmq.Again:
                # No message in 100 ms. Not counted here - the gap is
                # measured when the next one arrives.
                pass
            else:
                now = time.perf_counter()
                gap = (now - last_arrival) * 1000.0
                last_arrival = now
                n += 1
                n_report += 1
                gaps.append(gap)
                hist[_bucket(gap)] += 1
                worst_gap = max(worst_gap, gap)
                worst_report = max(worst_report, gap)
                if gap >= args.stall:
                    stalls.append((now - t0, gap))

                try:
                    raw = json.loads(msg)
                except ValueError:
                    continue
                seq = raw.get(SEQ_KEY)
                if isinstance(seq, (int, float)):
                    seq = int(seq)
                    if last_seq is not None and seq - last_seq > 1:
                        seq_gaps += seq - last_seq - 1
                    last_seq = seq
                sent = raw.get(TIME_KEY)
                if isinstance(sent, (int, float)):
                    delays.append((time.time() - sent) * 1000.0)

            now = time.perf_counter()
            if now - last_report >= args.report:
                rate = n_report / (now - last_report)
                print(f"  {now - t0:6.0f}s  {rate:5.1f} msg/s  worst gap {worst_report:6.1f} ms  "
                      f"stalls {len(stalls):3d}  seq gaps {seq_gaps}", flush=True)
                last_report = now
                n_report = 0
                worst_report = 0.0
    except KeyboardInterrupt:
        print()

    dur = time.perf_counter() - t0
    sock.close(linger=0)
    ctx.term()

    print()
    print("=" * 66)
    print(f"{n} messages in {dur:.0f} s  ({n / dur:.1f} msg/s; the host sends 60)")
    print("=" * 66)
    if gaps:
        print(f"inter-arrival: median {statistics.median(gaps):.1f}  p99 {_pct(gaps, 0.99):.1f}  "
              f"p99.9 {_pct(gaps, 0.999):.1f}  max {worst_gap:.1f}  ms")
    print(f"stalls (gap >= {args.stall:.0f} ms): {len(stalls)}")
    for t, g in stalls[:20]:
        print(f"    at {t:7.1f} s   {g:7.0f} ms")
    if len(stalls) > 20:
        print(f"    ... and {len(stalls) - 20} more")
    print(f"sequence gaps (messages that never arrived): {seq_gaps}")
    if delays:
        med = statistics.median(delays)
        print(f"one-way delay spread: p99 - median = {_pct(delays, 0.99) - med:.1f} ms, "
              f"max - median = {max(delays) - med:.1f} ms   "
              f"(absolute median {med:.0f} ms includes the clock offset; ignore it)")
    print()
    print("inter-arrival histogram (ms):")
    for i, c in enumerate(hist):
        bar = "#" * min(50, int(50 * c / max(1, n)))
        print(f"  {_label(i)} {c:7d}  {bar}")
    print()
    if stalls:
        print("FAIL: the link stalled. If this run was under video load and the quiet run")
        print("      was clean, the TOS marking is not being honoured somewhere between")
        print("      the two hosts.")
        return 2
    print("PASS: no stalls.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
