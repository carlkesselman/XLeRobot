#!/usr/bin/env python
"""Characterize the two-wheel ODrive base from first principles, then tune it.

    ./config/base-characterize.sh blocks   WHEELS OFF THE GROUND. Per wheel:
                                           motor constant (back-EMF), wheel
                                           inertia and bearing drag.
    ./config/base-characterize.sh floor    ON THE FLOOR, ~2 m clear ahead and
                                           behind, the floor you will drive on.
                                           Breakaway torque, cart inertia and
                                           rolling friction, friction and
                                           speed-estimate noise vs speed.
    ./config/base-characterize.sh design   No motion. From the saved
                                           measurements: current limit, gains,
                                           encoder bandwidth, feed-forward,
                                           acceleration and speed limits - each
                                           with the number that set it.
    ./config/base-characterize.sh step     ON THE FLOOR. Validate: speed steps
                                           through odrive_base, the code
                                           teleop uses, at 30 Hz like teleop.
                                           Rise time, overshoot, ripple,
                                           current jitter, stopping.

Run them in that order. Measurements accumulate in
~/.cache/xlerobot/base-model.json (per machine, like the calibration file);
each stage prints what it measured so the numbers can also be pasted.

Units are ODrive's: torque in "ODrive Nm" (Iq x the configured
torque_constant), speed in wheel turns/s, inertia in Nm per turn/s^2. The
controller lives in those units, so the design is self-consistent even if
the motor's true KV differs from the 16 assumed at setup - only the physical
readouts (newtons, kilograms) depend on it, and `blocks` measures it.

Every test restores the board's modes, limits and gains afterwards and
leaves both axes IDLE. Nothing is saved to flash here; `design` prints the
commands that do that.

The model and the reasoning: claude/xlerobot-base-performance.md in the
XLeRobot project.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import odrive_check as oc  # noqa: E402  - board discovery, enums, errors, yaml

MODEL = Path.home() / ".cache" / "xlerobot" / "base-model.json"

TORQUE, VELOCITY = 1, 2           # CONTROL_MODE_*
PASSTHROUGH, VEL_RAMP = 1, 2      # INPUT_MODE_*
IDLE, CLOSED = 1, 8               # AXIS_STATE_*


# ------------------------------------------------------------------ model file

def load_model() -> dict:
    try:
        return json.loads(MODEL.read_text())
    except Exception:  # noqa: BLE001
        return {}


def save_model(section: str, data: dict) -> None:
    m = load_model()
    m[section] = {**data, "measured_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    MODEL.parent.mkdir(parents=True, exist_ok=True)
    MODEL.write_text(json.dumps(m, indent=2))
    print(f"\nsaved to {MODEL} [{section}]")
    print(json.dumps({section: m[section]}, indent=2))


# ---------------------------------------------------------------- board access

def linfit(xs, ys):
    n = len(xs)
    if n < 2:
        return float("nan"), float("nan")
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return float("nan"), float("nan")
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return slope, my - slope * mx


def iq(ax) -> float:
    return getattr(ax.motor.current_control, "Iq_measured", 0.0)


def vq(ax):
    """q-axis voltage, if this firmware exposes one. 0.5.x names vary."""
    cc = ax.motor.current_control
    for name in ("Vq_setpoint", "final_v_q", "v_current_control_integral_q"):
        if hasattr(cc, name):
            return getattr(cc, name), name
    return None, None


class Board:
    """Find the board, remember everything a test may change, restore it."""

    FIELDS = (("controller.config", "control_mode"), ("controller.config", "input_mode"),
              ("controller.config", "vel_limit"), ("controller.config", "enable_torque_mode_vel_limit"),
              ("controller.config", "vel_ramp_rate"), ("controller.config", "vel_gain"),
              ("controller.config", "vel_integrator_gain"), ("encoder.config", "bandwidth"))

    def __init__(self, timeout: float):
        self.o = oc._find(timeout)
        self.axes = [(0, self.o.axis0), (1, self.o.axis1)]
        self.saved = {}
        for i, ax in self.axes:
            for path, name in self.FIELDS:
                obj = _get(ax, path)
                if hasattr(obj, name):
                    self.saved[(i, path, name)] = getattr(obj, name)
        rb = oc._robot_block()
        # forward sign per axis: +1 if a positive turns/s drives that wheel forward
        self.fwd = {0: -1.0 if rb.get("invert_right_wheel", False) else 1.0,
                    1: -1.0 if rb.get("invert_left_wheel", True) else 1.0}
        self.radius = float(rb.get("wheel_radius", 0.0825))
        self.wheelbase = float(rb.get("wheelbase", 0.527))
        self.circ = 2 * math.pi * self.radius
        a0 = self.o.axis0
        self.kt = a0.motor.config.torque_constant
        self.pp = a0.motor.config.pole_pairs
        self.cpr = a0.encoder.config.cpr

    def describe(self) -> dict:
        d = {"torque_constant": self.kt, "pole_pairs": self.pp, "cpr": self.cpr,
             "wheel_radius": self.radius, "wheelbase": self.wheelbase,
             "vbus": round(self.o.vbus_voltage, 2),
             "dc_max_positive_current": getattr(self.o.config, "dc_max_positive_current", None),
             "dc_bus_undervoltage_trip_level": getattr(self.o.config, "dc_bus_undervoltage_trip_level", None)}
        for i, ax in self.axes:
            m, c = ax.motor.config, ax.controller.config
            d[f"axis{i}"] = {"side": oc.SIDE[i], "torque_constant": m.torque_constant,
                             "phase_resistance": m.phase_resistance,
                             "phase_inductance": m.phase_inductance, "current_lim": m.current_lim,
                             "vel_gain": c.vel_gain, "vel_integrator_gain": c.vel_integrator_gain,
                             "encoder_bandwidth": ax.encoder.config.bandwidth}
        return d

    def arm(self, which=(0, 1)) -> None:
        oc._clear(self.o)
        for i, ax in self.axes:
            if i in which:
                ax.controller.input_torque = 0.0
                ax.controller.input_vel = 0.0
                ax.controller.config.input_mode = PASSTHROUGH
                ax.controller.config.control_mode = VELOCITY
                ax.requested_state = CLOSED
        time.sleep(0.5)
        for i, ax in self.axes:
            if i in which and ax.current_state != CLOSED:
                raise RuntimeError(f"axis{i} did not enter closed loop: {oc._fmt_errors(oc._axis_errors(ax))}")

    def torque_mode(self, ax, vel_limit: float) -> None:
        c = ax.controller.config
        ax.controller.input_torque = 0.0
        if hasattr(c, "enable_torque_mode_vel_limit"):
            c.enable_torque_mode_vel_limit = True
        c.vel_limit = vel_limit
        c.input_mode = PASSTHROUGH
        c.control_mode = TORQUE

    def soft_stop(self, which=(0, 1), decel: float = 3.0) -> None:
        """Hand a moving wheel back to the velocity loop at its own speed, then
        ramp to zero - a switch straight to input_vel 0 would brake hard."""
        sel = [(i, ax) for i, ax in self.axes if i in which]
        sp = {}
        for i, ax in sel:
            v = ax.encoder.vel_estimate
            ax.controller.config.input_mode = PASSTHROUGH
            ax.controller.input_vel = v
            if hasattr(ax.controller, "vel_integrator_torque"):
                try:
                    ax.controller.vel_integrator_torque = ax.controller.input_torque
                except Exception:  # noqa: BLE001
                    pass
            ax.controller.input_torque = 0.0
            ax.controller.config.control_mode = VELOCITY
            sp[i] = v
        t = time.time()
        while any(abs(v) > 1e-3 for v in sp.values()):
            time.sleep(0.02)
            now = time.time(); dt = now - t; t = now
            for i, ax in sel:
                v = sp[i]
                v = max(0.0, abs(v) - decel * dt) * (1 if v > 0 else -1)
                sp[i] = v
                ax.controller.input_vel = v
        time.sleep(0.4)

    def restore(self) -> None:
        for i, ax in self.axes:
            try:
                ax.controller.input_torque = 0.0
                ax.controller.input_vel = 0.0
                ax.requested_state = IDLE
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.2)
        for (i, path, name), v in self.saved.items():
            try:
                setattr(_get(dict(self.axes)[i], path), name, v)
            except Exception:  # noqa: BLE001
                pass

    def errors(self) -> dict:
        return {i: oc._fmt_errors(oc._axis_errors(ax)) for i, ax in self.axes}


def _get(obj, path: str):
    for p in path.split("."):
        obj = getattr(obj, p)
    return obj


def confirm(text: str) -> bool:
    print(text)
    return oc._yes("Ready?")


# ---------------------------------------------------------------------- blocks

def cmd_blocks(args) -> int:
    b = Board(args.timeout)
    if not confirm("\nBLOCKS: both wheels clear of the ground. Each wheel in turn spins up to\n"
                   "3 turns/s (~1.5 m/s at the tyre) and coasts. Keep hands clear."):
        return 1
    out = {"board": b.describe(), "axes": {}}
    try:
        for i, ax in b.axes:
            side = oc.SIDE[i]
            print(f"\n=== axis{i} ({side}) ===")
            b.arm(which=(i,))
            res = {}
            # 1. motor constant from back-EMF: Vq - R*Iq = lambda * omega_e
            R = ax.motor.config.phase_resistance
            pts = []
            vname = None
            for v in (1.0, 2.0, 3.0):
                ax.controller.input_vel = v
                time.sleep(1.2)
                vs, qs, ws = [], [], []
                t0 = time.time()
                while time.time() - t0 < 0.8:
                    val, vname = vq(ax)
                    if val is None:
                        break
                    vs.append(val); qs.append(iq(ax)); ws.append(ax.encoder.vel_estimate)
                    time.sleep(0.01)
                if not vs:
                    break
                w = statistics.mean(ws)
                pts.append((2 * math.pi * b.pp * w, statistics.mean(vs) - R * statistics.mean(qs),
                            statistics.mean(qs)))
                print(f"  {w:+.2f} turns/s: {vname} {statistics.mean(vs):+.3f} V, Iq {statistics.mean(qs):+.2f} A")
            b.soft_stop(which=(i,))
            if len(pts) >= 2:
                lam, icpt = linfit([p[0] for p in pts], [p[1] for p in pts])
                # The sign is the board's direction convention (Vq and Iq both read
                # negative for positive speed on this cart); the size is the constant.
                kt_meas = 1.5 * b.pp * abs(lam)
                kt_cfg = ax.motor.config.torque_constant
                res["kt_measured"] = kt_meas
                res["kv_measured"] = 8.27 / kt_meas if kt_meas > 0 else None
                res["backemf_intercept_v"] = icpt
                plaus = 0.1 < kt_meas < 3.0 and abs(icpt) < 0.5
                print(f"  motor constant: {kt_meas:.3f} Nm/A (KV {8.27 / kt_meas:.1f}), back-EMF line through "
                      f"{icpt:+.2f} V at zero speed - configured {kt_cfg:.3f} (KV {8.27 / kt_cfg:.1f})"
                      + ("" if plaus else
                         f"   IMPLAUSIBLE - {vname} is probably not in volts on this firmware; ignored"))
                if plaus and abs(kt_meas / kt_cfg - 1) > 0.1:
                    print(f"  -> differs by {100 * (kt_meas / kt_cfg - 1):+.0f}%. Set it, keeping behaviour identical:")
                    print("     ./config/odrive-check.sh --save-gains --kt <right>,<left>   then rerun `blocks`")
                if not plaus:
                    res["kt_measured"] = None
            else:
                print("  motor constant: this firmware exposes no q-axis voltage - skipped "
                      "(the controller does not need it; only physical readouts do)")
            # 2. wheel inertia + drag: spin up at a constant torque, then coast at zero torque.
            #    tau - f = J a_up,  -f = J a_dn   =>  J = tau / (a_up - a_dn), f = -J a_dn
            for tau in (0.4, 0.8, 1.2):
                b.torque_mode(ax, vel_limit=3.5)
                s = 1.0
                up, dn = [], []
                ax.controller.input_torque = tau * s
                t0 = time.time()
                while time.time() - t0 < 2.5:
                    v = ax.encoder.vel_estimate * s
                    up.append((time.time() - t0, v))
                    if v > 2.5:
                        break
                    time.sleep(0.01)
                ax.controller.input_torque = 0.0
                t1 = time.time()
                while time.time() - t1 < 4.0:
                    v = ax.encoder.vel_estimate * s
                    dn.append((time.time() - t1, v))
                    if v < 0.2:
                        break
                    time.sleep(0.01)
                b.soft_stop(which=(i,))
                upf = [(t, v) for t, v in up if 0.3 < v < 2.3]
                dnf = [(t, v) for t, v in dn if 0.3 < v < 2.3]
                if len(upf) < 5 or len(dnf) < 5:
                    print(f"  torque {tau} Nm: wheel did not spin up/coast through 0.3-2.3 turns/s - "
                          "trying more torque")
                    continue
                a_up, _ = linfit(*zip(*upf))
                a_dn, _ = linfit(*zip(*dnf))
                J = tau / (a_up - a_dn)
                drag = -J * a_dn
                res.update(J_wheel=J, J_wheel_si=J / (2 * math.pi), drag_wheel=drag,
                           a_up=a_up, a_dn=a_dn, tau_test=tau)
                print(f"  torque {tau} Nm: accel {a_up:+.2f}, coast {a_dn:+.2f} turns/s^2 -> wheel inertia "
                      f"{J:.4f} Nm per turn/s^2 ({J / (2 * math.pi):.4f} kg m^2), drag {drag:.3f} Nm")
                break
            out["axes"][str(i)] = res
            ax.requested_state = IDLE
            time.sleep(0.3)
    finally:
        b.restore()
    print("\nerrors after:", b.errors())
    save_model("blocks", out)
    print("\nNext: ./config/base-characterize.sh floor")
    return 0


# ----------------------------------------------------------------------- floor

def cmd_floor(args) -> int:
    b = Board(args.timeout)
    room = args.room
    if not confirm(f"\nFLOOR: the floor you will drive on, {room:g} m clear ahead AND behind.\n"
                   "Short pushes reach ~0.7 m/s for a moment.\n"
                   "The cart moves forward and back in short pushes, never more than ~"
                   f"{0.8 * room:.1f} m from its start.\nYour hand near the pack - "
                   "Ctrl-C stops everything."):
        return 1
    out = {"board": b.describe(), "room_m": room}
    fwd = b.fwd

    def cart_speed():
        return statistics.mean(fwd[i] * ax.encoder.vel_estimate for i, ax in b.axes)

    def set_torque(tau):
        for i, ax in b.axes:
            ax.controller.input_torque = fwd[i] * tau

    def to_torque_mode():
        for _, ax in b.axes:
            b.torque_mode(ax, vel_limit=1.8)

    try:
        b.arm()
        # 1. breakaway: ramp torque slowly until the cart moves, each direction
        print("\n--- breakaway torque (ramped 0.8 Nm/s until the cart moves) ---")
        brk = []
        for d in (+1, -1):
            to_torque_mode()
            t0 = time.time(); tau = 0.0; moving = 0
            while tau < 6.0:
                tau = 0.8 * (time.time() - t0)
                set_torque(d * tau)
                moving = moving + 1 if d * cart_speed() > 0.05 else 0
                if moving >= 3:
                    break
                time.sleep(0.01)
            set_torque(0.0)
            b.soft_stop()
            if tau >= 6.0:
                print(f"  {'forward' if d > 0 else 'backward'}: did not move at 6 Nm - stuck or braked?")
                continue
            brk.append(tau)
            print(f"  {'forward ' if d > 0 else 'backward'}: moved at {tau:.2f} Nm per wheel "
                  f"({tau / b.kt:.1f} A)")
        if not brk:
            print("no breakaway - stopping.")
            return 1
        tau_b = statistics.mean(brk)
        out["tau_break"] = tau_b

        # 2. inertia and rolling friction: two constant torques, both directions
        print("\n--- cart inertia and rolling friction (constant torque, two levels) ---")
        levels = [min(1.3 * tau_b, 4.0), min(2.0 * tau_b, 6.0)]
        acc = {}
        for tau in levels:
            alist = []
            for d in (+1, -1):
                to_torque_mode()
                set_torque(d * tau)
                pts = []; t0 = time.time()
                while time.time() - t0 < 2.5:
                    v = d * cart_speed()
                    pts.append((time.time() - t0, v))
                    if v > 1.3:
                        break
                    time.sleep(0.005)
                set_torque(0.0)
                b.soft_stop()
                # Fit well clear of the start: the speed estimate (a PLL on Hall
                # steps) lags the breakaway and would flatten the slope.
                fit = [(t, v) for t, v in pts if 0.35 < v < 1.25]
                if len(fit) < 5:
                    print(f"  {tau:.2f} Nm {'fwd' if d > 0 else 'back'}: too few samples in 0.35-1.25 turns/s")
                    continue
                a, _ = linfit(*zip(*fit))
                alist.append(a)
                print(f"  {tau:.2f} Nm {'fwd ' if d > 0 else 'back'}: {a:.2f} turns/s^2")
            if alist:
                acc[tau] = statistics.mean(alist)
        if len(acc) == 2:
            (t1, a1), (t2, a2) = sorted(acc.items())
            J = (t2 - t1) / (a2 - a1) if a2 != a1 else float("nan")
            tau_r = t1 - J * a1
            out.update(J_cart=J, tau_roll_accel=tau_r)
            print(f"  -> effective inertia per wheel {J:.3f} Nm per turn/s^2, rolling friction "
                  f"{tau_r:.2f} Nm per wheel while accelerating")

        # 3. friction and estimate noise vs speed, in velocity mode
        print("\n--- friction and speed-estimate noise vs speed (velocity mode, saved gains) ---")
        sweep = []
        for v_mps in args.speeds:
            v = v_mps / b.circ
            hold = 2.0
            dist = v_mps * (hold + v / 2.0)          # ramp at 2 turns/s^2 + hold
            if dist > 0.8 * room:
                print(f"  {v_mps} m/s needs {dist:.1f} m - skipped (raise --room)")
                continue
            row = {"v_mps": v_mps, "v_tps": v}
            taus, vsd, isd = [], [], []
            for d in (+1, -1):
                for _, ax in b.axes:
                    ax.controller.input_torque = 0.0
                    ax.controller.config.control_mode = VELOCITY
                    ax.controller.config.vel_ramp_rate = 2.0
                    ax.controller.config.input_mode = VEL_RAMP
                for i, ax in b.axes:
                    ax.controller.input_vel = fwd[i] * d * v
                time.sleep(v / 2.0 + hold - 1.0)
                smp = {0: [], 1: []}
                t0 = time.time()
                while time.time() - t0 < 1.0:
                    for i, ax in b.axes:
                        smp[i].append((fwd[i] * d * ax.encoder.vel_estimate,
                                       fwd[i] * d * iq(ax) * ax.motor.config.torque_constant))
                    time.sleep(0.01)
                for i, ax in b.axes:
                    ax.controller.input_vel = 0.0
                time.sleep(v / 2.0 + 0.6)
                for i in (0, 1):
                    vv = [x[0] for x in smp[i]]; tt = [x[1] for x in smp[i]]
                    taus.append(statistics.mean(tt))
                    vsd.append(statistics.pstdev(vv))
                    isd.append(statistics.pstdev(tt) / dict(b.axes)[i].motor.config.torque_constant)
            row.update(tau=statistics.mean(taus), vel_sd=statistics.mean(vsd), iq_sd=statistics.mean(isd))
            sweep.append(row)
            print(f"  {v_mps:.2f} m/s ({v:.2f} turns/s): torque {row['tau']:.2f} Nm/wheel, speed noise "
                  f"{row['vel_sd']:.3f} turns/s ({100 * row['vel_sd'] / v:.0f}%), current noise {row['iq_sd']:.2f} A")
        out["sweep"] = sweep
        if len(sweep) >= 2:
            bv, c = linfit([r["v_tps"] for r in sweep], [r["tau"] for r in sweep])
            out.update(coulomb=c, viscous=bv)
            print(f"  -> friction {c:.2f} Nm + {bv:.3f} Nm per turn/s")

        # 4. optional: estimate noise vs encoder bandwidth at one slow speed
        if args.bw_sweep:
            print("\n--- speed-estimate noise vs encoder bandwidth (0.15 m/s) ---")
            nb = []
            v = 0.15 / b.circ
            for bw in args.bw_sweep:
                vsd = []
                for d in (+1, -1):
                    for i, ax in b.axes:
                        ax.encoder.config.bandwidth = bw
                        ax.controller.config.vel_ramp_rate = 2.0
                        ax.controller.config.input_mode = VEL_RAMP
                        ax.controller.input_vel = fwd[i] * d * v
                    time.sleep(1.2)
                    smp = {0: [], 1: []}; t0 = time.time()
                    while time.time() - t0 < 1.0:
                        for i, ax in b.axes:
                            smp[i].append(ax.encoder.vel_estimate)
                        time.sleep(0.01)
                    for i, ax in b.axes:
                        ax.controller.input_vel = 0.0
                    time.sleep(0.8)
                    vsd += [statistics.pstdev(smp[0]), statistics.pstdev(smp[1])]
                nb.append({"bw": bw, "vel_sd": statistics.mean(vsd)})
                print(f"  bandwidth {bw:5.0f} rad/s: speed noise {statistics.mean(vsd):.3f} turns/s")
            out["noise_vs_bw"] = nb
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        b.restore()
    print("\nerrors after:", b.errors())
    save_model("floor", out)
    print("\nNext: ./config/base-characterize.sh design")
    return 0


# ---------------------------------------------------------------------- design

def cmd_design(args) -> int:
    m = load_model()
    bl, fl = m.get("blocks", {}), m.get("floor", {})
    if not fl:
        print("no floor measurements yet - run `floor` first", file=sys.stderr)
        return 1
    bd = fl.get("board") or bl.get("board") or {}

    def kt_of(board):
        """Mean configured torque constant of a section's board (per axis since 2026-10-24)."""
        per = [board.get(f"axis{i}", {}).get("torque_constant") for i in (0, 1)]
        per = [x for x in per if x]
        return statistics.mean(per) if per else board.get("torque_constant", 8.27 / 16)

    kt = kt_of(bd)
    # blocks may have run under a different configured torque constant: its
    # torques (wheel inertia, drag) are in that run's units - convert to ours
    bl_scale = kt / kt_of(bl.get("board", {})) if bl.get("board") else 1.0
    r = bd.get("wheel_radius", 0.0825)
    circ = 2 * math.pi * r
    cpr = bd.get("cpr", 90)
    R = max(bd.get(f"axis{i}", {}).get("phase_resistance", 0.41) for i in (0, 1))
    i_bus = bd.get("dc_max_positive_current") or 10.0
    v_min = max(bd.get("dc_bus_undervoltage_trip_level") or 16.5, 16.0)
    J = fl.get("J_cart")
    Jw = bl_scale * statistics.mean([a.get("J_wheel") for a in bl.get("axes", {}).values() if a.get("J_wheel")] or [0.06])
    c = fl.get("coulomb")
    bv = fl.get("viscous", 0.0) or 0.0
    if c is None or not math.isfinite(c) or c <= 0:
        sw = fl.get("sweep") or []
        c = min(sw, key=lambda r_: r_["v_mps"])["tau"] if sw else fl.get("tau_roll_accel")
        bv = 0.0
    if not math.isfinite(bv) or bv < 0:
        bv = 0.0
    tau_b = fl.get("tau_break")
    kts = [a.get("kt_measured") for a in bl.get("axes", {}).values() if a.get("kt_measured")]
    kt_phys = statistics.mean(kts) if kts else kt
    if J is None or c is None:
        print("floor results incomplete (inertia or friction missing) - rerun `floor`", file=sys.stderr)
        return 1

    P = print
    P("\n================ BASE DESIGN, from first principles ================")
    P(f"measured: cart inertia {J:.3f} Nm per turn/s^2 per wheel, wheel alone {Jw:.4f}; "
      f"breakaway {tau_b:.2f} Nm, rolling {c:.2f} Nm + {bv:.3f} Nm per turn/s")
    # Inertia is measured in ODrive units (configured torque constant), so the
    # mass it implies scales with true kt / configured kt. A scale reading
    # therefore measures the true torque constant - independently of `blocks`.
    mass_odrive = 2 * (J - Jw) / (2 * math.pi) / (r * r)       # as if configured kt were true
    if args.mass_kg:
        kt_mass = kt * args.mass_kg / mass_odrive
        P(f"   -> inertia + scale ({args.mass_kg:g} kg) imply a torque constant of {kt_mass:.3f} Nm/A "
          f"(KV {8.27 / kt_mass:.1f}); configured {kt:.3f} (KV {8.27 / kt:.1f})"
          + (f", back-EMF (blocks) {kt_phys:.3f} (KV {8.27 / kt_phys:.1f})" if kts else ""))
        if kts and abs(kt_mass / kt_phys - 1) > 0.25:
            P("      The two disagree by more than 25%: torque per amp is lower than the motor's")
            P("      back-EMF says - suspect the Hall commutation (encoder offset), and recalibrate.")
        kt_phys = kt_mass if not kts else kt_phys
        mass = args.mass_kg
    else:
        mass = mass_odrive * (kt_phys / kt)
        P(f"   -> cart mass as the wheels see it: ~{mass:.0f} kg"
          + ("" if kts else " (assuming KV 16 - run `blocks` to measure it, or pass --mass-kg)"))
    scale = kt_phys / kt                       # ODrive Nm -> real Nm
    drag = bl_scale * statistics.mean([a.get("drag_wheel") for a in bl.get("axes", {}).values() if a.get("drag_wheel")] or [0.0])
    ext = max(c - drag, 0.0)
    P(f"   -> friction per wheel {c:.2f} Nm: {drag:.2f} inside the motor (blocks coast-down), "
      f"{ext:.2f} from the floor and casters")
    rr = 2 * ext * scale / r / (mass * 9.81) if mass > 0 else float("nan")
    P(f"   -> floor + caster rolling resistance {2 * ext * scale / r:.0f} N = {rr:.3f} of the weight "
      "(hard casters on a hard floor: 0.01-0.03; carpet 0.05-0.1)")
    trac = 0.7 * args.drive_share * mass * 9.81
    P(f"   -> traction, if {100 * args.drive_share:.0f}% of the weight is on the drive wheels and tyre "
      f"grip is ~0.7: {trac:.0f} N, i.e. {trac * r / 2 / scale + drag:.2f} ODrive Nm per wheel before "
      "they slip - on a light cart grip, not current, is the obstacle limit")

    # 1. current limit: both wheels stalled, copper loss only, from the pack at v_min
    i_pack = math.sqrt(0.9 * v_min * i_bus / (2 * 1.5 * R))
    i_lim = math.floor(min(i_pack, args.motor_peak) * 2) / 2
    tau_max = kt * i_lim
    P("\n1. MOTOR CURRENT LIMIT")
    P(f"   Both wheels stalled at I amps burn 2 x 1.5 x R x I^2 in the windings (R {R:.3f} ohm), all of it")
    P(f"   from the pack. The pack limit {i_bus:g} A is a FAULT on fw 0.5.1, not a ceiling. At {v_min:g} V")
    P(f"   (undervoltage trip) with 90% inverter efficiency that allows {i_pack:.1f} A per motor.")
    P(f"   -> current_lim {i_lim:g} A  = {tau_max:.2f} Nm per wheel, "
      f"{tau_max / tau_b:.1f}x breakaway" + ("" if tau_max > 1.5 * tau_b else
      "   WARNING: little margin for obstacles - casters / weight distribution"))

    # 2. velocity loop
    P("\n2. VELOCITY LOOP  (per wheel: torque = Kp e + Ki integral(e) + feed-forward)")
    P(f"   Plant: torque -> inertia {J:.3f}. Closed-loop bandwidth wc = Kp / J (rad/s).")
    P(f"   Hall position steps 1/{cpr} turn; the speed estimate is a PLL of bandwidth wb.")
    P("   Rules: wc <= wb/4 (the estimate's lag stays out of the loop), Ki = Kp wc/4 (PI zero a")
    P("   quarter below crossover: ~60 deg phase margin). A higher wb gives a stiffer, faster loop")
    P("   and passes more Hall noise into the torque; which wins on this cart is measured, not")
    P("   assumed: `step --candidates` runs all three and scores them.")
    noise = {int(n["bw"]): n["vel_sd"] for n in fl.get("noise_vs_bw", [])}
    cands = []
    for name, wb in (("soft", 25), ("medium", 50), ("stiff", 100)):
        wc = wb / 4
        kp = J * wc
        ki = kp * wc / 4
        cands.append({"name": name, "enc_bw": wb, "vel_gain": round(kp, 2), "int_gain": round(ki, 2), "wc": wc})
        sd = noise.get(wb)
        P(f"   {name:<6} wb {wb:3d}: vel_gain {kp:5.2f}, vel_integrator_gain {ki:6.2f}, wc {wc:5.2f} rad/s, "
          f"rise ~{2.2 / wc:.2f} s, Kp x noise {kp * sd:.2f} Nm" if sd else
          f"   {name:<6} wb {wb:3d}: vel_gain {kp:5.2f}, vel_integrator_gain {ki:6.2f}, wc {wc:5.2f} rad/s, "
          f"rise ~{2.2 / wc:.2f} s")
    pick = cands[1]
    wb, kp, wc, ki = pick["enc_bw"], pick["vel_gain"], pick["wc"], pick["int_gain"]
    P(f"   -> provisional: {pick['name']} (validate with `step --candidates`)")
    P(f"   On blocks the {pick['name']} gains give wc {kp / Jw:.0f} rad/s "
      f"({'above' if kp / Jw > wb / 2 else 'below'} wb/2): blocks will look "
      f"{'lively or buzzy - expected, ignore' if kp / Jw > wb / 2 else 'calm'}.")
    v_cmd = 0.2 / circ
    t_full = (tau_max - 0.9 * c - kp * v_cmd) / (ki * v_cmd) if ki > 0 else float("inf")
    P(f"   Stalled at an edge while commanding 0.2 m/s: feed-forward {0.9 * c:.2f} + Kp x error "
      f"{kp * v_cmd:.2f} Nm at once, full {tau_max:.2f} Nm after {max(t_full, 0):.1f} s.")

    # 3. feed-forward
    ff_c = round(0.9 * c, 2)
    P("\n3. FEED-FORWARD")
    P(f"   friction_ff_nm {ff_c}  (90% of rolling friction: carries the cart without pushing it off")
    P(f"   a standstill; breakaway, {tau_b:.2f} Nm, is left to the loop)")
    P(f"   inertia_ff {J:.3f}  (the measured inertia: the ramp's acceleration costs no error)")

    # 4. acceleration and speed
    P("\n4. ACCELERATION AND SPEED")
    a_tq = (0.5 * tau_max - c) / J * circ
    P(f"   Half the torque budget for acceleration leaves the rest for disturbances: "
      f"{a_tq:.2f} m/s^2.")
    a = round(min(a_tq, args.accel_cap), 2)
    P(f"   -> max_accel_mps2 {a}  (capped at {args.accel_cap} for a cart carrying arms - raise only after a")
    P("      tip test: brake hard from top speed with the arms up)")
    kv = 8.27 / kt_phys
    v_volt = kv * v_min * 0.9 / 60 * circ
    v_top = round(min(args.speed_cap, 0.5 * v_volt), 2)
    P(f"   Back-EMF limit at {v_min:g} V: ~{v_volt:.1f} m/s (KV {kv:.1f}). Keep half for torque headroom.")
    P(f"   -> max_linear_mps {v_top}  (stopping distance {v_top ** 2 / (2 * a):.2f} m at {a} m/s^2)")
    v_smooth = wb / cpr * circ
    P(f"   Smooth-speed floor: below ~{v_smooth:.2f} m/s the estimate gets less than one Hall step per")
    P("   time constant - there the feed-forward carries the motion and the loop only trims.")
    stick = round(min(v_top, max(0.3, 0.6 * v_top)), 2)
    P(f"   -> gamepad base_speed_mps {stick} at full stick (scale R1/L2 0.25-2.0 => "
      f"{0.25 * stick:.2f}-{min(2 * stick, v_top):.2f} m/s)")

    design = {"candidates": cands, "current_lim": i_lim, "vel_gain": round(kp, 3), "vel_integrator_gain": round(ki, 3),
              "encoder_bandwidth": wb, "friction_ff_nm": ff_c, "inertia_ff": round(J, 3),
              "max_accel_mps2": a, "max_linear_mps": v_top, "base_speed_mps": stick,
              "mass_kg": round(mass, 1)}
    P("\n================ TO APPLY ================")
    P(f"./config/odrive-check.sh --save-gains --vel-gain {kp:.2f} --int-gain {ki:.2f} "
      f"--enc-bw {wb} --current-lim {i_lim:g}")
    P("config/cart-remote.yaml, robot block:")
    P(f"  max_linear_mps: {v_top}\n  max_accel_mps2: {a}\n  friction_ff_nm: {ff_c}\n  inertia_ff: {J:.3f}")
    P("config/operator-leader-host.yaml (rosie):")
    P(f"  base_speed_mps: {stick}")
    P("then validate:  ./config/base-characterize.sh step --candidates")
    save_model("design", design)
    return 0


# ------------------------------------------------------------------------ step

def cmd_step(args) -> int:
    from lerobot_robot_xlerobot_2wheels_odrive.odrive_base import ODriveBase
    rb = oc._robot_block()
    design = load_model().get("design", {})
    kw = dict(wheel_radius=float(rb.get("wheel_radius", 0.0825)), wheelbase=float(rb.get("wheelbase", 0.527)),
              invert_left=bool(rb.get("invert_left_wheel", True)), invert_right=bool(rb.get("invert_right_wheel", False)),
              max_linear_mps=float(rb.get("max_linear_mps", 0.5)),
              max_accel_mps2=float(rb.get("max_accel_mps2", 1.0)),
              friction_ff_nm=float(rb.get("friction_ff_nm", 0.0)), inertia_ff=float(rb.get("inertia_ff", 0.0)))
    if args.candidates:
        if not design.get("candidates"):
            print("no design yet - run `design` first", file=sys.stderr)
            return 1
        # validate the design as a whole: its feed-forward and limits too
        for k in ("max_accel_mps2", "friction_ff_nm", "inertia_ff", "max_linear_mps"):
            if k in design:
                kw[k] = design[k]
    for k in ("max_accel_mps2", "friction_ff_nm", "inertia_ff", "max_linear_mps"):
        v = getattr(args, k)
        if v is not None:
            kw[k] = v
    if args.candidates:
        cands = design["candidates"]
    else:
        cands = [{"name": "as saved" if args.vel_gain is None else "given", "vel_gain": args.vel_gain,
                  "int_gain": args.int_gain, "enc_bw": args.enc_bw}]
    speeds = [s_ for s_ in args.speeds if s_ <= kw["max_linear_mps"]]
    room = max(s_ * args.hold for s_ in speeds) + 0.3
    if not confirm(f"\nSTEP: on the floor, {room:.1f} m clear ahead and behind. For each of "
                   f"{len(cands)} setting(s), each speed {speeds} m/s: forward {args.hold:g} s, stop, "
                   f"back {args.hold:g} s, stop - it ends where it started.\nbase: {kw}"):
        return 1
    base = ODriveBase(**kw)
    base.connect()
    o = base.odrv
    circ = 2 * math.pi * kw["wheel_radius"]
    fwd = {"L": -1.0 if kw["invert_left"] else 1.0, "R": -1.0 if kw["invert_right"] else 1.0}
    dt = 1 / 30
    results = []
    try:
        for cand in cands:
            print(f"\n=== {cand['name']} ===")
            oc._show_or_try_gains(o, argparse.Namespace(vel_gain=cand.get("vel_gain"),
                                                       int_gain=cand.get("int_gain"), enc_bw=cand.get("enc_bw")))
            log = []
            t0 = time.monotonic()

            def run(x_mps, seconds, label):
                t_end = time.monotonic() + seconds
                nxt = time.monotonic()
                while time.monotonic() < t_end:
                    base.send_body_velocity(x_mps, 0.0)
                    base.read_body_velocity()
                    L, R = base.left, base.right
                    log.append({"t": time.monotonic() - t0, "seg": label,
                                "L": fwd["L"] * L.encoder.vel_estimate, "R": fwd["R"] * R.encoder.vel_estimate,
                                "LA": fwd["L"] * iq(L), "RA": fwd["R"] * iq(R)})
                    nxt += dt
                    time.sleep(max(0.0, nxt - time.monotonic()))

            for s_ in speeds:
                run(s_, args.hold, f"+{s_}")
                run(0.0, 1.2, f"stop+{s_}")
                run(-s_, args.hold, f"-{s_}")
                run(0.0, 1.2, f"stop-{s_}")
            rows = step_metrics(log, speeds, circ, args.hold)
            results.append({"cand": cand, "rows": rows})
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        base.stop()
        time.sleep(0.5)
        base.disconnect()
        # trial gains live in RAM until the board reboots: put the saved ones back
        b = Board(args.timeout)
        b.restore()

    # score: ripple at the speeds above the smooth floor, overshoot, rise
    print("\n================ SUMMARY ================")
    print(f" {'setting':<10} {'ripple@>=0.15':>13} {'ripple@slowest':>14} {'overshoot':>9} {'rise':>6} {'I jitter':>8}")
    scored = []
    for r in results:
        rows = r["rows"]
        if not rows:
            continue
        def agg(key, sel=lambda row: True):
            vals = [x for row in rows if sel(row) for x in row[key] if math.isfinite(x)]
            return statistics.mean(vals) if vals else float("nan")
        fast = agg("ripple", lambda row: row["v"] >= 0.15)
        slowest = min(row["v"] for row in rows)
        slow = agg("ripple", lambda row: row["v"] == slowest)
        over = agg("overshoot")
        rise = agg("rise")
        jit = agg("jitter")
        scored.append((r["cand"], fast, slow, over, rise, jit))
        print(f" {r['cand']['name']:<10} {100 * fast:12.0f}% {100 * slow:13.0f}% {100 * over:+8.0f}% "
              f"{rise:5.2f}s {jit:7.2f}A")
    ok = [x for x in scored if x[3] <= 0.15 and math.isfinite(x[1])]
    why = "lowest ripple at working speeds with overshoot <= 15%, then fastest"
    if not ok:
        ok, why = scored, "NONE kept overshoot <= 15% - lowest ripple regardless; look at the table"
    if ok:
        best = min(ok, key=lambda x: (round(x[1], 2), x[4]))
        c = best[0]
        print(f"\n-> best: {c['name']}  ({why})")
        if c.get("vel_gain") is not None:
            print(f"   ./config/odrive-check.sh --save-gains --vel-gain {c['vel_gain']} --int-gain {c['int_gain']} "
                  f"--enc-bw {c['enc_bw']}" + (f" --current-lim {design['current_lim']:g}" if design.get("current_lim") else ""))
        print("   and the feed-forward/limits used here into cart-remote.yaml:")
        for k in ("max_linear_mps", "max_accel_mps2", "friction_ff_nm", "inertia_ff"):
            print(f"     {k}: {kw[k]}")
    save_model("step", {"settings": kw, "results": [{"cand": c_, "fast_ripple": f, "slow_ripple": sl,
                                                       "overshoot": ov, "rise": ri, "jitter": j}
                                                      for c_, f, sl, ov, ri, j in scored]})
    return 0


def step_metrics(log, speeds, circ, hold):
    print("   segment  target   rise90 L|R   overshoot   ripple   current mean / jitter   stop")
    rows = []
    for s_ in speeds:
        for sg in (+1, -1):
            seg = f"{'+' if sg > 0 else '-'}{s_}"
            pts = [p for p in log if p["seg"] == seg]
            if not pts:
                continue
            tgt = s_ / circ
            ts = pts[0]["t"]
            row = {"v": s_, "rise": [], "overshoot": [], "ripple": [], "amps": [], "jitter": []}
            for w in ("L", "R"):
                raw = [p[w] * sg for p in pts]
                t = [p["t"] - ts for p in pts]
                # rise and overshoot on a 5-tick (~0.17 s) moving average: one Hall
                # noise spike is not an overshoot. Ripple stays on the raw estimate.
                v = [statistics.mean(raw[max(0, k - 4):k + 1]) for k in range(len(raw))]
                rise = next((tt for tt, vv in zip(t, v) if vv >= 0.9 * tgt), float("nan"))
                tail = [vv for tt, vv in zip(t, raw) if tt > hold - 1.0]
                amps = [p[w + "A"] * sg for p, tt in zip(pts, t) if tt > hold - 1.0]
                row["rise"].append(rise)
                row["overshoot"].append(max(v) / tgt - 1)
                row["ripple"].append(statistics.pstdev(tail) / tgt if len(tail) > 2 else float("nan"))
                row["amps"].append(statistics.mean(amps) if amps else float("nan"))
                row["jitter"].append(statistics.pstdev(amps) if len(amps) > 2 else float("nan"))
            st = [p for p in log if p["seg"] == "stop" + seg]
            row["stop"] = next((p["t"] - st[0]["t"] for p in st if abs(p["L"]) < 0.03 and abs(p["R"]) < 0.03),
                               float("nan")) if st else float("nan")
            rows.append(row)
            r_, o_, p_, a_, j_ = row["rise"], row["overshoot"], row["ripple"], row["amps"], row["jitter"]
            print(f"   {seg:>7} {tgt:5.2f}t/s  {r_[0]:4.2f}|{r_[1]:4.2f}s  {100 * o_[0]:+4.0f}|{100 * o_[1]:+4.0f}%  "
                  f"{100 * p_[0]:3.0f}|{100 * p_[1]:3.0f}%   {a_[0]:4.1f}|{a_[1]:4.1f} A / {j_[0]:3.1f}|{j_[1]:3.1f} A   "
                  f"{row['stop']:.2f}s")
    return rows


# ------------------------------------------------------------------------ main

def floats(s: str) -> list[float]:
    return [float(x) for x in s.split(",") if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=("blocks", "floor", "design", "step"))
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--room", type=float, default=2.0, help="floor: metres clear ahead and behind")
    ap.add_argument("--speeds", type=floats, default=None,
                    help="floor: friction sweep, m/s (0.1,0.2,0.3,0.5); step: (0.05,0.15,0.3)")
    ap.add_argument("--bw-sweep", type=floats, default=None,
                    help="floor: also measure estimate noise at these encoder bandwidths, e.g. 25,50,100")
    ap.add_argument("--motor-peak", type=float, default=15.0, help="design: motor current ceiling, A")
    ap.add_argument("--mass-kg", type=float, default=7.5,
                    help="design: cart mass from a scale (7.5 kg = 16.5 lb, weighed 2026-10-23); 0 = infer")
    ap.add_argument("--drive-share", type=float, default=0.6,
                    help="design: fraction of the weight on the drive wheels (traction estimate)")
    ap.add_argument("--accel-cap", type=float, default=1.0, help="design: acceleration cap, m/s^2")
    ap.add_argument("--speed-cap", type=float, default=1.0, help="design: top-speed cap, m/s")
    ap.add_argument("--hold", type=float, default=2.5, help="step: seconds at each speed")
    for k in ("max_accel_mps2", "friction_ff_nm", "inertia_ff", "max_linear_mps"):
        ap.add_argument("--" + k.replace("_", "-"), dest=k, type=float, default=None,
                        help="step: override cart-remote.yaml for this run")
    ap.add_argument("--candidates", action="store_true",
                    help="step: run the design's soft/medium/stiff gains with its feed-forward, and pick")
    ap.add_argument("--vel-gain", type=float, default=None, help="step: try this vel_gain (RAM only)")
    ap.add_argument("--int-gain", type=float, default=None)
    ap.add_argument("--enc-bw", type=float, default=None)
    args = ap.parse_args()
    if args.speeds is None:
        args.speeds = [0.1, 0.2, 0.3, 0.5] if args.stage == "floor" else [0.05, 0.15, 0.3]
    return {"blocks": cmd_blocks, "floor": cmd_floor, "design": cmd_design, "step": cmd_step}[args.stage](args)


if __name__ == "__main__":
    raise SystemExit(main())
