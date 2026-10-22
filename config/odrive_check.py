#!/usr/bin/env python
"""The ODESC on the bench: is it there, what is it set to, does it survive a cut?

    ./config/odrive-check.sh            report: serial, firmware, bus voltage,
                                        the bus-limit config, errors
    ./config/odrive-check.sh --apply    write the cart's bus limits (below),
                                        save, reboot, re-read and compare
    ./config/odrive-check.sh --watch    poll the board until Ctrl-C and report
                                        every disappearance and return - the
                                        XH-M609 relay-drop test

Those three touch no axis and draw milliamps: safe on any supply. The motor
bring-up below, in order, once the wheels are wired (phases on M0/M1, Halls
on M0_A/B/Z and M1_A/B/Z with their 5V and GND):

    --halls                       turn each wheel ONE revolution by hand:
                                  counts the Hall edges -> pole pairs, and
                                  checks for illegal Hall states. No current.
    --setup-motors --pole-pairs N write the hub-motor config to both axes
                                  (ODrive's hoverboard settings), save, reboot
    --calibrate [--axis 0|1]      motor + Hall offset calibration. The wheel
                                  beeps, then turns slowly. ON THE DRILL PACK,
                                  WHEELS OFF THE GROUND. Saves pre_calibrated
                                  so the robot can enter closed loop at boot.
    --spin --axis N [--speed 0.5] closed-loop velocity for a few seconds:
                                  commanded vs measured, direction, current

axis0 (M0) is the RIGHT wheel and axis1 (M1) the LEFT (odrive_base.py
RIGHT_AXIS / LEFT_AXIS). With the battery terminals on the left edge, M0 is
the right-hand phase block and M1 the left-hand one, so each motor wires
straight to its own side of the cart.

The limits are the ones recorded in the power doc for firmware 0.5.1, which
has no enable_brake_resistor: the resistor is in use whenever
brake_resistance > 0. They are written as a dict so --apply and the report's
"differs from target" column cannot drift apart.
"""

import argparse
import sys
import time

# axis -> wheel. Must match RIGHT_AXIS / LEFT_AXIS in odrive_base.py.
SIDE = {0: "RIGHT", 1: "LEFT"}

TARGET = {
    "brake_resistance": 2.0,                 # the fitted 2 ohm / 50 W
    "dc_max_positive_current": 10.0,         # XH-M609 relay rating at 16-21 V
    "dc_max_negative_current": -0.5,         # fault threshold; regen goes to the resistor
    "max_regen_current": 0.0,                # none back into the pack
    "dc_bus_undervoltage_trip_level": 16.5,  # above the XH-M609's 16.0 V cut
    "dc_bus_overvoltage_trip_level": 26.0,   # 5S pack maxes at 21 V
    "enable_dc_bus_overvoltage_ramp": False,
}


def _find(timeout: float):
    import odrive
    return odrive.find_any(timeout=timeout)


def _report(o) -> bool:
    """Print the state. Returns True when every target field matches."""
    from odrive.utils import dump_errors

    print(f"serial   {o.serial_number}")
    print(f"firmware {o.fw_version_major}.{o.fw_version_minor}.{o.fw_version_revision}")
    print(f"vbus     {o.vbus_voltage:.2f} V")
    armed = getattr(o, "brake_resistor_armed", None)
    if armed is not None:
        print(f"brake resistor armed: {armed}")
    print()
    ok = True
    print(f"  {'field':<34}{'now':>10}   {'target':>8}")
    for k, want in TARGET.items():
        have = getattr(o.config, k, None)
        match = have is not None and (have == want or (isinstance(want, float) and abs(have - want) < 1e-6))
        ok = ok and match
        flag = "" if match else "   <- differs"
        print(f"  {k:<34}{str(have):>10}   {str(want):>8}{flag}")
    print()
    for i, ax in enumerate((o.axis0, o.axis1)):
        print(f"  axis{i}: state {ax.current_state}  error 0x{ax.error:04x}  "
              f"motor 0x{ax.motor.error:04x}  encoder 0x{ax.encoder.error:04x}")
    print()
    dump_errors(o)
    return ok


def cmd_report(args) -> int:
    o = _find(args.timeout)
    ok = _report(o)
    print("config matches the cart's bus limits." if ok else
          "config differs from the cart's bus limits - run with --apply.")
    return 0 if ok else 1


def _uptime_ms(o):
    stats = getattr(o, "system_stats", None)
    return getattr(stats, "uptime", None) if stats is not None else None


def _save_and_reboot(o, timeout: float):
    """Save to flash, reboot, and PROVE both happened. Returns the fresh board.

    The first version of --apply (2026-10-11) called save_configuration(),
    ignored what it returned, swallowed the reboot's exception, and re-read
    the settings - from RAM, on a board that had not necessarily saved or
    rebooted. It printed "applied and verified." On 2026-10-19 the board
    reported factory bus limits. So: check save's return value, and accept
    the reboot only if the board's uptime went back to near zero.
    """
    saved = o.save_configuration()
    if saved is False:
        raise RuntimeError(
            "save_configuration() returned False - nothing was written to flash. "
            "On 0.5.x that usually means an axis was not IDLE.")
    before = _uptime_ms(o)
    print("saved. rebooting - the USB device goes away for a few seconds, which is normal.")
    try:
        o.reboot()
    except Exception:  # noqa: BLE001 - the reboot tears the connection down
        pass
    time.sleep(3.0)
    fresh = _find(max(timeout, 20.0))
    after = _uptime_ms(fresh)
    if before is not None and after is not None:
        if after >= before:
            raise RuntimeError(
                f"the board did not reboot (uptime {after} ms, was {before} ms) - the "
                "settings read back next would come from RAM, not flash. Power-cycle it "
                "and run this again.")
        print(f"rebooted (uptime {after} ms).")
    else:
        print("rebooted (this firmware does not report uptime, so not independently confirmed:")
        print("power-cycle the board and run the plain report to be sure the values stuck).")
    return fresh


def cmd_apply(args) -> int:
    o = _find(args.timeout)
    print(f"serial {o.serial_number}, vbus {o.vbus_voltage:.2f} V - writing bus limits")
    for k, v in TARGET.items():
        if not hasattr(o.config, k):
            print(f"  this firmware has no config.{k}; skipping", file=sys.stderr)
            continue
        setattr(o.config, k, v)
        print(f"  {k} = {v}")
    o = _save_and_reboot(o, args.timeout)
    print("\nre-reading from the rebooted board:\n")
    ok = _report(o)
    print("applied and verified." if ok else "SOMETHING DID NOT STICK - see the 'differs' rows.")
    return 0 if ok else 1


def cmd_watch(args) -> int:
    """Poll the board; say when it vanishes and when it is back.

    This is the XH-M609 test: with the board behind the module, raise the
    cut-off threshold above the pack voltage, confirm the board drops here,
    lower it, confirm the board comes back - and that it comes back to
    IDLE with no error, which is what the robot's re-arm logic expects.
    """
    import odrive
    print("watching; Ctrl-C to stop. Now trip the XH-M609 and restore it.\n")
    o = None
    present = False
    t_lost = None
    n_lost = 0
    try:
        while True:
            try:
                if o is None:
                    o = odrive.find_any(timeout=1.0)
                v = o.vbus_voltage
                states = (o.axis0.current_state, o.axis1.current_state)
                errs = (o.axis0.error, o.axis1.error)
                if not present:
                    present = True
                    gone = "" if t_lost is None else f" after {time.perf_counter() - t_lost:.1f} s away"
                    print(f"{time.strftime('%H:%M:%S')}  board present{gone}: vbus {v:.2f} V, "
                          f"axis states {states}, errors {errs}", flush=True)
                else:
                    print(f"\r{time.strftime('%H:%M:%S')}  vbus {v:6.2f} V  states {states}  "
                          f"errors {errs}   ", end="", flush=True)
                time.sleep(0.5)
            except (TimeoutError, Exception) as e:  # fibre: ChannelBrokenException, ObjectLostError
                if present:
                    present = False
                    t_lost = time.perf_counter()
                    n_lost += 1
                    print(f"\n{time.strftime('%H:%M:%S')}  board GONE ({type(e).__name__})", flush=True)
                o = None
                time.sleep(0.5)
    except KeyboardInterrupt:
        print(f"\n\n{n_lost} loss/return cycle(s) seen.")
    return 0


# ------------------------------------------------------------- motors
#
# Hub motors are hoverboard-class: many poles, low KV, Hall sensors. These
# are ODrive's own hoverboard-guide values for 0.5.x, which is the firmware
# here. pole_pairs and KV come from the wheel; --halls measures the first.

def _enum(name: str, default: int) -> int:
    try:
        import odrive.enums as e
        return getattr(e, name)
    except Exception:  # noqa: BLE001
        return default


def _axes(o, which):
    pairs = [(0, o.axis0), (1, o.axis1)]
    return [(i, a) for i, a in pairs if which in (None, i)]


def _clear(o) -> None:
    for fn in (getattr(o, "clear_errors", None),):
        if fn:
            try:
                fn()
                return
            except Exception:  # noqa: BLE001
                pass
    try:
        import contextlib
        import io
        from odrive.utils import dump_errors
        with contextlib.redirect_stdout(io.StringIO()):   # 0.5.1: no clear_errors(); this clears, loudly
            dump_errors(o, True)
    except Exception:  # noqa: BLE001
        pass


def _axis_errors(ax) -> dict[str, int]:
    return {
        "axis": getattr(ax, "error", 0),
        "motor": getattr(ax.motor, "error", 0),
        "encoder": getattr(ax.encoder, "error", 0),
        "controller": getattr(ax.controller, "error", 0),
    }


# The 0.5.x codes this bring-up actually meets, so the output says what
# happened instead of a hex word to look up.
_ERROR_NAMES = {
    "axis": {0x1: "INVALID_STATE", 0x40: "MOTOR_FAILED", 0x100: "ENCODER_FAILED",
             0x200: "CONTROLLER_FAILED", 0x800: "WATCHDOG_TIMER_EXPIRED"},
    "motor": {0x1: "PHASE_RESISTANCE_OUT_OF_RANGE", 0x2: "PHASE_INDUCTANCE_OUT_OF_RANGE",
              0x8: "DRV_FAULT", 0x10: "CONTROL_DEADLINE_MISSED", 0x80: "MODULATION_MAGNITUDE",
              0x400: "CURRENT_SENSE_SATURATION", 0x1000: "CURRENT_LIMIT_VIOLATION"},
    "encoder": {0x1: "UNSTABLE_GAIN", 0x2: "CPR_POLEPAIRS_MISMATCH", 0x4: "NO_RESPONSE",
                0x8: "UNSUPPORTED_ENCODER_MODE", 0x10: "ILLEGAL_HALL_STATE",
                0x20: "INDEX_NOT_FOUND_YET", 0x100: "HALL_NOT_CALIBRATED_YET"},
}


def _fmt_errors(errs: dict[str, int]) -> str:
    bad = {k: v for k, v in errs.items() if v}
    if not bad:
        return "none"
    out = []
    for k, v in bad.items():
        names = [n for bit, n in _ERROR_NAMES.get(k, {}).items() if v & bit]
        out.append(f"{k} 0x{v:x}" + (f" ({', '.join(names)})" if names else ""))
    return "  ".join(out)


def _yes(prompt: str) -> bool:
    try:
        return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def cmd_halls(args) -> int:
    o = _find(args.timeout)
    hall = _enum("ENCODER_MODE_HALL", 1)
    print("Hall check. No current flows; the wheels are turned by hand.\n")
    result = {}
    for i, ax in _axes(o, args.axis):
        side = SIDE[i]
        ax.encoder.config.mode = hall          # RAM only; --setup-motors saves it
        time.sleep(0.2)
        print(f"axis{i} ({side} wheel). Put a mark on the tyre and line it up with something fixed.")
        try:
            input("  ENTER to start counting, then turn the wheel exactly ONE revolution: ")
        except (EOFError, KeyboardInterrupt):
            print(); return 1
        start = ax.encoder.shadow_count
        seen_states = set()
        print("  turning... ENTER when the mark is back where it started.")
        import threading
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        while not done.is_set():
            st = getattr(ax.encoder, "hall_state", None)
            if st is not None:
                seen_states.add(int(st))
            cnt = ax.encoder.shadow_count - start
            print(f"\r  edges {cnt:+5d}   hall state {st}   states seen {sorted(seen_states)}   ",
                  end="", flush=True)
            time.sleep(0.01)
        edges = abs(ax.encoder.shadow_count - start)
        pp = edges / 6.0
        print(f"\n  {edges} Hall edges in one turn -> {pp:.2f} pole pairs")
        if edges == 0:
            print(f"  axis{i} saw NO movement. Either the wheel you turned is not on this axis")
            print(f"  (axis{i} should be the {side} wheel: M{i} phases AND M{i}_A/B/Z Halls), or")
            print("  this wheel's Halls are not connected.")
        illegal = seen_states & {0, 7}
        if illegal:
            print(f"  ILLEGAL Hall state(s) {sorted(illegal)}: a Hall line is unconnected, has no")
            print("  5 V, or is noisy. ODrive v3 boards often need ~22 nF from each Hall line to")
            print("  GND with hub motors. Fix before calibrating.")
        if seen_states and len(seen_states - {0, 7}) < 6:
            print(f"  only {len(seen_states - {0, 7})} of 6 Hall states seen - turn more slowly, or a")
            print("  sensor is not switching.")
        errs = _fmt_errors(_axis_errors(ax))
        if errs != "none":
            print(f"  errors: {errs}")
        result[i] = pp
        print()
    pps = {i: round(v) for i, v in result.items()}
    if pps:
        print("pole pairs:", ", ".join(f"axis{i} {v}" for i, v in pps.items()))
        if any(v == 0 for v in pps.values()):
            print("\nan axis counted nothing - fix that (see above) and repeat before --setup-motors")
        elif len(set(pps.values())) == 1 and all(abs(result[i] - pps[i]) < 0.25 for i in pps):
            print(f"\nnext:  ./config/odrive-check.sh --setup-motors --pole-pairs {next(iter(pps.values()))}")
        else:
            print("\nthe two do not agree, or are not whole numbers - repeat, counting more carefully")
    return 0


def cmd_setup_motors(args) -> int:
    if not args.pole_pairs:
        print("--setup-motors needs --pole-pairs (measure it with --halls)", file=sys.stderr)
        return 2
    o = _find(args.timeout)
    pp, kv = args.pole_pairs, args.kv
    cpr = 6 * pp
    kt = 8.27 / kv
    cfg = {
        "motor.config.pole_pairs": pp,
        "motor.config.resistance_calib_max_voltage": 6.0,   # 4 (hoverboard guide) rejects the left wheel at 412 mOhm
        "motor.config.requested_current_range": 25.0,
        "motor.config.current_control_bandwidth": 100.0,
        "motor.config.torque_constant": kt,
        "motor.config.current_lim": args.current_lim,
        "encoder.config.mode": _enum("ENCODER_MODE_HALL", 1),
        "encoder.config.cpr": cpr,
        "encoder.config.calib_scan_distance": 150.0,
        "encoder.config.bandwidth": 100.0,
        "controller.config.control_mode": _enum("CONTROL_MODE_VELOCITY_CONTROL", 2),
        "controller.config.pos_gain": 1.0,
        "controller.config.vel_gain": 0.02 * kt * cpr,
        "controller.config.vel_integrator_gain": 0.1 * kt * cpr,
        "controller.config.vel_limit": args.vel_limit,
    }
    print(f"pole pairs {pp}, KV {kv:g} -> torque constant {kt:.3f} Nm/A, Hall cpr {cpr}\n")
    for i, ax in _axes(o, None):
        for path, v in cfg.items():
            obj = ax
            *parents, leaf = path.split(".")
            for p_ in parents:
                obj = getattr(obj, p_)
            if not hasattr(obj, leaf):
                print(f"  axis{i}: firmware has no {path}; skipping")
                continue
            setattr(obj, leaf, v)
        print(f"  axis{i}: written")
    print()
    o = _save_and_reboot(o, args.timeout)
    for i, ax in _axes(o, None):
        print(f"  axis{i}: pole_pairs {ax.motor.config.pole_pairs}, cpr {ax.encoder.config.cpr}, "
              f"mode {ax.encoder.config.mode}, vel_gain {ax.controller.config.vel_gain:.3f}")
    print("\nnext, on the drill pack with the wheels off the ground:")
    print("  ./config/odrive-check.sh --calibrate")
    return 0


def _wait_idle(o, ax, what: str, timeout: float) -> float:
    """Wait for the axis to return to IDLE. Returns the lowest vbus seen."""
    idle = _enum("AXIS_STATE_IDLE", 1)
    t0 = time.time()
    vmin = o.vbus_voltage
    time.sleep(0.3)
    while ax.current_state != idle:
        vmin = min(vmin, o.vbus_voltage)
        if time.time() - t0 > timeout:
            ax.requested_state = idle
            raise TimeoutError(f"{what} did not finish in {timeout:.0f} s")
        time.sleep(0.1)
    return vmin


def cmd_calibrate(args) -> int:
    o = _find(args.timeout)
    v = o.vbus_voltage
    print(f"vbus {v:.2f} V")
    print("Motor calibration pushes several amps through each motor (it beeps), then the")
    print("wheel turns slowly for the Hall offset. Drill pack only - not a bench supply or")
    print("USB-C, which fold or get back-fed. Cart on blocks: the wheel WILL turn.")
    if not (16.5 <= v <= 21.6):
        print(f"\nvbus {v:.1f} V is outside a 5S pack's 16.5-21 V. Refusing.", file=sys.stderr)
        return 1
    if args.cal_max_voltage is not None and not (0 < args.cal_max_voltage <= min(8.0, 0.4 * v)):
        print(f"--cal-max-voltage must be above 0 and at most {min(8.0, 0.4 * v):.1f} V here",
              file=sys.stderr)
        return 2
    if not _yes("\nOn the drill pack, both wheels clear of the ground?"):
        return 1
    motor_cal = _enum("AXIS_STATE_MOTOR_CALIBRATION", 4)
    hall_pol = _enum("AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION", -1)
    offset_cal = _enum("AXIS_STATE_ENCODER_OFFSET_CALIBRATION", 7)
    ok_all = True
    succeeded = []
    for i, ax in _axes(o, args.axis):
        side = SIDE[i]
        print(f"\n--- axis{i} ({side}) ---")
        _clear(o)
        if args.cal_max_voltage is not None:
            # The test current stays the same; this only lets the board apply
            # more voltage to reach it, i.e. accept a higher-resistance motor.
            ax.motor.config.resistance_calib_max_voltage = args.cal_max_voltage
        # (label, state, optional). Hall polarity calibration exists in the
        # python package's enums but not in every 0.5.x firmware: 0.5.1
        # answers it with AXIS_ERROR_INVALID_STATE (seen 2026-10-19). Offset
        # calibration works on Halls without it.
        steps = [("motor calibration (beep)", motor_cal, False)]
        if hall_pol >= 0:
            steps.append(("Hall polarity", hall_pol, True))
        steps.append(("Hall offset (wheel turns)", offset_cal, False))
        ok = True
        for label, state, optional in steps:
            print(f"  {label}...", end="", flush=True)
            ax.requested_state = state
            try:
                vmin = _wait_idle(o, ax, label, 40.0)
            except TimeoutError as e:
                print(f" {e}"); ok = False; break
            errs = _axis_errors(ax)
            if optional and errs["axis"] == 0x1 and not any(v for k, v in errs.items() if k != "axis"):
                print(" not supported by this firmware - skipped")
                _clear(o)
                continue
            print(f" done, vbus min {vmin:.1f} V, errors {_fmt_errors(errs)}")
            if state == motor_cal:
                m = ax.motor.config
                i_cal = getattr(m, "calibration_current", float("nan"))
                v_max = getattr(m, "resistance_calib_max_voltage", float("nan"))
                if errs["motor"] and m.phase_resistance == 0:
                    print("    phase resistance not measured (the board keeps only a passing value)")
                else:
                    print(f"    measured phase resistance {m.phase_resistance * 1000:.0f} mOhm, "
                          f"inductance {m.phase_inductance * 1e6:.0f} uH")
                print(f"    (test current {i_cal:g} A, voltage allowed {v_max:g} V -> "
                      f"resistance must be under {1000 * v_max / i_cal:.0f} mOhm)")
                if errs["motor"] & 0x1:
                    print("    PHASE_RESISTANCE_OUT_OF_RANGE: the test could not drive its current")
                    print("    within the allowed voltage. An open or loose phase connection reads")
                    print("    as enormous resistance - check all three screw terminals and any")
                    print("    extension joints on this wheel. A genuinely high-resistance motor")
                    print("    instead reads just over the limit (compare with the other wheel).")
                    print("    To tell them apart: rerun this axis with --cal-max-voltage 6 (limit")
                    print("    600 mOhm). Passes near the other wheel's value = just the motor;")
                    print("    still fails = an open or loose phase.")
            if any(errs.values()):
                ok = False
                break
        if ok:
            m = ax.motor.config
            print(f"  Hall offset {getattr(ax.encoder.config, 'offset', '?')}")
            m.pre_calibrated = True
            ax.encoder.config.pre_calibrated = True
            succeeded.append(i)
            print("  marked pre_calibrated")
        else:
            print("  NOT calibrated. Common causes: a phase lead loose (motor error), Hall")
            print("  wiring or noise (encoder error: ILLEGAL_HALL_STATE, CPR_POLEPAIRS_MISMATCH),")
            print("  wrong pole_pairs, or the supply sagging (DC_BUS_UNDER_VOLTAGE).")
            ok_all = False
    if succeeded:
        # Save whatever passed, so a good wheel is not recalibrated because
        # the other one failed. A failed axis is simply not pre_calibrated.
        print(f"\nsaving axis {', '.join(map(str, succeeded))} (pre_calibrated survives reboot)")
        o = _save_and_reboot(o, args.timeout)
        for i, ax in _axes(o, args.axis):
            print(f"  axis{i}: motor pre_calibrated {ax.motor.config.pre_calibrated}, "
                  f"encoder pre_calibrated {ax.encoder.config.pre_calibrated}")
    if ok_all:
        print("next:  ./config/odrive-check.sh --spin --axis 0   (then --axis 1)")
    else:
        failed = [i for i, _ in _axes(o, args.axis) if i not in succeeded]
        print(f"\nfix axis {', '.join(map(str, failed))} and rerun with --axis "
              f"{failed[0] if len(failed) == 1 else '0/1'}; the saved one(s) need not be redone.")
    return 0 if ok_all else 1


def cmd_spin(args) -> int:
    if args.axis is None:
        print("--spin needs --axis 0 or 1", file=sys.stderr)
        return 2
    o = _find(args.timeout)
    ax = dict(_axes(o, None))[args.axis]
    side = SIDE[args.axis]
    if not _yes(f"Spin axis{args.axis} ({side}) at {args.speed:+g} turns/s for {args.seconds:g} s, wheel clear?"):
        return 1
    closed = _enum("AXIS_STATE_CLOSED_LOOP_CONTROL", 8)
    idle = _enum("AXIS_STATE_IDLE", 1)
    _clear(o)
    ax.controller.config.control_mode = _enum("CONTROL_MODE_VELOCITY_CONTROL", 2)
    ax.controller.config.input_mode = _enum("INPUT_MODE_PASSTHROUGH", 1)
    ax.controller.input_vel = 0.0
    ax.requested_state = closed
    time.sleep(0.5)
    if ax.current_state != closed:
        print(f"did not enter closed loop: state {ax.current_state}, errors {_fmt_errors(_axis_errors(ax))}")
        print("calibrated? (./config/odrive-check.sh --calibrate)")
        ax.requested_state = idle
        return 1
    vels, amps = [], []
    try:
        ax.controller.input_vel = args.speed
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            vels.append(ax.encoder.vel_estimate)
            amps.append(abs(getattr(ax.motor.current_control, "Iq_measured", 0.0)))
            time.sleep(0.02)
    finally:
        ax.controller.input_vel = 0.0
        time.sleep(0.5)
        ax.requested_state = idle
    steady = vels[len(vels) // 2:] or vels
    mean = sum(steady) / len(steady)
    print(f"commanded {args.speed:+.2f} turns/s, measured {mean:+.2f} turns/s "
          f"(steady half), current up to {max(amps):.1f} A, errors {_fmt_errors(_axis_errors(ax))}")
    print("Note which way the wheel turned relative to 'forward' for the cart - that")
    print("sets invert_left_wheel / invert_right_wheel in cart-remote.yaml.")
    good = abs(mean - args.speed) < 0.25 * abs(args.speed) and not any(_axis_errors(ax).values())
    print("PASS" if good else "FAIL - wheel bound, wrong pole pairs, or gains need tuning")
    return 0 if good else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--apply", action="store_true", help="write the bus limits, save, reboot, verify")
    g.add_argument("--watch", action="store_true", help="poll until Ctrl-C; report loss and return")
    g.add_argument("--halls", action="store_true", help="count Hall edges over one hand-turned revolution")
    g.add_argument("--setup-motors", action="store_true", help="write the hub-motor config, save, reboot")
    g.add_argument("--calibrate", action="store_true", help="motor + Hall calibration (drill pack, wheels up)")
    g.add_argument("--spin", action="store_true", help="closed-loop velocity test on one axis")
    ap.add_argument("--axis", type=int, choices=(0, 1), default=None, help="0 = right wheel (M0), 1 = left wheel (M1); default both")
    ap.add_argument("--pole-pairs", type=int, default=None)
    ap.add_argument("--cal-max-voltage", type=float, default=None,
                    help="--calibrate: voltage the resistance test may use (default as configured: 6 V from --setup-motors); "
                         "raises the resistance limit, not the current")
    ap.add_argument("--kv", type=float, default=16.0, help="motor KV, rpm/V (hoverboard-class ~16)")
    ap.add_argument("--current-lim", type=float, default=15.0, help="motor current limit, A")
    ap.add_argument("--vel-limit", type=float, default=10.0, help="turns/s")
    ap.add_argument("--speed", type=float, default=0.5, help="--spin, turns/s")
    ap.add_argument("--seconds", type=float, default=3.0, help="--spin duration")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the board")
    args = ap.parse_args()
    try:
        if args.apply:
            return cmd_apply(args)
        if args.watch:
            return cmd_watch(args)
        if args.halls:
            return cmd_halls(args)
        if args.setup_motors:
            return cmd_setup_motors(args)
        if args.calibrate:
            return cmd_calibrate(args)
        if args.spin:
            return cmd_spin(args)
        return cmd_report(args)
    except RuntimeError as e:
        print(f"\nFAILED: {e}", file=sys.stderr)
        return 1
    except TimeoutError:
        print("no ODrive found. Is the board powered (its USB only works with DC on the "
              "terminals), and is the udev rule installed? `lsusb | grep 1209`", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
