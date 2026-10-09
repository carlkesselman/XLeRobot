#!/usr/bin/env python
"""Find out what your gamepad's axes and buttons are actually numbered.

Run this BEFORE wiring anything to the robot. Indices differ by controller,
by firmware and by input mode - an 8BitDo pad reports one layout in X-input
and another in D-input, and an SF30 Pro in PlayStation mode even spoofs
Sony's vendor id. Guessing produces a robot that drives sideways when you
push forward.

    python -m lerobot_teleoperator_xlerobot_leader_gamepad.probe_gamepad
    python -m lerobot_teleoperator_xlerobot_leader_gamepad.probe_gamepad --live

The default walks you through each control one at a time and prints a YAML
block to paste into config/operator-leader-host.yaml. --live is the raw
stream, for when you just want to watch what a control does.

WHICHEVER MODE THE PAD IS IN WHEN YOU RUN THIS is the mode the numbers
describe. Note it down physically; if the pad ever power-cycles into a
different one, every value here becomes wrong at once.
"""

import argparse
import sys
import time


def _headless_sdl() -> None:
    """Let SDL start without a display.

    The gamepad is read over /dev/input, which needs no video at all, but
    pygame.init() brings up SDL's video subsystem and that fails on a bare
    SSH session - "No available video device". The leader host runs headless
    by design, so this is the normal case, not the exception.
    """
    import os

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")


def _open_pad():
    try:
        import pygame
    except ImportError:
        print("pygame is missing. Install the gamepad extra:", file=sys.stderr)
        print("  uv sync   (lerobot[gamepad] is already a dependency)", file=sys.stderr)
        return None, None

    _headless_sdl()
    pygame.init()
    pygame.joystick.init()

    n = pygame.joystick.get_count()
    if n == 0:
        print("No gamepad detected.", file=sys.stderr)
        print("  - is it powered on and connected?", file=sys.stderr)
        print("  - does it appear in /dev/input/js*?", file=sys.stderr)
        print("  - are you in the 'input' group?  id -nG", file=sys.stderr)
        return None, None

    for i in range(n):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f"[{i}] {j.get_name()}   axes={j.get_numaxes()} "
              f"buttons={j.get_numbuttons()} hats={j.get_numhats()}")
    if n > 1:
        print(f"\nMore than one pad. This probes [0]; set joystick_index to pick another.")
    print()
    return pygame, pygame.joystick.Joystick(0)


# Each step: (prompt, kind). Order matters - it is the order a person can
# follow without re-reading the screen.
STEPS = [
    ("left",   "axis", "LEFT stick  -  push FULLY UP and hold"),
    ("lx",     "axis", "LEFT stick  -  push FULLY RIGHT and hold"),
    ("ry",     "axis", "RIGHT stick -  push FULLY UP and hold"),
    ("rx",     "axis", "RIGHT stick -  push FULLY RIGHT and hold"),
    ("l1",     "any",  "LEFT shoulder (L1) - press and hold"),
    ("r1",     "any",  "RIGHT shoulder (R1) - press and hold"),
    ("l2",     "any",  "LEFT trigger (L2) - squeeze fully and hold"),
    ("r2",     "any",  "RIGHT trigger (R2) - squeeze fully and hold"),
]

AXIS_THRESHOLD = 0.55       # well past any resting drift
SETTLE = 0.25               # everything must return to rest before the next step


def _snapshot(pygame, js):
    pygame.event.pump()
    return (
        [js.get_axis(i) for i in range(js.get_numaxes())],
        [js.get_button(i) for i in range(js.get_numbuttons())],
    )


def _wait_rest(pygame, js, rest_axes, timeout=15.0):
    """Block until nothing is displaced, so one control cannot bleed into
    the reading of the next."""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        axes, btns = _snapshot(pygame, js)
        moved = any(abs(a - r) > 0.3 for a, r in zip(axes, rest_axes))
        if not moved and not any(btns):
            time.sleep(SETTLE)
            axes, btns = _snapshot(pygame, js)
            if not any(abs(a - r) > 0.3 for a, r in zip(axes, rest_axes)) and not any(btns):
                return True
        time.sleep(0.03)
    return False


def _capture(pygame, js, rest_axes, kind, timeout=30.0):
    """Return ('axis', index, value) or ('button', index, None)."""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        axes, btns = _snapshot(pygame, js)

        if kind in ("any",):
            pressed = [i for i, b in enumerate(btns) if b]
            if pressed:
                return ("button", pressed[0], None)

        # Largest displacement from rest wins, so a slightly-off-centre
        # neighbouring stick cannot win over the one being pushed.
        best_i, best_d = None, 0.0
        for i, (a, r) in enumerate(zip(axes, rest_axes)):
            d = abs(a - r)
            if d > best_d:
                best_i, best_d = i, d
        if best_i is not None and best_d > AXIS_THRESHOLD:
            return ("axis", best_i, axes[best_i])

        time.sleep(0.03)
    return (None, None, None)


def guided(pygame, js) -> int:
    print("Each control, one at a time. Let everything return to rest between")
    print("steps - the probe waits for that. Ctrl-C to abandon.\n")

    print("Leave every stick and trigger alone for a moment...", end="", flush=True)
    time.sleep(1.0)
    rest_axes, _ = _snapshot(pygame, js)
    print(" got it.")
    print(f"resting axis values: {[round(v, 2) for v in rest_axes]}\n")

    found: dict[str, tuple] = {}
    for key, kind, prompt in STEPS:
        if not _wait_rest(pygame, js, rest_axes):
            print("  (something is still displaced; carrying on anyway)")
        print(f"  {prompt} ... ", end="", flush=True)
        what, idx, val = _capture(pygame, js, rest_axes, kind)
        if what is None:
            print("nothing detected - skipped")
            continue
        if what == "axis":
            print(f"axis {idx}  ({val:+.2f})")
        else:
            print(f"button {idx}")
        found[key] = (what, idx, val)

    print("\n" + "=" * 62)
    print("PASTE INTO config/operator-leader-host.yaml")
    print("=" * 62)

    def axis_of(k):
        v = found.get(k)
        return v[1] if v and v[0] == "axis" else None

    def idx_of(k):
        v = found.get(k)
        return v[1] if v else None

    def sign_of(k):
        v = found.get(k)
        return v[2] if v and v[0] == "axis" else None

    ly, lx, ry, rx = axis_of("left"), axis_of("lx"), axis_of("ry"), axis_of("rx")
    print()
    print("joystick_index: 0")
    print(f"deadzone: 0.15")
    if ly is not None:
        print(f"axis_base_forward: {ly}")
    if lx is not None:
        print(f"axis_base_strafe: {lx}")
    if rx is not None:
        print(f"axis_base_turn: {rx}")
        print(f"axis_head_pan: {rx}")
    if ry is not None:
        print(f"axis_head_tilt: {ry}")

    # 'up' giving a negative reading is the usual convention; the invert
    # flags exist to turn that back into "up means forward".
    if sign_of("left") is not None:
        print(f"invert_base_forward: {str(sign_of('left') < 0).lower()}")
    if sign_of("ry") is not None:
        print(f"invert_head_tilt: {str(sign_of('ry') < 0).lower()}")

    for cfg_key, step in (("button_head_modifier", "l1"),
                          ("button_speed_up", "r1"),
                          ("button_speed_down", "l2")):
        v = found.get(step)
        if v and v[0] == "button":
            print(f"{cfg_key}: {v[1]}")
        elif v:
            print(f"# {cfg_key}: {step.upper()} is AXIS {v[1]}, not a button - "
                  f"pick a different control")

    print()
    print("Not settable here: base strafe and turn have no invert flag. If the")
    print("cart turns the wrong way in teleoperation, say so and we add one.")
    print()
    return 0


def live(pygame, js) -> int:
    print("Raw stream. Move one control at a time. Ctrl-C to stop.\n")
    seen_axes: dict[int, list] = {}
    seen_btns: list[int] = []
    try:
        while True:
            axes, btns = _snapshot(pygame, js)
            for i, v in enumerate(axes):
                lo, hi = seen_axes.get(i, [v, v])
                seen_axes[i] = [min(lo, v), max(hi, v)]
            for i, b in enumerate(btns):
                if b and i not in seen_btns:
                    seen_btns.append(i)
            moving = {i: round(v, 2) for i, v in enumerate(axes) if abs(v) > 0.2}
            down = [i for i, b in enumerate(btns) if b]
            print(f"\raxes={str(moving) if moving else '-':<42} "
                  f"buttons={str(down) if down else '-':<16}", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")

    print("range seen per axis:")
    for i, (lo, hi) in sorted(seen_axes.items()):
        bar = "  <- moved" if hi - lo > 0.5 else ""
        print(f"  axis {i}:  {lo:+.2f} .. {hi:+.2f}{bar}")
    print(f"buttons pressed, in order: {seen_btns or '(none)'}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true",
                    help="raw stream instead of the guided walkthrough")
    args = ap.parse_args()

    pygame, js = _open_pad()
    if js is None:
        return 1
    try:
        return live(pygame, js) if args.live else guided(pygame, js)
    except KeyboardInterrupt:
        print("\nabandoned")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
