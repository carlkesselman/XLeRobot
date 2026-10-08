#!/usr/bin/env python
"""Print live axis and button numbers for a connected gamepad.

Run this BEFORE wiring anything to the robot. Axis and button indices differ
by controller, by firmware, and by input mode - an 8BitDo pad reports a
different layout in X-input than in D-input or Switch mode. Rather than guess,
read the real numbers here and put them in the config.

    python -m lerobot_teleoperator_xlerobot_leader_gamepad.probe_gamepad

Push one stick at a time and note which index moves, then press each button.
Set axis_base_forward / axis_base_strafe / axis_base_turn / axis_head_pan /
axis_head_tilt and the button_* fields accordingly. If a stick moves the wrong
way, flip the matching invert_* flag.

For an 8BitDo pad, put it in X-input mode first - the button combination is on
the sticker underneath.
"""

import sys
import time


def main() -> int:
    try:
        import pygame
    except ImportError:
        print("pygame is missing. Install the gamepad extra:", file=sys.stderr)
        print("  uv pip install 'lerobot[gamepad]'", file=sys.stderr)
        return 1

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No gamepad detected.", file=sys.stderr)
        print("  - is it powered and paired?", file=sys.stderr)
        print("  - 8BitDo: X-input mode (see the sticker underneath)", file=sys.stderr)
        print("  - check it appears in /dev/input/js* or /dev/input/event*", file=sys.stderr)
        return 1

    for i in range(pygame.joystick.get_count()):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f"[{i}] {j.get_name()}  axes={j.get_numaxes()} buttons={j.get_numbuttons()} hats={j.get_numhats()}")

    js = pygame.joystick.Joystick(0)
    print("\nMove one stick at a time, then press each button. Ctrl-C to stop.\n")

    try:
        while True:
            pygame.event.pump()
            axes = {i: round(js.get_axis(i), 2) for i in range(js.get_numaxes())}
            live = {i: v for i, v in axes.items() if abs(v) > 0.2}
            btns = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            hats = [js.get_hat(i) for i in range(js.get_numhats())]
            print(f"\raxes(moving)={live or '-':<40} buttons={btns or '-':<18} hats={hats or '-'}   ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
