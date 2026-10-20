# xlerobot_2wheels_odrive

Two SO-101 arms, a 2-DOF head, and a **two-wheel differential base driven by an
ODrive** (brushless), with a ZMQ host/client split for running the robot on one
machine and teleoperating it from another.

This is `xlerobot_2wheels` with the base swapped. Arms, head, cameras, the
action space and the host/client protocol are unchanged, so datasets and
policies are interchangeable with the Feetech 2-wheel variant.

> **Partly tested (2026-10-11).** The ODESC (fw 0.5.1) enumerates, takes its
> bus-limit config and reads back; nothing with wheels has run yet. Expect
> to debug the motor side against the real board.

## One type, three bases

`XLerobotConfig.base` picks the cart's configuration and this type accepts
all three, so one yaml serves every setup and the choice can be made on the
command line (`--robot.base=none`, or `BASE=none` on the lerobot `config/`
scripts):

| `base:` | wheels | bus 2 | base features |
|---|---|---|---|
| `odrive` (default here) | two brushless on the ODESC, USB | right arm 1-6 | `x.vel`, `theta.vel` |
| `omniwheels` | three STS3215, ids 7-9 | right arm + wheels | `x.vel`, `y.vel`, `theta.vel` |
| `none` | — | right arm 1-6 | none; base commands ignored with one warning |

With `omniwheels` or `none` every base hook defers to the parent `XLerobot`
and the ODrive is never opened. The plain `type: xlerobot` accepts
`omniwheels` and `none` only.

## Motor layout

```
bus1 (port1):  left arm 1-6   +  head_motor_1/2 at 7-8      Feetech
bus2 (port2):  right arm 1-6                                 Feetech
ODrive:        axis0 = right wheel, axis1 = left wheel       brushless
```

Note bus2 carries **only** the right arm — the wheels are not on the Feetech
bus at all, unlike `xlerobot_2wheels` where they sit at IDs 9-10.

## Action / observation space

Identical to `xlerobot_2wheels`: 14 joint positions plus `x.vel` (m/s) and
`theta.vel` (deg/s). A differential drive cannot translate sideways, so there
is no `y.vel`.

## ODrive support

Targets **ODrive v3.x with 0.5.x firmware** — the API the scripts in
`software/examples/odrive/` use (`odrive.find_any()`, `odrv.axis0/axis1`,
`axis.controller.input_vel`, `axis.encoder.vel_estimate`).

ODrive 0.6+ (S1, Pro) removed `axis.encoder` and renamed much of this. All of
it is isolated in `odrive_base.py`; a newer board means rewriting that one
file, not the robot.

The plugin pins `odrive>=0.5.4,<0.6` for the same reason.

## Configuration worth checking first

```python
wheel_radius        = 0.0825   # metres. 165 mm wheel. MEASURE YOURS -
                               # an error here scales every velocity.
wheelbase           = 0.25     # metres between wheel contact points
invert_left_wheel   = True     # see below
invert_right_wheel  = False
max_linear_mps      = 1.0      # per-wheel clamp, safety
```

**The inversion flags are the thing to get wrong.** The two motors face
opposite directions mounted either side of the chassis, so exactly one axis
must be negated — which one depends on the wiring (axis0/M0 = right,
axis1/M1 = left). `config/odrive-check.sh --spin --axis N` shows which way
each wheel turns for a positive command.

If driving forward makes the robot **spin in place**, and turning makes it
**drive straight**, the flags are wrong. Flip them.

## Running split

On the robot:

```bash
python -m lerobot_robot_xlerobot_2wheels_odrive.xlerobot_2wheels_odrive_host \
  --robot.id=<id> --robot.port1=<left bus> --robot.port2=<right bus>
```

On the operator machine, `--robot.type=xlerobot_2wheels_odrive_client` with
`remote_ip` pointing at the robot.

## Install

```bash
uv sync                         # from the XLeRobot workspace root
```

A default dependency since the ODrive base became the standard cart base
(2026-10-11); it pulls the `odrive` package. On elroy install
`config/99-odrive-elroy.rules` (lerobot repo) so the board enumerates
without sudo.

## ODESC board wiring

Vendor wiring diagram: [odesc-wiring-diagram.png](odesc-wiring-diagram.png). Battery terminals on the left edge (upper = +); top edge left to right is M1, brake resistor, M0; Hall sensors go to the bottom-right encoder header (M1_A/B/Z and M0_A/B/Z with their own 5V and GND).

**Wheel assignment: M0 = RIGHT wheel, M1 = LEFT wheel.** Mount the board with M0 toward the cart's right side; each motor's phases and Halls then run to the board on its own side with no crossover. The code follows this (`RIGHT_AXIS = 0`, `LEFT_AXIS = 1` in `odrive_base.py`).
