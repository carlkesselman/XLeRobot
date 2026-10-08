# xlerobot_2wheels_odrive

Two SO-101 arms, a 2-DOF head, and a **two-wheel differential base driven by an
ODrive** (brushless), with a ZMQ host/client split for running the robot on one
machine and teleoperating it from another.

This is `xlerobot_2wheels` with the base swapped. Arms, head, cameras, the
action space and the host/client protocol are unchanged, so datasets and
policies are interchangeable with the Feetech 2-wheel variant.

> **Untested.** Written before the hardware was assembled. Expect to debug it
> against a real board — start with `software/examples/odrive/test_connection.py`.

## Motor layout

```
bus1 (port1):  left arm 1-6   +  head_motor_1/2 at 7-8      Feetech
bus2 (port2):  right arm 1-6                                 Feetech
ODrive:        axis0 = left wheel, axis1 = right wheel       brushless
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
must be negated — which one depends on your wiring and on how axis0/axis1 are
assigned in the board's own configuration.

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
uv sync --group odrive          # from the XLeRobot workspace root
```

Not a default dependency: it pulls the `odrive` package, which is only useful
with the board attached.
