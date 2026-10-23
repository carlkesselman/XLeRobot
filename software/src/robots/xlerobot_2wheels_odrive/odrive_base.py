#!/usr/bin/env python

"""Differential-drive base driven by an ODrive motor controller.

This is the only ODrive-specific code in the robot. Everything else - arms,
head, cameras, ZMQ host/client - is unchanged from the Feetech 2-wheel variant.

Targets ODrive **v3.x with 0.5.x firmware**, the same API the scripts under
``software/examples/odrive/`` use:

    odrive.find_any()                              discovery over USB
    odrv.axis0 / odrv.axis1                        RIGHT / LEFT wheel (M0 / M1)
    axis.controller.config.control_mode            velocity control
    axis.controller.input_vel                      command, in turns/s
    axis.encoder.vel_estimate                      feedback, in turns/s
    axis.requested_state                           closed loop

ODrive 0.6+ (S1, Pro) removed ``axis.encoder`` and renamed much of this. If you
move to a newer board or firmware, this file is what needs rewriting - nothing
else should.

Units at the boundary match the rest of lerobot: body velocity is metres per
second for x and **degrees** per second for theta. Conversions to turns/s live
here so the robot class never sees ODrive units.
"""

import logging
import math
import os
import sys
import time
from typing import Any

logger = logging.getLogger(__name__)

# Which ODrive axis drives which wheel. M0 (axis0) sits at the right-hand end
# of the board's top edge and M1 (axis1) at the left, so with the board
# mounted that way round each motor's phase wires go straight to its own side
# of the cart without crossing. config/odrive_check.py labels axes from these.
RIGHT_AXIS = 0
LEFT_AXIS = 1


class ODriveBase:
    """Two-wheel differential drive on one ODrive board."""

    def __init__(
        self,
        wheel_radius: float = 0.0825,
        wheelbase: float = 0.25,
        invert_left: bool = True,
        invert_right: bool = False,
        max_linear_mps: float = 1.0,
        max_accel_mps2: float = 1.0,
        friction_ff_nm: float = 0.0,
        inertia_ff: float = 0.0,
        connect_timeout_s: int = 30,
        serial_number: str | None = None,
    ):
        """
        Args:
            wheel_radius: metres. 0.0825 is the 165 mm wheel the ODrive examples assume.
            wheelbase: metres between the two wheel contact points.
            invert_left/invert_right: the two motors face opposite directions when
                mounted on either side of the chassis, so one of them has to be
                negated. Which one depends on your wiring - if driving forward
                makes the robot spin in place, flip these. The Feetech variant
                hard-codes the left wheel inverted; here it is configurable
                because it depends on how the phases and Halls were wired.
                Settle it with `odrive-check.sh --spin`.
            max_linear_mps: per-wheel clamp, a safety limit on a runaway command.
            max_accel_mps2: per-wheel acceleration limit, applied on the board
                (INPUT_MODE_VEL_RAMP). Teleop sends a new velocity every tick;
                passed straight through, each step is a torque spike, which at
                the high gains a carpet edge needs makes the cart jerk. The
                ramp smooths the command without softening the controller's
                response to a load. It applies to stops too: from 0.5 m/s at
                1.0 m/s^2 the wheels take 0.5 s and ~12 cm. 0 = passthrough.
            friction_ff_nm / inertia_ff: feed-forward torque, per wheel, from
                config/base_characterize.py. Hall feedback is too coarse at
                low speed (one count per 5.8 mm of travel) for the velocity
                loop to supply the torque a slow cart needs without lurching;
                feed-forward supplies the predictable part - rolling friction
                (Nm, Coulomb) and inertia x commanded acceleration (Nm per
                turn/s^2) - and leaves the loop only the error. Either one > 0
                switches the ramp from the board to this class (the board's
                VEL_RAMP overwrites input_torque), PASSTHROUGH with
                input_vel + input_torque each tick. Both 0 = as before.
            serial_number: pick a specific board when more than one is attached.
        """
        self.wheel_radius = wheel_radius
        self.wheelbase = wheelbase
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.max_linear_mps = max_linear_mps
        self.max_accel_mps2 = max_accel_mps2
        self.friction_ff_nm = friction_ff_nm
        self.inertia_ff = inertia_ff
        self._ff = friction_ff_nm > 0 or inertia_ff > 0
        # Software ramp state (feed-forward mode): wheel setpoints, raw turns/s.
        self._sp = [0.0, 0.0]            # left, right
        self._ff_torque = [0.0, 0.0]
        self._sp_t = None
        self.connect_timeout_s = connect_timeout_s
        self.serial_number = serial_number

        self.wheel_circumference = 2.0 * math.pi * wheel_radius

        self.odrv: Any = None
        self.left: Any = None
        self.right: Any = None
        self._connected = False

        # LEROBOT_BASE_TRACE=1 (or =<file>): one line every 0.2 s - the
        # command as it arrived, what each wheel was asked for, what it did,
        # its current, and the axis states - written to base-trace.log in the
        # current directory (a file: teleop's status line overwrites the
        # terminal). For "teleop drives worse than odrive-check --push does".
        tr = os.environ.get("LEROBOT_BASE_TRACE", "")
        self._trace = tr not in ("", "0")
        self._trace_path = "base-trace.log" if tr in ("1", "true", "yes") else tr
        self._trace_file = None
        self._trace_last = 0.0
        self._trace_t0 = time.monotonic()
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)

        # Re-arm bookkeeping. See _ensure_closed_loop().
        self._last_rearm = 0.0
        self._rearms = 0

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        """Find the board and put both axes into closed-loop velocity control.

        Raises ConnectionError rather than exiting, unlike the example script -
        the caller is a robot, not a CLI.
        """
        try:
            import odrive
            from odrive.enums import (
                CONTROL_MODE_VELOCITY_CONTROL,
                INPUT_MODE_PASSTHROUGH,
            )
            try:
                from odrive.enums import INPUT_MODE_VEL_RAMP
            except ImportError:  # same value on every 0.5.x
                INPUT_MODE_VEL_RAMP = 2
        except ImportError as e:
            raise ImportError(
                "The odrive package is required for xlerobot_2wheels_odrive. "
                "Install it with `uv pip install odrive`."
            ) from e

        try:
            if self.serial_number:
                self.odrv = odrive.find_any(
                    serial_number=self.serial_number, timeout=self.connect_timeout_s
                )
            else:
                self.odrv = odrive.find_any(timeout=self.connect_timeout_s)
        except Exception as e:
            raise ConnectionError(f"Could not find an ODrive: {e}") from e

        axes = (self.odrv.axis0, self.odrv.axis1)
        self.left = axes[LEFT_AXIS]
        self.right = axes[RIGHT_AXIS]

        logger.info(
            "ODrive %s connected, vbus %.1fV",
            getattr(self.odrv, "serial_number", "?"),
            self.odrv.vbus_voltage,
        )

        for axis in (self.left, self.right):
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            if self._ff:
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                axis.controller.input_torque = 0.0
            elif self.max_accel_mps2 > 0:
                axis.controller.config.vel_ramp_rate = self.max_accel_mps2 / self.wheel_circumference
                axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            else:
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            # The robot's own control loop is the watchdog; ODrive's would trip
            # on any pause in the lerobot loop.
            axis.config.enable_watchdog = False

        errors = self._enter_closed_loop(settle_s=0.5)
        if any(errors.values()):
            raise ConnectionError(
                f"ODrive axes reported errors after entering closed loop: "
                f"{ {k: hex(v) for k, v in errors.items()} }. "
                "The motors most likely need encoder calibration - see the ODrive docs."
            )

        self._connected = True

    def _enter_closed_loop(self, settle_s: float, clear_sleep_s: float = 0.2) -> dict[str, int]:
        """Clear errors, request closed loop on both axes, return their error words.

        connect() can afford to wait for the board to settle. The re-arm path
        runs INSIDE the control loop's read and must not: it passes zero for
        both sleeps and lets the next tick observe the result.
        """
        from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL

        for axis in (self.left, self.right):
            axis.clear_errors()
        if clear_sleep_s:
            time.sleep(clear_sleep_s)
        for axis in (self.left, self.right):
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        if settle_s:
            time.sleep(settle_s)
        return {
            "left": getattr(self.left, "error", 0),
            "right": getattr(self.right, "error", 0),
        }

    # Re-arm no more often than this, so a board that keeps faulting is not
    # hammered with requests at the control rate.
    REARM_INTERVAL_S = 2.0

    def _ensure_closed_loop(self) -> None:
        """Put the axes back into closed loop if something knocked them out.

        An ODrive that loses its DC bus and gets it back - a low-voltage
        disconnect module tripping and auto-reconnecting near the end of a
        pack, a battery contact bouncing - boots to IDLE. USB comes back,
        reads answer, input_vel is accepted and ignored: a base that is
        silently dead with no error anywhere. The board's own undervoltage
        trip and any fault it latches (overvoltage during regen, say) leave
        it in IDLE the same way.

        Called from the read path, which runs every tick, so it is also the
        detector. One attempt per REARM_INTERVAL_S, with a log line, so a
        genuinely broken axis shows up in the log as a repeating re-arm
        rather than as a mystery.
        """
        from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL

        states = {
            "left": getattr(self.left, "current_state", None),
            "right": getattr(self.right, "current_state", None),
        }
        if all(st == AXIS_STATE_CLOSED_LOOP_CONTROL for st in states.values()):
            return

        now = time.monotonic()
        if now - self._last_rearm < self.REARM_INTERVAL_S:
            return
        self._last_rearm = now
        self._rearms += 1

        errors_before = {
            "left": getattr(self.left, "error", 0),
            "right": getattr(self.right, "error", 0),
        }
        logger.warning(
            "ODrive axes out of closed loop (states %s, errors %s, vbus %.1fV) - re-arming (#%d)",
            states,
            {k: hex(v) for k, v in errors_before.items()},
            getattr(self.odrv, "vbus_voltage", float("nan")),
            self._rearms,
        )
        # No sleeps: this is the control loop's read path. A few USB
        # transactions, then the next tick sees whether it took.
        errors = self._enter_closed_loop(settle_s=0.0, clear_sleep_s=0.0)
        if any(errors.values()):
            logger.warning(
                "ODrive re-arm left errors %s; will retry in %.0fs",
                {k: hex(v) for k, v in errors.items()},
                self.REARM_INTERVAL_S,
            )
        else:
            logger.info("ODrive re-arm requested; next read confirms")

    def send_body_velocity(self, x_mps: float, theta_degps: float) -> None:
        """Command a body velocity. x in m/s, theta in deg/s (positive = left turn)."""
        if not self._connected:
            raise ConnectionError("ODriveBase is not connected")

        theta_rad = math.radians(theta_degps)

        # Differential drive: v_left = v - w*L/2, v_right = v + w*L/2
        left_mps = x_mps - theta_rad * self.wheelbase / 2.0
        right_mps = x_mps + theta_rad * self.wheelbase / 2.0

        left_mps = max(-self.max_linear_mps, min(self.max_linear_mps, left_mps))
        right_mps = max(-self.max_linear_mps, min(self.max_linear_mps, right_mps))

        if self.invert_left:
            left_mps = -left_mps
        if self.invert_right:
            right_mps = -right_mps

        # ODrive velocity is in turns/s.
        if self._ff:
            self._send_ff(left_mps / self.wheel_circumference, right_mps / self.wheel_circumference)
        else:
            self.left.controller.input_vel = left_mps / self.wheel_circumference
            self.right.controller.input_vel = right_mps / self.wheel_circumference
        if self._trace:
            self._last_cmd = (x_mps, theta_degps, left_mps / self.wheel_circumference,
                              right_mps / self.wheel_circumference)

    def read_body_velocity(self) -> dict[str, float]:
        """Measured body velocity as {"x.vel": m/s, "theta.vel": deg/s}."""
        if not self._connected:
            raise ConnectionError("ODriveBase is not connected")

        # Every tick: if the board rebooted or faulted out of closed loop,
        # get it back before reporting a velocity nobody is controlling.
        self._ensure_closed_loop()

        left_mps = self.left.encoder.vel_estimate * self.wheel_circumference
        right_mps = self.right.encoder.vel_estimate * self.wheel_circumference

        # Undo the mounting inversion so both read positive when driving forward.
        if self.invert_left:
            left_mps = -left_mps
        if self.invert_right:
            right_mps = -right_mps

        if self._trace:
            self._trace_line()

        x_vel = (left_mps + right_mps) / 2.0
        theta_rad = (right_mps - left_mps) / self.wheelbase

        return {"x.vel": x_vel, "theta.vel": math.degrees(theta_rad)}

    # Below this wheel speed the friction feed-forward fades out (tanh), so a
    # cart told to stand still is not pushed one way or the other.
    FF_FADE_TPS = 0.03

    def _send_ff(self, left_tps: float, right_tps: float) -> None:
        """Software ramp + feed-forward, one tick. Raw (signed) turns/s."""
        now = time.monotonic()
        dt = 0.0 if self._sp_t is None else min(max(now - self._sp_t, 0.0), 0.1)
        self._sp_t = now
        a_max = self.max_accel_mps2 / self.wheel_circumference if self.max_accel_mps2 > 0 else float("inf")
        for k, (axis, target) in enumerate(((self.left, left_tps), (self.right, right_tps))):
            sp = self._sp[k]
            step = target - sp
            if dt > 0 and a_max != float("inf"):
                step = max(-a_max * dt, min(a_max * dt, step))
            elif dt == 0 and a_max != float("inf"):
                step = 0.0                       # first tick: start from rest
            new = sp + step
            accel = step / dt if dt > 0 else 0.0
            torque = self.inertia_ff * accel + self.friction_ff_nm * math.tanh(new / self.FF_FADE_TPS)
            self._sp[k] = new
            self._ff_torque[k] = torque
            axis.controller.input_vel = new
            axis.controller.input_torque = torque

    def _trace_line(self) -> None:
        now = time.monotonic()
        if now - self._trace_last < 0.2:
            return
        if self._trace_file is None:
            self._trace_file = open(self._trace_path, "w", buffering=1)
            logger.info("base trace -> %s", os.path.abspath(self._trace_path))
            print("  t s   x cmd  th cmd |  L cmd  L vel  L A  L ff |  R cmd  R vel  R A  R ff | states"
                  "   (wheel columns in turns/s, raw - left is inverted; cmd = what the wheel was"
                  " sent this tick, after any software ramp; ff = feed-forward Nm)", file=self._trace_file)
        self._trace_last = now
        x, th, lc, rc = self._last_cmd
        if self._ff:
            lc, rc = self._sp
        lf, rf = self._ff_torque
        def amps(ax):
            return abs(getattr(ax.motor.current_control, "Iq_measured", float("nan")))
        print(f"{now - self._trace_t0:6.1f} {x:+6.3f} {th:+7.1f} | "
              f"{lc:+6.2f} {self.left.encoder.vel_estimate:+6.2f} {amps(self.left):4.1f} {lf:+5.2f} | "
              f"{rc:+6.2f} {self.right.encoder.vel_estimate:+6.2f} {amps(self.right):4.1f} {rf:+5.2f} | "
              f"{self.left.current_state},{self.right.current_state}", file=self._trace_file)

    def stop(self) -> None:
        """Zero both wheels. Safe to call when not connected."""
        if not self._connected:
            return
        try:
            for axis in (self.left, self.right):
                if self._ff:
                    axis.controller.input_torque = 0.0
                axis.controller.input_vel = 0.0
            self._sp = [0.0, 0.0]
            self._ff_torque = [0.0, 0.0]
            self._sp_t = None
        except Exception as e:
            logger.warning("Failed to stop ODrive wheels: %s", e)

    def disconnect(self) -> None:
        """Stop the wheels and drop out of closed loop."""
        if not self._connected:
            return
        self.stop()
        try:
            from odrive.enums import AXIS_STATE_IDLE

            self.left.requested_state = AXIS_STATE_IDLE
            self.right.requested_state = AXIS_STATE_IDLE
        except Exception as e:
            logger.warning("Failed to idle ODrive axes: %s", e)
        finally:
            self._connected = False
            self.odrv = None
            self.left = None
            self.right = None
