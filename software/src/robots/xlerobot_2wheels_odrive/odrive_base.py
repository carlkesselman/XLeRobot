#!/usr/bin/env python

"""Differential-drive base driven by an ODrive motor controller.

This is the only ODrive-specific code in the robot. Everything else - arms,
head, cameras, ZMQ host/client - is unchanged from the Feetech 2-wheel variant.

Targets ODrive **v3.x with 0.5.x firmware**, the same API the scripts under
``software/examples/odrive/`` use:

    odrive.find_any()                              discovery over USB
    odrv.axis0 / odrv.axis1                        left / right
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
import time
from typing import Any

logger = logging.getLogger(__name__)


class ODriveBase:
    """Two-wheel differential drive on one ODrive board."""

    def __init__(
        self,
        wheel_radius: float = 0.0825,
        wheelbase: float = 0.25,
        invert_left: bool = True,
        invert_right: bool = False,
        max_linear_mps: float = 1.0,
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
                because ODrive axis assignment is set in the board's own config.
            max_linear_mps: per-wheel clamp, a safety limit on a runaway command.
            serial_number: pick a specific board when more than one is attached.
        """
        self.wheel_radius = wheel_radius
        self.wheelbase = wheelbase
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.max_linear_mps = max_linear_mps
        self.connect_timeout_s = connect_timeout_s
        self.serial_number = serial_number

        self.wheel_circumference = 2.0 * math.pi * wheel_radius

        self.odrv: Any = None
        self.left: Any = None
        self.right: Any = None
        self._connected = False

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
                AXIS_STATE_CLOSED_LOOP_CONTROL,
                CONTROL_MODE_VELOCITY_CONTROL,
                INPUT_MODE_PASSTHROUGH,
            )
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

        self.left = self.odrv.axis0
        self.right = self.odrv.axis1

        logger.info(
            "ODrive %s connected, vbus %.1fV",
            getattr(self.odrv, "serial_number", "?"),
            self.odrv.vbus_voltage,
        )

        for axis in (self.left, self.right):
            axis.clear_errors()
        time.sleep(0.2)

        for axis in (self.left, self.right):
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            # The robot's own control loop is the watchdog; ODrive's would trip
            # on any pause in the lerobot loop.
            axis.config.enable_watchdog = False

        for axis in (self.left, self.right):
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.5)

        errors = {
            "left": getattr(self.left, "error", 0),
            "right": getattr(self.right, "error", 0),
        }
        if any(errors.values()):
            raise ConnectionError(
                f"ODrive axes reported errors after entering closed loop: "
                f"{ {k: hex(v) for k, v in errors.items()} }. "
                "The motors most likely need encoder calibration - see the ODrive docs."
            )

        self._connected = True

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
        self.left.controller.input_vel = left_mps / self.wheel_circumference
        self.right.controller.input_vel = right_mps / self.wheel_circumference

    def read_body_velocity(self) -> dict[str, float]:
        """Measured body velocity as {"x.vel": m/s, "theta.vel": deg/s}."""
        if not self._connected:
            raise ConnectionError("ODriveBase is not connected")

        left_mps = self.left.encoder.vel_estimate * self.wheel_circumference
        right_mps = self.right.encoder.vel_estimate * self.wheel_circumference

        # Undo the mounting inversion so both read positive when driving forward.
        if self.invert_left:
            left_mps = -left_mps
        if self.invert_right:
            right_mps = -right_mps

        x_vel = (left_mps + right_mps) / 2.0
        theta_rad = (right_mps - left_mps) / self.wheelbase

        return {"x.vel": x_vel, "theta.vel": math.degrees(theta_rad)}

    def stop(self) -> None:
        """Zero both wheels. Safe to call when not connected."""
        if not self._connected:
            return
        try:
            self.left.controller.input_vel = 0.0
            self.right.controller.input_vel = 0.0
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
