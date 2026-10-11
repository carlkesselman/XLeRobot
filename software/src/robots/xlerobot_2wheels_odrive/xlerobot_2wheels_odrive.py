# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""XLeRobot with a two-wheel brushless differential base on an ODrive.

Everything that is not the base - two SO-101 arms, the pan/tilt head, the
cameras, calibration, the bus retries, the disconnect ordering, the servo
motion profiles - is inherited from :class:`XLerobot`. This class overrides
only the base hooks, so the two robots cannot drift apart: a fix in the
parent is a fix here.

(This file used to be a 500-line copy of the parent with the base swapped.
It had none of the hardening the parent gained afterwards.)

The ODrive itself is in :mod:`odrive_base`; this class never sees turns/s.
"""

import logging

import numpy as np

from lerobot.motors import Motor, MotorNormMode
from lerobot_robot_xlerobot.xlerobot import XLerobot

from .config_xlerobot_2wheels_odrive import XLerobot2WheelsODriveConfig
from .odrive_base import ODriveBase

logger = logging.getLogger(__name__)


class XLerobot2WheelsODrive(XLerobot):
    config_class = XLerobot2WheelsODriveConfig
    name = "xlerobot_2wheels_odrive"

    def __init__(self, config: XLerobot2WheelsODriveConfig):
        # The base object must exist before the parent's __init__ runs,
        # because the parent consults the hooks below while building its
        # motor tables, and because a failed parent __init__ still leaves a
        # disconnect() that touches self.base.
        self.base = ODriveBase(
            wheel_radius=config.wheel_radius,
            wheelbase=config.wheelbase,
            invert_left=config.invert_left_wheel,
            invert_right=config.invert_right_wheel,
            max_linear_mps=config.max_linear_mps,
            connect_timeout_s=config.odrive_connect_timeout_s,
            serial_number=config.odrive_serial_number,
        )
        super().__init__(config)
        self.config: XLerobot2WheelsODriveConfig = config

    # ------------------------------------------------------------- base hooks

    @staticmethod
    def _bus2_motors(norm_mode_body: MotorNormMode) -> dict[str, Motor]:
        """Bus 2 is the right arm only; the wheels are on the ODrive."""
        return {
            "right_arm_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "right_arm_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "right_arm_elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "right_arm_wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "right_arm_wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "right_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }

    @property
    def _base_state_keys(self) -> tuple[str, ...]:
        # No y.vel: a differential drive cannot strafe. The operator side
        # must not emit one either (emit_y_vel: false in the leader config),
        # or the robot is sent a key it has no feature for.
        return ("x.vel", "theta.vel")

    @property
    def _base_is_connected(self) -> bool:
        return self.base.is_connected

    def _connect_base(self) -> None:
        self.base.connect()

    def _disconnect_base(self) -> None:
        self.base.disconnect()

    def _read_base_velocity(self) -> dict[str, float]:
        # The parent's read-retry logic catches ConnectionError, so an ODrive
        # that stops answering mid-run is ridden out on the last good state
        # like a dropped servo packet, rather than ending the episode.
        try:
            return self.base.read_body_velocity()
        except Exception as e:  # noqa: BLE001
            raise ConnectionError(f"ODrive read failed: {e}") from e

    def _send_base_velocity(self, x_mps: float, y_mps: float, theta_degps: float) -> None:
        if abs(y_mps) > 1e-6:
            # Tolerated, logged once: a stale operator config still emitting
            # y.vel should be loud but not fatal.
            if not getattr(self, "_warned_y", False):
                self._warned_y = True
                logger.warning("%s: received y.vel=%.3f; a differential drive ignores it", self, y_mps)
        self.base.send_body_velocity(x_mps, theta_degps)

    def stop_base(self):
        self.base.stop()
        logger.info("Base motors stopped")

    # -------------------------------------------------------------- keyboard

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        if self.teleop_keys["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)
        speed_setting = self.speed_levels[self.speed_index]
        x_speed = speed_setting["xy"]
        theta_speed = speed_setting["theta"]

        x_cmd = 0.0
        theta_cmd = 0.0
        if self.teleop_keys["forward"] in pressed_keys:
            x_cmd += x_speed
        if self.teleop_keys["backward"] in pressed_keys:
            x_cmd -= x_speed
        if self.teleop_keys["rotate_left"] in pressed_keys:
            theta_cmd += theta_speed
        if self.teleop_keys["rotate_right"] in pressed_keys:
            theta_cmd -= theta_speed

        return {"x.vel": x_cmd, "theta.vel": theta_cmd}
