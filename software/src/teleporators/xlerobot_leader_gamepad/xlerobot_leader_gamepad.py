#!/usr/bin/env python

"""Two SO-101 leader arms plus a gamepad, presented as one teleoperator."""

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_xlerobot_leader_gamepad import XLerobotLeaderGamepadConfig

logger = logging.getLogger(__name__)

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



class XLerobotLeaderGamepad(Teleoperator):
    """Leader arms drive the arms; a gamepad aims the head and drives the base.

    Action keys are remapped from the leaders' naming to the robot's. BiSOLeader
    emits `left_shoulder_pan.pos`, but XLerobot filters incoming actions with
    `startswith("left_arm_")` - so without this remap every arm key is silently
    dropped and the arms do not move at all. That failure is quiet: the robot
    connects, the loop runs, nothing happens.
    """

    config_class = XLerobotLeaderGamepadConfig
    name = "xlerobot_leader_gamepad"

    # BiSOLeader prefix -> XLerobot prefix
    _PREFIX = (("left_", "left_arm_"), ("right_", "right_arm_"))

    def __init__(self, config: XLerobotLeaderGamepadConfig):
        super().__init__(config)
        self.config = config

        self.leaders = BiSOLeader(
            BiSOLeaderConfig(
                # config.id verbatim, NOT f"{config.id}_leaders". BiSOLeader
                # appends _left / _right itself, so the suffix produced
                # xlerobot_leaders_leaders_left - calibration files nobody
                # has. Passing it through means this teleoperator reads the
                # SAME per-arm calibration as bi_so_leader does from
                # config/bi-arms.yaml: calibrate the leaders once, use them
                # from either config.
                id=config.id,
                calibration_dir=config.calibration_dir,
                left_arm_config=config.left_arm_config,
                right_arm_config=config.right_arm_config,
            )
        )

        self.joystick: Any = None
        self._pygame: Any = None

        # Head is position-controlled, but the stick gives a rate - integrate it
        # so the head holds still when the stick is centred.
        self.head_targets = {
            "head_motor_1": config.head_start_deg,
            "head_motor_2": config.head_start_deg,
        }
        self.speed_scale = config.base_speed_scale
        self._last_tick: float | None = None
        self._prev_buttons: set[int] = set()

    # ---------------------------------------------------------------- naming

    def _remap(self, key: str) -> str:
        if not self.config.remap_arm_prefix:
            return key
        for old, new in self._PREFIX:
            if key.startswith(old):
                return new + key[len(old) :]
        return key

    # ------------------------------------------------------------- features

    @cached_property
    def action_features(self) -> dict[str, type]:
        feats: dict[str, type] = {
            self._remap(k): v for k, v in self.leaders.action_features.items()
        }
        if self.config.emit_head:
            feats["head_motor_1.pos"] = float
            feats["head_motor_2.pos"] = float
        if self.config.emit_base:
            feats["x.vel"] = float
            if self.config.emit_y_vel:
                feats["y.vel"] = float
            feats["theta.vel"] = float
        return feats

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    # ------------------------------------------------------------ lifecycle

    @property
    def is_connected(self) -> bool:
        return self.leaders.is_connected and self.joystick is not None

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.leaders.connect(calibrate)

        try:
            import pygame
        except ImportError as e:
            raise ImportError(
                "pygame is required for the gamepad. Install the gamepad extra: "
                "`uv pip install 'lerobot[gamepad]'`."
            ) from e

        self._pygame = pygame
        _headless_sdl()
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise DeviceNotConnectedError(
                "No gamepad found. Check it is paired and powered, and that an "
                "8BitDo pad is in X-input mode (the button combination is on the "
                "sticker underneath)."
            )
        if self.config.joystick_index >= count:
            raise DeviceNotConnectedError(
                f"joystick_index={self.config.joystick_index} but only {count} "
                f"gamepad(s) are present."
            )

        self.joystick = pygame.joystick.Joystick(self.config.joystick_index)
        self.joystick.init()
        logger.info(
            "Gamepad: %s (%d axes, %d buttons)",
            self.joystick.get_name(),
            self.joystick.get_numaxes(),
            self.joystick.get_numbuttons(),
        )

    @property
    def is_calibrated(self) -> bool:
        return self.leaders.is_calibrated

    def calibrate(self) -> None:
        # Only the leader arms have anything to calibrate.
        self.leaders.calibrate()

    def configure(self) -> None:
        self.leaders.configure()

    # --------------------------------------------------------------- action

    def _axis(self, index: int) -> float:
        """Read one axis with the deadzone applied. Out-of-range -> 0."""
        if index < 0 or index >= self.joystick.get_numaxes():
            return 0.0
        v = self.joystick.get_axis(index)
        return 0.0 if abs(v) < self.config.deadzone else v

    def _button(self, index: int) -> bool:
        if index < 0 or index >= self.joystick.get_numbuttons():
            return False
        return bool(self.joystick.get_button(index))

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        cfg = self.config
        self._pygame.event.pump()

        now = time.perf_counter()
        dt = 0.0 if self._last_tick is None else min(now - self._last_tick, 0.2)
        self._last_tick = now

        if cfg.log_axes:
            axes = [round(self.joystick.get_axis(i), 2) for i in range(self.joystick.get_numaxes())]
            btns = [i for i in range(self.joystick.get_numbuttons()) if self.joystick.get_button(i)]
            logger.info("axes=%s buttons=%s", axes, btns)

        # --- speed, on button edges so a held button steps once -------------
        pressed = {i for i in range(self.joystick.get_numbuttons()) if self.joystick.get_button(i)}
        for btn, delta in ((cfg.button_speed_up, +1), (cfg.button_speed_down, -1)):
            if btn >= 0 and btn in pressed and btn not in self._prev_buttons:
                self.speed_scale = max(
                    cfg.base_speed_min,
                    min(cfg.base_speed_max, self.speed_scale + delta * cfg.base_speed_step),
                )
                logger.info("base speed scale: %.2f", self.speed_scale)
        self._prev_buttons = pressed

        head_mode = self._button(cfg.button_head_modifier) if cfg.button_head_modifier >= 0 else False

        # --- head: stick gives a rate, integrated into a target -------------
        pan = self._axis(cfg.axis_head_pan) if (head_mode or cfg.button_head_modifier < 0) else 0.0
        tilt = self._axis(cfg.axis_head_tilt) if (head_mode or cfg.button_head_modifier < 0) else 0.0
        if cfg.invert_head_tilt:
            tilt = -tilt

        self.head_targets["head_motor_1"] = max(
            cfg.head_min_deg,
            min(cfg.head_max_deg, self.head_targets["head_motor_1"] + pan * cfg.head_rate_degps * dt),
        )
        self.head_targets["head_motor_2"] = max(
            cfg.head_min_deg,
            min(cfg.head_max_deg, self.head_targets["head_motor_2"] + tilt * cfg.head_rate_degps * dt),
        )

        # --- base -----------------------------------------------------------
        fwd = self._axis(cfg.axis_base_forward)
        if cfg.invert_base_forward:
            fwd = -fwd
        strafe = self._axis(cfg.axis_base_strafe)
        turn = 0.0 if head_mode else self._axis(cfg.axis_base_turn)

        action: dict[str, Any] = {
            self._remap(k): v for k, v in self.leaders.get_action().items()
        }
        if cfg.emit_head:
            action["head_motor_1.pos"] = self.head_targets["head_motor_1"]
            action["head_motor_2.pos"] = self.head_targets["head_motor_2"]
        if cfg.emit_base:
            action["x.vel"] = fwd * cfg.base_speed_mps * self.speed_scale
            if cfg.emit_y_vel:
                action["y.vel"] = strafe * cfg.base_speed_mps * self.speed_scale
            action["theta.vel"] = -turn * cfg.base_turn_degps * self.speed_scale
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # Leaders can be actuated for handover, but nothing drives that here.
        pass

    def disconnect(self) -> None:
        if self.leaders.is_connected:
            self.leaders.disconnect()
        if self.joystick is not None:
            try:
                self.joystick.quit()
                self._pygame.joystick.quit()
            except Exception as e:
                logger.warning("Failed to release the gamepad: %s", e)
            finally:
                self.joystick = None
