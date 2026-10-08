#!/usr/bin/env python

"""Config for the leader-arms + gamepad composite teleoperator."""

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.so_leader import SOLeaderConfig


@TeleoperatorConfig.register_subclass("xlerobot_leader_gamepad")
@dataclass
class XLerobotLeaderGamepadConfig(TeleoperatorConfig):
    """Two SO-101 leader arms for the arms, a gamepad for the head and base.

    Why one composite rather than two teleoperators: lerobot's record and
    teleoperate CLIs take exactly one. record_loop does have a multi-teleop
    path, but it is gated to `robot.name == "lekiwi_client"` and to the
    single-arm leader classes, so it cannot serve this robot. Presenting both
    devices as ONE teleoperator sidesteps that and works with the stock CLI.

    Why a gamepad rather than the keyboard: lerobot's keyboard teleoperation
    uses pynput's global key capture, which does not work on Wayland (nor
    headless). A gamepad is read as a device, so the session type is
    irrelevant - and analog sticks suit driving a base far better than
    discrete key presses.
    """

    left_arm_config: SOLeaderConfig = field(default_factory=lambda: SOLeaderConfig(port=""))
    right_arm_config: SOLeaderConfig = field(default_factory=lambda: SOLeaderConfig(port=""))

    # --- gamepad ------------------------------------------------------
    joystick_index: int = 0
    # Sticks rest slightly off centre; ignore small magnitudes so the base
    # does not creep.
    deadzone: float = 0.15

    # Axis numbers vary by controller and driver. These are the common
    # xbox/XInput layout under SDL. Run with log_axes=true to see live values
    # and correct them for your pad.
    axis_base_forward: int = 1   # left stick Y  (usually inverted - see invert)
    axis_base_strafe: int = 0    # left stick X
    axis_base_turn: int = 2      # right stick X
    axis_head_pan: int = 2       # right stick X  (shared with turn by default;
    axis_head_tilt: int = 3      #   hold the modifier button to aim the head)
    invert_base_forward: bool = True
    invert_head_tilt: bool = True

    # While held, the right stick aims the head instead of turning the base.
    # Set to -1 to give the head its own axes instead and never share.
    button_head_modifier: int = 4   # left shoulder

    button_speed_up: int = 5        # right shoulder
    button_speed_down: int = 7      # right trigger on some pads

    # Print raw axis/button values each tick. Use once to map your controller.
    log_axes: bool = False

    # --- motion -------------------------------------------------------
    base_speed_mps: float = 0.2
    base_turn_degps: float = 60.0
    base_speed_scale: float = 1.0
    base_speed_min: float = 0.25
    base_speed_max: float = 2.0
    base_speed_step: float = 0.25

    # A 3-omniwheel base can strafe, so y.vel is emitted. Differential drive
    # cannot - set false for xlerobot_2wheels* and y.vel is dropped.
    emit_y_vel: bool = True

    # --- head ---------------------------------------------------------
    # The stick sets a RATE, integrated into a target position, so the head
    # holds still when the stick is centred.
    head_rate_degps: float = 30.0
    head_min_deg: float = -90.0
    head_max_deg: float = 90.0
    head_start_deg: float = 0.0
