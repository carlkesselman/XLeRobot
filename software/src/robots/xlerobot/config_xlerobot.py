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

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

from lerobot.robots.config import RobotConfig


def xlerobot_cameras_config() -> dict[str, CameraConfig]:
    return {
        # "left_wrist": OpenCVCameraConfig(
        #     index_or_path="/dev/video0", fps=30, width=640, height=480, rotation=Cv2Rotation.NO_ROTATION
        # ),

        # "right_wrist": OpenCVCameraConfig(
        #     index_or_path="/dev/video2", fps=30, width=640, height=480, rotation=Cv2Rotation.NO_ROTATION
        # ),  

        # "head(RGDB)": OpenCVCameraConfig(
        #     index_or_path="/dev/video2", fps=30, width=640, height=480, rotation=Cv2Rotation.NO_ROTATION
        # ),                     
        
        # "head": RealSenseCameraConfig(
        #     serial_number_or_name="125322060037",  # Replace with camera SN
        #     fps=30,
        #     width=1280,
        #     height=720,
        #     color_mode=ColorMode.BGR, # Request BGR output
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     use_depth=True
        # ),
    }


@RobotConfig.register_subclass("xlerobot")
@dataclass
class XLerobotConfig(RobotConfig):
    
    port1: str = "/dev/ttyACM0"  # port to connect to the bus (so101 + head camera)
    port2: str = "/dev/ttyACM1"  # port to connect to the bus (same as lekiwi setup)
    disable_torque_on_disconnect: bool = True

    # --- which base, if any ------------------------------------------------
    # The same cart runs in three configurations and this one field picks
    # between them, so a yaml never needs to change and the choice can be
    # made on the command line (--robot.base=none):
    #
    #   omniwheels  three STS3215 wheels, ids 7-9 on bus 2 (the stock XLeRobot)
    #   none        arms and head only: no wheel ids on bus 2, no x.vel /
    #               y.vel / theta.vel in the observation or action, base
    #               commands from the operator ignored (one warning)
    #   odrive      two brushless hub wheels on an ODESC/ODrive over USB -
    #               only with type: xlerobot_2wheels_odrive, which adds the
    #               wheel geometry and the ODrive fields; see that config
    #
    # The dataset's feature set follows this choice, so episodes recorded
    # under different bases are different datasets.
    base: str = "omniwheels"
    _allowed_bases = ("omniwheels", "none")

    def __post_init__(self) -> None:
        if hasattr(super(), "__post_init__"):
            super().__post_init__()
        if self.base not in self._allowed_bases:
            raise ValueError(
                f"base={self.base!r} is not one of {self._allowed_bases} for robot type "
                f"{getattr(self, 'type', type(self).__name__)!r}"
            )

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

    # --- head motion profile ---------------------------------------------
    # The head is commanded as a stream of absolute positions, ~1 unit apart
    # at 30 Hz when the operator holds the D-pad. configure_motors() leaves
    # every servo at Acceleration=254 (maximum) with no speed limit, so each
    # of those steps is a full-torque jump followed by a hold: thirty nudges
    # a second, which on a light pan/tilt carrying the camera you are
    # watching through reads as jerk. The arms get the same treatment and
    # hide it behind a human hand and their own inertia.
    #
    # These two registers make the servo interpolate across the tick instead.
    # Units are the servo's own: Goal_Velocity in steps/s (0 = unlimited),
    # Acceleration in units of 100 steps/s^2 (0 = unlimited). With a
    # calibrated head range of ~2000 steps, the operator's 30 units/s is
    # ~300 steps/s, so 600 keeps up with headroom while 40 (= 4000 steps/s^2)
    # reaches it in 0.15 s. They also cap the lunge after a link stall.
    head_goal_velocity: int = 600
    head_acceleration: int = 40

    # --- arm acceleration, which is also the inrush limit -------------------
    # configure_motors() leaves the arm servos at Acceleration=254, the
    # maximum. The first send_action() of a session hands all twelve arm
    # servos a goal at once - the followers snap to wherever the leaders
    # happen to be - and twelve STS3215s starting simultaneously from rest
    # at maximum acceleration is the largest current spike this robot can
    # produce. On a shared supply it is also how both buses went silent on
    # the first tick of a run: the socket tripped, not sagged.
    #
    # Acceleration is in units of 100 steps/s^2. 100 (= 10 000 steps/s^2)
    # reaches full speed in ~0.3 s, which tracks a human hand without
    # feeling slow, while spreading each servo's start-up current over
    # tens of milliseconds instead of a few. Lower is gentler on the
    # supply and softer to drive; 254 is the old behaviour.
    arm_acceleration: int = 100

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            # Speed control
            "speed_up": "n",
            "speed_down": "m",
            # quit teleop
            "quit": "b",
        }
    )



@dataclass
class XLerobotHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 3600

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30

@RobotConfig.register_subclass("xlerobot_client")
@dataclass
class XLerobotClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            # Speed control
            "speed_up": "n",
            "speed_down": "m",
            # quit teleop
            "quit": "b",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
