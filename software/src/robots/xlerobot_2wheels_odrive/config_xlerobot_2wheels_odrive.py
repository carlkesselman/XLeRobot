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


def xlerobot_2wheels_cameras_config() -> dict[str, CameraConfig]:
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


@RobotConfig.register_subclass("xlerobot_2wheels_odrive")
@dataclass
class XLerobot2WheelsODriveConfig(RobotConfig):
    
    port1: str = "/dev/ttyACM0"  # port to connect to the bus (so101 + head camera)
    port2: str = "/dev/ttyACM1"  # right arm only - the wheels are on the ODrive, not this bus
    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_2wheels_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

    # Differential drive parameters
    wheel_radius: float = 0.0825  # Wheel radius in metres (165 mm wheel)
    wheelbase: float = 0.25     # Distance between left and right wheel contact points, metres

    # --- ODrive ---------------------------------------------------------
    # Targets ODrive v3.x with 0.5.x firmware - see odrive_base.py. A newer
    # board (S1/Pro, 0.6+) needs that file rewritten, not this config.
    #
    # The two motors face opposite ways when mounted either side of the
    # chassis, so one axis must be negated. Which one depends on your wiring
    # and on how axis0/axis1 are assigned in the board's own config. If
    # driving forward spins the robot in place instead, flip these.
    invert_left_wheel: bool = True
    invert_right_wheel: bool = False

    # Per-wheel clamp, m/s. A safety limit against a runaway command.
    max_linear_mps: float = 1.0

    odrive_connect_timeout_s: int = 30
    # Set when more than one ODrive is attached; None takes the first found.
    odrive_serial_number: str | None = None

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement (differential drive)
            "forward": "i",
            "backward": "k",
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
class XLerobot2WheelsODriveHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 3600

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30

@RobotConfig.register_subclass("xlerobot_2wheels_odrive_client")
@dataclass
class XLerobot2WheelsODriveClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Differential drive parameters
    wheel_radius: float = 0.0825  # Wheel radius in metres (165 mm wheel)
    wheelbase: float = 0.25     # Distance between left and right wheels in meters

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement (differential drive)
            "forward": "i",
            "backward": "k",
            "rotate_left": "u",
            "rotate_right": "o",
            # Speed control
            "speed_up": "n",
            "speed_down": "m",
            # quit teleop
            "quit": "b",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_2wheels_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
