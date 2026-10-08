#!/usr/bin/env python

"""Config for the remote-leader teleoperator pair."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("xlerobot_leader_remote")
@dataclass
class XLerobotLeaderRemoteConfig(TeleoperatorConfig):
    """Leader arms on one machine, the robot and the dataset on another.

    This is the mirror image of xlerobot_host/xlerobot_client. There, the
    robot is remote and the teleoperator is local. Here the teleoperator is
    remote and the robot is local, which is what recording on the cart with
    the operator at a desk requires:

        operator station          cart
        ----------------          ----
        leader arms + gamepad     follower arms + cameras
        leader_host.py     --->   lerobot-record
                                    robot  = xlerobot (local)
                                    teleop = xlerobot_leader_remote (this)

    Only the action stream crosses the network, and only in one direction.
    Observations never do, so the dataset is written from full-rate local
    camera frames - no conflated socket between the sensors and the disk.

    What the link costs instead is teleoperation feel: wifi jitter lands in
    the action stream, and whatever the operator actually commanded is what
    gets recorded. The dataset stays self-consistent either way, but jerky
    input makes jerky demonstrations, so watch the reported staleness.
    """

    remote_ip: str = "127.0.0.1"
    port_zmq_actions: int = 5557

    # How long connect() waits for the first action before giving up. The
    # first action must be real: it carries the leaders' current pose, and
    # the followers move to it. Fabricating a neutral pose here would snap
    # the arms to a position nobody asked for.
    connect_timeout_s: float = 10.0

    # Past this with no new action, the link is considered stalled.
    stale_after_ms: int = 300

    # On a stall, zero the base velocities. The arms hold their last
    # commanded position - which is what they would do anyway - but a base
    # keeps driving on its last velocity until told otherwise, and the
    # operator who would have stopped it is on the far side of the break.
    stop_base_when_stale: bool = True

    # Log once per this many seconds while stalled, rather than per tick.
    stale_log_interval_s: float = 1.0
