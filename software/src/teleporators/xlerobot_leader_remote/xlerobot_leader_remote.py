#!/usr/bin/env python

"""Teleoperator that reads its actions off the network.

Runs on the cart, inside `lerobot-record`, and stands in for the leader arms
and gamepad that are physically at the operator station. See
`leader_host.py` for the other half.
"""

import json
import logging
import threading
import time
from functools import cached_property
from typing import Any

import zmq

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_xlerobot_leader_remote import XLerobotLeaderRemoteConfig

logger = logging.getLogger(__name__)

# Zeroed on a stalled link. Everything else holds its last value.
_BASE_KEYS = ("x.vel", "y.vel", "theta.vel")


class XLerobotLeaderRemote(Teleoperator):
    """Pulls actions from a leader_host over ZMQ.

    `get_action()` must never block: `record_loop` calls it once per tick at
    the dataset's fps, so a blocking receive would make the recording rate
    hostage to the wifi. A background thread owns the socket and keeps the
    newest action in a slot; `get_action()` reads the slot and returns.

    The socket is CONFLATE, so a backlog cannot build up either - a late
    action is worthless, only the current leader pose matters.
    """

    config_class = XLerobotLeaderRemoteConfig
    name = "xlerobot_leader_remote"

    def __init__(self, config: XLerobotLeaderRemoteConfig):
        super().__init__(config)
        self.config = config

        self._ctx: zmq.Context | None = None
        self._sock: zmq.Socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

        self._lock = threading.Lock()
        self._action: dict[str, Any] | None = None
        self._action_time: float = 0.0

        self._stale = False
        self._last_stale_log = 0.0
        self._keys_logged = False

    # ------------------------------------------------------------- features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Taken from the first action received, so there is one definition
        of the action space and it lives at the operator end.

        Note that `lerobot-record` builds the dataset's action columns from
        `robot.action_features`, not from this - so this is used for
        validation and logging rather than for the dataset schema. That is
        also why a key mismatch between the two ends is worth shouting
        about: it would not show up as a schema error, only as a column of
        zeros nobody notices until training.
        """
        if self._action is None:
            raise DeviceNotConnectedError(
                f"{self} has no action yet; action_features is only known after connect()."
            )
        return {k: float for k in self._action}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    # ------------------------------------------------------------ lifecycle

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self, calibrate: bool = True) -> None:
        """Open the socket and wait for the first real action.

        Waiting matters. The first action carries the leaders' current pose
        and the followers move to it; inventing a neutral pose to return
        early would throw the arms somewhere nobody asked for. Calibration
        is the operator end's business - the leaders are not here.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        cfg = self.config
        addr = f"tcp://{cfg.remote_ip}:{cfg.port_zmq_actions}"

        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PULL)
        # Must be set before connect(), and keeps only the newest message.
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.RCVTIMEO, 100)
        self._sock.connect(addr)

        self._stop.clear()
        self._thread = threading.Thread(target=self._receive_loop, name="leader-remote-rx", daemon=True)
        self._thread.start()

        logger.info("Waiting up to %.0fs for the first action from %s", cfg.connect_timeout_s, addr)
        deadline = time.perf_counter() + cfg.connect_timeout_s
        while time.perf_counter() < deadline:
            with self._lock:
                if self._action is not None:
                    logger.info(
                        "Leader host at %s is sending %d action keys", addr, len(self._action)
                    )
                    return
            time.sleep(0.02)

        self.disconnect()
        raise DeviceNotConnectedError(
            f"No action from the leader host at {addr} within {cfg.connect_timeout_s}s.\n"
            f"  - is leader_host.py running at the operator station?\n"
            f"  - does {cfg.remote_ip} resolve and answer ping from here?\n"
            f"  - is port {cfg.port_zmq_actions} open?"
        )

    @property
    def is_calibrated(self) -> bool:
        # The leaders are calibrated where they are plugged in.
        return True

    def calibrate(self) -> None:
        logger.info("Nothing to calibrate here - the leader arms are at the operator station.")

    def configure(self) -> None:
        pass

    # --------------------------------------------------------------- action

    def _receive_loop(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                msg = self._sock.recv_string()
            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                if self._stop.is_set():
                    return
                logger.error("Action receive failed: %s", e)
                time.sleep(0.05)
                continue

            try:
                action = {k: float(v) for k, v in json.loads(msg).items()}
            except (ValueError, TypeError, AttributeError) as e:
                logger.error("Malformed action message dropped: %s", e)
                continue

            with self._lock:
                self._action = action
                self._action_time = time.perf_counter()

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with self._lock:
            action = dict(self._action) if self._action is not None else None
            age_ms = (time.perf_counter() - self._action_time) * 1000.0

        if action is None:
            # connect() guarantees one action arrived, so this is unreachable
            # unless disconnect raced us.
            raise DeviceNotConnectedError(f"{self} has no action available.")

        if not self._keys_logged:
            self._keys_logged = True
            logger.info("Action keys from the leader host: %s", sorted(action))

        stale = age_ms > self.config.stale_after_ms
        if stale:
            if self.config.stop_base_when_stale:
                for k in _BASE_KEYS:
                    if k in action:
                        action[k] = 0.0
            now = time.perf_counter()
            if not self._stale or now - self._last_stale_log > self.config.stale_log_interval_s:
                logger.warning(
                    "Leader link stalled: last action %.0fms ago%s",
                    age_ms,
                    " - base stopped, arms holding" if self.config.stop_base_when_stale else "",
                )
                self._last_stale_log = now
        elif self._stale:
            logger.info("Leader link recovered (%.0fms)", age_ms)
        self._stale = stale

        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # record_loop only calls this for unitree_g1, and there is nothing to
        # send back anyway - the operator's video comes from the recording
        # process's own display stream, not from here.
        pass

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None
        if self._ctx is not None:
            self._ctx.term()
            self._ctx = None
