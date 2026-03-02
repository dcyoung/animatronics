"""Real-hardware robot backend for Personality Core.

Controls 3 Dynamixel XL430 servos via :class:`~personality_core.motor_controller.XL430Controller`:

  - Servo 0 : PR0  (pitch/roll leg 0)
  - Servo 1 : PR1  (pitch/roll leg 1)
  - Servo 2 : Yaw  (bevel gear)

The control loop runs at 50 Hz.
"""

import json
import logging
import time
from dataclasses import dataclass
from typing import Annotated, Any

import numpy as np
import numpy.typing as npt

from reachy_mini.daemon.backend.abstract import Backend, MotorControlMode

from personality_core.kinematics import AnalyticalKinematics
from personality_core.motor_controller import XL430Controller


class RobotBackend(Backend):
    """Real-hardware backend using :class:`XL430Controller`."""

    def __init__(
        self,
        serialport: str,
        log_level: str = "INFO",
        use_audio: bool = False,
    ) -> None:
        # Initialise the reachy_mini Backend base class then immediately swap
        # in our own 3-DOF kinematics engine.
        super().__init__(
            log_level=log_level,
            check_collision=False,
            kinematics_engine="AnalyticalKinematics",
            use_audio=use_audio,
            wireless_version=False,
        )
        self.head_kinematics = AnalyticalKinematics()  # type: ignore[assignment]

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(log_level)

        # -- Motor controller (mirrors ReachyMiniPyControlLoop usage) --------
        self.c: XL430Controller | None = XL430Controller(serial_port=serialport)

        self.name2id = self.c.get_motor_name_id()

        self.control_loop_frequency = 50.0  # Hz
        self.motor_control_mode = MotorControlMode.Disabled

        self._status = RobotBackendStatus(
            motor_control_mode=self.motor_control_mode,
            ready=False,
            last_alive=None,
            control_loop_stats={},
        )
        self._stats: dict[str, Any] = {"timestamps": [], "nb_error": 0}
        self._stats_record_period = 1.0
        self._stats_record_t0 = time.time()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Run the 50 Hz control loop."""
        assert self.c is not None, "Motor controller not initialized or already closed."

        period = 1.0 / self.control_loop_frequency

        # Bootstrap FK from current servo positions
        pos = self.c.get_last_position()
        initial_joints = np.array(pos.head, dtype=np.float64)
        self.current_head_pose = self.head_kinematics.fk(initial_joints)
        assert self.current_head_pose is not None

        while not self.should_stop.is_set():
            start_t = time.time()
            self._stats["timestamps"].append(start_t)

            self._update()

            elapsed = time.time() - start_t
            sleep_time = max(0.001, period - elapsed)
            self.should_stop.wait(sleep_time)

    def _update(self) -> None:
        """Single control-loop tick."""
        assert self.c is not None, "Motor controller not initialized or already closed."

        # ── Write targets ────────────────────────────────────────────
        if self.motor_control_mode == MotorControlMode.Enabled:
            if self.target_head_joint_positions is not None:
                self.c.set_head_position(self.target_head_joint_positions)

        # ── Read present positions & publish ─────────────────────────
        if (
            self.joint_positions_publisher is not None
            and self.pose_publisher is not None
        ):
            try:
                pos = self.c.get_last_position()
                head_joints = np.array(pos.head, dtype=np.float64)
                self.update_head_kinematics_model(head_joints)

                if self.ik_required:
                    try:
                        self.update_target_head_joints_from_ik(
                            pose=self.target_head_pose,
                            # reachy_mini compat: Body yaw is not supported on this robot.
                            body_yaw=None,
                        )
                    except ValueError as e:
                        self.logger.warning(f"IK error: {e}")

                if not self.is_shutting_down:
                    self.joint_positions_publisher.put(
                        json.dumps(
                            {
                                "head_joint_positions": pos.head,
                                # reachy_mini compat: ZenohClient expects this key
                                "antennas_joint_positions": [],
                            }
                        )
                    )
                    self.pose_publisher.put(
                        json.dumps({"head_pose": self.get_present_head_pose().tolist()})
                    )

                self._status.last_alive = time.time()
                self.ready.set()

            except RuntimeError as e:
                self._stats["nb_error"] += 1
                if (
                    self._status.last_alive is not None
                    and self._status.last_alive + 1 < time.time()
                ):
                    self.error = "No response from servos for the last second."
                    self.logger.error(self.error)
                    raise e

        # ── Stats bookkeeping ────────────────────────────────────────
        if time.time() - self._stats_record_t0 > self._stats_record_period:
            dt = np.diff(self._stats["timestamps"])
            if len(dt) > 1:
                self._status.control_loop_stats["mean_control_loop_frequency"] = float(
                    np.mean(1.0 / dt)
                )
                self._status.control_loop_stats["max_control_loop_interval"] = float(
                    np.max(dt)
                )
                self._status.control_loop_stats["nb_error"] = self._stats["nb_error"]
                self._status.control_loop_stats["motor_controller"] = str(
                    self.c.get_stats()
                )
            self._stats["timestamps"].clear()
            self._stats["nb_error"] = 0
            self._stats_record_t0 = time.time()

    # ------------------------------------------------------------------
    # Motor enable / disable  (mirrors reachy_mini RobotBackend)
    # ------------------------------------------------------------------

    def enable_motors(self) -> None:
        """Enable the motors by turning torque on."""
        assert self.c is not None, "Motor controller not initialized or already closed."
        self.c.enable_torque()

    def disable_motors(self) -> None:
        """Disable the motors by turning torque off."""
        assert self.c is not None, "Motor controller not initialized or already closed."
        self.c.disable_torque()

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Release the serial connection."""
        if self.c is not None:
            self.c.close()
        self.c = None
        super().close()

    def get_status(self) -> "RobotBackendStatus":
        """Return the current backend status."""
        self._status.error = self.error
        self._status.motor_control_mode = self.motor_control_mode
        return self._status

    def get_present_head_joint_positions(
        self,
    ) -> Annotated[npt.NDArray[np.float64], (3,)]:
        """Return current head joint angles ``(3,)`` in radians."""
        assert self.c is not None, "Motor controller not initialized or already closed."
        pos = self.c.get_last_position()
        return np.array(pos.head, dtype=np.float64)

    def get_motor_control_mode(self) -> MotorControlMode:
        """Get the motor control mode."""
        return self.motor_control_mode

    def set_motor_control_mode(self, mode: MotorControlMode) -> None:
        """Set the motor control mode."""
        if mode == self.motor_control_mode:
            return
        if mode == MotorControlMode.Enabled:
            self.enable_motors()
        elif mode == MotorControlMode.Disabled:
            self.disable_motors()
        elif mode == MotorControlMode.GravityCompensation:
            raise NotImplementedError(
                "Gravity compensation is not supported on this robot."
            )
        self.motor_control_mode = mode

    def set_motor_torque_ids(self, ids: list[str], on: bool) -> None:
        """Set torque for named motors (``pr0``, ``pr1``, ``yaw``)."""
        assert self.c is not None, "Motor controller not initialized or already closed."

        ids_int = []
        for name in ids:
            sid = self.name2id.get(name)
            if sid is not None:
                ids_int.append(sid)
            else:
                self.logger.warning(f"Unknown motor name: {name!r}")

        if ids_int:
            if on:
                self.c.enable_torque_on_ids(ids_int)
            else:
                self.c.disable_torque_on_ids(ids_int)

    # ------------------------------------------------------------------
    # Wake / sleep
    # ------------------------------------------------------------------

    INIT_HEAD_POSE = np.eye(4)

    _sleep_rotation = np.array(
        [
            [0.911, 0.0, 0.413],
            [0.0, 1.0, 0.0],
            [-0.413, 0.0, 0.911],
        ]
    )
    SLEEP_HEAD_POSE = np.eye(4)
    SLEEP_HEAD_POSE[:3, :3] = _sleep_rotation

    async def wake_up(self) -> None:
        """Move head to the init (forward-looking) pose."""
        import asyncio

        await asyncio.sleep(0.1)
        await self.goto_target(self.INIT_HEAD_POSE, duration=1.0)
        await asyncio.sleep(0.1)

    async def goto_sleep(self) -> None:
        """Tilt head forward into the sleep pose."""
        import asyncio

        await self.goto_target(self.SLEEP_HEAD_POSE, duration=2.0)
        await asyncio.sleep(0.5)

    # ------------------------------------------------------------------
    # Compat shims — required by the reachy_mini Backend base class
    # ------------------------------------------------------------------

    def get_present_antenna_joint_positions(
        self,
    ) -> Annotated[npt.NDArray[np.float64], (0,)]:
        """Return empty array (this robot has no antennas)."""
        return np.array([], dtype=np.float64)

    def get_present_body_yaw(self) -> float:
        """Return 0 (this robot has no separate body-rotation motor)."""
        return 0.0


@dataclass
class RobotBackendStatus:
    """Status of the Robot Backend."""

    ready: bool
    motor_control_mode: MotorControlMode
    last_alive: float | None
    control_loop_stats: dict[str, Any]
    error: str | None = None
