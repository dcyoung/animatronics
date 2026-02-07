"""Lightweight simulation backend for Personality Core.

Target positions become current positions immediately (no physics).
The kinematics engine is still used for FK/IK.
"""

import json
import time
from dataclasses import dataclass
from typing import Annotated

import numpy as np
import numpy.typing as npt

from reachy_mini.daemon.backend.abstract import Backend, MotorControlMode

from personality_core.kinematics import AnalyticalKinematics


class MockupSimBackend(Backend):
    """Lightweight simulated backend — no physics engine required.

    Target positions are applied instantly each tick.
    """

    def __init__(self, use_audio: bool = False) -> None:
        super().__init__(
            check_collision=False,
            kinematics_engine="AnalyticalKinematics",
            use_audio=use_audio,
        )
        self.head_kinematics = AnalyticalKinematics()  # type: ignore[assignment]

        # Start at the init (identity) pose
        self._head_joint_positions: npt.NDArray[np.float64] = np.array(
            self.head_kinematics.ik(np.eye(4)), dtype=np.float64
        )

        self._motor_control_mode = MotorControlMode.Enabled
        self.control_frequency = 50.0

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Run the simulation loop at 50 Hz."""
        period = 1.0 / self.control_frequency
        self.update_head_kinematics_model(self._head_joint_positions)

        while not self.should_stop.is_set():
            start_t = time.time()

            if self.target_head_joint_positions is not None:
                self._head_joint_positions = self.target_head_joint_positions.copy()

            self.current_head_joint_positions = self._head_joint_positions.copy()
            self.update_head_kinematics_model(self.current_head_joint_positions)

            if self.ik_required:
                try:
                    self.update_target_head_joints_from_ik(self.target_head_pose)
                except ValueError:
                    pass

            if (
                self.joint_positions_publisher is not None
                and self.pose_publisher is not None
                and not self.is_shutting_down
            ):
                self.joint_positions_publisher.put(
                    json.dumps(
                        {
                            "head_joint_positions": self.current_head_joint_positions.tolist(),
                            # reachy_mini compat: ZenohClient expects this key
                            "antennas_joint_positions": [],
                        }
                    ).encode("utf-8")
                )
                self.pose_publisher.put(
                    json.dumps(
                        {"head_pose": self.get_present_head_pose().tolist()}
                    ).encode("utf-8")
                )

            self.ready.set()
            elapsed = time.time() - start_t
            time.sleep(max(0, period - elapsed))

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------

    def get_status(self) -> "MockupSimBackendStatus":
        """Return backend status."""
        return MockupSimBackendStatus(motor_control_mode=self._motor_control_mode)

    def get_present_head_joint_positions(
        self,
    ) -> Annotated[npt.NDArray[np.float64], (3,)]:
        """Return the current head joint positions ``(3,)``."""
        return self._head_joint_positions.copy()  # type: ignore[return-value]

    def get_motor_control_mode(self) -> MotorControlMode:
        """Get the motor control mode."""
        return self._motor_control_mode

    def set_motor_control_mode(self, mode: MotorControlMode) -> None:
        """Set the motor control mode (no-op in simulation)."""
        self._motor_control_mode = mode

    def set_motor_torque_ids(self, ids: list[str], on: bool) -> None:
        """No-op in simulation."""
        pass

    # ------------------------------------------------------------------
    # Wake / sleep
    # ------------------------------------------------------------------

    INIT_HEAD_POSE = np.eye(4)

    async def wake_up(self) -> None:
        """Move head to the init pose."""
        import asyncio

        await asyncio.sleep(0.1)
        await self.goto_target(self.INIT_HEAD_POSE, duration=0.5)
        await asyncio.sleep(0.1)

    async def goto_sleep(self) -> None:
        """Tilt head forward into the sleep pose."""
        import asyncio
        from scipy.spatial.transform import Rotation as R

        sleep_pose = np.eye(4)
        sleep_pose[:3, :3] = R.from_euler("xyz", [0, 25, 0], degrees=True).as_matrix()
        await self.goto_target(sleep_pose, duration=1.0)
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
class MockupSimBackendStatus:
    """Status of the MockupSim backend."""

    motor_control_mode: MotorControlMode
    error: str | None = None
