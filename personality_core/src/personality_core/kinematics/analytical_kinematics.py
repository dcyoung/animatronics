"""Robot-level analytical kinematics adapter for the Personality Core head.

Wraps the platform IK solver (``ik_solver``) to present the ``ik`` / ``fk``
interface expected by the reachy_mini ``Backend`` base class.

Joint convention ``(3,)``: ``[PR0, PR1, yaw_servo]``

Pose convention: 4×4 homogeneous matrix (orientation only — ball-joint head).
"""

from __future__ import annotations

import logging
from typing import Annotated

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation as R

from personality_core.kinematics.stewart_platform_3dof.ik_solver import (
    HeadConfig,
    default_config,
    solve_ik_from_config,
)

NUM_JOINTS = 3  # [PR0, PR1, yaw_servo]


class AnalyticalKinematics:
    """Analytical kinematics for the 3-DOF personality-core head.

    Parameters
    ----------
    head_config : HeadConfig | None
        Physical geometry.  ``None`` uses ``default_config()``.

    """

    def __init__(self, head_config: HeadConfig | None = None) -> None:
        """Initialize with the given head geometry (or the default)."""
        self.logger = logging.getLogger(__name__)
        self.config = head_config if head_config is not None else default_config()

        # Warm-start cache for the FK numerical solver
        self._last_rpy: NDArray[np.float64] = np.zeros(3)

    # ------------------------------------------------------------------
    # IK  :  4×4 pose  ->  (3,) joint angles
    # ------------------------------------------------------------------

    def ik(
        self,
        pose: Annotated[NDArray[np.float64], (4, 4)],
        # reachy_mini compat: Backend.update_target_head_joints_from_ik passes these
        body_yaw: float = 0.0,
        check_collision: bool = False,
        no_iterations: int = 0,
    ) -> Annotated[NDArray[np.float64], (3,)]:
        """Inverse kinematics: 4×4 head pose -> ``(3,)`` joint angles."""
        rpy = R.from_matrix(pose[:3, :3]).as_euler("xyz", degrees=False)
        return solve_ik_from_config(rpy, self.config)

    # ------------------------------------------------------------------
    # FK  :  (3,) joint angles  ->  4×4 pose
    # ------------------------------------------------------------------

    def fk(
        self,
        joint_angles: Annotated[NDArray[np.float64], (3,)],
        # reachy_mini compat: Backend.update_head_kinematics_model passes these
        check_collision: bool = False,
        no_iterations: int = 3,
    ) -> Annotated[NDArray[np.float64], (4, 4)]:
        """Forward kinematics: ``(3,)`` joint angles -> 4×4 head pose.

        Yaw is recovered analytically; roll and pitch are found via a
        small ``scipy.fsolve`` root-find that converges quickly.
        """
        joint_angles = np.asarray(joint_angles, dtype=np.float64)
        target_pr = joint_angles[:2]
        head_yaw = joint_angles[2] * self.config.yaw_ratio

        def _residual(rp: NDArray[np.float64]) -> NDArray[np.float64]:
            rpy = np.array([rp[0], rp[1], head_yaw])
            joints = solve_ik_from_config(rpy, self.config)
            return joints[:2] - target_pr

        rp_solution, _info, ier, _msg = fsolve(
            _residual, self._last_rpy[:2].copy(), full_output=True
        )
        if ier != 1:
            self.logger.warning("FK solver did not converge; using best estimate.")

        rpy = np.array([rp_solution[0], rp_solution[1], head_yaw])
        self._last_rpy = rpy

        pose = np.eye(4)
        pose[:3, :3] = R.from_euler("xyz", rpy, degrees=False).as_matrix()
        return pose

    # ------------------------------------------------------------------
    # reachy_mini compat: ZenohServer calls this on the kinematics engine
    # when it receives an "automatic_body_yaw" command.
    # ------------------------------------------------------------------

    def set_automatic_body_yaw(self, automatic_body_yaw: bool) -> None:
        """No-op — this robot has no separate body-rotation motor."""
        pass
