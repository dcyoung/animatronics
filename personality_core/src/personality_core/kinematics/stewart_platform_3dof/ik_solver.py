"""3-DOF platform inverse kinematics: head orientation -> actuator angles.

Pure geometry / math — no robot-specific interface concerns.

Frame
-----
Head centre at rest is the origin.  The head has no translational DOF
(ball-joint); only orientation (roll, pitch, yaw) changes.

- **B** : horn rotation origins (servo positions), head-centred frame.
- **P** : rod attachment points on the head, head-local frame.
- Leg vector ``L_i = R @ P[:,i] - B[:,i]``.

Convention
----------
IK input  : ``(roll, pitch, yaw)`` in radians.
IK output : ``[PR0, PR1, yaw_servo]`` in radians.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ======================================================================
# Configuration
# ======================================================================


@dataclass
class HeadConfig:
    """Physical geometry for a 3-DOF head (2 PR servos + bevel yaw).

    All lengths in consistent units.  Frame: head centre at rest = (0, 0, 0).

    Attributes:
        head_center: (3,) head centre in world frame (for visualisation only).
        head_anchors: (3, 2) rod attachment points on the head (P), head-local.
        servo_offsets: (3, 2) horn rotation origins relative to head centre (B).
        lhl: Horn length(s) — scalar or (2,).
        ldl: Rod length(s) — scalar or (2,).
        yaw_ratio: Bevel gear ratio ``head_yaw = servo_yaw * yaw_ratio``.

    """

    head_center: np.ndarray
    head_anchors: np.ndarray
    servo_offsets: np.ndarray
    lhl: float | np.ndarray
    ldl: float | np.ndarray
    yaw_ratio: float

    def __post_init__(self) -> None:
        self.head_center = np.asarray(self.head_center).reshape(3)
        self.head_anchors = np.asarray(self.head_anchors)
        self.servo_offsets = np.asarray(self.servo_offsets)
        if self.head_anchors.shape != (3, 2) or self.servo_offsets.shape != (3, 2):
            raise ValueError("head_anchors and servo_offsets must be (3, 2)")
        self.lhl = np.broadcast_to(np.asarray(self.lhl, dtype=float), 2)
        self.ldl = np.broadcast_to(np.asarray(self.ldl, dtype=float), 2)


# ======================================================================
# Default geometry (measured from physical robot)
# ======================================================================
# Frame: +X = forward (face), +Y = left, +Z = up.
# Servo plane at z=0; head_center is derived above.

#: Servo horn rotation origins (world frame, z=0 servo plane).
#: Columns: [PR0 (left), PR1 (right)].
#: Measured: 24.5 mm behind head centre, ±14.5 mm left/right.
_DEFAULT_SERVO_POSITIONS: np.ndarray = np.array(
    [
        [-24.5, -24.5],  # x: behind head centre
        [+14.5, -14.5],  # y: PR0 left, PR1 right
        [0.0, 0.0],  # z: servo plane
    ]
)

#: Rod attachment points on the head plate (head-local frame).
#: Columns: [PR0 (left), PR1 (right)].
#: Measured: 29.2 mm behind head centre, ±46.7 mm left/right.
_DEFAULT_HEAD_ANCHORS: np.ndarray = np.array(
    [
        [-29.2, -29.2],  # x: behind head centre
        [+46.7, -46.7],  # y: PR0 left, PR1 right
        [0.0, 0.0],  # z
    ]
)


def default_config(
    servo_positions: np.ndarray | None = None,
    head_anchors: np.ndarray | None = None,
    lhl: float | np.ndarray = 35.0,
    ldl: float | np.ndarray = 47.0,
    yaw_ratio: float = 0.5,
) -> HeadConfig:
    """Build a :class:`HeadConfig` from explicit Cartesian positions.

    Parameters
    ----------
    servo_positions : (3, 2), optional
        Horn rotation origins (servo positions) in world frame.
        Defaults to :data:`_DEFAULT_SERVO_POSITIONS` (measured from robot).
    head_anchors : (3, 2), optional
        Rod attachment points on the head, in head-local frame.
        Defaults to :data:`_DEFAULT_HEAD_ANCHORS`.
    lhl : float or (2,)
        Horn (crank) length(s).
    ldl : float or (2,)
        Connecting rod length(s).
    yaw_ratio : float
        Bevel gear ratio (``head_yaw = servo_yaw * yaw_ratio``).

    Returns
    -------
    HeadConfig

    """
    if servo_positions is None:
        servo_positions = _DEFAULT_SERVO_POSITIONS.copy()
    if head_anchors is None:
        head_anchors = _DEFAULT_HEAD_ANCHORS.copy()

    servo_positions = np.asarray(servo_positions)
    head_anchors = np.asarray(head_anchors)
    lhl_arr = np.broadcast_to(np.asarray(lhl, dtype=float), 2)
    ldl_arr = np.broadcast_to(np.asarray(ldl, dtype=float), 2)

    # Derive head_center z so that rods are taut at the rest pose.
    dx = head_anchors[0, 0] - servo_positions[0, 0]
    dy = head_anchors[1, 0] - servo_positions[1, 0]
    z = float(np.sqrt(ldl_arr[0] ** 2 + lhl_arr[0] ** 2 - dx**2 - dy**2))
    head_center = np.array([0.0, 0.0, z])
    servo_offsets = servo_positions - head_center[:, np.newaxis]

    return HeadConfig(
        head_center=head_center,
        head_anchors=head_anchors,
        servo_offsets=servo_offsets,
        lhl=lhl,
        ldl=ldl,
        yaw_ratio=yaw_ratio,
    )


# ======================================================================
# IK solver
# ======================================================================


def rotation_matrix_rpy(rpy: np.ndarray) -> np.ndarray:
    """Roll-pitch-yaw (rad) -> 3×3 rotation matrix ``Rz · Ry · Rx``."""
    rpy = np.asarray(rpy).reshape(3)
    c, s = np.cos(rpy[2]), np.sin(rpy[2])
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    c, s = np.cos(rpy[1]), np.sin(rpy[1])
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    c, s = np.cos(rpy[0]), np.sin(rpy[0])
    Rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return Rz @ Ry @ Rx


def solve_ik(
    rpy: np.ndarray,
    B: np.ndarray,
    P: np.ndarray,
    lhl: np.ndarray,
    ldl: np.ndarray,
    yaw_ratio: float,
) -> np.ndarray:
    """Solve IK: head RPY (rad) -> ``[PR0, PR1, yaw_servo]`` (rad).

    Parameters
    ----------
    rpy : (3,) roll, pitch, yaw in radians.
    B   : (3, 2) horn rotation origins.
    P   : (3, 2) head anchors in head-local frame.
    lhl : (2,) horn lengths.
    ldl : (2,) rod lengths.
    yaw_ratio : bevel gear ratio.

    Returns
    -------
    (3,) actuator angles ``[PR0, PR1, yaw_servo]`` in radians.

    """
    rot = rotation_matrix_rpy(rpy)
    L = (rot @ P) - B

    pr_angles = np.zeros(2)
    for k in range(2):
        ly, lz = L[1, k], L[2, k]
        leg_len = np.linalg.norm(L[:, k])
        g = leg_len**2 - (ldl[k] ** 2 - lhl[k] ** 2)
        e = 2 * lhl[k] * lz
        fk = 2 * lhl[k] * ly
        denom = np.sqrt(e**2 + fk**2)
        if k == 0:
            angle = np.arcsin(g / denom) - np.arctan2(fk, e)
        else:
            angle = np.arctan2(-e, fk) + np.arccos(np.clip(-g / denom, -1.0, 1.0))
        pr_angles[k] = angle

    yaw_servo = rpy[2] / yaw_ratio
    return np.array([pr_angles[0], pr_angles[1], yaw_servo])


def solve_ik_from_config(
    rpy: np.ndarray,
    config: HeadConfig,
) -> np.ndarray:
    """Convenience wrapper: solve IK using a ``HeadConfig``."""
    return solve_ik(
        rpy,
        B=config.servo_offsets,
        P=config.head_anchors,
        lhl=config.lhl,
        ldl=config.ldl,
        yaw_ratio=config.yaw_ratio,
    )
