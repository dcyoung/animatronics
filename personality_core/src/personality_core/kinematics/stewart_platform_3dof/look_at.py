"""Look-at orientation: compute head roll/pitch/yaw to point at a target.

Uses the same RPY convention as Head: ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.

Forward convention: head forward = +X (face), back = -X (PR anchors).
"""

import numpy as np

#: Head forward axis: +X (face direction).
FORWARD_AXIS_HEAD = np.array([1.0, 0.0, 0.0])

_DEFAULT_FORWARD = FORWARD_AXIS_HEAD


def look_at_rpy(
    head_pos: np.ndarray,
    target: np.ndarray,
    forward_axis: np.ndarray | None = None,
    up_world: np.ndarray | None = None,
) -> np.ndarray:
    """Compute roll, pitch, yaw (radians) so the head's forward axis points at *target*.

    Parameters
    ----------
    head_pos : (3,) position of head/ball-joint in world frame.
    target : (3,) position of the look-at target in world frame.
    forward_axis : (3,) head's forward direction in local frame (default +X).
    up_world : (3,) world up for roll disambiguation (default +Z).

    Returns
    -------
    rpy : (3,) roll, pitch, yaw in radians.

    """
    head_pos = np.asarray(head_pos).reshape(3)
    target = np.asarray(target).reshape(3)
    if forward_axis is None:
        forward_axis = _DEFAULT_FORWARD.copy()
    else:
        forward_axis = np.asarray(forward_axis).reshape(3)
    if up_world is None:
        up_world = np.array([0.0, 0.0, 1.0])
    else:
        up_world = np.asarray(up_world).reshape(3)

    d = target - head_pos
    dist = np.linalg.norm(d)
    if dist < 1e-9:
        return np.zeros(3)
    d = d / dist

    right = np.cross(up_world, d)
    rn = np.linalg.norm(right)
    if rn < 1e-9:
        right = (
            np.array([1.0, 0.0, 0.0])
            if abs(d[0]) < 0.9
            else np.array([0.0, 1.0, 0.0])
        )
        right = right - np.dot(right, d) * d
        rn = np.linalg.norm(right)
        if rn >= 1e-9:
            right = right / rn
        else:
            return np.zeros(3)
    else:
        right = right / rn
    up = np.cross(d, right)

    if np.abs(forward_axis[0]) > 0.9:  # +X
        R = np.column_stack([d, right, up])
    elif np.abs(forward_axis[1]) > 0.9:  # +Y
        R = np.column_stack([right, d, up])
    else:  # +Z
        R = np.column_stack([right, up, d])

    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    cp = np.cos(pitch)
    if np.abs(cp) < 1e-9:
        yaw = 0.0
        roll = np.arctan2(-R[1, 2], R[0, 2])
    else:
        yaw = np.arctan2(R[1, 0] / cp, R[0, 0] / cp)
        roll = np.arctan2(R[2, 1] / cp, R[2, 2] / cp)
    return np.array([roll, pitch, yaw])
