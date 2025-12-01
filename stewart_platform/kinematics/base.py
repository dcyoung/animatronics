from dataclasses import dataclass
from typing import Protocol
import numpy as np


@dataclass
class PlatformState:
    """Type-safe platform state for kinematics solvers."""

    B: np.ndarray  # (3, n) base anchors in local frame
    P: np.ndarray  # (3, n) platform anchors in local frame
    L: np.ndarray  # (3, n) leg vectors in global frame
    platform_vertices_global: np.ndarray  # (3, m) platform vertices in global frame
    base_vertices: np.ndarray  # (3, k) base vertices in local frame
    home_pos: np.ndarray  # (3,) home position offset
    trans: np.ndarray  # (3,) current translation
    rot: np.ndarray  # (3,) current rotation (roll, pitch, yaw)


@dataclass
class KinematicsResult:
    """Result from kinematics solver."""

    angles: np.ndarray  # (n,) servo angles
    H: np.ndarray  # (3, n) spherical joint points
    metadata: dict | None = None  # optional extra info


class IKinematics(Protocol):
    def solve(self, state: PlatformState) -> KinematicsResult:
        """
        Compute servo/joint states given a platform state.
        """
        ...
