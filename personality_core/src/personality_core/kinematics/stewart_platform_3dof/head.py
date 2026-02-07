"""Head geometry and pose for visualization.

Maintains base/head anchors, vertices, and transforms.  Provides derived
leg vectors and global anchor/head vertex positions for 3-D rendering.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class HeadState:
    """Full head state at a pose: geometry and transforms for visualization.

    Used to draw the head, base, legs, and horn tips.  For IK, pass
    ``state.rot`` to ``solve_ik_from_config`` (orientation only).
    """

    B: np.ndarray  # (3, n) base anchors (horn origins)
    P: np.ndarray  # (3, n) head anchors in local frame
    L: np.ndarray  # (3, n) leg vectors in global frame
    head_vertices_global: np.ndarray
    base_vertices: np.ndarray
    home_pos: np.ndarray
    trans: np.ndarray
    rot: np.ndarray  # roll, pitch, yaw (rad)


class Head:
    """Head geometry and pose: base/head anchors, vertices, and transforms.

    - Hold base and head polygon vertices for visualization
    - Hold base (B) and head (P) anchor locations in local frames
    - Maintain pose (translation + rotation) and compute global transforms
    - Provide derived leg vectors and global anchor/head vertices
    """

    def __init__(
        self,
        B: np.ndarray,
        P: np.ndarray,
        head_vertices: np.ndarray,
        base_vertices: np.ndarray,
        home_pos: np.ndarray | None = None,
    ) -> None:
        assert B.shape[0] == 3 and P.shape[0] == 3, "B and P must be (3, n)"
        assert head_vertices.shape[0] == 3, "head_vertices must be (3, m)"
        assert base_vertices.shape[0] == 3, "base_vertices must be (3, k)"

        self.B = B  # base anchors (local)
        self.P = P  # head anchors (local)
        self.base_vertices = base_vertices
        self.head_vertices_local = head_vertices

        self.trans = np.zeros(3)
        self.rot = np.zeros(3)  # roll, pitch, yaw

        self.head_vertices_global = np.zeros_like(head_vertices)
        self.L = np.zeros_like(B)  # leg vectors in global frame

        if home_pos is None:
            self.home_pos = np.zeros(3)
        else:
            self.home_pos = home_pos

    def set_pose(self, trans: np.ndarray, rot: np.ndarray) -> None:
        """Set the head pose and recompute global transforms."""
        self.trans = np.asarray(trans).reshape(3)
        self.rot = np.asarray(rot).reshape(3)

        R = self._rotation_matrix(self.rot)
        self.L = (
            np.repeat(self.trans[:, np.newaxis], self.P.shape[1], axis=1)
            + np.repeat(self.home_pos[:, np.newaxis], self.P.shape[1], axis=1)
            + np.matmul(R, self.head_anchors_local())
            - self.base_anchors_local()
        )

        self.head_vertices_global = (
            np.repeat(
                self.trans[:, np.newaxis], self.head_vertices_local.shape[1], axis=1
            )
            + np.repeat(
                self.home_pos[:, np.newaxis],
                self.head_vertices_local.shape[1],
                axis=1,
            )
            + np.matmul(R, self.head_vertices_local)
        )

    def base_anchors_local(self) -> np.ndarray:
        """Return the base anchor positions."""
        return self.B

    def head_anchors_local(self) -> np.ndarray:
        """Return the head anchor positions in local frame."""
        return self.P

    def state(self) -> HeadState:
        """Current head state for visualization."""
        return HeadState(
            B=self.B,
            P=self.P,
            L=self.L,
            head_vertices_global=self.head_vertices_global,
            base_vertices=self.base_vertices,
            home_pos=self.home_pos,
            trans=self.trans,
            rot=self.rot,
        )

    @staticmethod
    def rotX(phi: float) -> np.ndarray:
        """Rotation matrix about X axis."""
        return np.array(
            [[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]]
        )

    @staticmethod
    def rotY(theta: float) -> np.ndarray:
        """Rotation matrix about Y axis."""
        return np.array(
            [
                [np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta), 0, np.cos(theta)],
            ]
        )

    @staticmethod
    def rotZ(psi: float) -> np.ndarray:
        """Rotation matrix about Z axis."""
        return np.array(
            [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
        )

    @classmethod
    def _rotation_matrix(cls, rpy: np.ndarray) -> np.ndarray:
        rpy = np.asarray(rpy).reshape(3)
        return cls.rotZ(rpy[2]) @ cls.rotY(rpy[1]) @ cls.rotX(rpy[0])
