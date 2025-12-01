import numpy as np
from kinematics.base import PlatformState


class Platform:
    """
    Core Stewart platform model: geometry + pose transforms.

    Responsibilities:
    - Hold base and platform polygon vertices for visualization
    - Hold base (`B`) and platform (`P`) anchor locations in local frames
    - Maintain pose (translation + rotation) and compute global transforms
    - Provide derived leg vectors and global anchor/platform vertices

    No servo/kinematics math lives here.
    """

    def __init__(
        self,
        B: np.ndarray,
        P: np.ndarray,
        platform_vertices: np.ndarray,
        base_vertices: np.ndarray,
        home_pos: np.ndarray | None = None,
    ) -> None:
        assert B.shape[0] == 3 and P.shape[0] == 3, "B and P must be (3, n)"
        assert platform_vertices.shape[0] == 3, "platform_vertices must be (3, m)"
        assert base_vertices.shape[0] == 3, "base_vertices must be (3, k)"

        self.B = B  # base anchors (local)
        self.P = P  # platform anchors (local)
        self.base_vertices = base_vertices
        self.platform_vertices_local = platform_vertices

        # Pose
        self.trans = np.zeros(3)
        self.rot = np.zeros(3)  # roll, pitch, yaw

        # Derived
        self.platform_vertices_global = np.zeros_like(platform_vertices)
        self.L = np.zeros_like(B)  # leg vectors in global frame

        # Home position offset (z lift etc.)
        if home_pos is None:
            self.home_pos = np.zeros(3)
        else:
            self.home_pos = home_pos

    def set_pose(self, trans: np.ndarray, rot: np.ndarray) -> None:
        self.trans = np.asarray(trans).reshape(3)
        self.rot = np.asarray(rot).reshape(3)

        R = self._rotation_matrix(self.rot)
        # Leg vectors in global frame: T + home + R*P - B
        self.L = (
            np.repeat(self.trans[:, np.newaxis], self.P.shape[1], axis=1)
            + np.repeat(self.home_pos[:, np.newaxis], self.P.shape[1], axis=1)
            + np.matmul(R, self.platform_anchors_local())
            - self.base_anchors_local()
        )

        # Update platform vertices
        self.platform_vertices_global = (
            np.repeat(
                self.trans[:, np.newaxis], self.platform_vertices_local.shape[1], axis=1
            )
            + np.repeat(
                self.home_pos[:, np.newaxis],
                self.platform_vertices_local.shape[1],
                axis=1,
            )
            + np.matmul(R, self.platform_vertices_local)
        )

    def base_anchors_local(self) -> np.ndarray:
        return self.B

    def platform_anchors_local(self) -> np.ndarray:
        return self.P

    def state(self) -> PlatformState:
        """Expose the current platform state to kinematics solvers."""
        return PlatformState(
            B=self.B,
            P=self.P,
            L=self.L,
            platform_vertices_global=self.platform_vertices_global,
            base_vertices=self.base_vertices,
            home_pos=self.home_pos,
            trans=self.trans,
            rot=self.rot,
        )

    @staticmethod
    def rotX(phi: float) -> np.ndarray:
        return np.array(
            [[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]]
        )

    @staticmethod
    def rotY(theta: float) -> np.ndarray:
        return np.array(
            [
                [np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta), 0, np.cos(theta)],
            ]
        )

    @staticmethod
    def rotZ(psi: float) -> np.ndarray:
        return np.array(
            [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
        )

    @classmethod
    def _rotation_matrix(cls, rpy: np.ndarray) -> np.ndarray:
        rpy = np.asarray(rpy).reshape(3)
        return cls.rotZ(rpy[2]) @ cls.rotY(rpy[1]) @ cls.rotX(rpy[0])
