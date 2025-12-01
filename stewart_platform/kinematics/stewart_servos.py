import numpy as np
from .base import IKinematics, KinematicsResult, PlatformState


class StewartServosKinematics(IKinematics):
    """
    Kinematics implementation matching current Nx rotational servos.

    Parameters per servo:
    - lhl: servo horn length
    - ldl: rod length
    - beta: servo plane/orientation angle
    """

    def __init__(self, lhl: np.ndarray, ldl: np.ndarray, beta: np.ndarray) -> None:
        self.lhl = np.asarray(lhl)
        self.ldl = np.asarray(ldl)
        self.beta = np.asarray(beta)

    def solve(self, state: PlatformState) -> KinematicsResult:
        B: np.ndarray = state.B
        L: np.ndarray = state.L

        num_servos = B.shape[1]
        angles = np.zeros(num_servos)
        H = np.zeros_like(B)

        lx = L[0, :]
        ly = L[1, :]
        lz = L[2, :]
        leg_lengths = np.linalg.norm(L, axis=0)

        for k in range(num_servos):
            angle, Hk = self._calculate_servo_angle(
                k, lx[k], ly[k], lz[k], leg_lengths[k], B
            )
            angles[k] = angle
            H[:, k] = Hk

        return KinematicsResult(angles=angles, H=H)

    def _calculate_servo_angle(
        self,
        k: int,
        lx: float,
        ly: float,
        lz: float,
        leg_length: float,
        B: np.ndarray,
    ):
        lhl_k = self.lhl[k]
        ldl_k = self.ldl[k]
        beta_k = self.beta[k]

        g = leg_length**2 - (ldl_k**2 - lhl_k**2)
        e = 2 * lhl_k * lz
        fk = 2 * lhl_k * (np.cos(beta_k) * lx + np.sin(beta_k) * ly)

        angle = np.arcsin(g / np.sqrt(e**2 + fk**2)) - np.arctan2(fk, e)

        H = np.array(
            [
                lhl_k * np.cos(angle) * np.cos(beta_k) + B[0, k],
                lhl_k * np.cos(angle) * np.sin(beta_k) + B[1, k],
                lhl_k * np.sin(angle),
            ]
        )

        return angle, H
