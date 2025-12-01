import numpy as np
from .base import IKinematics, KinematicsResult, PlatformState


class YawServoKinematics(IKinematics):
    """
    Kinematics for a single servo controlling platform yaw (rotation around z-axis).

    The servo is mounted on the base with its horn rotating in the XY plane.
    A connecting rod links the servo horn to an anchor point that rotates with
    the platform around the Z axis.

    Parameters:
    -----------
    servo_base : np.ndarray (3,)
        Position of the servo center on the base (where horn rotates)
    anchor_rest : np.ndarray (3,)
        Position of the platform anchor at rest (relative to base origin)
    lhl : float
        Length of servo horn
    ldl : float
        Length of connecting rod
    beta : float
        Servo horn reference angle (radians) - angle of horn at zero position in XY plane
    """

    def __init__(
        self,
        servo_base: np.ndarray,
        anchor_rest: np.ndarray,
        lhl: float,
        ldl: float,
        beta: float = 0.0,
    ) -> None:
        self.servo_base = np.asarray(servo_base).reshape(3)
        self.anchor_rest = np.asarray(anchor_rest).reshape(3)
        self.lhl = lhl
        self.ldl = ldl
        self.beta = beta

    def solve(self, state: PlatformState) -> KinematicsResult:
        """
        Solve for the yaw servo angle given platform yaw rotation.

        The anchor rotates around the Z axis matching the platform's yaw (rot[2]).
        """
        # Extract yaw from platform rotation
        yaw = state.rot[2]

        # Calculate anchor position after yaw rotation around Z axis
        # Anchor rotates in XY plane around base origin
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        anchor_current = np.array(
            [
                self.anchor_rest[0] * cos_yaw - self.anchor_rest[1] * sin_yaw,
                self.anchor_rest[0] * sin_yaw + self.anchor_rest[1] * cos_yaw,
                self.anchor_rest[2],  # Z remains constant
            ]
        )

        # Vector from servo base to anchor
        vec_to_anchor = anchor_current - self.servo_base

        # Since both servo and anchor are in XY plane (or close to it),
        # we solve in 2D XY space
        dx = vec_to_anchor[0]
        dy = vec_to_anchor[1]
        dz = vec_to_anchor[2]

        # Distance to anchor
        dist_xy = np.sqrt(dx**2 + dy**2)
        dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)

        # Check if reachable
        if dist_3d > self.lhl + self.ldl or dist_3d < abs(self.lhl - self.ldl):
            # If not reachable, return a safe angle
            angle = self.beta
        else:
            # Use law of cosines to find servo angle
            # This is simplified for XY plane rotation
            # Angle from servo base to anchor in XY plane
            angle_to_anchor = np.arctan2(dy, dx)

            # Use law of cosines to find the angle offset from direct line
            # cos(offset) = (lhl^2 + dist^2 - ldl^2) / (2 * lhl * dist)
            cos_offset = (self.lhl**2 + dist_3d**2 - self.ldl**2) / (
                2 * self.lhl * dist_3d
            )
            cos_offset = np.clip(cos_offset, -1, 1)
            offset = np.arccos(cos_offset)

            # Servo angle (choosing + offset solution for natural upward swing)
            angle = angle_to_anchor + offset

        # Calculate horn tip position (spherical joint)
        H = np.array(
            [
                self.servo_base[0] + self.lhl * np.cos(angle),
                self.servo_base[1] + self.lhl * np.sin(angle),
                self.servo_base[2],  # Stays at base height
            ]
        ).reshape(3, 1)

        angles = np.array([angle])

        return KinematicsResult(angles=angles, H=H, metadata={"anchor": anchor_current})
