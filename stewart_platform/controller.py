import math
from typing import Optional, Tuple
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt


class Servo_Configuration(object):
    """Configuration for a single servo in the Stewart Platform"""

    def __init__(
        self, B: np.ndarray, P: np.ndarray, lhl: float, ldl: float, beta: float
    ):
        """
        Initialize a servo configuration.

        Parameters:
        -----------
        B : np.ndarray (3,)
            Base anchor position in local frame (where servo arm connects to base)
        P : np.ndarray (3,)
            Platform anchor position in local frame (where rod connects to platform)
        lhl : float
            Length of servo horn
        ldl : float
            Length of connecting rod
        beta : float
            Servo arm orientation angle (radians) - angle between the plane in which
            the servo arm moves and the xz-plane of the base CS
        """
        self.B = B
        self.P = P
        self.lhl = lhl
        self.ldl = ldl
        self.beta = beta


class RotationServo_Configuration(object):
    """Configuration for a single yaw servo in the Stewart Platform"""

    def __init__(
        self, B: np.ndarray, P: np.ndarray, lhl: float, ldl: float, beta: float
    ):
        """
        Initialize a servo configuration.

        Parameters:
        -----------
        B : np.ndarray (3,)
            Base anchor position in local frame (where servo arm connects to base)
        P : np.ndarray (3,)
            Base anchor position in locale frame (where connecting rod connects to rotatable part of base)
        lhl : float
            Length of servo horn
        ldl : float
            Length of connecting rod
        beta : float
            Servo arm orientation angle (radians) - starting angle of the servo horn in the xy-plane
        """
        self.B = B
        self.P = P
        self.lhl = lhl
        self.ldl = ldl
        self.beta = beta


class Stewart_Platform(object):
    """
    Stewart Platform Python Implementation using Rotational Servos

    This class accepts an array of servo configurations, platform vertices,
    and base vertices to model a Stewart Platform with arbitrary servo arrangements.
    """

    def __init__(
        self,
        servo_configs: list[Servo_Configuration],
        platform_vertices: np.ndarray,
        base_vertices: np.ndarray,
    ):
        """
        Initialize Stewart Platform from servo configurations and geometry.

        Parameters:
        -----------
        servo_configs : list[Servo_Configuration]
            List of servo configurations, each containing B, P, lhl, ldl, beta
        platform_vertices : np.ndarray (3, n)
            Platform polygon vertices in local frame for visualization
        base_vertices : np.ndarray (3, m)
            Base polygon vertices in local frame for visualization
        """
        # Store servo configurations
        self.servo_configs = servo_configs
        self.num_servos = len(servo_configs)

        # Extract servo data into arrays for efficient computation
        self.B = np.zeros((3, self.num_servos))
        self.P = np.zeros((3, self.num_servos))
        self.beta = np.zeros(self.num_servos)
        self.lhl = np.zeros(self.num_servos)
        self.ldl = np.zeros(self.num_servos)

        for i, config in enumerate(servo_configs):
            self.B[:, i] = config.B
            self.P[:, i] = config.P
            self.beta[i] = config.beta
            self.lhl[i] = config.lhl
            self.ldl[i] = config.ldl

        # Allocate arrays for servo calculations
        self.l = np.zeros((3, self.num_servos))
        self.lll = np.zeros(self.num_servos)
        self.angles = np.zeros(self.num_servos)
        self.H = np.zeros((3, self.num_servos))

        # Store visualization geometry
        self.base_vertices = base_vertices
        self.platform_vertices_local = platform_vertices
        self.platform_vertices_global = np.zeros_like(platform_vertices)

        # Calculate home position based on first servo's geometry
        z = np.sqrt(
            self.ldl[0] ** 2
            + self.lhl[0] ** 2
            - (self.P[0, 0] - self.B[0, 0]) ** 2
            - (self.P[1, 0] - self.B[1, 0]) ** 2
        )
        self.home_pos = np.array([0, 0, z])

    @classmethod
    def create_6dof_platform(
        cls,
        r_B: float,
        r_P: float,
        lhl: float,
        ldl: float,
        gamma_B: float,
        gamma_P: float,
        ref_rotation: float = 0.0,
    ):
        """
        Factory method to create a standard 6-DOF Stewart Platform.

        This creates a symmetric 6-servo configuration commonly used in Stewart Platforms.

        Parameters:
        -----------
        r_B : float
            Radius for circumscribed circle where base servo anchors lie
        r_P : float
            Radius for circumscribed circle where platform anchors lie
        lhl : float
            Length of servo horn
        ldl : float
            Length of connecting rod
        gamma_B : float
            Half-angle for base anchor spacing (radians)
        gamma_P : float
            Half-angle for platform anchor spacing (radians)
        ref_rotation : float, optional
            Reference rotation to apply to entire configuration (radians)

        Returns:
        --------
        Stewart_Platform
            Configured 6-DOF Stewart Platform instance
        """
        pi = np.pi

        # Beta (Angle) - Servo arm orientation angles
        # Angle between the plane in which the servo arm moves and the xz-plane
        beta = (
            np.array(
                [
                    pi / 2 + pi,
                    pi / 2,
                    2 * pi / 3 + pi / 2 + pi,
                    2 * pi / 3 + pi / 2,
                    4 * pi / 3 + pi / 2 + pi,
                    4 * pi / 3 + pi / 2,
                ]
            )
            + ref_rotation
        )

        # Psi_B - Base anchor polar angles
        psi_B = (
            np.array(
                [
                    -gamma_B,
                    gamma_B,
                    2 * pi / 3 - gamma_B,
                    2 * pi / 3 + gamma_B,
                    2 * pi / 3 + 2 * pi / 3 - gamma_B,
                    2 * pi / 3 + 2 * pi / 3 + gamma_B,
                ]
            )
            + ref_rotation
        )

        # Psi_P - Platform anchor polar angles
        psi_P = (
            np.array(
                [
                    pi / 3 + 2 * pi / 3 + 2 * pi / 3 + gamma_P,
                    pi / 3 + -gamma_P,
                    pi / 3 + gamma_P,
                    pi / 3 + 2 * pi / 3 - gamma_P,
                    pi / 3 + 2 * pi / 3 + gamma_P,
                    pi / 3 + 2 * pi / 3 + 2 * pi / 3 - gamma_P,
                ]
            )
            + ref_rotation
        )

        # Create servo configurations
        servo_configs = []
        for i in range(6):
            B = r_B * np.array([np.cos(psi_B[i]), np.sin(psi_B[i]), 0])
            P = r_P * np.array([np.cos(psi_P[i]), np.sin(psi_P[i]), 0])
            servo_configs.append(Servo_Configuration(B, P, lhl, ldl, beta[i]))

        # Create hexagonal base vertices for visualization
        psi_base_vertices = (
            np.array([0, pi / 3, 2 * pi / 3, pi, 4 * pi / 3, 5 * pi / 3]) + ref_rotation
        )
        base_vertices = np.transpose(
            r_B * np.array([[np.cos(v), np.sin(v), 0] for v in psi_base_vertices])
        )

        # Create hexagonal platform vertices for visualization
        psi_platform_vertices = (
            np.array([0, pi / 3, 2 * pi / 3, pi, 4 * pi / 3, 5 * pi / 3]) + ref_rotation
        )
        platform_vertices = np.transpose(
            r_P * np.array([[np.cos(v), np.sin(v), 0] for v in psi_platform_vertices])
        )

        return cls(servo_configs, platform_vertices, base_vertices)

    def calculate_servo_angle(self, k, lx, ly, lz, leg_length):
        """
        Calculate the servo angle for a single servo.

        Parameters:
        -----------
        k : int
            Servo index
        lx : float
            X component of leg vector relative to base anchor
        ly : float
            Y component of leg vector relative to base anchor
        lz : float
            Z component of leg vector relative to base anchor
        leg_length : float
            Length of the leg (magnitude of leg vector)

        Returns:
        --------
        angle : float
            Servo angle in radians
        H : np.ndarray
            Position of the spherical joint connecting servo arm and rod (3D point)
        """
        # Get servo-specific parameters
        lhl_k = self.lhl[k]
        ldl_k = self.ldl[k]
        beta_k = self.beta[k]

        # Calculate auxiliary quantities g, f and e
        g = leg_length**2 - (ldl_k**2 - lhl_k**2)
        e = 2 * lhl_k * lz
        fk = 2 * lhl_k * (np.cos(beta_k) * lx + np.sin(beta_k) * ly)

        # Calculate servo angle
        # The wanted position could be achieved if the solution of this
        # equation is real for all i
        angle = np.arcsin(g / np.sqrt(e**2 + fk**2)) - np.arctan2(fk, e)

        # Get position of the point where a spherical joint connects servo arm and rod
        H = np.array(
            [
                lhl_k * np.cos(angle) * np.cos(beta_k) + self.B[0, k],
                lhl_k * np.cos(angle) * np.sin(beta_k) + self.B[1, k],
                lhl_k * np.sin(angle),
            ]
        )

        return angle, H

    def calculate(self, trans, rotation):
        trans = np.transpose(trans)
        rotation = np.transpose(rotation)

        # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
        R = np.matmul(
            np.matmul(self.rotZ(rotation[2]), self.rotY(rotation[1])),
            self.rotX(rotation[0]),
        )
        # R = np.matmul( np.matmul(self.rotX(rotation[0]), self.rotY(rotation[1])), self.rotZ(rotation[2]) )

        # Get leg length for each leg
        # leg = np.repeat(trans[:, np.newaxis], num_servos, axis=1) + np.repeat(home_pos[:, np.newaxis], num_servos, axis=1) + np.matmul(np.transpose(R), P) - B

        # Get leg length for each leg
        self.l = (
            np.repeat(trans[:, np.newaxis], self.num_servos, axis=1)
            + np.repeat(self.home_pos[:, np.newaxis], self.num_servos, axis=1)
            + np.matmul(R, self.P)
            - self.B
        )
        self.lll = np.linalg.norm(self.l, axis=0)

        # Position of leg in global frame (anchor points on platform)
        self.L = self.l + self.B

        # Update platform vertices in global frame (same transformation as anchors)
        num_platform_vertices = self.platform_vertices_local.shape[1]
        self.platform_vertices_global = (
            np.repeat(trans[:, np.newaxis], num_platform_vertices, axis=1)
            + np.repeat(self.home_pos[:, np.newaxis], num_platform_vertices, axis=1)
            + np.matmul(R, self.platform_vertices_local)
        )

        # Position of legs, wrt to their individual bases, split for clarity.
        lx = self.l[0, :]
        ly = self.l[1, :]
        lz = self.l[2, :]

        # Calculate servo angles for each leg using the extracted method
        for k in range(self.num_servos):
            self.angles[k], self.H[:, k] = self.calculate_servo_angle(
                k, lx[k], ly[k], lz[k], self.lll[k]
            )

        return self.angles

    def plot3D_line(self, ax, vec_arr_origin, vec_arr_dest, color_):
        num_lines = vec_arr_origin.shape[1]
        for i in range(num_lines):
            ax.plot(
                [vec_arr_origin[0, i], vec_arr_dest[0, i]],
                [vec_arr_origin[1, i], vec_arr_dest[1, i]],
                [vec_arr_origin[2, i], vec_arr_dest[2, i]],
                color=color_,
            )

    def plot_platform(self, ax=None):
        if ax is None:
            ax = plt.axes(projection="3d")  # Data for a three-dimensional line
        ax.set_xlim3d(-100, 100)
        ax.set_ylim3d(-100, 100)
        ax.set_zlim3d(0, 200)
        ax.set_xlabel("x-axis")
        ax.set_ylabel("y-axis")
        ax.set_zlabel("z-axis")

        # Draw base (using base vertices)
        ax.add_collection3d(
            Poly3DCollection(
                [list(np.transpose(self.base_vertices))], facecolors="green", alpha=0.25
            )
        )

        # Draw platform (using platform vertices, not anchor points)
        ax.add_collection3d(
            Poly3DCollection(
                [list(np.transpose(self.platform_vertices_global))],
                facecolors="blue",
                alpha=0.25,
            )
        )

        # Draw servo components
        self.plot3D_line(ax, self.B, self.H, "red")  # Servo horns
        self.plot3D_line(ax, self.H, self.L, "black")  # Rods
        self.plot3D_line(ax, self.B, self.L, "orange")  # Virtual legs
        return ax

    def plot_platform_g(self):
        ax = plt.axes(projection="3d")  # Data for a three-dimensional line
        ax.set_xlim3d(-400, 400)
        ax.set_ylim3d(-400, 400)
        ax.set_zlim3d(0, 200)
        ax.set_xlabel("x-axis")
        ax.set_ylabel("y-axis")
        ax.set_zlabel("z-axis")

        # Draw base (using base vertices)
        ax.add_collection3d(
            Poly3DCollection(
                [list(np.transpose(self.base_vertices))], facecolors="green", alpha=0.25
            )
        )

        # Draw platform (using platform vertices, not anchor points)
        ax.add_collection3d(
            Poly3DCollection(
                [list(np.transpose(self.platform_vertices_global))],
                facecolors="blue",
                alpha=0.25,
            )
        )

        # Draw servo components
        self.plot3D_line(ax, self.B, self.H, "red")  # Servo horns
        self.plot3D_line(ax, self.H, self.L, "black")  # Rods
        self.plot3D_line(ax, self.B, self.L, "orange")  # Virtual legs
        return ax

    def rotX(self, phi):
        rotx = np.array(
            [[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]]
        )
        return rotx

    def rotY(self, theta):
        roty = np.array(
            [
                [np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta), 0, np.cos(theta)],
            ]
        )
        return roty

    def rotZ(self, psi):
        rotz = np.array(
            [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
        )
        return rotz

    # Roll yaw pitch notation
    # def rotX(self, phi):
    #     rotx = np.array([
    #         [1,     0    ,    0    ],
    #         [0,  np.cos(phi), np.sin(phi)],
    #         [0, -np.sin(phi), np.cos(phi)] ])
    #     return rotx

    # def rotY(self, theta):
    #     roty = np.array([
    #         [np.cos(theta), 0, -np.sin(theta) ],
    #         [0         , 1,     0       ],
    #         [np.sin(theta), 0,  np.cos(theta) ] ])
    #     return roty

    # def rotZ(self, psi):
    #     rotz = np.array([
    #         [ np.cos(psi), np.sin(psi), 0 ],
    #         [-np.sin(psi), np.cos(psi), 0 ],
    #         [   0        ,     0      , 1 ] ])
    #     return rotz


def solve_joint_point_np(Q, P, H, L):
    """
    NumPy version.

    Q : array-like of shape (3,)   (servo base point, ie: center of servo horn rotation)
    P : array-like of shape (3,)   (target base anchor)
    H : float                      (servo horn length)
    L : float                      (rod length)

    Returns:
        (J1, J2) where each is a (3,) NumPy array
        or None if no valid solutions exist.
    """

    Q = np.asarray(Q, dtype=float)
    P = np.asarray(P, dtype=float)

    # vertical separation
    dz = P[2] - Q[2]

    # Check if the rod can reach vertically
    if abs(dz) > L:
        return None

    # Effective rod length in XY
    Lxy = np.sqrt(L**2 - dz**2)

    # XY vectors
    Qxy = Q[:2]
    Pxy = P[:2]
    d_vec = Pxy - Qxy
    d = np.linalg.norm(d_vec)

    # Check XY circle reach
    if d > H + Lxy or d < abs(H - Lxy):
        return None

    # Distance along the line between Q and P to chord midpoint
    a = (H**2 - Lxy**2 + d**2) / (2 * d)

    # XY midpoint between the two possible joint points
    midpoint = Qxy + (a / d) * d_vec

    # Distance from midpoint to intersection
    h_sq = H**2 - a**2
    h_sq = max(h_sq, 0.0)  # guard against tiny negatives
    h = np.sqrt(h_sq)

    # Perpendicular direction
    perp = np.array([-d_vec[1], d_vec[0]]) / d  # rotate 90°

    # Two solutions
    J1_xy = midpoint + h * perp
    J2_xy = midpoint - h * perp

    # Full 3D points (z stays at Qz)
    J1 = np.array([J1_xy[0], J1_xy[1], Q[2]])
    J2 = np.array([J2_xy[0], J2_xy[1], Q[2]])

    return J1, J2
