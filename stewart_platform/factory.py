"""Factory functions for creating standard Stewart Platform configurations."""

import numpy as np
from platform import Platform
from kinematics.stewart_servos import StewartServosKinematics
from kinematics.yaw_servo import YawServoKinematics


def create_6dof_platform(
    r_B: float,
    r_P: float,
    lhl: float,
    ldl: float,
    gamma_B: float,
    gamma_P: float,
    ref_rotation: float = 0.0,
) -> tuple[Platform, StewartServosKinematics]:
    """
    Factory to create a standard 6-DOF Stewart Platform with rotational servos.

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
    tuple[Platform, StewartServosKinematics]
        Configured platform and kinematics solver
    """
    pi = np.pi

    # Beta (Angle) - Servo arm orientation angles
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

    # Build B and P anchor arrays
    B = np.zeros((3, 6))
    P = np.zeros((3, 6))
    for i in range(6):
        B[:, i] = r_B * np.array([np.cos(psi_B[i]), np.sin(psi_B[i]), 0])
        P[:, i] = r_P * np.array([np.cos(psi_P[i]), np.sin(psi_P[i]), 0])

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

    # Calculate home position based on geometry
    z = np.sqrt(ldl**2 + lhl**2 - (P[0, 0] - B[0, 0]) ** 2 - (P[1, 0] - B[1, 0]) ** 2)
    home_pos = np.array([0, 0, z])

    # Create platform and kinematics
    platform = Platform(
        B=B,
        P=P,
        platform_vertices=platform_vertices,
        base_vertices=base_vertices,
        home_pos=home_pos,
    )

    # Create kinematics with uniform servo parameters
    lhl_array = np.full(6, lhl)
    ldl_array = np.full(6, ldl)
    kinematics = StewartServosKinematics(lhl=lhl_array, ldl=ldl_array, beta=beta)

    return platform, kinematics


def create_3dof_platform(
    r_B: float,
    r_P: float,
    lhl: float,
    ldl: float,
    gamma_B: float,
    gamma_P: float,
    ref_rotation: float = 0.0,
) -> tuple[Platform, StewartServosKinematics]:
    """
    Factory to create a 3-DOF platform with 2 adjacent servos.

    Uses only the first two servos from the 6-DOF configuration pattern,
    providing pitch and roll control with limited DOF.

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
    tuple[Platform, StewartServosKinematics]
        Configured platform with 2 servos and kinematics solver
    """
    pi = np.pi

    # Beta (Angle) - Servo arm orientation angles for first 2 servos only
    beta = np.array([pi / 2 + pi, pi / 2]) + ref_rotation

    # Psi_B - Base anchor polar angles for first 2 servos
    psi_B = np.array([-gamma_B, gamma_B]) + ref_rotation

    # Psi_P - Platform anchor polar angles for first 2 servos
    psi_P = (
        np.array(
            [
                pi / 3 + 2 * pi / 3 + 2 * pi / 3 + gamma_P,
                pi / 3 + -gamma_P,
            ]
        )
        + ref_rotation
    )

    # Build B and P anchor arrays (only 2 servos)
    B = np.zeros((3, 2))
    P = np.zeros((3, 2))
    for i in range(2):
        B[:, i] = r_B * np.array([np.cos(psi_B[i]), np.sin(psi_B[i]), 0])
        P[:, i] = r_P * np.array([np.cos(psi_P[i]), np.sin(psi_P[i]), 0])

    # Create triangular base vertices for visualization (simpler geometry)
    psi_base_vertices = np.array([0, 2 * pi / 3, 4 * pi / 3]) + ref_rotation
    base_vertices = np.transpose(
        r_B * np.array([[np.cos(v), np.sin(v), 0] for v in psi_base_vertices])
    )

    # Create triangular platform vertices for visualization
    psi_platform_vertices = np.array([0, 2 * pi / 3, 4 * pi / 3]) + ref_rotation
    platform_vertices = np.transpose(
        r_P * np.array([[np.cos(v), np.sin(v), 0] for v in psi_platform_vertices])
    )

    # Calculate home position based on geometry
    z = np.sqrt(ldl**2 + lhl**2 - (P[0, 0] - B[0, 0]) ** 2 - (P[1, 0] - B[1, 0]) ** 2)
    home_pos = np.array([0, 0, z])

    # Create platform and kinematics
    platform = Platform(
        B=B,
        P=P,
        platform_vertices=platform_vertices,
        base_vertices=base_vertices,
        home_pos=home_pos,
    )

    # Create kinematics with uniform servo parameters (only 2 servos)
    lhl_array = np.full(2, lhl)
    ldl_array = np.full(2, ldl)
    kinematics = StewartServosKinematics(lhl=lhl_array, ldl=ldl_array, beta=beta)

    return platform, kinematics


def create_3dof_platform_with_yaw(
    r_B: float,
    r_P: float,
    lhl: float,
    ldl: float,
    gamma_B: float,
    gamma_P: float,
    yaw_servo_pos: np.ndarray | None = None,
    yaw_anchor_rest: np.ndarray | None = None,
    yaw_lhl: float | None = None,
    yaw_ldl: float | None = None,
    ref_rotation: float = 0.0,
) -> tuple[Platform, StewartServosKinematics, YawServoKinematics]:
    """
    Factory to create a 3-DOF platform with 2 pitch/roll servos + 1 yaw servo.

    Parameters:
    -----------
    r_B : float
        Radius for circumscribed circle where base servo anchors lie
    r_P : float
        Radius for circumscribed circle where platform anchors lie
    lhl : float
        Length of servo horn for pitch/roll servos
    ldl : float
        Length of connecting rod for pitch/roll servos
    gamma_B : float
        Half-angle for base anchor spacing (radians)
    gamma_P : float
        Half-angle for platform anchor spacing (radians)
    yaw_servo_pos : np.ndarray, optional
        Position of yaw servo on base. Defaults to [0, -r_B, 0]
    yaw_anchor_rest : np.ndarray, optional
        Rest position of yaw anchor. Defaults to [0, r_P, 0]
    yaw_lhl : float, optional
        Length of yaw servo horn. Defaults to lhl
    yaw_ldl : float, optional
        Length of yaw connecting rod. Defaults to ldl
    ref_rotation : float, optional
        Reference rotation to apply to entire configuration (radians)

    Returns:
    --------
    tuple[Platform, StewartServosKinematics, YawServoKinematics]
        Configured platform, pitch/roll kinematics, and yaw kinematics
    """
    # Create base 3-DOF platform with 2 servos
    platform, pitch_roll_kinematics = create_3dof_platform(
        r_B, r_P, lhl, ldl, gamma_B, gamma_P, ref_rotation
    )

    pi = np.pi

    # Set defaults for yaw servo
    # Position yaw servo at back (180°) between the two pitch/roll servos at -gamma_B and +gamma_B
    if yaw_servo_pos is None:
        yaw_servo_pos = r_B * np.array(
            [np.cos(pi / 2 + ref_rotation), np.sin(pi / 2 + ref_rotation), 0]
        )  # At 90° - adjacent side
    if yaw_anchor_rest is None:
        # Anchor perpendicular to horn: horn points +X, so anchor should be along +Y from horn tip
        # Horn tip at rest: servo_pos + lhl*[cos(beta), sin(beta), 0] = [30, 66, 0]
        # Place anchor directly above (+Y direction from horn tip)
        yaw_anchor_rest = np.array([30, 66 - r_P, 20])  # Perpendicular configuration
    if yaw_lhl is None:
        yaw_lhl = lhl
    if yaw_ldl is None:
        yaw_ldl = ldl * 0.4  # Shorter rod for more direct control

    # Create yaw servo kinematics
    yaw_kinematics = YawServoKinematics(
        servo_base=yaw_servo_pos,
        anchor_rest=yaw_anchor_rest,
        lhl=yaw_lhl,
        ldl=yaw_ldl,
        beta=np.pi / 2,  # Horn points along +X axis at rest
    )

    return platform, pitch_roll_kinematics, yaw_kinematics
