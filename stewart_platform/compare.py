"""Comparison of 6-DOF and 3-DOF platform configurations."""

import numpy as np
import matplotlib.pyplot as plt
from factory import create_6dof_platform, create_3dof_platform_with_yaw
import render


def main():
    # Create both platforms
    platform_6dof, kinematics_6dof = create_6dof_platform(
        r_B=132 / 2, r_P=100 / 2, lhl=30, ldl=130, gamma_B=0.2269, gamma_P=0.82
    )

    platform_3dof, pitch_roll_kin, yaw_kin = create_3dof_platform_with_yaw(
        r_B=132 / 2, r_P=100 / 2, lhl=30, ldl=130, gamma_B=0.2269, gamma_P=0.82
    )

    # Set same pose for both
    rot = np.array([0.2, 0.15, 0.1])
    trans = np.array([0, 0, 0])

    platform_6dof.set_pose(trans, rot)
    platform_3dof.set_pose(trans, rot)

    state_6dof = platform_6dof.state()
    state_3dof = platform_3dof.state()

    result_6dof = kinematics_6dof.solve(state_6dof)
    pitch_roll_result = pitch_roll_kin.solve(state_3dof)
    yaw_result = yaw_kin.solve(state_3dof)

    # Create side-by-side plots
    fig = plt.figure(figsize=(16, 7))

    # 6-DOF plot
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.set_xlim3d(-100, 100)
    ax1.set_ylim3d(-100, 100)
    ax1.set_zlim3d(0, 200)
    ax1.set_title("6-DOF Platform (6 servos)", fontsize=14, fontweight="bold")
    render.draw_base(ax1, state_6dof.base_vertices)
    render.draw_platform(ax1, state_6dof.platform_vertices_global)
    render.draw_kinematics(ax1, state_6dof.B, result_6dof.H, state_6dof.L)

    # 3-DOF plot
    ax2 = fig.add_subplot(122, projection="3d")
    ax2.set_xlim3d(-100, 100)
    ax2.set_ylim3d(-100, 100)
    ax2.set_zlim3d(0, 200)
    ax2.set_title(
        "3-DOF Platform (2 pitch/roll + 1 yaw)", fontsize=14, fontweight="bold"
    )
    render.draw_base(ax2, state_3dof.base_vertices)
    render.draw_platform(ax2, state_3dof.platform_vertices_global)
    render.draw_kinematics(ax2, state_3dof.B, pitch_roll_result.H, state_3dof.L)

    yaw_anchor = yaw_result.metadata["anchor"]
    render.draw_yaw_kinematics(ax2, yaw_kin.servo_base, yaw_result.H, yaw_anchor)

    plt.tight_layout()
    plt.show()

    print("\n6-DOF Platform:")
    print(f"  Servos: {len(result_6dof.angles)}")
    print(f"  Angles (deg): {np.degrees(result_6dof.angles)}")

    print("\n3-DOF Platform:")
    print(f"  Pitch/Roll Servos: {len(pitch_roll_result.angles)}")
    print(f"  Angles (deg): {np.degrees(pitch_roll_result.angles)}")
    print(f"  Yaw Servo Angle (deg): {np.degrees(yaw_result.angles[0]):.1f}°")

    print(f"\nPlatform Orientation:")
    print(f"  Roll:  {np.degrees(rot[0]):.1f}°")
    print(f"  Pitch: {np.degrees(rot[1]):.1f}°")
    print(f"  Yaw:   {np.degrees(rot[2]):.1f}°")


if __name__ == "__main__":
    main()
