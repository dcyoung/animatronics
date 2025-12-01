"""Animated visualization of Stewart Platform configurations."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from factory import create_6dof_platform, create_3dof_platform_with_yaw
import render


def main_6dof(fps=30):
    """Run 6-DOF platform animation with 6 rotational servos."""
    platform, kinematics = create_6dof_platform(
        r_B=132 / 2,
        r_P=100 / 2,
        lhl=30,
        ldl=130,
        gamma_B=0.2269,
        gamma_P=0.82,
        ref_rotation=0.0,
    )

    rx_amp, rx_freq = np.pi / 12, 0.5
    ry_amp, ry_freq = np.pi / 12, 1.0
    rz_amp, rz_freq = np.pi / 12, 0.25

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Track window state
    running = {"active": True}

    def on_close(event):
        running["active"] = False

    fig.canvas.mpl_connect("close_event", on_close)

    # Loop through various angles
    frame = 0
    while running["active"]:
        frame += 1
        ax.cla()
        ax.set_xlim3d(-100, 100)
        ax.set_ylim3d(-100, 100)
        ax.set_zlim3d(0, 200)
        ax.set_xlabel("x-axis")
        ax.set_ylabel("y-axis")
        ax.set_zlabel("z-axis")
        ax.set_title("6-DOF Platform: 6 Rotational Servos", fontsize=12)

        t = frame / fps
        trans = np.array([0, 0, 0])
        rot = np.array(
            [
                rx_amp * np.sin(2 * np.pi * rx_freq * t),
                ry_amp * np.cos(2 * np.pi * ry_freq * t),
                rz_amp * np.sin(2 * np.pi * rz_freq * t),
            ]
        )

        # Update platform pose
        platform.set_pose(trans, rot)

        # Solve kinematics
        state = platform.state()
        result = kinematics.solve(state)

        # Draw everything
        render.draw_base(ax, state.base_vertices)
        render.draw_platform(ax, state.platform_vertices_global)
        render.draw_kinematics(ax, state.B, result.H, state.L)

        plt.pause(1 / fps)
        plt.draw()


def main_3dof(fps=30):
    """Run 3-DOF platform animation with 2 pitch/roll servos + 1 yaw servo."""
    platform, pitch_roll_kin, yaw_kin = create_3dof_platform_with_yaw(
        r_B=132 / 2,
        r_P=100 / 2,
        lhl=30,
        ldl=130,
        gamma_B=0.2269,
        gamma_P=0.82,
        ref_rotation=0.0,
    )

    rx_amp, rx_freq = np.pi / 12, 0.5
    ry_amp, ry_freq = np.pi / 12, 1.0
    rz_amp, rz_freq = np.pi / 6, 0.3

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Track window state
    running = {"active": True}

    def on_close(event):
        running["active"] = False

    fig.canvas.mpl_connect("close_event", on_close)

    # Loop through various angles
    frame = 0
    while running["active"]:
        frame += 1
        ax.cla()
        ax.set_xlim3d(-100, 100)
        ax.set_ylim3d(-100, 100)
        ax.set_zlim3d(0, 200)
        ax.set_xlabel("x-axis")
        ax.set_ylabel("y-axis")
        ax.set_zlabel("z-axis")
        ax.set_title("3-DOF Platform: 2 Pitch/Roll Servos + 1 Yaw Servo", fontsize=12)

        t = frame / fps
        trans = np.array([0, 0, 0])
        rot = np.array(
            [
                rx_amp * np.sin(2 * np.pi * rx_freq * t),
                ry_amp * np.cos(2 * np.pi * ry_freq * t),
                rz_amp * np.sin(2 * np.pi * rz_freq * t),
            ]
        )

        # Update platform pose
        platform.set_pose(trans, rot)

        # Solve kinematics
        state = platform.state()
        pitch_roll_result = pitch_roll_kin.solve(state)
        yaw_result = yaw_kin.solve(state)

        # Draw base and platform
        render.draw_base(ax, state.base_vertices)
        render.draw_platform(ax, state.platform_vertices_global)

        # Draw pitch/roll servos
        render.draw_kinematics(ax, state.B, pitch_roll_result.H, state.L)

        # Draw yaw servo
        yaw_anchor = yaw_result.metadata["anchor"]
        render.draw_yaw_kinematics(ax, yaw_kin.servo_base, yaw_result.H, yaw_anchor)

        # Display detailed info
        info_text = (
            f"Pitch/Roll Servos:\n"
            f"  S1: {np.degrees(pitch_roll_result.angles[0]):6.1f}°\n"
            f"  S2: {np.degrees(pitch_roll_result.angles[1]):6.1f}°\n"
            f"Yaw Servo:\n"
            f"  S3: {np.degrees(yaw_result.angles[0]):6.1f}°\n"
            f"\nPlatform Rotation:\n"
            f"  Roll:  {np.degrees(rot[0]):6.1f}°\n"
            f"  Pitch: {np.degrees(rot[1]):6.1f}°\n"
            f"  Yaw:   {np.degrees(rot[2]):6.1f}°"
        )
        ax.text2D(
            0.02,
            0.98,
            info_text,
            transform=ax.transAxes,
            fontsize=9,
            verticalalignment="top",
            family="monospace",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

        plt.pause(1 / fps)
        plt.draw()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Animate Stewart Platform configurations"
    )
    parser.add_argument(
        "--config",
        type=str,
        choices=["6dof", "3dof"],
        default="6dof",
        help="Platform configuration: 6dof (6 servos) or 3dof (2 pitch/roll + 1 yaw)",
    )
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    args = parser.parse_args()

    if args.config == "6dof":
        main_6dof(fps=args.fps)
    else:
        main_3dof(fps=args.fps)
