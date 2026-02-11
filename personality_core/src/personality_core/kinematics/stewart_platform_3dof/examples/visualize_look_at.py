"""Visualization: head tracks a look-at target on a circle.

Uses the 3-DOF Stewart platform head configuration, IK solver, and
visualization helpers to animate a head following a target that moves
in a circle in front of the head (+X).

Run via CLI::

    personality-core-visualize-look-at --fps 30 --speed 0.5
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from personality_core.kinematics.stewart_platform_3dof import render
from personality_core.kinematics.stewart_platform_3dof.factory import head_from_config
from personality_core.kinematics.stewart_platform_3dof.ik_solver import (
    default_config,
    solve_ik_from_config,
)
from personality_core.kinematics.stewart_platform_3dof.look_at import look_at_rpy


def circle_target(
    head_pos: np.ndarray,
    t: float,
    forward_distance: float,
    radius: float,
) -> np.ndarray:
    """Return a target point on a circle in the YZ plane, forward along +X."""
    head_pos = np.asarray(head_pos).reshape(3)
    center = head_pos.copy()
    center[0] += forward_distance
    return center + radius * np.array([0.0, np.cos(t), np.sin(t)])


def draw_circle(
    ax,
    head_pos: np.ndarray,
    forward_distance: float,
    radius: float,
    n_pts: int = 64,
    color: str = "gray",
    linestyle: str = "--",
    alpha: float = 0.7,
):
    """Draw the look-at target circle in 3-D."""
    ts = np.linspace(0, 2 * np.pi, n_pts, endpoint=False)
    pts = np.array(
        [circle_target(head_pos, ti, forward_distance, radius) for ti in ts]
    )
    ax.plot(
        pts[:, 0], pts[:, 1], pts[:, 2],
        color=color, linestyle=linestyle, alpha=alpha,
    )


def main(
    fps: int = 30,
    forward_distance: float = 80.0,
    radius: float = 40.0,
    speed: float = 0.5,
):
    """Run look-at visualization: head tracks a target moving on a circle."""
    config = default_config()
    head = head_from_config(config)
    head_pos = head.home_pos.copy()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    running = {"active": True}

    def on_close(event):
        running["active"] = False

    fig.canvas.mpl_connect("close_event", on_close)
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
        ax.set_title("Head tracking look-at target (circle)", fontsize=12)

        t = (frame / fps) * speed * 2 * np.pi
        target = circle_target(head_pos, t, forward_distance, radius)
        rpy = look_at_rpy(head_pos, target)
        trans = np.zeros(3)
        head.set_pose(trans, rpy)

        state = head.state()
        angles = solve_ik_from_config(state.rot, config)
        H = render.horn_tips(state.B, config.lhl, angles[:2])

        render.draw_3dof_bevel_yaw(
            ax,
            state.base_vertices,
            state.head_vertices_global,
            state.B,
            H,
            state.L,
            roll=rpy[0],
            pitch=rpy[1],
            yaw=rpy[2],
            yaw_servo_deg=np.degrees(angles[2]),
        )

        # Telemetry: rod lengths (horn tip -> head anchor) should match config.ldl
        head_anchors_world = state.B + state.L
        rod_lengths = np.array([
            np.linalg.norm(head_anchors_world[:, k] - H[:, k]) for k in range(2)
        ])
        errors = rod_lengths - config.ldl
        telemetry_rod = "  |  ".join(
            [
                f"Rod{k}: {rod_lengths[k]:.4f} "
                f"(nom. {config.ldl[k]:.4f}, err {errors[k]:+.6f})"
                for k in range(2)
            ]
        )
        telemetry_angles = (
            f"PR0: {angles[0]:.4f} rad ({np.degrees(angles[0]):.2f}\u00b0)  |  "
            f"PR1: {angles[1]:.4f} rad ({np.degrees(angles[1]):.2f}\u00b0)  |  "
            f"yaw: {angles[2]:.4f} rad ({np.degrees(angles[2]):.2f}\u00b0)"
        )
        telemetry = telemetry_rod + "\n" + telemetry_angles
        if not hasattr(ax, "_telemetry_text"):
            ax._telemetry_text = ax.figure.text(
                0.02, 0.02, "", fontsize=9, family="monospace",
                verticalalignment="bottom", transform=ax.figure.transFigure,
            )
        ax._telemetry_text.set_text(telemetry)

        # Draw the circle and current target
        draw_circle(
            ax, head_pos, forward_distance, radius,
            color="gray", linestyle="--", alpha=0.7,
        )
        ax.scatter(
            [target[0]], [target[1]], [target[2]],
            color="red", s=80, marker="o", label="Target", zorder=5,
        )

        plt.pause(1 / fps)
        plt.draw()


def cli() -> None:
    """CLI entry point for ``personality-core-visualize-look-at``."""
    parser = argparse.ArgumentParser(
        description="Visualize head tracking a look-at target on a circle"
    )
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument(
        "--forward-distance",
        type=float,
        default=80.0,
        help="Distance of circle plane from head along +X",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=40.0,
        help="Radius of the target circle",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.5,
        help="Revolutions per second of target along circle",
    )
    args = parser.parse_args()
    main(
        fps=args.fps,
        forward_distance=args.forward_distance,
        radius=args.radius,
        speed=args.speed,
    )


if __name__ == "__main__":
    cli()
