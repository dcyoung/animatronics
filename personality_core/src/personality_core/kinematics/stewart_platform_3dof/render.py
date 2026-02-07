"""Matplotlib 3-D rendering helpers for the 3-DOF Stewart platform head."""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def horn_tips(B: np.ndarray, lhl: np.ndarray, pr_angles: np.ndarray) -> np.ndarray:
    """Compute horn-tip positions ``(3, 2)`` for the two PR servos (drawing only).

    Horns rotate in the YZ plane (about X); servo 1 is mirrored for symmetry.
    """
    H = np.zeros((3, 2))
    for k in range(2):
        sign = 1 if k == 0 else -1
        H[:, k] = (
            B[0, k],
            B[1, k] + sign * lhl[k] * np.cos(pr_angles[k]),
            lhl[k] * np.sin(pr_angles[k]),
        )
    return H


def draw_base(ax, base_vertices: np.ndarray):
    """Draw the base polygon."""
    ax.add_collection3d(
        Poly3DCollection(
            [list(np.transpose(base_vertices))], facecolors="green", alpha=0.25
        )
    )


def draw_head(ax, head_vertices_global: np.ndarray):
    """Draw the head polygon."""
    ax.add_collection3d(
        Poly3DCollection(
            [list(np.transpose(head_vertices_global))],
            facecolors="blue",
            alpha=0.25,
        )
    )


def draw_lines(ax, origin: np.ndarray, dest: np.ndarray, color: str):
    """Draw lines connecting origin to dest columns."""
    num_lines = origin.shape[1]
    for i in range(num_lines):
        ax.plot(
            [origin[0, i], dest[0, i]],
            [origin[1, i], dest[1, i]],
            [origin[2, i], dest[2, i]],
            color=color,
        )


def draw_kinematics(ax, B: np.ndarray, H: np.ndarray, L: np.ndarray):
    """Draw servo horns, rods, and virtual legs."""
    draw_lines(ax, B, H, "red")  # Servo horns
    draw_lines(ax, H, B + L, "black")  # Rods
    draw_lines(ax, B, B + L, "orange")  # Virtual legs


def draw_3dof_bevel_yaw(
    ax,
    base_vertices: np.ndarray,
    head_vertices_global: np.ndarray,
    B: np.ndarray,
    H: np.ndarray,
    L: np.ndarray,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    yaw_servo_deg: float | None = None,
):
    """Draw 3-DOF head with bevel-gear yaw.

    Renders the base circle, head, two PR servos (horns + rods).
    Yaw is not drawn (invisible drive); optional text shows R/P/Y and
    yaw servo angle.
    """
    draw_base(ax, base_vertices)
    draw_head(ax, head_vertices_global)
    draw_kinematics(ax, B, H, L)
    parts = [
        f"Roll:  {np.degrees(roll):6.1f}\u00b0",
        f"Pitch: {np.degrees(pitch):6.1f}\u00b0",
        f"Yaw:   {np.degrees(yaw):6.1f}\u00b0",
    ]
    if yaw_servo_deg is not None:
        parts.append(f"Yaw servo (bevel): {yaw_servo_deg:6.1f}\u00b0")
    ax.text2D(
        0.02,
        0.98,
        "\n".join(parts),
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        family="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )


def default_axes(lims=(-100, 100), zlim=(0, 200)):
    """Create default 3-D axes with standard limits."""
    ax = plt.axes(projection="3d")
    ax.set_xlim3d(lims[0], lims[1])
    ax.set_ylim3d(lims[0], lims[1])
    ax.set_zlim3d(zlim[0], zlim[1])
    ax.set_xlabel("x-axis")
    ax.set_ylabel("y-axis")
    ax.set_zlabel("z-axis")
    return ax
