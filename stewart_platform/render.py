import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def draw_base(ax, base_vertices: np.ndarray):
    ax.add_collection3d(
        Poly3DCollection(
            [list(np.transpose(base_vertices))], facecolors="green", alpha=0.25
        )
    )


def draw_platform(ax, platform_vertices_global: np.ndarray):
    ax.add_collection3d(
        Poly3DCollection(
            [list(np.transpose(platform_vertices_global))],
            facecolors="blue",
            alpha=0.25,
        )
    )


def draw_lines(ax, origin: np.ndarray, dest: np.ndarray, color: str):
    num_lines = origin.shape[1]
    for i in range(num_lines):
        ax.plot(
            [origin[0, i], dest[0, i]],
            [origin[1, i], dest[1, i]],
            [origin[2, i], dest[2, i]],
            color=color,
        )


def draw_kinematics(ax, B: np.ndarray, H: np.ndarray, L: np.ndarray):
    draw_lines(ax, B, H, "red")  # Servo horns
    draw_lines(ax, H, B + L, "black")  # Rods
    draw_lines(ax, B, B + L, "orange")  # Virtual legs


def draw_yaw_kinematics(
    ax, servo_base: np.ndarray, H: np.ndarray, anchor: np.ndarray, color: str = "purple"
):
    """Draw yaw servo components with distinct color."""
    # Servo horn
    ax.plot(
        [servo_base[0], H[0, 0]],
        [servo_base[1], H[1, 0]],
        [servo_base[2], H[2, 0]],
        color=color,
        linewidth=2,
        label="Yaw servo horn",
    )
    # Connecting rod
    ax.plot(
        [H[0, 0], anchor[0]],
        [H[1, 0], anchor[1]],
        [H[2, 0], anchor[2]],
        color="cyan",
        linewidth=2,
        linestyle="--",
        label="Yaw rod",
    )
    # Mark anchor point
    ax.scatter([anchor[0]], [anchor[1]], [anchor[2]], color="red", s=50, marker="o")


def default_axes(lims=(-100, 100), zlim=(0, 200)):
    ax = plt.axes(projection="3d")
    ax.set_xlim3d(lims[0], lims[1])
    ax.set_ylim3d(lims[0], lims[1])
    ax.set_zlim3d(zlim[0], zlim[1])
    ax.set_xlabel("x-axis")
    ax.set_ylabel("y-axis")
    ax.set_zlabel("z-axis")
    return ax
