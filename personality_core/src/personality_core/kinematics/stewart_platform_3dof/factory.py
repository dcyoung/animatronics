"""Build visualization-ready Head objects from a HeadConfig.

Bridge between the IK solver (``ik_solver.py``) and the 3-D visualization
layer (``head.py``, ``render.py``).
"""

import numpy as np

from personality_core.kinematics.stewart_platform_3dof.head import Head
from personality_core.kinematics.stewart_platform_3dof.ik_solver import HeadConfig

FORWARD_AXIS_HEAD = np.array([1.0, 0.0, 0.0])


def head_from_config(config: HeadConfig) -> Head:
    """Build a :class:`Head` from a :class:`HeadConfig` for visualization."""
    B = config.head_center[:, np.newaxis] + config.servo_offsets
    P = config.head_anchors.copy()
    # Base: circle in XY plane at z=0 that everything sits on
    n = 64
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    r_base = max(np.max(np.linalg.norm(B[:2, :], axis=0)) + 30, 80)
    base_vertices = np.array([
        r_base * np.cos(theta),
        r_base * np.sin(theta),
        np.zeros(n),
    ])
    head_vertices = np.hstack([np.zeros((3, 1)), P])
    return Head(
        B=B,
        P=P,
        head_vertices=head_vertices,
        base_vertices=base_vertices,
        home_pos=config.head_center.copy(),
    )
