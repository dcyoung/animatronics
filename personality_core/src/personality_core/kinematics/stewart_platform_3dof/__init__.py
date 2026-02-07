"""3-DOF Stewart platform head: IK solver, geometry, visualization, and look-at utilities."""

from personality_core.kinematics.stewart_platform_3dof.factory import (
    FORWARD_AXIS_HEAD,
    from_config,
    head_from_config,
)
from personality_core.kinematics.stewart_platform_3dof.head import Head, HeadState
from personality_core.kinematics.stewart_platform_3dof.ik_solver import (
    HeadConfig,
    default_config,
    solve_ik,
    solve_ik_from_config,
    symmetric_config,
)
from personality_core.kinematics.stewart_platform_3dof.look_at import look_at_rpy

__all__ = [
    "FORWARD_AXIS_HEAD",
    "Head",
    "HeadConfig",
    "HeadState",
    "default_config",
    "from_config",
    "head_from_config",
    "look_at_rpy",
    "solve_ik",
    "solve_ik_from_config",
    "symmetric_config",
]
