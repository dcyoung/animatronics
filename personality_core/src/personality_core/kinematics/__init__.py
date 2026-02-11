"""Kinematics for the Personality Core 3-DOF head."""

from personality_core.kinematics.analytical_kinematics import AnalyticalKinematics
from personality_core.kinematics.stewart_platform_3dof.ik_solver import (
    HeadConfig,
    default_config,
)

__all__ = [
    "AnalyticalKinematics",
    "HeadConfig",
    "default_config",
]
