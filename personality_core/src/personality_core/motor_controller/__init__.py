"""Motor controllers for the Personality Core robot."""

from personality_core.motor_controller.xl430 import (
    DEFAULT_BAUDRATE,
    DEFAULT_MOTOR_NAME_ID,
    DEFAULT_SERVO_IDS,
    SERVO_RESOLUTION,
    HeadPosition,
    XL430Controller,
    pos_to_rad,
    rad_to_pos,
)

__all__ = [
    "DEFAULT_BAUDRATE",
    "DEFAULT_MOTOR_NAME_ID",
    "DEFAULT_SERVO_IDS",
    "SERVO_RESOLUTION",
    "HeadPosition",
    "XL430Controller",
    "pos_to_rad",
    "rad_to_pos",
]
