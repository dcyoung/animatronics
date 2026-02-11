"""Motor controllers for the Personality Core robot."""

from personality_core.motor_controller.xl430 import (
    DEFAULT_BAUDRATE,
    DEFAULT_SERVO_CONFIGS,
    SERVO_PR0,
    SERVO_PR1,
    SERVO_RESOLUTION,
    SERVO_YAW,
    HeadPosition,
    ServoConfig,
    XL430Controller,
)

__all__ = [
    "DEFAULT_BAUDRATE",
    "DEFAULT_SERVO_CONFIGS",
    "SERVO_PR0",
    "SERVO_PR1",
    "SERVO_RESOLUTION",
    "SERVO_YAW",
    "HeadPosition",
    "ServoConfig",
    "XL430Controller",
]
