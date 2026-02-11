import time
from rustypot import Xl430PyController
import numpy as np
from dataclasses import dataclass


RESOLUTION = 4096


def pos2rad(pos: int) -> float:
    return float(pos) / RESOLUTION * 2 * np.pi


def rad2pos(rad):
    # 0 rad => 0,
    # 2pi => RESOLUTION
    return int((rad % (2 * np.pi)) / (2 * np.pi) * RESOLUTION)


@dataclass
class Servo:
    id: int
    name: str
    step_center: int
    step_limits: tuple[int, int]

    @property
    def rad_center(self) -> float:
        return pos2rad(self.step_center)

    @property
    def rad_limits(self) -> tuple[float, float]:
        return (
            pos2rad(self.step_limits[0]),
            pos2rad(self.step_limits[1]),
        )


SERVO_PR_LEFT = Servo(id=1, name="pr0", step_limits=(512, 1536), step_center=1024)
SERVO_PR_RIGHT = Servo(id=0, name="pr1", step_limits=(2560, 3584), step_center=3072)
SERVO_YAW = Servo(id=2, name="yaw", step_limits=(1536, 2560), step_center=2048)


def get_servo_controller() -> Xl430PyController:
    return Xl430PyController(
        serial_port="/dev/tty.usbserial-FTAAMLB8",
        # serial_port="/dev/cu.usbserial-FTAAMLB8",
        baudrate=57_600,
        timeout=0.1,
    )
