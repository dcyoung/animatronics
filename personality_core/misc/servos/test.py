import time
from rustypot import Xl430PyController
import numpy as np

SERVO_IDS = [0, 1, 2]
RESOLUTION = 4096


def rad2pos(rad):
    # 0 rad => 0,
    # 2pi => RESOLUTION
    return int((rad % (2 * np.pi)) / (2 * np.pi) * RESOLUTION)


# def deg2pos(deg):
#     rad = np.deg2rad(deg)
#     return rad2pos(rad)


def main():
    print("Starting servo test...")
    c = Xl430PyController(
        serial_port="/dev/tty.usbserial-FTAAMLB8",
        # serial_port="/dev/cu.usbserial-FTAAMLB8",
        baudrate=57_600,
        timeout=0.1,
    )

    print("Enabling torque...")
    for servo_id in SERVO_IDS:
        c.write_torque_enable(servo_id, True)

    amp = np.deg2rad(90.0)
    center = np.deg2rad(180)
    freq = 0.1
    print("Starting sinusoidal motion...")
    while True:
        t = time.time()
        phase_shifts = [id * (1 / freq) / len(SERVO_IDS) for id in SERVO_IDS]
        # phase_shifts = [0] * len(SERVO_IDS)
        pos = [
            rad2pos(center + amp * np.sin(2 * np.pi * freq * (t + phase_shift)))
            for phase_shift in phase_shifts
        ]
        c.sync_write_goal_position(
            SERVO_IDS,
            pos,
        )

        # print(pos)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
