import time
import numpy as np
from servo import (
    SERVO_PR_LEFT,
    SERVO_PR_RIGHT,
    SERVO_YAW,
    rad2pos,
    get_servo_controller,
)

SERVOS = [SERVO_PR_LEFT, SERVO_PR_RIGHT, SERVO_YAW]


def main():
    print("Starting servo test...")
    c = get_servo_controller()

    print("Enabling torque...")
    SERVO_IDS = [servo.id for servo in SERVOS]
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
