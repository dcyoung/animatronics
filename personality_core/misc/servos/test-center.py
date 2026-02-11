import time
from servo import (
    SERVO_PR_LEFT,
    SERVO_PR_RIGHT,
    SERVO_YAW,
    get_servo_controller,
)

SERVOS = [SERVO_PR_LEFT, SERVO_PR_RIGHT, SERVO_YAW]


def main():
    print("Starting servo test...")
    c = get_servo_controller()

    print("Enabling torque...")
    for servo in SERVOS:
        c.write_torque_enable(servo.id, True)

    print(f"Centering")
    print("Ctrl-C to stop.\n")

    for servo in SERVOS:
        c.sync_write_goal_position([servo.id], [servo.step_center])

    time.sleep(2)
    for servo in SERVOS:
        c.write_torque_enable(servo.id, False)
    print("Done.")


if __name__ == "__main__":
    main()
