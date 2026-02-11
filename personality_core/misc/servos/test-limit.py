import time
import numpy as np
from servo import (
    SERVO_PR_LEFT,
    SERVO_PR_RIGHT,
    rad2pos,
    RESOLUTION,
    get_servo_controller,
)

SERVOS = [SERVO_PR_LEFT, SERVO_PR_RIGHT]


def main():
    print("Starting servo test...")
    c = get_servo_controller()

    print("Enabling torque...")
    for servo in SERVOS:
        c.write_torque_enable(servo.id, True)

    freq = 0.1  # Hz — one full cycle every 10 seconds
    print(f"Sweeping full resolution (0..{RESOLUTION - 1}) as sin wave @ {freq} Hz")
    print("Ctrl-C to stop.\n")

    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            # sin in [-1, 1] → map to [0, RESOLUTION-1]
            raw = (np.sin(2 * np.pi * freq * t) + 1.0) / 2.0  # 0..1
            target_pos = int(raw * (RESOLUTION - 1))

            for servo in SERVOS:
                c.sync_write_goal_position([servo.id], [target_pos])

            for servo in SERVOS:
                p = c.read_present_position(servo.id)
                # rustypot may return int or list — normalise to int
                if isinstance(p, (list, tuple)):
                    p = p[0]
                lo, hi = servo.step_limits
                clamped = "CLAMPED" if (target_pos < lo or target_pos > hi) else "ok"
                print(
                    f"  {servo.name}(id={servo.id}): "
                    f"target={target_pos:4d}  actual={int(p):4d}  "
                    f"limits=[{lo}, {hi}]  {clamped}",
                )
            print()

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
