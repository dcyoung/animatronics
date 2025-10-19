import numpy as np
from controller import Stewart_Platform
import matplotlib.pyplot as plt


def main():
    FPS = 30
    platform = Stewart_Platform.create_6dof_platform(
        r_B=132 / 2,
        r_P=100 / 2,
        lhl=30,
        ldl=130,
        gamma_B=0.2269,
        gamma_P=0.82,
        ref_rotation=0.0,
    )
    rx_amp, rx_freq = np.pi / 12, 0.5
    ry_amp, ry_freq = np.pi / 12, 1.0
    rz_amp, rz_freq = np.pi / 12, 0.25

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Loop through various angles
    frame = 0
    while True:
        frame += 1
        ax.cla()
        t = frame / FPS
        servo_angles = platform.calculate(
            np.array([0, 0, 0]),
            np.array(
                [
                    rx_amp * np.sin(2 * np.pi * rx_freq * t),
                    ry_amp * np.cos(2 * np.pi * ry_freq * t),
                    rz_amp * np.sin(2 * np.pi * rz_freq * t),
                ]
            ),
        )
        platform.plot_platform(ax=ax)
        plt.pause(1 / FPS)
        plt.draw()


if __name__ == "__main__":
    main()
