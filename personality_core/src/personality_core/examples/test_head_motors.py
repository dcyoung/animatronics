"""Track a look-at target on a rotating circle with real servos.

Usage
-----
    personality-core-test-head-motors
    python -m personality_core.examples.test_head_motors

The head follows a target that moves in a circle in front of it,
exercising pitch, roll, and yaw together — same motion as the
``personality-core-visualize-look-at`` visualization but on hardware.
"""

from __future__ import annotations

import logging
import signal
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from personality_core.daemon.utils import find_serial_port
from personality_core.kinematics.analytical_kinematics import AnalyticalKinematics
from personality_core.kinematics.stewart_platform_3dof.look_at import look_at_rpy
from personality_core.motor_controller.xl430 import XL430Controller

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
)
log = logging.getLogger(__name__)

_shutdown = False

# -- Circle parameters --------------------------------------------------------
FORWARD_DISTANCE = 300.0  # mm in front of head
CIRCLE_RADIUS = 150.0  # mm
SPEED = 0.25  # revolutions per second
LOOP_HZ = 50  # command update rate


def _on_signal(sig: int, _f: object) -> None:
    global _shutdown
    _shutdown = True


def _fmt(joints: np.ndarray) -> str:
    d = np.rad2deg(joints)
    return f"PR0={d[0]:+7.2f}°  PR1={d[1]:+7.2f}°  yaw={d[2]:+7.2f}°"


def circle_target(t: float) -> np.ndarray:
    """Target point on a circle in the YZ plane, forward along +X."""
    return np.array([
        FORWARD_DISTANCE,
        CIRCLE_RADIUS * np.cos(t),
        CIRCLE_RADIUS * np.sin(t),
    ])


def run() -> None:
    """Track a circling look-at target on the real servos."""
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    kin = AnalyticalKinematics()
    head_pos = np.zeros(3)  # head at origin

    # -- Connect --------------------------------------------------------------
    ports = find_serial_port()
    if not ports:
        log.error("No serial port found.")
        sys.exit(1)
    port = ports[0]
    log.info("Auto-detected serial port: %s", port)

    ctrl = XL430Controller(serial_port=port)
    log.info("Motor map: %s", ctrl.get_motor_name_id())
    for s in ctrl.servo_configs:
        log.info(
            "  %s (id=%d): centre=%d  inverted=%s",
            s.name, s.id, s.step_center, s.inverted,
        )

    pos = ctrl.get_last_position()
    log.info("Current joints: %s", _fmt(np.array(pos.head)))

    # -- Enable torque --------------------------------------------------------
    log.info("Enabling torque...")
    ctrl.enable_torque()

    dt = 1.0 / LOOP_HZ
    t0 = time.time()

    try:
        log.info(
            "Tracking circle (r=%.0f, d=%.0f, %.2f rev/s) — Ctrl-C to stop.\n",
            CIRCLE_RADIUS,
            FORWARD_DISTANCE,
            SPEED,
        )
        while not _shutdown:
            t = time.time() - t0
            angle = 2 * np.pi * SPEED * t
            target = circle_target(angle)

            rpy = look_at_rpy(head_pos, target)
            pose = np.eye(4)
            pose[:3, :3] = R.from_euler("xyz", rpy, degrees=False).as_matrix()
            joints = kin.ik(pose)

            ctrl.set_head_position(joints)

            rpy_deg = np.rad2deg(rpy)
            log.info(
                "target=(%.0f,%.0f,%.0f)  RPY=(%+5.1f°,%+5.1f°,%+5.1f°)  joints: %s",
                *target,
                *rpy_deg,
                _fmt(joints),
            )

            time.sleep(dt)
    finally:
        log.info("\nReturning to rest and disabling torque...")
        rest = kin.ik(np.eye(4))
        ctrl.set_head_position(rest)
        time.sleep(0.5)
        ctrl.disable_torque()
        ctrl.close()
        log.info("Done.")


def cli() -> None:
    """Entry point for ``personality-core-test-head-motors``."""
    run()


if __name__ == "__main__":
    cli()
