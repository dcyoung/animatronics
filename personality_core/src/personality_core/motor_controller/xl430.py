"""Low-level driver for Dynamixel XL430 servos via ``rustypot``.

Encapsulates all protocol-level communication so that higher-level code
(e.g. the daemon backend) only deals with radians and motor names.

The interface intentionally mirrors the style of reachy_mini's
``ReachyMiniPyControlLoop`` (from the ``reachy_mini_motor_controller``
Rust binding) so that usage in the daemon backend feels familiar::

    ctrl = XL430Controller("/dev/ttyUSB0")
    ctrl.enable_torque()
    ctrl.set_head_position(np.array([1.0, 2.0, 0.5]))
    pos = ctrl.get_last_position()
    print(pos.head)  # [1.0, 2.0, 0.5]
    ctrl.disable_torque()
    ctrl.close()
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Default servo IDs for the Personality Core head (PR0, PR1, yaw).
DEFAULT_SERVO_IDS: list[int] = [0, 1, 2]

#: Human-readable motor names -> servo IDs.
DEFAULT_MOTOR_NAME_ID: dict[str, int] = {"pr0": 0, "pr1": 1, "yaw": 2}

#: Encoder resolution (positions per revolution) for the XL430.
SERVO_RESOLUTION: int = 4096

#: Default baud rate for XL430 communication.
DEFAULT_BAUDRATE: int = 57_600

#: Default serial timeout in seconds.
DEFAULT_TIMEOUT: float = 0.1


# ---------------------------------------------------------------------------
# Unit conversion helpers
# ---------------------------------------------------------------------------


def rad_to_pos(rad: float) -> int:
    """Convert radians to XL430 servo position (0 -- 4095)."""
    return int((rad % (2 * np.pi)) / (2 * np.pi) * SERVO_RESOLUTION)


def pos_to_rad(pos: int) -> float:
    """Convert XL430 servo position (0 -- 4095) to radians."""
    return float(pos) / SERVO_RESOLUTION * (2 * np.pi)


# ---------------------------------------------------------------------------
# Position container  (mirrors ReachyMiniPyControlLoop.get_last_position())
# ---------------------------------------------------------------------------


@dataclass
class HeadPosition:
    """Structured snapshot of head joint positions (radians).

    Analogous to the position object returned by
    ``ReachyMiniPyControlLoop.get_last_position()`` in reachy_mini,
    but simplified for the Personality Core's 3-DOF head.

    Attributes
    ----------
    head:
        Joint angles ``[PR0, PR1, yaw]`` in radians.

    """

    head: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


@dataclass
class XL430Controller:
    """High-level controller for Dynamixel XL430 servos.

    The public API follows the same conventions as reachy_mini's
    ``ReachyMiniPyControlLoop`` so that the daemon backend can use
    either controller in a familiar way.

    Parameters
    ----------
    serial_port:
        OS device path (e.g. ``/dev/ttyUSB0``).
    servo_ids:
        List of Dynamixel servo IDs to manage.
    baudrate:
        Serial baud rate.
    timeout:
        Read timeout in seconds.

    """

    serial_port: str
    servo_ids: list[int] = field(default_factory=lambda: list(DEFAULT_SERVO_IDS))
    baudrate: int = DEFAULT_BAUDRATE
    timeout: float = DEFAULT_TIMEOUT

    def __post_init__(self) -> None:
        """Open the serial connection via rustypot."""
        self.logger = logging.getLogger(__name__)

        from rustypot import Xl430PyController as _Controller

        self._controller = _Controller(
            serial_port=self.serial_port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

        self._torque_enabled: bool = False
        self._last_position = HeadPosition()
        self._stats: dict[str, float | int] = {
            "read_count": 0,
            "write_count": 0,
            "error_count": 0,
            "last_read_time": 0.0,
        }

        self.logger.info(
            "XL430Controller opened on %s @ %d baud (servos %s)",
            self.serial_port,
            self.baudrate,
            self.servo_ids,
        )

    # ------------------------------------------------------------------
    # Position I/O  (mirrors set_stewart_platform_position / get_last_position)
    # ------------------------------------------------------------------

    def get_last_position(self) -> HeadPosition:
        """Read present positions and return a structured :class:`HeadPosition`.

        Analogous to ``ReachyMiniPyControlLoop.get_last_position()``.

        """
        raw = [
            self._controller.read_present_position(sid) for sid in self.servo_ids
        ]
        positions = [pos_to_rad(p) for p in raw]
        self._last_position = HeadPosition(head=positions)
        self._stats["read_count"] = int(self._stats["read_count"]) + 1
        self._stats["last_read_time"] = time.time()
        return self._last_position

    def set_head_position(self, radians: npt.NDArray[np.float64]) -> None:
        """Write goal positions for the head servos (input in radians).

        Analogous to ``ReachyMiniPyControlLoop.set_stewart_platform_position()``.

        """
        positions = [rad_to_pos(float(r)) for r in radians]
        self._controller.sync_write_goal_position(self.servo_ids, positions)
        self._stats["write_count"] = int(self._stats["write_count"]) + 1

    # ------------------------------------------------------------------
    # Torque control  (mirrors enable_torque / disable_torque)
    # ------------------------------------------------------------------

    def enable_torque(self) -> None:
        """Enable torque on **all** managed servos.

        Analogous to ``ReachyMiniPyControlLoop.enable_torque()``.

        """
        for sid in self.servo_ids:
            self._controller.write_torque_enable(sid, True)
        self._torque_enabled = True

    def disable_torque(self) -> None:
        """Disable torque on **all** managed servos.

        Analogous to ``ReachyMiniPyControlLoop.disable_torque()``.

        """
        for sid in self.servo_ids:
            self._controller.write_torque_enable(sid, False)
        self._torque_enabled = False

    def enable_torque_on_ids(self, ids: list[int]) -> None:
        """Enable torque on specific servo IDs.

        Analogous to ``ReachyMiniPyControlLoop.enable_torque_on_ids()``.

        """
        for sid in ids:
            self._controller.write_torque_enable(sid, True)

    def disable_torque_on_ids(self, ids: list[int]) -> None:
        """Disable torque on specific servo IDs.

        Analogous to ``ReachyMiniPyControlLoop.disable_torque_on_ids()``.

        """
        for sid in ids:
            self._controller.write_torque_enable(sid, False)

    def is_torque_enabled(self) -> bool:
        """Return whether torque is currently enabled.

        Analogous to ``ReachyMiniPyControlLoop.is_torque_enabled()``.

        """
        return self._torque_enabled

    # ------------------------------------------------------------------
    # Introspection  (mirrors get_motor_name_id / get_stats)
    # ------------------------------------------------------------------

    def get_motor_name_id(self) -> dict[str, int]:
        """Return a mapping of motor names to servo IDs.

        Analogous to ``ReachyMiniPyControlLoop.get_motor_name_id()``.

        """
        return dict(DEFAULT_MOTOR_NAME_ID)

    def get_stats(self) -> dict[str, float | int]:
        """Return controller statistics.

        Analogous to ``ReachyMiniPyControlLoop.get_stats()``.

        """
        return dict(self._stats)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Release the serial connection.

        Analogous to ``ReachyMiniPyControlLoop.close()``.

        """
        self._controller = None  # type: ignore[assignment]
        self.logger.info("XL430Controller closed.")
