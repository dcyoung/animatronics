"""Low-level driver for Dynamixel XL430 servos via ``rustypot``.

Encapsulates all protocol-level communication so that higher-level code
(e.g. the daemon backend) only deals with radians and motor names.

Kinematics angles are **offsets from each servo's centre position**.
The centre step is the encoder value that corresponds to a kinematic
angle of 0 rad.  Reads and writes are transparently converted::

    step = centre + int(offset_rad / 2π × 4096)
    offset_rad = (step − centre) / 4096 × 2π

The interface intentionally mirrors the style of reachy_mini's
``ReachyMiniPyControlLoop`` (from the ``reachy_mini_motor_controller``
Rust binding) so that usage in the daemon backend feels familiar::

    ctrl = XL430Controller("/dev/ttyUSB0")
    ctrl.enable_torque()
    ctrl.set_head_position(np.array([0.1, -0.05, 0.3]))
    pos = ctrl.get_last_position()
    print(pos.head)  # [0.1, -0.05, 0.3]  (radians, offset from centre)
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

#: Encoder resolution (positions per revolution) for the XL430.
SERVO_RESOLUTION: int = 4096

#: Default baud rate for XL430 communication.
DEFAULT_BAUDRATE: int = 57_600

#: Default serial timeout in seconds.
DEFAULT_TIMEOUT: float = 0.1


# ---------------------------------------------------------------------------
# Servo configuration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ServoConfig:
    """Physical configuration for a single Dynamixel XL430 servo.

    Attributes
    ----------
    id:
        Dynamixel bus ID.
    name:
        Human-readable name (``"pr0"``, ``"pr1"``, ``"yaw"``).
    step_center:
        Encoder step that corresponds to kinematic angle 0 rad.
    step_limits:
        ``(min_step, max_step)`` firmware position limits.
    inverted:
        If ``True``, negate the offset when converting between radians
        and encoder steps.  Use this when the servo is physically
        mounted so that a positive kinematic offset corresponds to a
        decreasing encoder step (or vice-versa).

    """

    id: int
    name: str
    step_center: int
    step_limits: tuple[int, int]
    inverted: bool = False

    @property
    def _sign(self) -> int:
        return -1 if self.inverted else 1

    @property
    def rad_center(self) -> float:
        """Centre position in absolute radians (for reference only)."""
        return float(self.step_center) / SERVO_RESOLUTION * (2 * np.pi)

    @property
    def rad_limits(self) -> tuple[float, float]:
        """Position limits as offsets from centre, in radians."""
        return (
            self._step_to_offset_rad(self.step_limits[0]),
            self._step_to_offset_rad(self.step_limits[1]),
        )

    # -- conversion helpers (offset from centre) ----------------------------

    def offset_rad_to_step(self, rad: float) -> int:
        """Convert a kinematic offset (radians from centre) to an encoder step."""
        return self.step_center + int(self._sign * rad / (2 * np.pi) * SERVO_RESOLUTION)

    def step_to_offset_rad(self, step: int) -> float:
        """Convert an encoder step to a kinematic offset (radians from centre)."""
        return self._step_to_offset_rad(step)

    def _step_to_offset_rad(self, step: int) -> float:
        return self._sign * float(step - self.step_center) / SERVO_RESOLUTION * (2 * np.pi)


# -- Default servo configs (ordered to match kinematics: PR0, PR1, yaw) -----
#
# Physical wiring (facing forward):
#   Servo ID 1 → PR0 (left side,  +Y in kinematics), centre @ step 1024
SERVO_PR1 = ServoConfig(id=0, name="pr1", step_center=3072, step_limits=(2560, 3584))
#   Servo ID 0 → PR1 (right side, −Y in kinematics), centre @ step 3072
SERVO_PR0 = ServoConfig(id=1, name="pr0", step_center=1024, step_limits=(512, 1536), inverted=True)
#   Servo ID 2 → yaw,                                 centre @ step 2048
SERVO_YAW = ServoConfig(id=2, name="yaw", step_center=2048, step_limits=(1536, 2560))


#: Default servo configs, ordered ``[PR0, PR1, yaw]`` to match kinematics output.
DEFAULT_SERVO_CONFIGS: list[ServoConfig] = [SERVO_PR0, SERVO_PR1, SERVO_YAW]


# ---------------------------------------------------------------------------
# Position container  (mirrors ReachyMiniPyControlLoop.get_last_position())
# ---------------------------------------------------------------------------


@dataclass
class HeadPosition:
    """Structured snapshot of head joint positions (radians, offset from centre).

    Analogous to the position object returned by
    ``ReachyMiniPyControlLoop.get_last_position()`` in reachy_mini,
    but simplified for the Personality Core's 3-DOF head.

    Attributes
    ----------
    head:
        Joint angles ``[PR0, PR1, yaw]`` in radians (offset from centre).

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
    servo_configs:
        Per-servo configuration (IDs, centres, limits).
    baudrate:
        Serial baud rate.
    timeout:
        Read timeout in seconds.

    """

    serial_port: str
    servo_configs: list[ServoConfig] = field(
        default_factory=lambda: list(DEFAULT_SERVO_CONFIGS),
    )
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

        self._servo_ids = [s.id for s in self.servo_configs]
        self._torque_enabled: bool = False
        self._last_position = HeadPosition()
        self._stats: dict[str, float | int] = {
            "read_count": 0,
            "write_count": 0,
            "error_count": 0,
            "last_read_time": 0.0,
        }

        self.logger.info(
            "XL430Controller opened on %s @ %d baud  servos: %s",
            self.serial_port,
            self.baudrate,
            [(s.name, s.id, f"centre={s.step_center}") for s in self.servo_configs],
        )

    # ------------------------------------------------------------------
    # Position I/O  (mirrors set_stewart_platform_position / get_last_position)
    # ------------------------------------------------------------------

    def get_last_position(self) -> HeadPosition:
        """Read present positions and return a structured :class:`HeadPosition`.

        Returned angles are **offsets from centre** in radians.

        Analogous to ``ReachyMiniPyControlLoop.get_last_position()``.

        """
        raw = [self._controller.read_present_position(s.id) for s in self.servo_configs]
        # rustypot may return int or single-element list — normalise to int
        raw = [r[0] if isinstance(r, (list, tuple)) else r for r in raw]

        positions = [
            s.step_to_offset_rad(int(p)) for s, p in zip(self.servo_configs, raw)
        ]
        self._last_position = HeadPosition(head=positions)
        self._stats["read_count"] = int(self._stats["read_count"]) + 1
        self._stats["last_read_time"] = time.time()
        return self._last_position

    def set_head_position(self, radians: npt.NDArray[np.float64]) -> None:
        """Write goal positions for the head servos.

        *radians* are **offsets from centre** — the controller converts
        them to absolute encoder steps using each servo's ``step_center``.

        Analogous to ``ReachyMiniPyControlLoop.set_stewart_platform_position()``.

        """
        steps = [
            s.offset_rad_to_step(float(r)) for s, r in zip(self.servo_configs, radians)
        ]
        self._controller.sync_write_goal_position(self._servo_ids, steps)
        self._stats["write_count"] = int(self._stats["write_count"]) + 1

    # ------------------------------------------------------------------
    # Torque control  (mirrors enable_torque / disable_torque)
    # ------------------------------------------------------------------

    def enable_torque(self) -> None:
        """Enable torque on **all** managed servos.

        Analogous to ``ReachyMiniPyControlLoop.enable_torque()``.

        """
        for sid in self._servo_ids:
            self._controller.write_torque_enable(sid, True)
        self._torque_enabled = True

    def disable_torque(self) -> None:
        """Disable torque on **all** managed servos.

        Analogous to ``ReachyMiniPyControlLoop.disable_torque()``.

        """
        for sid in self._servo_ids:
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
        return {s.name: s.id for s in self.servo_configs}

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
