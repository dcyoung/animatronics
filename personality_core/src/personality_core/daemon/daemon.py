"""Daemon for the Personality Core robot.

Orchestrates backend lifecycle, Zenoh server, and status publishing.
Follows the same structure as the reachy_mini Daemon but is simplified:
  - No wireless version / WebRTC
  - No motor reflashing
  - 3-DOF head (2 PR servos + bevel yaw)
"""

import asyncio
import json
import logging
import time
from dataclasses import asdict, dataclass
from enum import Enum
from importlib.metadata import PackageNotFoundError, version
from threading import Event, Thread
from typing import Any, Optional

from reachy_mini.daemon.backend.abstract import MotorControlMode
from reachy_mini.io import ZenohServer

from personality_core.daemon.backend.mockup_sim.backend import (
    MockupSimBackend,
    MockupSimBackendStatus,
)
from personality_core.daemon.backend.mujoco.backend import (
    MujocoBackend,
    MujocoBackendStatus,
)
from personality_core.daemon.backend.robot.backend import (
    RobotBackend,
    RobotBackendStatus,
)
from personality_core.daemon.utils import convert_enum_to_dict, find_serial_port


class Daemon:
    """Daemon for the Personality Core robot.

    Manages the backend (real hardware, MuJoCo, or mockup sim), the Zenoh
    communication server, and the status-publishing thread.
    """

    def __init__(
        self,
        log_level: str = "INFO",
        robot_name: str = "personality_core",
    ) -> None:
        """Initialize the daemon."""
        self.log_level = log_level
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(self.log_level)

        self.robot_name = robot_name
        self.backend: RobotBackend | MujocoBackend | MockupSimBackend | None = None

        # Package version
        try:
            package_version = version("personality_core")
            self.logger.info(f"Daemon version: {package_version}")
        except PackageNotFoundError:
            package_version = None

        self._status = DaemonStatus(
            robot_name=robot_name,
            state=DaemonState.NOT_INITIALIZED,
            simulation_enabled=None,
            mockup_sim_enabled=None,
            backend_status=None,
            error=None,
            version=package_version,
        )
        self._thread_event_publish_status = Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(
        self,
        sim: bool = False,
        mockup_sim: bool = False,
        serialport: str = "auto",
        scene: str = "empty",
        localhost_only: bool = True,
        wake_up_on_start: bool = True,
        headless: bool = False,
        use_audio: bool = False,
    ) -> "DaemonState":
        """Start the daemon.

        Args:
            sim: Run in MuJoCo simulation mode.
            mockup_sim: Run in lightweight simulation mode (no MuJoCo).
            serialport: Serial port for real servos (``"auto"`` to detect).
            scene: MuJoCo scene name.
            localhost_only: Restrict Zenoh server to localhost.
            wake_up_on_start: Move head to init pose on start.
            headless: Run MuJoCo without GUI.
            use_audio: Initialise audio subsystem.

        Returns:
            The daemon state after the start attempt.

        """
        if self._status.state == DaemonState.RUNNING:
            self.logger.warning("Daemon is already running.")
            return self._status.state

        self.logger.info(
            f"Daemon start: sim={sim}, mockup_sim={mockup_sim}, "
            f"serialport={serialport}, localhost_only={localhost_only}"
        )

        self._status.simulation_enabled = sim
        self._status.mockup_sim_enabled = mockup_sim

        self._start_params: dict[str, Any] = {
            "sim": sim,
            "mockup_sim": mockup_sim,
            "serialport": serialport,
            "scene": scene,
            "headless": headless,
            "use_audio": use_audio,
            "localhost_only": localhost_only,
        }

        self.logger.info("Starting Personality Core daemon…")
        self._status.state = DaemonState.STARTING

        # -- Create backend ------------------------------------------------
        try:
            self.backend = self._setup_backend(
                sim=sim,
                mockup_sim=mockup_sim,
                serialport=serialport,
                scene=scene,
                headless=headless,
                use_audio=use_audio,
            )
        except Exception as e:
            self._status.state = DaemonState.ERROR
            self._status.error = str(e)
            raise

        # -- Start Zenoh server --------------------------------------------
        self.zenoh_server = ZenohServer(
            prefix=self.robot_name,
            backend=self.backend,
            localhost_only=localhost_only,
        )
        self.zenoh_server.start()

        # -- Status publisher thread ---------------------------------------
        self._thread_publish_status = Thread(
            target=self._publish_status, daemon=True
        )
        self._thread_publish_status.start()

        # -- Backend run thread --------------------------------------------
        def _backend_run() -> None:
            assert self.backend is not None
            try:
                self.backend.wrapped_run()
            except Exception as e:
                self.logger.error(f"Backend error: {e}")
                self._status.state = DaemonState.ERROR
                self._status.error = str(e)
                self.zenoh_server.stop()
                self.backend = None

        self.backend_run_thread = Thread(target=_backend_run)
        self.backend_run_thread.start()

        if not self.backend.ready.wait(timeout=3.0):
            self.logger.error("Backend not ready after 3 s.")
            self._status.state = DaemonState.ERROR
            self._status.error = self.backend.error
            return self._status.state

        # -- Wake up -------------------------------------------------------
        if wake_up_on_start:
            try:
                self.logger.info("Waking up…")
                self.backend.set_motor_control_mode(MotorControlMode.Enabled)
                await self.backend.wake_up()
            except Exception as e:
                self.logger.error(f"Error during wake-up: {e}")
                self._status.state = DaemonState.ERROR
                self._status.error = str(e)
                return self._status.state

        self.logger.info("Daemon started successfully.")
        self._status.state = DaemonState.RUNNING
        return self._status.state

    async def stop(self, goto_sleep_on_stop: bool = True) -> "DaemonState":
        """Stop the daemon."""
        if self._status.state == DaemonState.STOPPED:
            self.logger.warning("Daemon is already stopped.")
            return self._status.state

        if self.backend is None:
            self._status.state = DaemonState.STOPPED
            return self._status.state

        try:
            if self._status.state in (DaemonState.STOPPING, DaemonState.ERROR):
                goto_sleep_on_stop = False

            self.logger.info("Stopping daemon…")
            self._status.state = DaemonState.STOPPING
            self.backend.is_shutting_down = True
            self._thread_event_publish_status.set()

            if goto_sleep_on_stop:
                try:
                    self.backend.set_motor_control_mode(MotorControlMode.Enabled)
                    await self.backend.goto_sleep()
                    self.backend.set_motor_control_mode(MotorControlMode.Disabled)
                except Exception as e:
                    self.logger.error(f"Error during sleep: {e}")

            self.backend.should_stop.set()
            self.backend_run_thread.join(timeout=5.0)
            if self.backend_run_thread.is_alive():
                self.logger.warning("Backend did not stop in time.")
                self._status.state = DaemonState.ERROR

            self.backend.close()
            self.backend.ready.clear()
            self.zenoh_server.stop()

            if self._status.state != DaemonState.ERROR:
                self.logger.info("Daemon stopped successfully.")
                self._status.state = DaemonState.STOPPED
        except Exception as e:
            self.logger.error(f"Error while stopping: {e}")
            self._status.state = DaemonState.ERROR
            self._status.error = str(e)

        if self.backend is not None:
            backend_status = self.backend.get_status()
            if backend_status.error:
                self._status.state = DaemonState.ERROR
            self.backend = None

        return self._status.state

    async def restart(self, **kwargs: Any) -> "DaemonState":
        """Restart the daemon, optionally overriding start parameters."""
        await self.stop(goto_sleep_on_stop=kwargs.pop("goto_sleep_on_stop", False))
        params = {**self._start_params, **kwargs}
        params.setdefault("wake_up_on_start", False)
        return await self.start(**params)

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def status(self) -> "DaemonStatus":
        """Get the current daemon status."""
        if self.backend is not None:
            self._status.backend_status = self.backend.get_status()
            if self._status.backend_status.error:
                self._status.state = DaemonState.ERROR
            self._status.error = self._status.backend_status.error
        else:
            self._status.backend_status = None
        return self._status

    def _publish_status(self) -> None:
        self._thread_event_publish_status.clear()
        while not self._thread_event_publish_status.is_set():
            json_str = json.dumps(
                asdict(self.status(), dict_factory=convert_enum_to_dict)
            )
            self.zenoh_server.pub_status.put(json_str)
            time.sleep(1)

    # ------------------------------------------------------------------
    # Backend factory
    # ------------------------------------------------------------------

    def _setup_backend(
        self,
        sim: bool,
        mockup_sim: bool,
        serialport: str,
        scene: str,
        headless: bool,
        use_audio: bool,
    ) -> RobotBackend | MujocoBackend | MockupSimBackend:
        if mockup_sim:
            return MockupSimBackend(use_audio=use_audio)
        elif sim:
            return MujocoBackend(
                scene=scene,
                headless=headless,
                use_audio=use_audio,
            )
        else:
            if serialport == "auto":
                ports = find_serial_port()
                if len(ports) == 0:
                    raise RuntimeError(
                        "No Personality Core serial port found. "
                        "Check USB connection and permissions, "
                        "or specify --serialport explicitly."
                    )
                if len(ports) > 1:
                    raise RuntimeError(
                        f"Multiple serial ports found: {ports}. "
                        "Please specify --serialport explicitly."
                    )
                serialport = ports[0]
                self.logger.info(f"Found serial port: {serialport}")

            return RobotBackend(
                serialport=serialport,
                log_level=self.log_level,
                use_audio=use_audio,
            )


# ======================================================================
# Status data types
# ======================================================================


class DaemonState(Enum):
    """State of the daemon."""

    NOT_INITIALIZED = "not_initialized"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class DaemonStatus:
    """Status of the daemon."""

    robot_name: str
    state: DaemonState
    simulation_enabled: Optional[bool]
    mockup_sim_enabled: Optional[bool]
    backend_status: Optional[
        RobotBackendStatus | MujocoBackendStatus | MockupSimBackendStatus
    ]
    error: Optional[str] = None
    version: Optional[str] = None
