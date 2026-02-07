"""Daemon entry point for the Personality Core robot.

Provides a CLI and a FastAPI server (port 8000 by default) that manages
the daemon lifecycle and exposes a REST + WebSocket API for robot control.

The API routes intentionally mirror reachy_mini's structure so that
clients written for one robot can easily adapt to the other.
"""

import argparse
import asyncio
import logging
import types
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, AsyncGenerator

import numpy as np
import uvicorn
from fastapi import APIRouter, Depends, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from reachy_mini.daemon.backend.abstract import Backend, MotorControlMode

from personality_core.daemon.app.dependencies import get_backend, get_daemon
from personality_core.daemon.daemon import Daemon, DaemonStatus


# ======================================================================
# CLI arguments
# ======================================================================


@dataclass
class Args:
    """Arguments for configuring the Personality Core daemon."""

    log_level: str = "INFO"
    log_file: str | None = None

    serialport: str = "auto"

    sim: bool = False
    mockup_sim: bool = False
    scene: str = "empty"
    headless: bool = False
    use_audio: bool = False

    autostart: bool = True

    wake_up_on_start: bool = True
    goto_sleep_on_stop: bool = True

    robot_name: str = "personality_core"

    fastapi_host: str = "0.0.0.0"
    fastapi_port: int = 8000

    localhost_only: bool = True


# ======================================================================
# FastAPI application factory
# ======================================================================


def create_app(args: Args) -> FastAPI:
    """Create and configure the FastAPI application."""

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
        """Start the daemon on app startup; stop on shutdown."""
        args: Args = app.state.args

        try:
            if args.autostart:
                await app.state.daemon.start(
                    serialport=args.serialport,
                    sim=args.sim,
                    mockup_sim=args.mockup_sim,
                    scene=args.scene,
                    headless=args.headless,
                    use_audio=args.use_audio,
                    wake_up_on_start=args.wake_up_on_start,
                    localhost_only=args.localhost_only,
                )
            yield
        finally:
            try:
                logging.info("Shutting down daemon…")
                await app.state.daemon.stop(
                    goto_sleep_on_stop=args.goto_sleep_on_stop,
                )
            except Exception as e:
                logging.exception(f"Error stopping daemon: {e}")

    app = FastAPI(lifespan=lifespan)

    app.state.args = args
    app.state.daemon = Daemon(
        robot_name=args.robot_name,
        log_level=args.log_level,
    )

    # -- CORS ----------------------------------------------------------
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # -- Routes --------------------------------------------------------
    router = APIRouter(prefix="/api")

    # ── Daemon lifecycle ──────────────────────────────────────────────

    @router.get("/daemon/status")
    async def daemon_status(
        daemon: Daemon = Depends(get_daemon),
    ) -> DaemonStatus:
        """Get the current daemon status."""
        return daemon.status()

    @router.post("/daemon/start")
    async def daemon_start(
        request: Request,
        wake_up: bool = True,
        daemon: Daemon = Depends(get_daemon),
    ) -> dict[str, str]:
        """Start the daemon."""
        await daemon.start(
            sim=request.app.state.args.sim,
            mockup_sim=request.app.state.args.mockup_sim,
            serialport=request.app.state.args.serialport,
            scene=request.app.state.args.scene,
            headless=request.app.state.args.headless,
            use_audio=request.app.state.args.use_audio,
            wake_up_on_start=wake_up,
            localhost_only=request.app.state.args.localhost_only,
        )
        return {"status": "started"}

    @router.post("/daemon/stop")
    async def daemon_stop(
        goto_sleep: bool = True,
        daemon: Daemon = Depends(get_daemon),
    ) -> dict[str, str]:
        """Stop the daemon."""
        await daemon.stop(goto_sleep_on_stop=goto_sleep)
        return {"status": "stopped"}

    @router.post("/daemon/restart")
    async def daemon_restart(
        daemon: Daemon = Depends(get_daemon),
    ) -> dict[str, str]:
        """Restart the daemon."""
        await daemon.restart()
        return {"status": "restarted"}

    # ── State queries ─────────────────────────────────────────────────

    @router.get("/state/head_pose")
    async def get_head_pose(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, Any]:
        """Get the current head pose as a flattened 4x4 matrix."""
        pose = backend.get_present_head_pose()
        return {"head_pose": pose.tolist()}

    @router.get("/state/head_joint_positions")
    async def get_head_joints(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, Any]:
        """Get the current head joint positions [PR0, PR1, yaw] in radians."""
        joints = backend.get_present_head_joint_positions()
        return {"head_joint_positions": joints.tolist()}

    @router.get("/state/full")
    async def get_full_state(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, Any]:
        """Get the full robot state."""
        return {
            "head_pose": backend.get_present_head_pose().tolist(),
            "head_joint_positions": backend.get_present_head_joint_positions().tolist(),
            "motor_control_mode": backend.get_motor_control_mode().value,
        }

    # ── Motor control ─────────────────────────────────────────────────

    @router.post("/motors/enable")
    async def enable_motors(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Enable motor torque."""
        backend.set_motor_control_mode(MotorControlMode.Enabled)
        return {"motor_control_mode": MotorControlMode.Enabled.value}

    @router.post("/motors/disable")
    async def disable_motors(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Disable motor torque."""
        backend.set_motor_control_mode(MotorControlMode.Disabled)
        return {"motor_control_mode": MotorControlMode.Disabled.value}

    @router.get("/motors/mode")
    async def get_motor_mode(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Get the current motor control mode."""
        return {"motor_control_mode": backend.get_motor_control_mode().value}

    # ── Commands ──────────────────────────────────────────────────────

    @router.post("/command/set_target")
    async def set_target(
        request: Request,
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Set the target head pose (4x4 matrix, flattened)."""
        body = await request.json()
        if "head_pose" in body:
            pose = np.array(body["head_pose"]).reshape(4, 4)
            backend.set_target_head_pose(pose)
        if "head_joint_positions" in body:
            joints = np.array(body["head_joint_positions"])
            backend.set_target_head_joint_positions(joints)
        return {"status": "ok"}

    @router.post("/command/goto_target")
    async def goto_target(
        request: Request,
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Smoothly go to a target head pose over a duration."""
        body = await request.json()
        head = (
            np.array(body["head_pose"]).reshape(4, 4)
            if "head_pose" in body
            else None
        )
        duration = float(body.get("duration", 0.5))
        await backend.goto_target(head=head, duration=duration)
        return {"status": "ok"}

    @router.post("/command/wake_up")
    async def wake_up(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Wake up the robot."""
        backend.set_motor_control_mode(MotorControlMode.Enabled)
        await backend.wake_up()
        return {"status": "ok"}

    @router.post("/command/goto_sleep")
    async def goto_sleep(
        backend: Backend = Depends(get_backend),
    ) -> dict[str, str]:
        """Put the robot to sleep."""
        await backend.goto_sleep()
        backend.set_motor_control_mode(MotorControlMode.Disabled)
        return {"status": "ok"}

    app.include_router(router)
    return app


# ======================================================================
# Server runner
# ======================================================================


def run_app(args: Args) -> None:
    """Run the FastAPI app with Uvicorn."""
    import sys

    root_logger = logging.getLogger()
    root_logger.setLevel(args.log_level)

    handler = logging.StreamHandler(sys.stderr)
    handler.setLevel(args.log_level)
    handler.setFormatter(
        logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    )
    root_logger.addHandler(handler)

    # Install exception hook
    def exception_hook(
        exc_type: type[BaseException],
        exc_value: BaseException,
        exc_traceback: types.TracebackType | None,
    ) -> None:
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return
        root_logger.critical(
            "Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback)
        )
        sys.stderr.flush()

    sys.excepthook = exception_hook

    async def _run_server() -> None:
        app = create_app(args)
        config = uvicorn.Config(
            app,
            host=args.fastapi_host,
            port=args.fastapi_port,
            log_config=None,
        )
        server = uvicorn.Server(config)
        try:
            await server.serve()
        except KeyboardInterrupt:
            logging.info("Ctrl-C received, shutting down.")

    try:
        asyncio.run(_run_server())
    except KeyboardInterrupt:
        logging.info("Shutdown complete.")


# ======================================================================
# CLI entry point
# ======================================================================


def main() -> None:
    """CLI entry point for ``personality-core-daemon``."""
    default = Args()
    parser = argparse.ArgumentParser(
        description="Run the Personality Core daemon."
    )

    parser.add_argument(
        "--robot-name",
        type=str,
        default=default.robot_name,
        help="Zenoh topic prefix / robot name.",
    )

    # -- Hardware -------------------------------------------------------
    parser.add_argument(
        "-p",
        "--serialport",
        type=str,
        default=default.serialport,
        help="Serial port for servos (default: auto-detect).",
    )

    # -- Simulation -----------------------------------------------------
    parser.add_argument(
        "--sim",
        action="store_true",
        default=default.sim,
        help="Run in MuJoCo simulation mode.",
    )
    parser.add_argument(
        "--mockup-sim",
        action="store_true",
        default=default.mockup_sim,
        help="Run in lightweight mockup simulation mode.",
    )
    parser.add_argument(
        "--scene",
        type=str,
        default=default.scene,
        help="MuJoCo scene name.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=default.headless,
        help="Run MuJoCo without GUI.",
    )
    parser.add_argument(
        "--deactivate-audio",
        action="store_false",
        dest="use_audio",
        default=default.use_audio,
        help="Deactivate audio subsystem.",
    )

    # -- Daemon behaviour -----------------------------------------------
    parser.add_argument(
        "--autostart",
        action="store_true",
        default=default.autostart,
        help="Auto-start daemon on launch.",
    )
    parser.add_argument(
        "--no-autostart",
        action="store_false",
        dest="autostart",
    )
    parser.add_argument(
        "--wake-up-on-start",
        action="store_true",
        default=default.wake_up_on_start,
    )
    parser.add_argument(
        "--no-wake-up-on-start",
        action="store_false",
        dest="wake_up_on_start",
    )
    parser.add_argument(
        "--goto-sleep-on-stop",
        action="store_true",
        default=default.goto_sleep_on_stop,
    )
    parser.add_argument(
        "--no-goto-sleep-on-stop",
        action="store_false",
        dest="goto_sleep_on_stop",
    )

    # -- Zenoh ----------------------------------------------------------
    parser.add_argument(
        "--localhost-only",
        action="store_true",
        default=default.localhost_only,
        help="Restrict Zenoh to localhost.",
    )
    parser.add_argument(
        "--no-localhost-only",
        action="store_false",
        dest="localhost_only",
    )

    # -- FastAPI --------------------------------------------------------
    parser.add_argument(
        "--fastapi-host",
        type=str,
        default=default.fastapi_host,
    )
    parser.add_argument(
        "--fastapi-port",
        type=int,
        default=default.fastapi_port,
    )

    # -- Logging --------------------------------------------------------
    parser.add_argument(
        "--log-level",
        type=str,
        default=default.log_level,
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=default.log_file,
    )

    parsed = parser.parse_args()

    if parsed.log_file:
        fh = logging.FileHandler(parsed.log_file, mode="a")
        fh.setFormatter(
            logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        )
        logging.getLogger().addHandler(fh)
        logging.getLogger().setLevel(parsed.log_level)

    run_app(Args(**vars(parsed)))


if __name__ == "__main__":
    main()
