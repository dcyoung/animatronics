"""FastAPI dependency injection for the Personality Core daemon."""

from fastapi import HTTPException, Request, WebSocket

from reachy_mini.daemon.backend.abstract import Backend

from personality_core.daemon.daemon import Daemon


def get_daemon(request: Request) -> Daemon:
    """Get the daemon instance from the app state."""
    daemon = request.app.state.daemon
    assert isinstance(daemon, Daemon)
    return daemon


def get_backend(request: Request) -> Backend:
    """Get the running backend instance from the app state."""
    backend = request.app.state.daemon.backend

    if backend is None or not backend.ready.is_set():
        raise HTTPException(status_code=503, detail="Backend not running")

    assert isinstance(backend, Backend)
    return backend


def ws_get_backend(websocket: WebSocket) -> Backend:
    """Get the running backend — WebSocket variant."""
    backend = websocket.app.state.daemon.backend

    if backend is None or not backend.ready.is_set():
        raise HTTPException(status_code=503, detail="Backend not running")

    assert isinstance(backend, Backend)
    return backend
