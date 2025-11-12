"""Utility helpers for locating waypoint storage directories."""

from __future__ import annotations

import os
from pathlib import Path


def _ensure_directory(path: Path) -> Path:
    """Create *path* if needed and return it."""

    path.mkdir(parents=True, exist_ok=True)
    return path


def waypoint_storage_dir() -> Path:
    """Return a writable directory for waypoint artifacts.

    Preference order:

    1. ``ROVER_GPS_WAYPOINT_DIR`` environment variable if set.
    2. The package's ``data/waypoints`` directory (useful in a checkout).
    3. ``~/.local/share/rover_gps/waypoints`` as a final fallback.
    """

    override = os.getenv("ROVER_GPS_WAYPOINT_DIR")
    if override:
        return _ensure_directory(Path(override).expanduser())

    package_dir = Path(__file__).resolve().parents[2] / "data" / "waypoints"
    try:
        return _ensure_directory(package_dir)
    except OSError:
        fallback = Path.home() / ".local" / "share" / "rover_gps" / "waypoints"
        return _ensure_directory(fallback)


def resolve_waypoint_path(filename: str | os.PathLike[str]) -> Path:
    """Resolve *filename* into the waypoint storage directory.

    Absolute paths and paths containing a drive/root element are returned as-is
    (after ``expanduser``). All other paths are treated as relative to the
    waypoint storage directory.
    """

    path = Path(filename).expanduser()
    if path.is_absolute():
        return path
    if path.anchor:  # Handles Windows drive-letter-only paths.
        return path
    return waypoint_storage_dir() / path

