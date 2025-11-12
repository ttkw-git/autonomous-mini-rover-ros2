"""Pytest configuration for locating the rover_gps source package."""

import sys
from pathlib import Path

PACKAGE_SRC = Path(__file__).resolve().parents[2]
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
