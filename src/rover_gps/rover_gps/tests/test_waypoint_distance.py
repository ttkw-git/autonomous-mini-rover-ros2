"""Unit tests for waypoint distance calculations."""

from importlib import resources

import pytest

from rover_gps.rover_control.waypoint_navigator import WaypointNavigator


def load_data_file(filename: str) -> str:
    """Return the filesystem path for a packaged waypoint file."""
    with resources.as_file(
        resources.files("rover_gps.data.waypoints") / filename
    ) as file_path:
        return str(file_path)


def test_total_distance_matches_expected_rectangle():
    navigator = WaypointNavigator()
    waypoint_file = load_data_file("test_rectangle.json")

    assert navigator.load_waypoints_from_file(waypoint_file)
    assert len(navigator.waypoints) == 3
    assert navigator.total_distance == pytest.approx(
        navigator.calculate_total_distance(), rel=1e-6
    )
