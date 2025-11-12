"""Integration-style tests that exercise waypoint navigation state transitions."""

from importlib import resources

from rover_gps.rover_control.waypoint_navigator import NavigationState, WaypointNavigator


def load_data_file(filename: str) -> str:
    with resources.as_file(
        resources.files("rover_gps.data.waypoints") / filename
    ) as file_path:
        return str(file_path)


def test_indoor_navigation_completes_with_simulated_rover():
    navigator = WaypointNavigator(waypoint_tolerance=1.0)
    waypoint_file = load_data_file("test_rectangle.json")

    assert navigator.load_waypoints_from_file(waypoint_file)
    assert navigator.start_navigation()

    for expected_count, target in enumerate(navigator.waypoints, start=1):
        for _ in range(3):
            navigator.calculate_control_command(
                target["latitude"], target["longitude"], 0.0
            )
            if navigator.waypoints_reached >= expected_count:
                break
        assert (
            navigator.waypoints_reached >= expected_count
        ), f"Failed to mark waypoint {target['id']} as reached"

    status = navigator.get_navigation_status()
    assert status["waypoints_reached"] == status["total_waypoints"]
    assert status["state"] in {
        NavigationState.COMPLETED.value,
        NavigationState.REACHED_WAYPOINT.value,
    }
