#!/usr/bin/env python3
"""Test waypoint distance calculation"""

import json
import sys
sys.path.insert(0, '/home/hiwonder/rover_ws/src/rover_control')  # Adjust path if needed

from waypoint_navigator import WaypointNavigator

# Test with your 4m+2m file
navigator = WaypointNavigator()

print("Testing waypoint distance calculation...")
print("=" * 60)

# Load your file
result = navigator.load_waypoints_from_file('test_rectangle.json')

if result:
    print(f"\n✓ File loaded successfully")
    print(f"  Waypoints: {len(navigator.waypoints)}")
    print(f"  Total distance: {navigator.total_distance:.2f}m")
    
    # Show each waypoint
    print("\nWaypoint details:")
    for i, wp in enumerate(navigator.waypoints):
        print(f"  {i+1}. lat={wp['latitude']:.6f}, lon={wp['longitude']:.6f}")
        if 'x' in wp:
            print(f"     x={wp['x']}, y={wp['y']}")
    
    # Manual calculation
    print("\nManual distance verification:")
    for i in range(1, len(navigator.waypoints)):
        lat1 = navigator.waypoints[i-1]['latitude']
        lon1 = navigator.waypoints[i-1]['longitude']
        lat2 = navigator.waypoints[i]['latitude']
        lon2 = navigator.waypoints[i]['longitude']
        
        dist, bearing = navigator._calculate_distance_and_bearing(lat1, lon1, lat2, lon2)
        print(f"  Segment {i}: {dist:.2f}m at {bearing:.1f}°")
else:
    print("✗ Failed to load file")
