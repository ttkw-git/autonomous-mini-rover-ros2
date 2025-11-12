#!/usr/bin/env python3
"""
GPS Waypoint Navigator
Autonomously navigates through recorded GPS waypoints
"""

import json
import math
import time
from enum import Enum


class NavigationState(Enum):
    """Navigation states"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    REACHED_WAYPOINT = "reached_waypoint"
    COMPLETED = "completed"
    PAUSED = "paused"
    ERROR = "error"


class WaypointNavigator:
    """Handles autonomous waypoint navigation"""
    
    def __init__(self, waypoint_tolerance=2.0, heading_tolerance=10.0):
        """
        Initialize waypoint navigator
        
        Args:
            waypoint_tolerance: Distance in meters to consider waypoint reached
            heading_tolerance: Heading tolerance in degrees
        """
        # Waypoint data
        self.waypoints = []
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.current_waypoint_index = 0
        
        # Navigation parameters
        self.waypoint_tolerance = waypoint_tolerance  # meters
        self.heading_tolerance = heading_tolerance  # degrees
        
        # Control parameters
        self.max_linear_speed = 0.3  # m/s
        self.min_linear_speed = 0.1  # m/s
        self.max_angular_speed = 0.5  # rad/s
        
        # State
        self.state = NavigationState.IDLE
        self.navigation_active = False
        
        # Statistics
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.waypoints_reached = 0
        
    def load_waypoints_from_file(self, filename):
        """
        Load waypoints from JSON file
        
        Args:
            filename: Path to waypoint JSON file
            
        Returns:
            True if loaded successfully, False otherwise
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.waypoints = data['waypoints']
            self.origin_lat = data['origin']['latitude']
            self.origin_lon = data['origin']['longitude']
            self.current_waypoint_index = 0
            self.waypoints_reached = 0
            
            # Calculate total distance
            self.total_distance = self._calculate_total_distance()
            self.distance_traveled = 0.0
            
            print(f"Loaded {len(self.waypoints)} waypoints")
            print(f"Origin: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
            print(f"Total path distance: {self.total_distance:.2f}m")
            
            return True
            
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            self.state = NavigationState.ERROR
            return False
    
    def start_navigation(self):
        """Start autonomous navigation"""
        if len(self.waypoints) == 0:
            print("No waypoints loaded!")
            return False
        
        self.navigation_active = True
        self.current_waypoint_index = 0
        self.waypoints_reached = 0
        self.distance_traveled = 0.0
        self.state = NavigationState.NAVIGATING
        print("Navigation started!")
        return True
    
    def stop_navigation(self):
        """Stop autonomous navigation"""
        self.navigation_active = False
        self.state = NavigationState.IDLE
        print("Navigation stopped")
    
    def pause_navigation(self):
        """Pause navigation"""
        if self.navigation_active:
            self.state = NavigationState.PAUSED
            print("Navigation paused")
    
    def resume_navigation(self):
        """Resume navigation"""
        if self.state == NavigationState.PAUSED:
            self.state = NavigationState.NAVIGATING
            print("Navigation resumed")
    
    def get_current_target(self):
        """
        Get current target waypoint
        
        Returns:
            Dictionary with target waypoint info or None
        """
        if self.current_waypoint_index >= len(self.waypoints):
            return None
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        target_info = {
            'index': self.current_waypoint_index,
            'total': len(self.waypoints),
            'latitude': waypoint['latitude'],
            'longitude': waypoint['longitude']
        }

        if 'x' in waypoint:
            target_info['x'] = waypoint['x']
            target_info['y'] = waypoint['y']
        else:
            target_info['x'] = 0.0
            target_info['y'] = 0.0

        return target_info
    
    def calculate_control_command(self, current_lat, current_lon, current_heading=None):
        """
        Calculate velocity commands to reach current waypoint
        
        Args:
            current_lat: Current GPS latitude
            current_lon: Current GPS longitude
            current_heading: Current heading in degrees (optional, from IMU)
            
        Returns:
            (linear, angular, reached): Tuple of velocities and reached flag
        """
        if not self.navigation_active or self.state == NavigationState.PAUSED:
            return (0.0, 0.0, False)
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.state = NavigationState.COMPLETED
            self.navigation_active = False
            print("Navigation completed! All waypoints reached.")
            return (0.0, 0.0, True)
        
        # Get target waypoint
        target = self.waypoints[self.current_waypoint_index]
        target_lat = target['latitude']
        target_lon = target['longitude']
        
        # Calculate distance and bearing to target
        distance, bearing = self._calculate_distance_and_bearing(
            current_lat, current_lon, target_lat, target_lon
        )
        
        # Check if waypoint reached
        if distance < self.waypoint_tolerance:
            self.waypoints_reached += 1
            self.current_waypoint_index += 1
            self.state = NavigationState.REACHED_WAYPOINT
            
            print(f"Waypoint {self.waypoints_reached}/{len(self.waypoints)} reached!")
            print(f"Distance to next: {distance:.2f}m")
            
            # Move to next waypoint
            if self.current_waypoint_index < len(self.waypoints):
                self.state = NavigationState.NAVIGATING
                # Recalculate for next waypoint
                return self.calculate_control_command(current_lat, current_lon, current_heading)
            else:
                return (0.0, 0.0, True)
        
        # Calculate control commands
        linear, angular = self._calculate_velocities(distance, bearing, current_heading)
        
        return (linear, angular, False)
    
    def _calculate_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance and bearing between two GPS coordinates
        
        Args:
            lat1, lon1: Start coordinates
            lat2, lon2: End coordinates
            
        Returns:
            (distance_m, bearing_deg): Distance in meters and bearing in degrees
        """
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula for distance
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        # Earth radius in meters
        earth_radius = 6371000
        distance = earth_radius * c
        
        # Calculate bearing
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = (math.degrees(bearing_rad) + 360) % 360
        
        return distance, bearing_deg
    
    def _calculate_velocities(self, distance, target_bearing, current_heading=None):
        """
        Calculate linear and angular velocities
        
        Args:
            distance: Distance to target (meters)
            target_bearing: Bearing to target (degrees)
            current_heading: Current heading (degrees, optional)
            
        Returns:
            (linear, angular): Velocity commands
        """
        # Linear velocity - slow down as approaching waypoint
        if distance > 5.0:
            linear = self.max_linear_speed
        elif distance > 2.0:
            linear = self.max_linear_speed * 0.7
        else:
            linear = self.min_linear_speed
        
        # Angular velocity - turn towards target
        if current_heading is not None:
            # Calculate heading error
            heading_error = target_bearing - current_heading
            
            # Normalize to -180 to 180
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            # Proportional control for angular velocity
            angular = (heading_error / 180.0) * self.max_angular_speed
            
            # Clamp angular velocity
            angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
            
            # Reduce linear speed when turning sharply
            if abs(heading_error) > 30:
                linear *= 0.5
        else:
            # FIXED: GPS-based navigation without IMU heading
            # Normalize bearing to -180 to 180 range for steering
            if target_bearing > 180:
                normalized_bearing = target_bearing - 360
            else:
                normalized_bearing = target_bearing
            
            # Proportional steering: larger bearing error = sharper turn
            angular = (normalized_bearing / 90.0) * self.max_angular_speed
            
            # Clamp to max angular speed
            angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
            
            # Reduce forward speed when making sharp turns
            if abs(normalized_bearing) > 45:
                linear *= 0.5
        
        return linear, angular
    
    def _calculate_total_distance(self):
        """Calculate total distance of waypoint path"""
        if len(self.waypoints) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(self.waypoints)):
            lat1 = self.waypoints[i-1]['latitude']
            lon1 = self.waypoints[i-1]['longitude']
            lat2 = self.waypoints[i]['latitude']
            lon2 = self.waypoints[i]['longitude']

            distance, _ = self._calculate_distance_and_bearing(lat1, lon1, lat2, lon2)
            total += distance

        return total

    def calculate_total_distance(self):
        """Return the total path length for the currently loaded waypoints."""
        return self._calculate_total_distance()
    
    def get_navigation_status(self):
        """
        Get current navigation status
        
        Returns:
            Dictionary with navigation status info
        """
        progress_pct = 0.0
        if len(self.waypoints) > 0:
            progress_pct = (self.waypoints_reached / len(self.waypoints)) * 100
        
        return {
            'state': self.state.value,
            'active': self.navigation_active,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'waypoints_reached': self.waypoints_reached,
            'progress_percent': progress_pct,
            'total_distance': self.total_distance,
            'distance_traveled': self.distance_traveled
        }


# Example usage
if __name__ == '__main__':
    # Create navigator
    navigator = WaypointNavigator(waypoint_tolerance=2.0)
    
    # Load waypoints from file
    if navigator.load_waypoints_from_file('recorded_path.json'):
        # Start navigation
        navigator.start_navigation()
        
        # Simulate navigation loop
        # In real use, this would be called from your main control loop
        current_lat = 49.891364
        current_lon = -97.153644
        current_heading = 45.0  # degrees
        
        while navigator.navigation_active:
            linear, angular, reached = navigator.calculate_control_command(
                current_lat, current_lon, current_heading
            )
            
            print(f"Command: linear={linear:.2f}, angular={angular:.2f}")
            
            # In real system: send these commands to rover
            # ros_node.send_cmd_vel(linear, angular)
            
            # Get status
            status = navigator.get_navigation_status()
            print(f"Progress: {status['waypoints_reached']}/{status['total_waypoints']}")
            
            time.sleep(0.1)
