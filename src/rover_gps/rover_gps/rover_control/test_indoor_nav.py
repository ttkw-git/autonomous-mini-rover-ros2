#!/usr/bin/env python3
"""
Indoor Navigation Test
Tests waypoint navigation logic without GPS or real robot movement
"""

import sys
import json
import time
import math

# Add the rover_control directory to path
sys.path.insert(0, '/home/ubuntu/ros2_ws/src/rover_gps/rover_gps/rover_control')

from waypoint_navigator import WaypointNavigator, NavigationState


class SimulatedRover:
    """Simulates a rover moving through waypoints"""
    
    def __init__(self):
        # Start at a simulated position
        self.latitude = 49.891130
        self.longitude = -97.153738
        self.heading = 0.0  # degrees, 0 = North
        
        # Movement parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.update_rate = 0.1  # 10 Hz
        
    def update_position(self, linear, angular):
        """Update simulated position based on velocity commands"""
        self.linear_velocity = linear
        self.angular_velocity = angular
        
        # Update heading (simplified)
        self.heading += math.degrees(angular * self.update_rate)
        self.heading = self.heading % 360
        
        # Update position (simplified GPS movement)
        # Convert linear velocity to GPS coordinate change
        # Rough approximation: 1 m/s ≈ 0.00001 degrees latitude
        heading_rad = math.radians(self.heading)
        
        lat_change = linear * math.cos(heading_rad) * self.update_rate * 0.00001
        lon_change = linear * math.sin(heading_rad) * self.update_rate * 0.00001
        
        self.latitude += lat_change
        self.longitude += lon_change
    
    def get_position(self):
        """Get current position"""
        return self.latitude, self.longitude, self.heading
    
    def display_status(self, target_lat, target_lon, distance):
        """Display current status"""
        print(f"\n{'='*60}")
        print(f"Rover Position: {self.latitude:.6f}, {self.longitude:.6f}")
        print(f"Heading: {self.heading:.1f}°")
        print(f"Target: {target_lat:.6f}, {target_lon:.6f}")
        print(f"Distance to target: {distance:.2f}m")
        print(f"Commands: Linear={self.linear_velocity:.2f} m/s, Angular={self.angular_velocity:.2f} rad/s")
        print(f"{'='*60}")


def test_navigation_indoor(waypoint_file):
    """Test navigation logic indoors with simulation"""
    
    print("="*60)
    print("INDOOR NAVIGATION TEST")
    print("="*60)
    print("\nThis simulates the rover navigating through waypoints")
    print("without needing GPS signal or actual robot movement.\n")
    
    # Create navigator
    navigator = WaypointNavigator(waypoint_tolerance=0.5)  # 0.5m tolerance
    
    # Load waypoints
    print(f"Loading waypoints from: {waypoint_file}")
    if not navigator.load_waypoints_from_file(waypoint_file):
        print("❌ Failed to load waypoints")
        return False
    
    print(f"✓ Loaded {len(navigator.waypoints)} waypoints")
    print(f"✓ Total distance: {navigator.total_distance:.2f}m\n")
    
    # Create simulated rover
    rover = SimulatedRover()
    
    # Start navigation
    navigator.start_navigation()
    print("🚀 Starting simulated navigation...\n")
    
    iteration = 0
    max_iterations = 500  # Safety limit (50 seconds at 10Hz)
    
    try:
        while navigator.navigation_active and iteration < max_iterations:
            iteration += 1
            
            # Get current position
            lat, lon, heading = rover.get_position()
            
            # Calculate control commands
            linear, angular, reached = navigator.calculate_control_command(
                lat, lon, heading
            )
            
            # Update simulated rover
            rover.update_position(linear, angular)
            
            # Display status every 10 iterations (1 second)
            if iteration % 10 == 0 or reached:
                target = navigator.get_current_target()
                if target:
                    # Calculate distance for display
                    distance = navigator._calculate_distance_and_bearing(
                        lat, lon, target['latitude'], target['longitude']
                    )[0]
                    
                    rover.display_status(
                        target['latitude'], 
                        target['longitude'],
                        distance
                    )
                    
                    # Show navigation status
                    status = navigator.get_navigation_status()
                    print(f"Progress: {status['current_waypoint']}/{status['total_waypoints']} "
                          f"({status['progress_percent']:.1f}%)")
            
            # Simulate 10Hz update rate
            time.sleep(0.1)
        
        if reached:
            print("\n" + "="*60)
            print("✅ NAVIGATION COMPLETED!")
            print("="*60)
            status = navigator.get_navigation_status()
            print(f"Waypoints reached: {status['waypoints_reached']}/{status['total_waypoints']}")
            print(f"Total iterations: {iteration} ({iteration/10:.1f} seconds)")
            return True
        else:
            print("\n⚠️ Navigation stopped (max iterations reached)")
            return False
            
    except KeyboardInterrupt:
        print("\n\n⚠️ Test interrupted by user")
        return False


def main():
    """Main entry point"""
    
    # Default waypoint file
    waypoint_file = '/home/ubuntu/ros2_ws/src/rover_gps/rover_gps/rover_control/rover_waypoints_20251004_042535.json'
    
    # Allow custom file as argument
    if len(sys.argv) > 1:
        waypoint_file = sys.argv[1]
    
    print(f"\nUsing waypoint file: {waypoint_file}\n")
    
    # Run test
    success = test_navigation_indoor(waypoint_file)
    
    if success:
        print("\n✅ Indoor test successful!")
        print("\nThe navigation logic is working correctly.")
        print("Next steps:")
        print("1. Test outdoors with real GPS")
        print("2. Ensure robot controller is running:")
        print("   ros2 launch ros_robot_controller ros_robot_controller.launch.py")
    else:
        print("\n❌ Indoor test failed")
        print("Check the waypoint file and navigation logic")
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
