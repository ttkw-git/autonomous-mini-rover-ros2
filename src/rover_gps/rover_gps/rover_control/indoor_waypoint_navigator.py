#!/usr/bin/env python3
"""
Indoor Waypoint Navigator
Follows waypoints using IMU heading and odometry (no GPS needed!)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
import json
import math
import time


class IndoorWaypointNavigator(Node):
    """
    Autonomous indoor navigation using IMU + odometry
    """
    
    def __init__(self, waypoint_file=None):
        super().__init__('indoor_waypoint_navigator')
        
        # Navigation parameters
        self.waypoint_tolerance = 0.3  # meters
        self.heading_tolerance = 5.0   # degrees
        
        # Control parameters
        self.max_linear_speed = 0.25
        self.max_angular_speed = 0.4
        
        # Waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_heading = 0.0
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.imu_valid = False
        self.odom_valid = False
        
        # Navigation state
        self.navigation_active = False
        self.waypoints_reached = 0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Vector3,
            '/imu/rpy/filtered',
            self.imu_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Navigation timer (10 Hz)
        self.nav_timer = self.create_timer(0.1, self.navigation_update)
        
        self.get_logger().info('Indoor Navigator initialized')
        
        # Load waypoints if provided
        if waypoint_file:
            self.load_waypoints(waypoint_file)
    
    def imu_callback(self, msg):
        """Handle IMU updates"""
        self.current_heading = msg.z % 360
        self.imu_valid = True
    
    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_valid = True
    
    def load_waypoints(self, filename):
        """Load waypoints from JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            if data.get('mode') != 'indoor_imu_odometry':
                self.get_logger().warn(
                    f'Waypoint file mode is "{data.get("mode")}" - '
                    'expected "indoor_imu_odometry"'
                )
            
            self.waypoints = data['waypoints']
            self.origin_x = data['origin']['x']
            self.origin_y = data['origin']['y']
            self.origin_heading = data['origin']['heading']
            
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'✓ Loaded {len(self.waypoints)} waypoints')
            self.get_logger().info(f'  Origin: x={self.origin_x:.2f}, y={self.origin_y:.2f}, h={self.origin_heading:.1f}°')
            self.get_logger().info(f'  Total distance: {data.get("total_distance_m", 0):.2f}m')
            self.get_logger().info('=' * 60)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return False
    
    def start_navigation(self):
        """Start autonomous navigation"""
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints loaded!')
            return False
        
        if not self.imu_valid or not self.odom_valid:
            self.get_logger().error('Waiting for sensor data!')
            return False
        
        # Reset navigation state
        self.current_waypoint_idx = 0
        self.waypoints_reached = 0
        self.navigation_active = True
        
        self.get_logger().info('')
        self.get_logger().info('🚀 Navigation STARTED')
        self.get_logger().info(f'   Following {len(self.waypoints)} waypoints')
        self.get_logger().info('')
        
        return True
    
    def stop_navigation(self):
        """Stop navigation"""
        self.navigation_active = False
        self.stop_rover()
        self.get_logger().info('Navigation STOPPED')
    
    def navigation_update(self):
        """Main navigation loop (called at 10 Hz)"""
        if not self.navigation_active:
            return
        
        if not self.imu_valid or not self.odom_valid:
            return
        
        # Check if all waypoints reached
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('')
            self.get_logger().info('✓✓✓ All waypoints reached! ✓✓✓')
            self.get_logger().info(f'    Completed {self.waypoints_reached}/{len(self.waypoints)} waypoints')
            self.stop_navigation()
            return
        
        # Get target waypoint (in absolute coordinates)
        target = self.waypoints[self.current_waypoint_idx]
        target_x = self.origin_x + target['x']
        target_y = self.origin_y + target['y']
        target_heading = target['heading']
        
        # Calculate distance to target
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint reached
        if distance < self.waypoint_tolerance:
            self.waypoints_reached += 1
            self.current_waypoint_idx += 1
            
            self.get_logger().info(
                f'✓ Waypoint {self.waypoints_reached}/{len(self.waypoints)} reached! '
                f'(distance: {distance:.2f}m)'
            )
            
            return
        
        # Calculate desired heading to target
        desired_heading = math.degrees(math.atan2(dy, dx)) % 360
        
        # Calculate heading error
        heading_error = self._normalize_angle(desired_heading - self.current_heading)
        
        # Calculate control commands
        linear, angular = self._calculate_control(distance, heading_error)
        
        # Send command
        self.send_cmd_vel(linear, angular)
        
        # Log status every 2 seconds
        if int(time.time()) % 2 == 0:
            self.get_logger().info(
                f'WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
                f'dist={distance:.2f}m, heading_err={heading_error:.1f}°, '
                f'cmd=[{linear:.2f}, {angular:.2f}]'
            )
    
    def _calculate_control(self, distance, heading_error):
        """
        Calculate velocity commands
        
        Args:
            distance: Distance to target (m)
            heading_error: Heading error (degrees, -180 to 180)
        
        Returns:
            (linear, angular) velocities
        """
        # Linear velocity - slow down as approaching
        if distance > 2.0:
            linear = self.max_linear_speed
        elif distance > 1.0:
            linear = self.max_linear_speed * 0.7
        elif distance > 0.5:
            linear = self.max_linear_speed * 0.5
        else:
            linear = self.max_linear_speed * 0.3
        
        # Angular velocity - proportional to heading error
        kp = 0.02  # Proportional gain
        angular = kp * heading_error
        
        # Clamp angular velocity
        angular = max(-self.max_angular_speed, 
                     min(self.max_angular_speed, angular))
        
        # If heading error is large, reduce forward speed
        if abs(heading_error) > 45:
            linear *= 0.3
        elif abs(heading_error) > 20:
            linear *= 0.6
        
        return linear, angular
    
    def _normalize_angle(self, angle):
        """Normalize angle to -180 to 180 degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def send_cmd_vel(self, linear, angular):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)
    
    def stop_rover(self):
        """Stop the rover"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)


def main():
    """Main function"""
    import sys
    
    rclpy.init()
    
    # Check if waypoint file provided
    if len(sys.argv) > 1:
        waypoint_file = sys.argv[1]
    else:
        waypoint_file = None
    
    navigator = IndoorWaypointNavigator(waypoint_file)
    
    if waypoint_file:
        # Wait for sensors
        print('\nWaiting for sensors...')
        time.sleep(2)
        
        # Start navigation
        if navigator.start_navigation():
            print('\nNavigation started! Press Ctrl+C to stop.\n')
    else:
        print('\nUsage: python3 indoor_waypoint_navigator.py <waypoint_file.json>')
        print('Waiting for manual waypoint load...\n')
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print('\nStopping...')
    
    navigator.stop_rover()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
