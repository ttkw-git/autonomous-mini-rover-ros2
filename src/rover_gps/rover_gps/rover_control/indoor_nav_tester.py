#!/usr/bin/env python3
"""
Indoor Navigation Tester
Simulates GPS while using real rover motors
Perfect for testing navigation logic indoors!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class IndoorNavigationTester(Node):
    def __init__(self):
        super().__init__('indoor_nav_tester')
        
        # Fake GPS publisher
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Subscribe to cmd_vel to track movement
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/controller/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Starting position (Winnipeg example)
        self.lat = 49.8951
        self.lon = -97.1384
        self.heading = 0.0  # North
        
        # Publish GPS at 1 Hz
        self.timer = self.create_timer(0.2, self.publish_fake_gps)
        
        self.get_logger().info('🏠 Indoor Navigation Tester Started')
        self.get_logger().info(f'   Start: {self.lat:.6f}, {self.lon:.6f}')
        self.get_logger().info('   Drive rover - GPS will update based on movement!')
     
    def get_meters_per_degree(self, latitude):
        lat_rad = math.radians(latitude)
        meters_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
        meters_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)
        return meters_per_deg_lat, meters_per_deg_lon


    def cmd_vel_callback(self, msg):
        """Update fake GPS based on rover movement"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Time step (1 second between GPS updates)
        dt = 1.0
        
        # Update heading based on angular velocity
        self.heading += angular * dt
        
        # Calculate distance moved
        distance = linear * dt
        
        # Update lat/lon based on heading
        # Rough approximation
        # meters_per_deg_lat = 111000
        # meters_per_deg_lon = 70000
        
        meters_per_deg_lat, meters_per_deg_lon = self.get_meters_per_degree(self.lat)

        delta_lat = distance * math.cos(self.heading) / meters_per_deg_lat
        delta_lon = distance * math.sin(self.heading) / meters_per_deg_lon
        
        self.lat += delta_lat
        self.lon += delta_lon
        
    def publish_fake_gps(self):
        """Publish fake GPS position"""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        msg.latitude = self.lat
        msg.longitude = self.lon
        msg.altitude = 239.0
        
        msg.status.status = 1  # Fix
        msg.status.service = 1
        
        self.gps_pub.publish(msg)
        self.get_logger().info(f'GPS: {self.lat:.6f}, {self.lon:.6f}')

def main():
    rclpy.init()
    node = IndoorNavigationTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
