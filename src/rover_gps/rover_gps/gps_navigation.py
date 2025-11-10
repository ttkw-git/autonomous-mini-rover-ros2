#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

class GPSNavigation(Node):
    """Convert GPS coordinates to navigation goals"""
    
    def __init__(self):
        super().__init__('gps_navigation')
        
        # GPS reference point (first GPS fix will be origin)
        self.origin_lat = None
        self.origin_lon = None
        self.origin_set = False
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info('GPS Navigation node started')
        
    def gps_callback(self, msg):
        """Store first GPS fix as origin"""
        if not self.origin_set and msg.status.status >= 0:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_set = True
            self.get_logger().info(f'GPS origin set: {self.origin_lat:.6f}, {self.origin_lon:.6f}')
            
    def gps_to_map(self, target_lat, target_lon):
        """Convert GPS coordinates to map coordinates"""
        if not self.origin_set:
            self.get_logger().error('GPS origin not set yet!')
            return None, None
            
        # Simple flat earth approximation (good for local navigation)
        R = 6371000  # Earth radius in meters
        
        # Convert to radians
        lat1 = math.radians(self.origin_lat)
        lon1 = math.radians(self.origin_lon)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)
        
        # Calculate x, y in meters from origin
        x = R * (lon2 - lon1) * math.cos((lat1 + lat2) / 2)
        y = R * (lat2 - lat1)
        
        return x, y
        
    def navigate_to_gps(self, target_lat, target_lon):
        """Send navigation goal to GPS coordinates"""
        x, y = self.gps_to_map(target_lat, target_lon)
        
        if x is None:
            return False
            
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Navigating to GPS: {target_lat:.6f}, {target_lon:.6f}')
        self.get_logger().info(f'Map coordinates: x={x:.2f}, y={y:.2f}')
        
        # Send goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
