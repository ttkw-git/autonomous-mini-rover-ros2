#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
import sys

class GPSNavService(Node):
    """Service to handle GPS navigation requests"""
    
    def __init__(self):
        super().__init__('gps_nav_service')
        
        # Service to set GPS target
        self.srv = self.create_service(
            Empty, 
            'navigate_to_gps_target', 
            self.navigate_callback
        )
        
        # Target coordinates (set via parameters or command line)
        self.target_lat = self.declare_parameter('target_lat', 0.0).value
        self.target_lon = self.declare_parameter('target_lon', 0.0).value
        
        self.get_logger().info(f'GPS Nav Service ready - Target: {self.target_lat}, {self.target_lon}')
        
    def navigate_callback(self, request, response):
        """Handle navigation request"""
        self.get_logger().info(f'Navigation requested to: {self.target_lat}, {self.target_lon}')
        # Call main GPS navigation node
        return response

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) >= 3:
        target_lat = float(sys.argv[1])
        target_lon = float(sys.argv[2])
    else:
        target_lat = 0.0
        target_lon = 0.0
        
    node = GPSNavService()
    node.set_parameters([
        rclpy.Parameter('target_lat', rclpy.Parameter.Type.DOUBLE, target_lat),
        rclpy.Parameter('target_lon', rclpy.Parameter.Type.DOUBLE, target_lon)
    ])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
