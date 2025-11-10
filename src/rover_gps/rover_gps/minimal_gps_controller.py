#!/usr/bin/env python3

"""
Minimal GPS + Manual Control System
Simple GPS waypoint navigation with manual override
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import math
import threading

class MinimalGPSController(Node):
    def __init__(self):
        super().__init__('minimal_gps_controller')
        
        # Simple state variables
        self.mode = 'MANUAL'  # MANUAL, AUTO, STOPPED
        self.current_gps = None
        self.target_gps = None
        self.emergency_stop = False
        
        # Simple navigation settings
        self.speed = 0.2  # Fixed slow speed for safety
        
        # Publishers - using cmd_vel (standard topic)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/gps_status', 10)
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10
        )
        self.manual_cmd_sub = self.create_subscription(
            Twist, '/manual_cmd', self.manual_cmd_callback, 10
        )
        
        # Services
        self.create_service(Trigger, '/set_manual_mode', self.set_manual_callback)
        self.create_service(Trigger, '/emergency_stop', self.emergency_stop_callback)
        
        # Simple timer for GPS navigation
        self.nav_timer = self.create_timer(1.0, self.navigation_loop)  # 1Hz - slow and safe
        
        self.get_logger().info("Minimal GPS Controller started - Manual mode")
        self.publish_status("Ready - Manual mode")

    def gps_callback(self, msg):
        """Update current GPS position"""
        if msg.status.status >= 0:  # Valid GPS fix
            self.current_gps = {
                'lat': msg.latitude,
                'lon': msg.longitude
            }
            # Log GPS updates every 10 seconds to avoid spam
            if hasattr(self, '_last_gps_log'):
                if self.get_clock().now().nanoseconds - self._last_gps_log > 10_000_000_000:  # 10 seconds
                    self.get_logger().info(f"GPS: {msg.latitude:.6f}, {msg.longitude:.6f}")
                    self._last_gps_log = self.get_clock().now().nanoseconds
            else:
                self._last_gps_log = self.get_clock().now().nanoseconds
        else:
            self.current_gps = None

    def manual_cmd_callback(self, msg):
        """Handle manual control commands"""
        self.get_logger().info(f"Received manual command: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        self.get_logger().info(f"Mode: {self.mode}, Emergency stop: {self.emergency_stop}")

        if self.mode == 'MANUAL' and not self.emergency_stop:
            self.get_logger().info("Publishing to /cmd_vel")
            self.cmd_vel_pub.publish(msg)
        else:
            self.get_logger().info(f"Not publishing - mode: {self.mode}, emergency_stop: {self.emergency_stop}")

    def set_target_gps(self, latitude, longitude):
        """Set target GPS coordinates - called from external interface"""
        self.target_gps = {
            'lat': latitude,
            'lon': longitude
        }
        self.mode = 'AUTO'
        self.get_logger().info(f"GPS target set: {latitude:.6f}, {longitude:.6f}")
        self.publish_status(f"AUTO mode - Target: {latitude:.6f}, {longitude:.6f}")

    def set_manual_callback(self, request, response):
        """Switch to manual mode"""
        self.mode = 'MANUAL'
        self.target_gps = None
        self.stop_rover()
        self.publish_status("Manual mode activated")
        self.get_logger().info("Switched to manual mode")
        response.success = True
        response.message = "Manual mode activated"
        return response

    def emergency_stop_callback(self, request, response):
        """Emergency stop"""
        self.emergency_stop = True
        self.mode = 'STOPPED'
        self.stop_rover()
        self.publish_status("EMERGENCY STOP")
        self.get_logger().error("EMERGENCY STOP ACTIVATED")
        response.success = True
        response.message = "Emergency stop activated"
        return response

    def stop_rover(self):
        """Stop all movement"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Simple distance calculation between GPS points"""
        # Convert to radians
        lat1_r = math.radians(lat1)
        lon1_r = math.radians(lon1)
        lat2_r = math.radians(lat2)
        lon2_r = math.radians(lon2)
        
        # Simple distance calculation
        dlat = lat2_r - lat1_r
        dlon = lon2_r - lon1_r
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters
        
        return distance

    def navigation_loop(self):
        """Simple GPS navigation loop"""
        if self.emergency_stop or self.mode != 'AUTO':
            return
            
        if not self.current_gps or not self.target_gps:
            return
            
        # Calculate distance to target
        distance = self.calculate_distance(
            self.current_gps['lat'], self.current_gps['lon'],
            self.target_gps['lat'], self.target_gps['lon']
        )
        
        # Check if we reached the target
        if distance < 2.0:  # Within 2 meters - success!
            self.stop_rover()
            self.mode = 'MANUAL'
            self.get_logger().info(f"Target reached! Distance: {distance:.2f}m")
            self.publish_status(f"Target reached! Distance: {distance:.2f}m - Switched to manual")
            return
        
        # Simple movement toward target
        twist = Twist()
        
        # Very simple logic: just move forward slowly
        # Note: This doesn't account for direction, it's just a basic proof of concept
        twist.linear.x = self.speed
        twist.angular.z = 0.0  # No turning for now
        
        self.cmd_vel_pub.publish(twist)
        
        # Update status
        status = f"AUTO: Distance to target: {distance:.1f}m"
        self.publish_status(status)

    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def get_status(self):
        """Get current status for external interfaces"""
        return {
            'mode': self.mode,
            'emergency_stop': self.emergency_stop,
            'current_gps': self.current_gps,
            'target_gps': self.target_gps
        }

def main(args=None):
    rclpy.init(args=args)
    controller = MinimalGPSController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
    finally:
        controller.stop_rover()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
