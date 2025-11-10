#!/usr/bin/env python3
"""
LiDAR Safety Diagnostic Script
Diagnoses why LiDAR safety monitoring isn't preventing collisions
"""

import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarSafetyDiagnostic(Node):
    def __init__(self):
        super().__init__('lidar_safety_diagnostic')
        
        # Diagnostic data
        self.scan_count = 0
        self.cmd_vel_count = 0
        self.last_scan_time = None
        self.last_cmd_time = None
        self.min_distances = []
        
        # Subscribe to LiDAR topics
        self.lidar_topics = ['/scan', '/lidar/scan', '/laser_scan', '/ldlidar_node/scan']
        self.scan_subs = {}
        
        # Subscribe to cmd_vel to monitor commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/controller/cmd_vel', self.cmd_vel_callback, 10)
        
        # Setup LiDAR subscriptions
        self.setup_lidar_subscriptions()
        
        # Diagnostic timer
        self.diagnostic_timer = self.create_timer(2.0, self.print_diagnostics)
        
        self.get_logger().info('LiDAR Safety Diagnostic started')
    
    def setup_lidar_subscriptions(self):
        """Subscribe to all possible LiDAR topics"""
        for topic in self.lidar_topics:
            try:
                sub = self.create_subscription(
                    LaserScan, topic, 
                    lambda msg, t=topic: self.scan_callback(msg, t), 10)
                self.scan_subs[topic] = sub
                self.get_logger().info(f'Monitoring LiDAR topic: {topic}')
            except Exception as e:
                self.get_logger().warn(f'Could not subscribe to {topic}: {e}')
    
    def scan_callback(self, msg, topic):
        """Process LiDAR scan data"""
        self.scan_count += 1
        self.last_scan_time = time.time()
        
        # Analyze scan
        import numpy as np
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
        
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
            self.min_distances.append(min_dist)
            
            # Keep only recent distances
            if len(self.min_distances) > 10:
                self.min_distances = self.min_distances[-10:]
            
            # Log dangerous situations
            if min_dist < 0.5:
                self.get_logger().warn(f'🚨 CLOSE OBSTACLE on {topic}: {min_dist:.2f}m')
        
        # Log first few scans for debugging
        if self.scan_count <= 3:
            self.get_logger().info(f'Scan #{self.scan_count} from {topic}: {len(valid_ranges)} valid points, min: {np.min(valid_ranges):.2f}m')
    
    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel commands"""
        self.cmd_vel_count += 1
        self.last_cmd_time = time.time()
        
        # Log commands that should be dangerous
        if msg.linear.x > 0.1:
            min_dist = min(self.min_distances) if self.min_distances else float('inf')
            if min_dist < 0.5:
                self.get_logger().error(f'🚨 DANGEROUS COMMAND: linear={msg.linear.x:.2f} with obstacle at {min_dist:.2f}m')
        
        # Log first few commands
        if self.cmd_vel_count <= 5:
            self.get_logger().info(f'Cmd #{self.cmd_vel_count}: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
    
    def print_diagnostics(self):
        """Print diagnostic information"""
        current_time = time.time()
        
        print("\n" + "="*60)
        print("🔍 LIDAR SAFETY DIAGNOSTIC REPORT")
        print("="*60)
        
        # LiDAR status
        print(f"📡 LiDAR Status:")
        print(f"   Scans received: {self.scan_count}")
        if self.last_scan_time:
            scan_age = current_time - self.last_scan_time
            print(f"   Last scan: {scan_age:.1f}s ago")
            print(f"   Scan rate: {'OK' if scan_age < 1.0 else 'SLOW/STOPPED'}")
        else:
            print(f"   Last scan: NEVER")
        
        # Distance status
        if self.min_distances:
            current_min = min(self.min_distances)
            avg_min = sum(self.min_distances) / len(self.min_distances)
            print(f"   Current min distance: {current_min:.2f}m")
            print(f"   Average min distance: {avg_min:.2f}m")
            print(f"   Obstacle status: {'🚨 DANGER' if current_min < 0.5 else '⚠️ WARNING' if current_min < 1.0 else '✅ SAFE'}")
        else:
            print(f"   Distance data: NONE")
        
        # Command status
        print(f"\n🎮 Command Status:")
        print(f"   Commands sent: {self.cmd_vel_count}")
        if self.last_cmd_time:
            cmd_age = current_time - self.last_cmd_time
            print(f"   Last command: {cmd_age:.1f}s ago")
        else:
            print(f"   Last command: NEVER")
        
        # Safety analysis
        print(f"\n🛡️ Safety Analysis:")
        if self.scan_count == 0:
            print(f"   ❌ NO LIDAR DATA - Safety system cannot work!")
        elif self.cmd_vel_count == 0:
            print(f"   ℹ️ No movement commands detected")
        else:
            if self.min_distances and min(self.min_distances) < 0.5:
                print(f"   🚨 OBSTACLES DETECTED - Commands should be blocked!")
            else:
                print(f"   ✅ No immediate obstacles")
        
        # Recommendations
        print(f"\n💡 Recommendations:")
        if self.scan_count == 0:
            print(f"   1. Check LiDAR connection and topics")
            print(f"   2. Verify LiDAR node is running")
            print(f"   3. Check available topics: ros2 topic list | grep scan")
        
        if self.min_distances and min(self.min_distances) < 0.5 and self.cmd_vel_count > 0:
            print(f"   1. Safety system NOT working - commands not blocked")
            print(f"   2. Check safety monitor integration")
            print(f"   3. Verify safety logic is being called")
        
        print("="*60)


def run_diagnostic():
    """Run the diagnostic"""
    rclpy.init()
    
    diagnostic = LidarSafetyDiagnostic()
    
    print("🔍 LiDAR Safety Diagnostic Starting...")
    print("Move the rover towards obstacles to test safety system")
    print("Press Ctrl+C to stop")
    
    try:
        rclpy.spin(diagnostic)
    except KeyboardInterrupt:
        print("\n🛑 Diagnostic stopped by user")
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_diagnostic()
