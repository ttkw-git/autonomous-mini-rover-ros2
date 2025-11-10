#!/usr/bin/env python3
"""
Indoor Waypoint Recorder
Records waypoints using IMU heading and distance traveled (no GPS needed!)
Perfect for indoor navigation testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
import json
import time
import math
from datetime import datetime


class IndoorWaypointRecorder(Node):
    """
    Records waypoints based on distance traveled and IMU heading
    No GPS required - works perfectly indoors!
    """
    
    def __init__(self):
        super().__init__('indoor_waypoint_recorder')
        
        # Recording state
        self.recording = False
        self.waypoints = []
        self.waypoint_id = 0
        
        # Origin (starting point)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_heading = 0.0
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.current_distance = 0.0
        
        # IMU data
        self.imu_heading = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_valid = False
        
        # Odometry data
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_valid = False
        
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
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Indoor Waypoint Recorder initialized')
        self.get_logger().info('Waiting for IMU and odometry data...')
        self.get_logger().info('=' * 60)
    
    def imu_callback(self, msg):
        """Handle IMU updates (heading in degrees)"""
        self.imu_roll = msg.x
        self.imu_pitch = msg.y
        self.imu_heading = msg.z % 360  # Normalize to 0-360
        self.imu_valid = True
    
    def odom_callback(self, msg):
        """Handle odometry updates (position in meters)"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_valid = True
        
        # Update current position
        if self.recording:
            self.current_x = self.odom_x
            self.current_y = self.odom_y
            self.current_heading = self.imu_heading
            
            # Calculate distance from origin
            dx = self.current_x - self.origin_x
            dy = self.current_y - self.origin_y
            self.current_distance = math.sqrt(dx**2 + dy**2)
    
    def start_recording(self):
        """Start recording waypoints"""
        if not self.imu_valid or not self.odom_valid:
            self.get_logger().error('Cannot start - waiting for sensor data!')
            self.get_logger().info(f'  IMU valid: {self.imu_valid}')
            self.get_logger().info(f'  Odom valid: {self.odom_valid}')
            return False
        
        # Set origin
        self.origin_x = self.odom_x
        self.origin_y = self.odom_y
        self.origin_heading = self.imu_heading
        
        self.current_x = self.origin_x
        self.current_y = self.origin_y
        self.current_heading = self.origin_heading
        self.current_distance = 0.0
        
        # Clear previous waypoints
        self.waypoints = []
        self.waypoint_id = 0
        self.recording = True
        
        self.get_logger().info('')
        self.get_logger().info('✓ Recording STARTED')
        self.get_logger().info(f'  Origin: x={self.origin_x:.3f}m, y={self.origin_y:.3f}m')
        self.get_logger().info(f'  Initial heading: {self.origin_heading:.1f}°')
        self.get_logger().info('')
        
        # Automatically record first waypoint (origin)
        self.record_waypoint()
        
        return True
    
    def record_waypoint(self):
        """Record current position as waypoint"""
        if not self.recording:
            self.get_logger().warn('Not recording! Call start_recording() first')
            return False
        
        if not self.imu_valid or not self.odom_valid:
            self.get_logger().warn('Skipping - invalid sensor data')
            return False
        
        self.waypoint_id += 1
        
        # Calculate relative position from origin
        rel_x = self.current_x - self.origin_x
        rel_y = self.current_y - self.origin_y
        
        # Calculate heading relative to origin
        rel_heading = (self.current_heading - self.origin_heading) % 360
        
        waypoint = {
            'id': self.waypoint_id,
            'timestamp': time.time(),
            'datetime': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'x': rel_x,
            'y': rel_y,
            'heading': self.current_heading,
            'relative_heading': rel_heading,
            'distance_from_origin': self.current_distance,
            'roll': self.imu_roll,
            'pitch': self.imu_pitch
        }
        
        self.waypoints.append(waypoint)
        
        self.get_logger().info(
            f'✓ Waypoint {self.waypoint_id}: '
            f'x={rel_x:.2f}m, y={rel_y:.2f}m, '
            f'heading={self.current_heading:.1f}°, '
            f'dist={self.current_distance:.2f}m'
        )
        
        return True
    
    def stop_recording(self):
        """Stop recording"""
        if not self.recording:
            self.get_logger().warn('Not currently recording')
            return
        
        self.recording = False
        self.get_logger().info('')
        self.get_logger().info('✓ Recording STOPPED')
        self.get_logger().info(f'  Total waypoints: {len(self.waypoints)}')
        self.get_logger().info('')
    
    def save_to_file(self, filename=None):
        """Save waypoints to JSON file"""
        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints to save!')
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'indoor_waypoints_{timestamp}.json'
        
        # Calculate total path distance
        total_distance = 0.0
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i]['x'] - self.waypoints[i-1]['x']
            dy = self.waypoints[i]['y'] - self.waypoints[i-1]['y']
            total_distance += math.sqrt(dx**2 + dy**2)
        
        data = {
            'origin': {
                'x': self.origin_x,
                'y': self.origin_y,
                'heading': self.origin_heading
            },
            'waypoints': self.waypoints,
            'total_waypoints': len(self.waypoints),
            'total_distance_m': round(total_distance, 2),
            'recording_time': datetime.now().strftime('%Y%m%d_%H%M%S'),
            'mode': 'indoor_imu_odometry',
            'coordinate_system': 'relative_xy_meters'
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✓ SAVED: {filename}')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Total distance: {total_distance:.2f}m')
        self.get_logger().info('=' * 60)
        
        return filename
    
    def get_current_status(self):
        """Get current recording status"""
        return {
            'recording': self.recording,
            'waypoint_count': len(self.waypoints),
            'current_x': self.current_x - self.origin_x if self.recording else 0.0,
            'current_y': self.current_y - self.origin_y if self.recording else 0.0,
            'current_heading': self.current_heading,
            'current_distance': self.current_distance,
            'imu_valid': self.imu_valid,
            'odom_valid': self.odom_valid
        }


def main():
    """Interactive recording session"""
    rclpy.init()
    
    recorder = IndoorWaypointRecorder()
    
    # Spin in background
    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(recorder), daemon=True)
    spin_thread.start()
    
    # Wait for sensors
    print('\nWaiting for sensor data...')
    time.sleep(2)
    
    print('\n' + '=' * 60)
    print('INDOOR WAYPOINT RECORDER - Interactive Mode')
    print('=' * 60)
    print('Commands:')
    print('  s - Start recording')
    print('  w - Record waypoint')
    print('  x - Stop recording')
    print('  f - Save to file')
    print('  i - Show status')
    print('  q - Quit')
    print('=' * 60 + '\n')
    
    try:
        while rclpy.ok():
            cmd = input('Command [s/w/x/f/i/q]: ').strip().lower()
            
            if cmd == 's':
                recorder.start_recording()
            elif cmd == 'w':
                recorder.record_waypoint()
            elif cmd == 'x':
                recorder.stop_recording()
            elif cmd == 'f':
                recorder.save_to_file()
            elif cmd == 'i':
                status = recorder.get_current_status()
                print(f'\nStatus:')
                print(f'  Recording: {status["recording"]}')
                print(f'  Waypoints: {status["waypoint_count"]}')
                print(f'  Position: x={status["current_x"]:.2f}m, y={status["current_y"]:.2f}m')
                print(f'  Heading: {status["current_heading"]:.1f}°')
                print(f'  Distance: {status["current_distance"]:.2f}m')
                print(f'  IMU: {status["imu_valid"]}, Odom: {status["odom_valid"]}\n')
            elif cmd == 'q':
                print('Exiting...')
                break
            else:
                print('Unknown command\n')
                
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    
    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
