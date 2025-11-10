#!/usr/bin/env python3
"""
Sensor Fusion Handler for ROS2
Integrates IMU (/imu/rpy/filtered) with GPS for accurate navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
import math
import time
import threading
from collections import deque


class SensorFusionHandler(Node):
    """
    Subscribes to IMU and GPS topics and provides fused pose data
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_handler')
        
        # Fusion weights
        self.gps_weight = 0.2  # GPS bearing weight
        self.imu_weight = 0.8  # IMU heading weight (more reliable)
        
        # Current state
        self.fused_heading = 0.0  # degrees, 0=North, 90=East
        self.fused_lat = 0.0
        self.fused_lon = 0.0
        
        # Raw sensor data
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_bearing = 0.0
        self.gps_valid = False
        self.gps_timestamp = 0.0
        
        self.imu_yaw = 0.0  # From IMU
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_valid = False
        self.imu_timestamp = 0.0
        
        # History for bearing calculation
        self.position_history = deque(maxlen=3)
        self.heading_history = deque(maxlen=5)
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Vector3,
            '/imu/rpy/filtered',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('✓ Sensor Fusion Handler initialized')
        self.get_logger().info(f'  IMU weight: {self.imu_weight:.1%}, GPS weight: {self.gps_weight:.1%}')
    
    def gps_callback(self, msg):
        """Handle GPS updates"""
        with self.lock:
            if abs(msg.latitude) > 90 or abs(msg.longitude) > 180:
                return
            
            self.gps_lat = msg.latitude
            self.gps_lon = msg.longitude
            self.gps_valid = (msg.status.status >= 0)
            self.gps_timestamp = time.time()
            
            # Calculate bearing from movement
            if len(self.position_history) > 0:
                prev_lat, prev_lon = self.position_history[-1]
                self.gps_bearing = self._calculate_bearing(
                    prev_lat, prev_lon, 
                    msg.latitude, msg.longitude
                )
            
            self.position_history.append((msg.latitude, msg.longitude))
    
    def imu_callback(self, msg):
        """
        Handle IMU RPY updates
        msg.x = roll, msg.y = pitch, msg.z = yaw (in degrees)
        """
        with self.lock:
            self.imu_roll = msg.x
            self.imu_pitch = msg.y
            self.imu_yaw = msg.z  # This is the heading!
            
            # Normalize to 0-360
            self.imu_yaw = self.imu_yaw % 360
            if self.imu_yaw < 0:
                self.imu_yaw += 360
            
            self.imu_valid = True
            self.imu_timestamp = time.time()
    
    def get_fused_pose(self):
        """
        Get current fused pose with heading
        
        Returns:
            dict with lat, lon, heading, valid flags
        """
        with self.lock:
            self._fuse_sensors()
            
            return {
                'latitude': self.fused_lat,
                'longitude': self.fused_lon,
                'heading': self.fused_heading,
                'valid': self.gps_valid and self.imu_valid,
                'gps_only': self.gps_valid and not self.imu_valid,
                'imu_only': self.imu_valid and not self.gps_valid,
                'roll': self.imu_roll,
                'pitch': self.imu_pitch
            }
    
    def _fuse_sensors(self):
        """Perform sensor fusion"""
        # Position: Always use GPS (most accurate)
        if self.gps_valid:
            self.fused_lat = self.gps_lat
            self.fused_lon = self.gps_lon
        
        # Heading: Fuse IMU and GPS bearing
        if self.imu_valid and self.gps_valid and len(self.position_history) >= 2:
            # Both available - use weighted fusion
            imu_rad = math.radians(self.imu_yaw)
            gps_rad = math.radians(self.gps_bearing)
            
            # Unit vector fusion
            imu_x = math.cos(imu_rad)
            imu_y = math.sin(imu_rad)
            gps_x = math.cos(gps_rad)
            gps_y = math.sin(gps_rad)
            
            fused_x = self.imu_weight * imu_x + self.gps_weight * gps_x
            fused_y = self.imu_weight * imu_y + self.gps_weight * gps_y
            
            fused_heading = math.degrees(math.atan2(fused_y, fused_x))
            self.fused_heading = (fused_heading + 360) % 360
            
            # Smooth with history
            self.heading_history.append(self.fused_heading)
            if len(self.heading_history) >= 3:
                self.fused_heading = self._circular_mean(list(self.heading_history))
                
        elif self.imu_valid:
            # Only IMU - use it directly (most common case)
            self.fused_heading = self.imu_yaw
            
        elif self.gps_valid and len(self.position_history) >= 2:
            # Only GPS - use bearing
            self.fused_heading = self.gps_bearing
    
    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two GPS points"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    def _circular_mean(self, angles):
        """Calculate mean of angles"""
        radians = [math.radians(a) for a in angles]
        sin_sum = sum(math.sin(r) for r in radians)
        cos_sum = sum(math.cos(r) for r in radians)
        mean_rad = math.atan2(sin_sum, cos_sum)
        return (math.degrees(mean_rad) + 360) % 360


# Test standalone
if __name__ == '__main__':
    rclpy.init()
    
    fusion_handler = SensorFusionHandler()
    
    def print_status():
        while rclpy.ok():
            pose = fusion_handler.get_fused_pose()
            print(f"\r Fused: Lat={pose['latitude']:.6f}, "
                  f"Lon={pose['longitude']:.6f}, "
                  f"Heading={pose['heading']:.1f}° | "
                  f"Valid={pose['valid']}", end='')
            time.sleep(0.5)
    
    import threading
    status_thread = threading.Thread(target=print_status, daemon=True)
    status_thread.start()
    
    try:
        rclpy.spin(fusion_handler)
    except KeyboardInterrupt:
        pass
    
    fusion_handler.destroy_node()
    rclpy.shutdown()
