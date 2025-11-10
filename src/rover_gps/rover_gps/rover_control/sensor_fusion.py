#!/usr/bin/env python3
"""
Sensor Fusion Module
Combines IMU (heading) with GPS (position) for accurate navigation
Uses complementary filter for smooth, accurate pose estimation
"""

import math
import time
import threading
import numpy as np
from collections import deque


class SensorFusion:
    """
    Fuses IMU and GPS data for accurate position and heading
    """
    
    def __init__(self, gps_weight=0.3, imu_weight=0.7):
        """
        Initialize sensor fusion
        
        Args:
            gps_weight: Weight for GPS bearing (0-1), default 0.3
            imu_weight: Weight for IMU heading (0-1), default 0.7
        """
        # Fusion weights (must sum to 1.0)
        self.gps_weight = gps_weight
        self.imu_weight = imu_weight
        
        # Current state
        self.fused_heading = 0.0  # degrees, 0=North
        self.fused_lat = 0.0
        self.fused_lon = 0.0
        
        # Raw sensor data
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_bearing = 0.0
        self.gps_valid = False
        self.gps_timestamp = 0.0
        
        self.imu_heading = 0.0
        self.imu_valid = False
        self.imu_timestamp = 0.0
        
        # History for smoothing
        self.heading_history = deque(maxlen=10)
        self.position_history = deque(maxlen=5)
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Statistics
        self.fusion_count = 0
        self.last_fusion_time = time.time()
        
        print("✓ Sensor Fusion initialized")
        print(f"  GPS weight: {gps_weight:.1%}, IMU weight: {imu_weight:.1%}")
    
    def update_gps(self, latitude, longitude, status=1):
        """
        Update GPS data
        
        Args:
            latitude: GPS latitude
            longitude: GPS longitude  
            status: GPS fix status (>= 0 = valid)
        """
        with self.lock:
            self.gps_lat = latitude
            self.gps_lon = longitude
            self.gps_valid = (status >= 0)
            self.gps_timestamp = time.time()
            
            # Calculate bearing from previous position if available
            if len(self.position_history) > 0:
                prev_lat, prev_lon = self.position_history[-1]
                self.gps_bearing = self._calculate_bearing(
                    prev_lat, prev_lon, latitude, longitude
                )
            
            self.position_history.append((latitude, longitude))
    
    def update_imu(self, heading_degrees):
        """
        Update IMU heading
        
        Args:
            heading_degrees: Compass heading in degrees (0=North, 90=East)
        """
        with self.lock:
            # Normalize to 0-360
            self.imu_heading = heading_degrees % 360
            self.imu_valid = True
            self.imu_timestamp = time.time()
    
    def get_fused_pose(self):
        """
        Get fused position and heading
        
        Returns:
            dict with 'lat', 'lon', 'heading', 'valid'
        """
        with self.lock:
            # Perform fusion
            self._fuse_sensors()
            
            return {
                'latitude': self.fused_lat,
                'longitude': self.fused_lon,
                'heading': self.fused_heading,
                'valid': self.gps_valid and self.imu_valid,
                'gps_only': self.gps_valid and not self.imu_valid,
                'imu_only': self.imu_valid and not self.gps_valid
            }
    
    def _fuse_sensors(self):
        """
        Perform sensor fusion (complementary filter)
        """
        current_time = time.time()
        
        # Position: Use GPS directly (most accurate for position)
        if self.gps_valid:
            self.fused_lat = self.gps_lat
            self.fused_lon = self.gps_lon
        
        # Heading: Fuse IMU and GPS bearing
        if self.imu_valid and self.gps_valid:
            # Both sensors available - use complementary filter
            
            # Handle angle wrapping (e.g., 359° and 1° should be close)
            imu_rad = math.radians(self.imu_heading)
            gps_rad = math.radians(self.gps_bearing)
            
            # Convert to unit vectors for smooth interpolation
            imu_x = math.cos(imu_rad)
            imu_y = math.sin(imu_rad)
            gps_x = math.cos(gps_rad)
            gps_y = math.sin(gps_rad)
            
            # Weighted average
            fused_x = self.imu_weight * imu_x + self.gps_weight * gps_x
            fused_y = self.imu_weight * imu_y + self.gps_weight * gps_y
            
            # Convert back to angle
            fused_heading = math.degrees(math.atan2(fused_y, fused_x))
            
            # Normalize to 0-360
            self.fused_heading = fused_heading % 360
            
            # Add to history for smoothing
            self.heading_history.append(self.fused_heading)
            
            # Apply moving average filter
            if len(self.heading_history) >= 3:
                self.fused_heading = self._circular_mean(list(self.heading_history))
            
            self.fusion_count += 1
            
        elif self.imu_valid:
            # Only IMU available - use it directly
            self.fused_heading = self.imu_heading
            
        elif self.gps_valid and len(self.position_history) >= 2:
            # Only GPS available - use bearing from movement
            self.fused_heading = self.gps_bearing
        
        self.last_fusion_time = current_time
    
    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate bearing between two GPS points
        
        Returns:
            Bearing in degrees (0-360)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    def _circular_mean(self, angles):
        """
        Calculate mean of angles (handles 0/360 wrap)
        
        Args:
            angles: List of angles in degrees
            
        Returns:
            Mean angle in degrees
        """
        # Convert to radians
        radians = [math.radians(a) for a in angles]
        
        # Sum of unit vectors
        sin_sum = sum(math.sin(r) for r in radians)
        cos_sum = sum(math.cos(r) for r in radians)
        
        # Mean angle
        mean_rad = math.atan2(sin_sum, cos_sum)
        mean_deg = math.degrees(mean_rad)
        
        return (mean_deg + 360) % 360
    
    def get_statistics(self):
        """
        Get fusion statistics
        
        Returns:
            dict with fusion stats
        """
        with self.lock:
            age_gps = time.time() - self.gps_timestamp if self.gps_timestamp > 0 else 999
            age_imu = time.time() - self.imu_timestamp if self.imu_timestamp > 0 else 999
            
            return {
                'fusion_count': self.fusion_count,
                'gps_valid': self.gps_valid,
                'imu_valid': self.imu_valid,
                'gps_age_sec': age_gps,
                'imu_age_sec': age_imu,
                'heading_history_size': len(self.heading_history),
                'position_history_size': len(self.position_history)
            }
    
    def reset(self):
        """Reset fusion state"""
        with self.lock:
            self.heading_history.clear()
            self.position_history.clear()
            self.fusion_count = 0
            print("Sensor fusion reset")


# Test/demo code
if __name__ == '__main__':
    # Create fusion module
    fusion = SensorFusion(gps_weight=0.3, imu_weight=0.7)
    
    # Simulate GPS updates
    print("\n--- Simulating sensor updates ---")
    
    # GPS: Moving north
    fusion.update_gps(49.8951, -97.1384, status=1)
    time.sleep(0.1)
    fusion.update_gps(49.8952, -97.1384, status=1)
    
    # IMU: Heading northeast (45°)
    fusion.update_imu(45.0)
    
    # Get fused result
    pose = fusion.get_fused_pose()
    print(f"\nFused Pose:")
    print(f"  Position: {pose['latitude']:.6f}, {pose['longitude']:.6f}")
    print(f"  Heading: {pose['heading']:.1f}°")
    print(f"  Valid: {pose['valid']}")
    
    # Statistics
    stats = fusion.get_statistics()
    print(f"\nStatistics:")
    print(f"  Fusion count: {stats['fusion_count']}")
    print(f"  GPS age: {stats['gps_age_sec']:.2f}s")
    print(f"  IMU age: {stats['imu_age_sec']:.2f}s")
