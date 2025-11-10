#!/usr/bin/env python3
"""
LiDAR Safety Monitor - Production Version
Component class optimized for /scan_raw topic and 2Hz operation
Handles NaN values, low update rates, and provides smooth safety control
"""

import math
import numpy as np
import threading
import time


class LidarSafetyMonitor:
    """
    LiDAR safety monitor component optimized for real-world conditions
    - Handles /scan_raw topic (and fallbacks)
    - Works with 2Hz LiDAR data
    - Robust NaN and outlier handling
    - Smooth movement control
    """
    
    def __init__(self, safety_distance=0.5, warning_distance=1.0, logger=None):
        """
        Initialize LiDAR safety monitor
        
        Args:
            safety_distance: Minimum safe distance in meters (stop threshold)
            warning_distance: Distance to start slowing down in meters
            logger: ROS logger to use (from parent node)
        """
        # Safety parameters
        self.safety_distance = safety_distance
        self.warning_distance = warning_distance
        self.logger = logger
        
        # Detection zones (in degrees from front)
        self.front_zone = 45  # ±45° in front (wider for safety)
        self.side_zone = 90   # ±90° for sides
        
        # LiDAR data
        self.latest_scan = None
        self.latest_scan_time = None
        self.obstacle_detected = False
        self.obstacle_in_warning_zone = False
        self.min_distance = float('inf')
        self.obstacle_angle = 0.0
        
        # Direction flags
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        
        # Safety state
        self.enabled = True
        self.data_timeout = 2.0  # Allow 2 seconds for 2Hz data (was 1.0)
        
        # Low-rate optimization
        self.last_command_time = 0
        self.command_smoothing = True
        self.min_command_interval = 0.1  # Don't change commands faster than 10Hz
        
        # Data quality tracking
        self.valid_ranges_count = 0
        self.total_ranges_count = 0
        self.data_quality_threshold = 0.3  # Need 30% valid ranges minimum
        
        # Thread safety
        self._lock = threading.Lock()
        
        self.log_info(f'LiDAR Safety Monitor initialized for /scan_raw')
        self.log_info(f'Safety: {safety_distance}m, Warning: {warning_distance}m')
        self.log_info(f'Optimized for 2Hz LiDAR operation')
    
    def log_info(self, message):
        """Log info message"""
        if self.logger:
            self.logger.info(message)
        else:
            print(f"[INFO] {message}")
    
    def log_warn(self, message):
        """Log warning message"""
        if self.logger:
            self.logger.warn(message)
        else:
            print(f"[WARN] {message}")
    
    def log_error(self, message):
        """Log error message"""
        if self.logger:
            self.logger.error(message)
        else:
            print(f"[ERROR] {message}")
    
    def update_scan_data(self, scan_msg):
        """
        Update LiDAR scan data (called by parent ROS node)
        
        Args:
            scan_msg: LaserScan message from /scan_raw
        """
        with self._lock:
            self.latest_scan = scan_msg
            self.latest_scan_time = time.time()
            if self.enabled:
                self._analyze_scan(scan_msg)
    
    def _analyze_scan(self, scan):
        """
        Analyze LiDAR scan for obstacles with robust handling
        
        Args:
            scan: LaserScan message
        """
        try:
            # Convert to numpy array for efficient processing
            ranges = np.array(scan.ranges)
            self.total_ranges_count = len(ranges)
            
            # Find valid ranges (not NaN, not inf, within sensor limits)
            valid_mask = (
                np.isfinite(ranges) & 
                (ranges >= scan.range_min) & 
                (ranges <= scan.range_max) &
                (ranges > 0.05)  # Filter out very close noise
            )
            
            valid_indices = np.where(valid_mask)[0]
            self.valid_ranges_count = len(valid_indices)
            
            # Check data quality
            if self.valid_ranges_count == 0:
                self.log_warn("No valid LiDAR ranges in scan")
                self._set_safe_defaults()
                return
            
            data_quality = self.valid_ranges_count / self.total_ranges_count
            if data_quality < self.data_quality_threshold:
                self.log_warn(f"Low LiDAR data quality: {data_quality:.1%}")
            
            # Get valid ranges and their angles
            valid_ranges = ranges[valid_indices]
            angles = scan.angle_min + valid_indices * scan.angle_increment
            
            # Normalize angles to [-pi, pi]
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            
            # Find minimum distance overall
            min_idx = np.argmin(valid_ranges)
            self.min_distance = valid_ranges[min_idx]
            self.obstacle_angle = angles[min_idx]
            
            # Check if obstacle in safety zones
            self.obstacle_detected = self.min_distance < self.safety_distance
            self.obstacle_in_warning_zone = self.min_distance < self.warning_distance
            
            # Determine obstacle direction with improved logic
            self._detect_obstacle_direction_robust(valid_ranges, angles)
            
            # Log significant detections (but not too frequently)
            if self.obstacle_detected:
                current_time = time.time()
                if not hasattr(self, '_last_warn_time') or (current_time - self._last_warn_time) > 1.0:
                    self.log_warn(
                        f'OBSTACLE: {self.min_distance:.2f}m at {math.degrees(self.obstacle_angle):.0f}°'
                    )
                    self._last_warn_time = current_time
                    
        except Exception as e:
            self.log_error(f'Error analyzing LiDAR scan: {e}')
            # Safe fallback - assume obstacle present for safety
            self.obstacle_detected = True
            self.min_distance = 0.1
            self.obstacle_front = True
    
    def _set_safe_defaults(self):
        """Set safe default values when no valid data"""
        self.obstacle_detected = False
        self.obstacle_in_warning_zone = False
        self.min_distance = float('inf')
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
    
    def _detect_obstacle_direction_robust(self, ranges, angles):
        """
        Detect obstacle direction with robust algorithm
        
        Args:
            ranges: Array of valid distance measurements
            angles: Array of corresponding angles (normalized to [-pi, pi])
        """
        try:
            # Convert to degrees for easier threshold handling
            angles_deg = np.degrees(angles)
            
            # Front zone: ±45° from 0° (front of robot)
            front_mask = np.abs(angles_deg) <= self.front_zone
            if np.any(front_mask):
                front_ranges = ranges[front_mask]
                min_front_distance = np.min(front_ranges)
                self.obstacle_front = min_front_distance < self.safety_distance
            else:
                self.obstacle_front = False
            
            # Left zone: 45° to 135° (left side)
            left_mask = (angles_deg >= self.front_zone) & (angles_deg <= 135)
            if np.any(left_mask):
                left_ranges = ranges[left_mask]
                min_left_distance = np.min(left_ranges)
                self.obstacle_left = min_left_distance < self.safety_distance
            else:
                self.obstacle_left = False
            
            # Right zone: -135° to -45° (right side)
            right_mask = (angles_deg <= -self.front_zone) & (angles_deg >= -135)
            if np.any(right_mask):
                right_ranges = ranges[right_mask]
                min_right_distance = np.min(right_ranges)
                self.obstacle_right = min_right_distance < self.safety_distance
            else:
                self.obstacle_right = False
                
        except Exception as e:
            self.log_error(f'Error detecting obstacle direction: {e}')
            # Safe fallback
            self.obstacle_front = self.obstacle_detected
            self.obstacle_left = False
            self.obstacle_right = False
    
    def is_safe_to_move(self, linear, angular):
        """
        Check if movement is safe with smooth command handling
        
        Args:
            linear: Desired linear velocity (m/s)
            angular: Desired angular velocity (rad/s)
            
        Returns:
            (safe, modified_linear, modified_angular): Tuple of safety flag and modified commands
        """
        with self._lock:
            current_time = time.time()
            
            # If disabled, allow all movement
            if not self.enabled:
                return (True, linear, angular)
            
            # Check if we have recent LiDAR data
            if self.latest_scan is None:
                self.log_warn('No LiDAR data available for safety check')
                return (True, linear, angular)
            
            # Check if data is too old (account for 2Hz rate)
            data_age = current_time - self.latest_scan_time if self.latest_scan_time else float('inf')
            if data_age > self.data_timeout:
                self.log_warn(f'LiDAR data is stale ({data_age:.1f}s old)')
                # Still allow movement but be more conservative
                safety_margin = 1.5
            else:
                safety_margin = 1.0
            
            # Rate limiting for smooth operation
            if (self.command_smoothing and 
                hasattr(self, '_last_safety_check') and
                (current_time - self._last_safety_check) < self.min_command_interval):
                # Return previous result to avoid rapid changes
                return getattr(self, '_last_safety_result', (True, linear, angular))
            
            self._last_safety_check = current_time
            
            # Emergency stop if obstacle too close
            if self.obstacle_detected:
                if self._is_moving_towards_obstacle(linear, angular):
                    self.log_warn(f'EMERGENCY STOP: Obstacle at {self.min_distance:.2f}m!')
                    result = (False, 0.0, 0.0)
                    self._last_safety_result = result
                    return result
            
            # Progressive slowdown in warning zone
            if self.obstacle_in_warning_zone:
                if self._is_moving_towards_obstacle(linear, angular):
                    # Calculate slowdown factor based on distance
                    distance_ratio = self.min_distance / self.warning_distance
                    scale_factor = max(0.2, distance_ratio / safety_margin)  # Minimum 20% speed
                    
                    modified_linear = linear * scale_factor
                    modified_angular = angular * scale_factor
                    
                    self.log_info(
                        f'Slowing down: {self.min_distance:.2f}m ahead, '
                        f'speed: {scale_factor:.0%}'
                    )
                    
                    result = (True, modified_linear, modified_angular)
                    self._last_safety_result = result
                    return result
            
            # Safe to proceed at full speed
            result = (True, linear, angular)
            self._last_safety_result = result
            return result
    
    def _is_moving_towards_obstacle(self, linear, angular):
        """
        Check if current movement would bring rover towards obstacle
        
        Args:
            linear: Linear velocity
            angular: Angular velocity
            
        Returns:
            True if moving towards obstacle, False otherwise
        """
        # Not moving
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            return False
        
        # Moving forward and obstacle in front
        if linear > 0.05 and self.obstacle_front:
            return True
        
        # Moving backward - generally allow (front-facing LiDAR)
        if linear < -0.05:
            return False
        
        # Pure rotation with minimal forward motion - generally safe
        if abs(linear) < 0.1 and abs(angular) > 0.1:
            return False
        
        # Check turning towards obstacles (only when moving forward significantly)
        if linear > 0.1:
            # Turning left and obstacle on left
            if angular > 0.3 and self.obstacle_left:
                return True
            
            # Turning right and obstacle on right  
            if angular < -0.3 and self.obstacle_right:
                return True
        
        return False
    
    def get_obstacle_info(self):
        """
        Get current obstacle detection info
        
        Returns:
            Dictionary with comprehensive obstacle information
        """
        with self._lock:
            data_age = float('inf')
            if self.latest_scan_time:
                data_age = time.time() - self.latest_scan_time
            
            data_quality = 0.0
            if self.total_ranges_count > 0:
                data_quality = self.valid_ranges_count / self.total_ranges_count
            
            return {
                'detected': self.obstacle_detected,
                'warning': self.obstacle_in_warning_zone,
                'distance': self.min_distance,
                'angle_deg': math.degrees(self.obstacle_angle) if self.obstacle_angle else 0,
                'front': self.obstacle_front,
                'left': self.obstacle_left,
                'right': self.obstacle_right,
                'enabled': self.enabled,
                'data_age': data_age,
                'data_stale': data_age > self.data_timeout,
                'data_quality': data_quality,
                'valid_ranges': self.valid_ranges_count,
                'total_ranges': self.total_ranges_count
            }
    
    def set_safety_distance(self, distance):
        """Update safety distance threshold"""
        with self._lock:
            self.safety_distance = max(0.1, distance)
            self.log_info(f'Safety distance updated to {self.safety_distance}m')
    
    def set_warning_distance(self, distance):
        """Update warning distance threshold"""
        with self._lock:
            self.warning_distance = max(self.safety_distance, distance)
            self.log_info(f'Warning distance updated to {self.warning_distance}m')
    
    def set_enabled(self, enabled):
        """Enable or disable safety monitoring"""
        with self._lock:
            self.enabled = enabled
            self.log_info(f'LiDAR safety {"enabled" if enabled else "disabled"}')
    
    def is_enabled(self):
        """Check if safety monitoring is enabled"""
        return self.enabled
    
    def has_recent_data(self):
        """Check if we have recent LiDAR data"""
        if self.latest_scan_time is None:
            return False
        return (time.time() - self.latest_scan_time) < self.data_timeout
    
    def get_status_summary(self):
        """Get a summary for debugging"""
        with self._lock:
            return {
                'enabled': self.enabled,
                'has_data': self.latest_scan is not None,
                'data_recent': self.has_recent_data(),
                'obstacle_detected': self.obstacle_detected,
                'min_distance': self.min_distance,
                'data_quality': f"{self.valid_ranges_count}/{self.total_ranges_count}"
            }


# Test the component (when run standalone)
if __name__ == '__main__':
    print("Testing LiDAR Safety Monitor for /scan_raw...")
    
    # Create monitor
    monitor = LidarSafetyMonitor(safety_distance=0.5, warning_distance=1.0)
    
    # Simulate LiDAR data similar to what we saw
    class MockScan:
        def __init__(self, ranges, angle_min=0.0, angle_max=6.28):
            self.ranges = ranges
            self.angle_min = angle_min
            self.angle_max = angle_max
            self.angle_increment = (angle_max - angle_min) / len(ranges)
            self.range_min = 0.02
            self.range_max = 25.0
    
    print("\n1. Testing with realistic /scan_raw data:")
    
    # Create realistic data with some NaN values (like real LiDAR)
    realistic_ranges = [1.0] * 350  # 350 points like real data
    realistic_ranges[0:10] = [0.3] * 10  # Close obstacle in front
    realistic_ranges[50:60] = [float('nan')] * 10  # Some NaN values
    realistic_ranges[100:110] = [0.8] * 10  # Warning zone obstacle
    
    realistic_scan = MockScan(realistic_ranges)
    monitor.update_scan_data(realistic_scan)
    
    # Test movement commands
    safe, linear, angular = monitor.is_safe_to_move(0.3, 0.0)
    info = monitor.get_obstacle_info()
    
    print(f"   Safe: {safe}, Commands: {linear:.2f}, {angular:.2f}")
    print(f"   Obstacle: {info['detected']}, Distance: {info['distance']:.2f}m")
    print(f"   Data quality: {info['data_quality']:.1%}")
    print(f"   Directions - Front: {info['front']}, Left: {info['left']}, Right: {info['right']}")
    
    print("\n✅ Component test completed for /scan_raw integration!")
