#!/usr/bin/env python3
"""
Ackermann steering constraint handler
Manages steering angle limits and minimum speed requirements
"""

import math


class AckermannMovementHandler:
    """Handles Ackermann steering constraints for proper rover control"""
    
    def __init__(self, wheelbase=0.213, max_steering_deg=36, max_linear=0.6, min_linear=0.15):
        """
        Initialize Ackermann handler
        
        Args:
            wheelbase: Distance between front and rear axles (meters)
            max_steering_deg: Maximum steering angle (degrees)
            max_linear: Maximum linear velocity (m/s)
            min_linear: Minimum speed required for steering (m/s)
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = math.radians(max_steering_deg)
        self.max_linear = max_linear
        self.min_linear_for_steering = min_linear
        
    def calculate_safe_command(self, linear, angular, debug=False):
        """
        Convert desired linear/angular to Ackermann-safe commands
        
        Args:
            linear: Desired linear velocity (m/s)
            angular: Desired angular velocity (rad/s)
            debug: Enable debug output
            
        Returns:
            (safe_linear, safe_angular): Constrained velocities
        """
        # Clamp linear velocity
        safe_linear = max(-self.max_linear, min(self.max_linear, linear))
        
        # If not moving forward, don't allow turning
        if abs(safe_linear) < 0.01:
            #    if debug and abs(angular) > 0.01:
            #    print(f"[Ackermann] No forward motion, blocking turn command")
        #    return (0.0, 0.0)
            pass

        # Calculate desired steering angle from angular velocity
        # angular_z = v * tan(steering_angle) / wheelbase
        if abs(safe_linear) > 0.01:
            desired_steering_angle = math.atan(angular * self.wheelbase / safe_linear)
        else:
            desired_steering_angle = 0.0
            
        # Clamp steering angle to physical limits
        safe_steering_angle = max(-self.max_steering_angle, 
                                   min(self.max_steering_angle, desired_steering_angle))
        
        # Convert back to angular velocity
        safe_angular = safe_linear * math.tan(safe_steering_angle) / self.wheelbase
        
        # Ensure minimum speed when steering
        if abs(safe_angular) > 0.01 and abs(safe_linear) < self.min_linear_for_steering:
            safe_linear = self.min_linear_for_steering if safe_linear > 0 else -self.min_linear_for_steering
            if debug:
                print(f"[Ackermann] Increased speed to minimum: {safe_linear:.2f} m/s")
        
        if debug:
            print(f"[Ackermann] Input: L={linear:.2f}, A={angular:.2f} -> Output: L={safe_linear:.2f}, A={safe_angular:.2f}")
        
        return (safe_linear, safe_angular)
    
    def get_steering_angle_deg(self, angular, linear):
        """
        Get current steering angle in degrees for display
        
        Args:
            angular: Angular velocity (rad/s)
            linear: Linear velocity (m/s)
            
        Returns:
            Steering angle in degrees
        """
        if abs(linear) < 0.01:
            return 0.0
        steering_angle = math.atan(angular * self.wheelbase / linear)
        return math.degrees(steering_angle)
    
    def get_max_angular_for_speed(self, linear):
        """
        Calculate maximum angular velocity for given linear speed
        
        Args:
            linear: Linear velocity (m/s)
            
        Returns:
            Maximum safe angular velocity (rad/s)
        """
        if abs(linear) < 0.01:
            return 0.0
        return linear * math.tan(self.max_steering_angle) / self.wheelbase
