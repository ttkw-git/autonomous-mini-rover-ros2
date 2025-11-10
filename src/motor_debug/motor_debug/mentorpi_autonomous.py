#!/usr/bin/env python3
"""
MentorPi Ackermann Autonomous Controller
Based on observed motor configuration:
- Motor 1: Unused
- Motor 2: Right rear wheel (positive RPS = forward)  
- Motor 3: Front steering (position control)
- Motor 4: Left rear wheel (negative RPS = forward, reverse mounted)

This controller provides autonomous navigation capabilities while maintaining
compatibility with the existing teleop system.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math
import numpy as np

class MentorPiAutonomousController(Node):
    def __init__(self):
        super().__init__('mentorpi_autonomous_controller')
        
        # Import motor control messages
        try:
            from ros_robot_controller_msgs.msg import MotorsState, MotorState
            self.MotorsState = MotorsState
            self.MotorState = MotorState
            self.get_logger().info("Motor control messages imported successfully")
        except ImportError as e:
            self.get_logger().error(f"Could not import motor messages: {e}")
            return
        
        # Publishers
        self.motor_pub = self.create_publisher(
            self.MotorsState, 
            '/ros_robot_controller/set_motor', 
            10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.lidar_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Robot parameters (based on teleop observations)
        self.wheelbase = 0.30  # Distance between front and rear axles
        self.max_speed = 0.55  # Max speed from teleop observations
        self.max_turn_rate = 1.1  # Max turn rate from teleop observations
        self.max_steering_angle = math.radians(30)  # Maximum steering angle
        
        # Speed scaling factors (observed from teleop)
        self.rear_wheel_scale = 5.36  # Based on 2.96 RPS at speed 0.55
        
        # Autonomous navigation parameters
        self.autonomous_mode = False
        self.obstacle_threshold = 0.5  # meters
        self.current_lidar_data = None
        self.current_imu_data = None
        self.current_odom = None
        
        # Navigation state
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        self.get_logger().info("MentorPi Autonomous Controller Started")
        self.get_logger().info("Listening to /cmd_vel for autonomous commands")
        self.get_logger().info("Use /cmd_vel for autonomous control, teleop still works on /controller/cmd_vel")

    def cmd_vel_callback(self, msg):
        """Handle autonomous navigation commands from /cmd_vel"""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        
        # Apply obstacle avoidance if enabled
        if self.autonomous_mode and self.current_lidar_data:
            linear_cmd, angular_cmd = self.apply_obstacle_avoidance(
                self.target_linear, self.target_angular)
        else:
            linear_cmd = self.target_linear
            angular_cmd = self.target_angular
        
        # Convert to motor commands
        self.send_motor_commands(linear_cmd, angular_cmd)

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection"""
        self.current_lidar_data = msg
        
        if self.autonomous_mode:
            # Check for obstacles in front
            front_ranges = self.get_front_lidar_ranges(msg)
            min_distance = min(front_ranges) if front_ranges else float('inf')
            
            if min_distance < self.obstacle_threshold:
                self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m")

    def imu_callback(self, msg):
        """Process IMU data for orientation"""
        self.current_imu_data = msg

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_odom = msg

    def get_front_lidar_ranges(self, lidar_msg):
        """Extract front-facing LIDAR ranges (90-degree cone in front)"""
        ranges = np.array(lidar_msg.ranges)
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment
        
        # Calculate indices for front-facing region (-45 to +45 degrees)
        front_angle_range = math.radians(45)
        total_points = len(ranges)
        
        # Find center index (0 degrees)
        center_idx = int((-angle_min) / angle_increment)
        
        # Calculate range for front-facing cone
        half_cone_points = int(front_angle_range / angle_increment)
        
        start_idx = max(0, center_idx - half_cone_points)
        end_idx = min(total_points, center_idx + half_cone_points)
        
        # Filter out invalid ranges
        front_ranges = ranges[start_idx:end_idx]
        valid_ranges = front_ranges[(front_ranges > lidar_msg.range_min) & 
                                   (front_ranges < lidar_msg.range_max)]
        
        return valid_ranges.tolist()

    def apply_obstacle_avoidance(self, target_linear, target_angular):
        """Apply obstacle avoidance to target commands"""
        if not self.current_lidar_data:
            return target_linear, target_angular
        
        front_ranges = self.get_front_lidar_ranges(self.current_lidar_data)
        
        if not front_ranges:
            return target_linear, target_angular
        
        min_distance = min(front_ranges)
        
        # If obstacle is too close, modify commands
        if min_distance < self.obstacle_threshold:
            # Reduce forward speed based on distance
            speed_factor = max(0.0, min_distance / self.obstacle_threshold)
            modified_linear = target_linear * speed_factor
            
            # Add avoidance turn if moving forward
            if target_linear > 0:
                # Simple obstacle avoidance - turn away from obstacle
                avoidance_turn = 0.5 if target_angular >= 0 else -0.5
                modified_angular = target_angular + avoidance_turn
            else:
                modified_angular = target_angular
            
            self.get_logger().info(f"Obstacle avoidance: distance={min_distance:.2f}m, "
                                 f"speed={modified_linear:.2f}, turn={modified_angular:.2f}")
            
            return modified_linear, modified_angular
        
        return target_linear, target_angular

    def send_motor_commands(self, linear_velocity, angular_velocity):
        """Convert velocity commands to motor commands (matching teleop behavior)"""
        
        # Calculate rear wheel speeds for ackermann steering
        # Based on observed teleop behavior and motor configuration
        
        # Scale velocities to match teleop ranges
        scaled_linear = linear_velocity * self.rear_wheel_scale
        
        # For ackermann steering, both rear wheels get same speed for straight motion
        # Differential speed is minimal for steering (most steering is done by front wheel)
        wheel_speed_differential = angular_velocity * 0.5  # Small differential for stability
        
        # Motor speeds (based on observed configuration)
        right_wheel_speed = scaled_linear + wheel_speed_differential  # Motor 2
        left_wheel_speed = -(scaled_linear - wheel_speed_differential)  # Motor 4 (negative because reverse mounted)
        
        # Steering angle calculation for front wheel (Motor 3)
        # Convert angular velocity to steering position
        steering_position = angular_velocity / self.max_turn_rate  # Normalize to -1 to 1
        steering_position = max(-1.0, min(1.0, steering_position))  # Clamp
        
        # Create motor command message
        motors_msg = self.MotorsState()
        
        # Motor 1: Unused
        motor1 = self.MotorState()
        motor1.id = 1
        motor1.rps = 0.0
        
        # Motor 2: Right rear wheel
        motor2 = self.MotorState()
        motor2.id = 2
        motor2.rps = right_wheel_speed
        
        # Motor 3: Front steering (position control)
        motor3 = self.MotorState()
        motor3.id = 3
        motor3.rps = steering_position  # May need to be a position field instead of rps
        
        # Motor 4: Left rear wheel (reverse mounted)
        motor4 = self.MotorState()
        motor4.id = 4
        motor4.rps = left_wheel_speed
        
        motors_msg.data = [motor1, motor2, motor3, motor4]
        
        # Publish motor commands
        self.motor_pub.publish(motors_msg)
        
        self.get_logger().debug(f"Motor commands: R={right_wheel_speed:.2f}, "
                              f"L={left_wheel_speed:.2f}, S={steering_position:.2f}")

    def enable_autonomous_mode(self):
        """Enable autonomous obstacle avoidance"""
        self.autonomous_mode = True
        self.get_logger().info("Autonomous mode ENABLED - obstacle avoidance active")

    def disable_autonomous_mode(self):
        """Disable autonomous mode"""
        self.autonomous_mode = False
        self.get_logger().info("Autonomous mode DISABLED - manual control only")

    def stop_robot(self):
        """Emergency stop"""
        self.send_motor_commands(0.0, 0.0)
        self.get_logger().info("EMERGENCY STOP")

def main():
    rclpy.init()
    
    controller = MentorPiAutonomousController()
    
    # Enable autonomous mode by default
    controller.enable_autonomous_mode()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Autonomous Controller shutting down...")
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
