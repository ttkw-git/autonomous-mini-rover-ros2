#!/usr/bin/env python3
"""
Proper MentorPi Ackermann Steering Controller
Correct configuration based on observation:
- Motors 2 & 4: Rear wheels (propulsion)  
- PWM Servo ID 3: Front steering (position 1320-1679)
- Motor 4 reverse mounted: negative RPS = forward
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math
import numpy as np

class ProperAckermannController(Node):
    def __init__(self):
        super().__init__('proper_ackermann_controller')
        
        # Import message types
        try:
            from ros_robot_controller_msgs.msg import MotorsState, MotorState
            from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
            self.MotorsState = MotorsState
            self.MotorState = MotorState
            self.SetPWMServoState = SetPWMServoState
            self.PWMServoState = PWMServoState
            self.get_logger().info("Motor and servo control messages imported successfully")
        except ImportError as e:
            self.get_logger().error(f"Could not import control messages: {e}")
            return
        
        # Publishers
        self.motor_pub = self.create_publisher(
            self.MotorsState, 
            '/ros_robot_controller/set_motor', 
            10
        )
        
        self.servo_pub = self.create_publisher(
            self.SetPWMServoState,
            '/ros_robot_controller/pwm_servo/set_state',
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
        
        # Ackermann steering parameters (based on observations)
        self.wheelbase = 0.30  # Distance between front and rear axles
        self.max_speed = 0.55  # Max speed from teleop
        
        # Rear wheel parameters (from teleop observations)
        self.wheel_scale = 4.32  # Speed scaling factor (2.375 RPS at 0.55 m/s)
        
        # Front steering servo parameters (from observations)
        self.servo_center = 1500  # Center position (estimated)
        self.servo_left = 1320    # Left turn position (observed)
        self.servo_right = 1679   # Right turn position (observed)
        self.servo_range = self.servo_right - self.servo_left  # Total range: 359
        self.max_steering_angle = math.radians(25)  # Max steering angle
        
        # Autonomous navigation parameters
        self.autonomous_mode = False
        self.obstacle_threshold = 0.5  # meters
        self.current_lidar_data = None
        self.current_imu_data = None
        self.current_odom = None
        
        # Navigation state
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Servo control state
        self.current_servo_position = self.servo_center
        self.servo_command_updated = False
        
        self.get_logger().info("Proper Ackermann Controller Started")
        self.get_logger().info("Configuration: Motors 2&4 (rear), PWM Servo 3 (front steering)")
        self.get_logger().info("Servo range: Left=1320, Center~=1500, Right=1679")
        self.get_logger().info("Listening to /cmd_vel for autonomous commands")
        
        # Create timer for continuous servo publishing (like teleop)
        self.servo_timer = self.create_timer(0.02, self.publish_servo_command)  # 50Hz

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
        
        # Convert to Ackermann steering commands
        self.send_ackermann_commands(linear_cmd, angular_cmd)

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
        """Extract front-facing LIDAR ranges"""
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

    def send_ackermann_commands(self, linear_velocity, angular_velocity):
        """Convert velocity commands to Ackermann steering (motors + servo)"""
        
        # Calculate steering angle and rear wheel speeds
        if abs(angular_velocity) < 0.001:
            # Straight line motion - no steering
            steering_angle = 0.0
            rear_speed = linear_velocity
        else:
            # Turning motion
            if abs(linear_velocity) < 0.1:
                # Pure rotation - use maximum steering, both rear wheels same direction
                steering_angle = self.max_steering_angle if angular_velocity < 0 else -self.max_steering_angle
                
                # Both rear wheels move in same direction for pure rotation
                rear_speed = angular_velocity * 0.3  # Small speed for rotation
            else:
                # Normal Ackermann steering with forward motion
                turning_radius = abs(linear_velocity / angular_velocity)
                steering_angle = math.atan(self.wheelbase / turning_radius)
                
                # Apply sign based on turn direction
                if angular_velocity < 0:  # Right turn
                    steering_angle = -steering_angle
                
                # Limit steering angle
                steering_angle = max(-self.max_steering_angle, 
                                   min(self.max_steering_angle, steering_angle))
                
                # Both rear wheels get same speed for Ackermann steering
                rear_speed = linear_velocity
        
        # Send motor commands (both rear wheels same speed)
        self.send_motor_commands(rear_speed, rear_speed)
        
        # Send steering servo command
        self.send_steering_command(steering_angle)

    def send_motor_commands(self, left_speed, right_speed):
        """Send commands to rear wheel motors"""
        
        # Scale speeds to RPS values (based on teleop observations)
        left_rps = -(left_speed * self.wheel_scale)   # Motor 4 (negative because reverse mounted)
        right_rps = right_speed * self.wheel_scale    # Motor 2
        
        # Create motor command message
        motors_msg = self.MotorsState()
        
        # Motor 1: Unused
        motor1 = self.MotorState()
        motor1.id = 1
        motor1.rps = 0.0
        
        # Motor 2: Right rear wheel
        motor2 = self.MotorState()
        motor2.id = 2
        motor2.rps = right_rps
        
        # Motor 3: Unused (steering is handled by servo)
        motor3 = self.MotorState()
        motor3.id = 3
        motor3.rps = 0.0
        
        # Motor 4: Left rear wheel (reverse mounted)
        motor4 = self.MotorState()
        motor4.id = 4
        motor4.rps = left_rps
        
        motors_msg.data = [motor1, motor2, motor3, motor4]
        
        # Publish motor commands
        self.motor_pub.publish(motors_msg)
        
        self.get_logger().debug(f"Motor commands: Left={left_rps:.2f}, Right={right_rps:.2f}")

    def send_steering_command(self, steering_angle):
        """Update servo position (published continuously by timer)"""
        
        # Convert steering angle to servo position
        normalized_angle = steering_angle / self.max_steering_angle
        normalized_angle = max(-1.0, min(1.0, normalized_angle))
        
        # Convert to servo position
        if normalized_angle >= 0:
            # Right turn (positive angle)
            servo_position = self.servo_center + int(normalized_angle * (self.servo_right - self.servo_center))
        else:
            # Left turn (negative angle)  
            servo_position = self.servo_center + int(normalized_angle * (self.servo_center - self.servo_left))
        
        # Ensure position is within bounds
        servo_position = max(self.servo_left, min(self.servo_right, servo_position))
        
        # Update current servo position for continuous publishing
        if servo_position != self.current_servo_position:
            self.current_servo_position = servo_position
            self.servo_command_updated = True
            self.get_logger().info(f"Servo target: angle={math.degrees(steering_angle):.1f}°, pos={servo_position}")

    def publish_servo_command(self):
        """Continuously publish servo commands (called by timer)"""
        try:
            # Import PWMServoState for individual servo state
            from ros_robot_controller_msgs.msg import PWMServoState
            
            # Create message matching exact teleop format
            servo_msg = self.SetPWMServoState()
            
            # Create individual servo state
            servo_state = PWMServoState()
            servo_state.id = [3]
            servo_state.position = [self.current_servo_position] 
            servo_state.offset = []
            
            # Set state array and duration
            servo_msg.state = [servo_state]
            servo_msg.duration = 0.02
            
            # Publish servo command continuously
            self.servo_pub.publish(servo_msg)
            
            # Log only when command changes
            if self.servo_command_updated:
                self.get_logger().debug(f"Publishing servo pos: {self.current_servo_position}")
                self.servo_command_updated = False
                
        except Exception as e:
            self.get_logger().error(f"Servo publish error: {e}")

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
        self.send_ackermann_commands(0.0, 0.0)
        self.get_logger().info("EMERGENCY STOP")

def main():
    rclpy.init()
    
    controller = ProperAckermannController()
    
    # Enable autonomous mode by default
    controller.enable_autonomous_mode()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Ackermann Controller shutting down...")
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
