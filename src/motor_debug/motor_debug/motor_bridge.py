#!/usr/bin/env python3
"""
Motor Control Bridge Node for MentorPi Rover
Bridges cmd_vel commands to the actual motor controller

This script converts Twist messages to motor commands
and can be used to debug motor control issues.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import math

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        
        # Subscribe to cmd_vel commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/controller/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Create publisher for motor commands (we'll figure out the correct message type)
        # Try different potential motor command topics
        self.motor_pub = self.create_publisher(
            Float32MultiArray,  # Assuming motor commands are float arrays
            '/ros_robot_controller/set_motor',
            10
        )
        
        # Parameters for mecanum wheel kinematics
        self.wheel_radius = 0.05  # 5cm wheel radius (adjust as needed)
        self.robot_width = 0.3    # 30cm robot width (adjust as needed)
        self.robot_length = 0.3   # 30cm robot length (adjust as needed)
        
        # Motor speed limits
        self.max_motor_speed = 100.0  # Adjust based on your motor controller
        
        self.get_logger().info("Motor Bridge Node Started")
        self.get_logger().info("Listening to /controller/cmd_vel")
        self.get_logger().info("Publishing to /ros_robot_controller/set_motor")

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor commands"""
        self.get_logger().info(f"Received cmd_vel: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x      # Forward/backward velocity
        linear_y = msg.linear.y      # Left/right velocity (for mecanum)
        angular_z = msg.angular.z    # Rotation velocity
        
        # Calculate individual wheel speeds for mecanum drive
        # Standard mecanum wheel configuration:
        # Front-left, Front-right, Rear-left, Rear-right
        
        # Mecanum wheel kinematics
        # v_fl = linear_x - linear_y - angular_z * (width + length) / 2
        # v_fr = linear_x + linear_y + angular_z * (width + length) / 2  
        # v_rl = linear_x + linear_y - angular_z * (width + length) / 2
        # v_rr = linear_x - linear_y + angular_z * (width + length) / 2
        
        robot_diagonal = (self.robot_width + self.robot_length) / 2
        
        v_fl = linear_x - linear_y - angular_z * robot_diagonal
        v_fr = linear_x + linear_y + angular_z * robot_diagonal
        v_rl = linear_x + linear_y - angular_z * robot_diagonal
        v_rr = linear_x - linear_y + angular_z * robot_diagonal
        
        # Convert velocities to motor speeds (normalize to motor controller range)
        motor_speeds = [v_fl, v_fr, v_rl, v_rr]
        
        # Scale to motor controller range
        max_speed = max(abs(speed) for speed in motor_speeds)
        if max_speed > 1.0:
            motor_speeds = [speed / max_speed for speed in motor_speeds]
        
        # Scale to motor controller's expected range
        motor_commands = [speed * self.max_motor_speed for speed in motor_speeds]
        
        self.get_logger().info(f"Motor commands: FL={motor_commands[0]:.1f}, FR={motor_commands[1]:.1f}, RL={motor_commands[2]:.1f}, RR={motor_commands[3]:.1f}")
        
        # Publish motor commands
        motor_msg = Float32MultiArray()
        motor_msg.data = motor_commands
        self.motor_pub.publish(motor_msg)

    def simple_differential_drive(self, linear_x, angular_z):
        """Alternative: Simple differential drive calculation"""
        # For robots that don't have mecanum wheels
        left_speed = linear_x - angular_z * self.robot_width / 2
        right_speed = linear_x + angular_z * self.robot_width / 2
        
        return [left_speed * self.max_motor_speed, right_speed * self.max_motor_speed]

def main():
    rclpy.init()
    
    # Create and run the motor bridge node
    motor_bridge = MotorBridge()
    
    try:
        rclpy.spin(motor_bridge)
    except KeyboardInterrupt:
        motor_bridge.get_logger().info("Motor Bridge shutting down...")
    finally:
        motor_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
