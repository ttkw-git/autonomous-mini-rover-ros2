#!/usr/bin/env python3
"""
Proper Motor Controller Test using MotorsState messages
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ProperMotorTest(Node):
    def __init__(self):
        super().__init__('proper_motor_test')
        
        # Import the correct message types
        try:
            from ros_robot_controller_msgs.msg import MotorsState, MotorState
            self.MotorsState = MotorsState
            self.MotorState = MotorState
            self.get_logger().info("Successfully imported MotorsState and MotorState")
        except ImportError as e:
            self.get_logger().error(f"Could not import motor messages: {e}")
            return
        
        # Create publisher for proper motor commands
        self.motor_pub = self.create_publisher(
            self.MotorsState, 
            '/ros_robot_controller/set_motor', 
            10
        )
        
        # Subscribe to cmd_vel for monitoring
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/controller/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info("Proper Motor Test Node Started")

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to proper MotorsState message"""
        self.get_logger().info(f"Converting cmd_vel: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
        
        # Convert to motor speeds (simple differential drive for testing)
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Differential drive kinematics
        wheel_base = 0.3  # 30cm between wheels
        left_speed = linear_x - angular_z * wheel_base / 2
        right_speed = linear_x + angular_z * wheel_base / 2
        
        # Scale to reasonable RPS values (revolutions per second)
        scale_factor = 2.0  # Adjust this based on your wheel radius and desired speed
        left_rps = left_speed * scale_factor
        right_rps = right_speed * scale_factor
        
        # Create MotorsState message
        motors_msg = self.MotorsState()
        
        # Motor 1 (Left front)
        motor1 = self.MotorState()
        motor1.id = 1
        motor1.rps = left_rps
        
        # Motor 2 (Right front)  
        motor2 = self.MotorState()
        motor2.id = 2
        motor2.rps = right_rps
        
        # Motor 3 (Left rear)
        motor3 = self.MotorState()
        motor3.id = 3
        motor3.rps = left_rps
        
        # Motor 4 (Right rear)
        motor4 = self.MotorState()
        motor4.id = 4
        motor4.rps = right_rps
        
        # Add all motors to the message
        motors_msg.data = [motor1, motor2, motor3, motor4]
        
        # Publish the motor commands
        self.motor_pub.publish(motors_msg)
        self.get_logger().info(f"Published MotorsState: Motor1={left_rps:.2f}, Motor2={right_rps:.2f}, Motor3={left_rps:.2f}, Motor4={right_rps:.2f}")

    def test_direct_motor_control(self):
        """Test direct motor control without cmd_vel"""
        self.get_logger().info("Testing direct motor control...")
        
        # Test 1: Move forward
        self.get_logger().info("Test 1: Moving forward...")
        self.send_motor_command(1.0, 1.0, 1.0, 1.0)  # All wheels forward
        time.sleep(2)
        
        # Stop
        self.get_logger().info("Stopping...")
        self.send_motor_command(0.0, 0.0, 0.0, 0.0)
        time.sleep(1)
        
        # Test 2: Turn right
        self.get_logger().info("Test 2: Turning right...")
        self.send_motor_command(-1.0, 1.0, -1.0, 1.0)  # Left wheels backward, right forward
        time.sleep(2)
        
        # Stop
        self.get_logger().info("Stopping...")
        self.send_motor_command(0.0, 0.0, 0.0, 0.0)
        time.sleep(1)
        
        # Test 3: Turn left
        self.get_logger().info("Test 3: Turning left...")
        self.send_motor_command(1.0, -1.0, 1.0, -1.0)  # Left wheels forward, right backward
        time.sleep(2)
        
        # Final stop
        self.get_logger().info("Final stop...")
        self.send_motor_command(0.0, 0.0, 0.0, 0.0)
        
        self.get_logger().info("Direct motor test complete!")

    def send_motor_command(self, motor1_rps, motor2_rps, motor3_rps, motor4_rps):
        """Send direct motor commands"""
        motors_msg = self.MotorsState()
        
        motor1 = self.MotorState()
        motor1.id = 1
        motor1.rps = motor1_rps
        
        motor2 = self.MotorState()
        motor2.id = 2
        motor2.rps = motor2_rps
        
        motor3 = self.MotorState()
        motor3.id = 3
        motor3.rps = motor3_rps
        
        motor4 = self.MotorState()
        motor4.id = 4
        motor4.rps = motor4_rps
        
        motors_msg.data = [motor1, motor2, motor3, motor4]
        self.motor_pub.publish(motors_msg)
        self.get_logger().info(f"Direct command: M1={motor1_rps:.1f}, M2={motor2_rps:.1f}, M3={motor3_rps:.1f}, M4={motor4_rps:.1f}")

def main():
    rclpy.init()
    
    motor_test = ProperMotorTest()
    
    # Wait a bit for setup
    time.sleep(1)
    
    # Run direct motor test
    try:
        motor_test.test_direct_motor_control()
    except Exception as e:
        motor_test.get_logger().error(f"Error in direct motor test: {e}")
    
    # Then spin to handle cmd_vel messages
    try:
        motor_test.get_logger().info("Now listening for cmd_vel messages...")
        rclpy.spin(motor_test)
    except KeyboardInterrupt:
        motor_test.get_logger().info("Motor test shutting down...")
    finally:
        motor_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
