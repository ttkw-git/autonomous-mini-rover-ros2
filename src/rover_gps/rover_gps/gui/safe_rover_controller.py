#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading

class SafeRoverController(Node):
    def __init__(self):
        super().__init__('safe_rover_controller')
        
        # Create publisher for motor control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer for continuous publishing (20 Hz)
        self.publish_rate = 20  # Hz
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Current command state
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.lock = threading.Lock()
        
        # Safety parameters
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.min_linear_for_steering = 0.1  # Minimum speed needed for steering
        
        # Steering reset parameters
        self.steering_center_threshold = 0.05  # rad/s
        
        self.get_logger().info(f'Safe Rover Controller initialized (publishing at {self.publish_rate} Hz)')
        
    def timer_callback(self):
        """Continuously publish current command at fixed rate"""
        with self.lock:
            twist = Twist()
            twist.linear.x = self.target_linear
            twist.angular.z = self.target_angular
            self.cmd_vel_pub.publish(twist)
    
    def set_velocity(self, linear_x, angular_z):
        """Thread-safe velocity setter"""
        with self.lock:
            self.target_linear = linear_x
            self.target_angular = angular_z
            
    def get_velocity(self):
        """Thread-safe velocity getter"""
        with self.lock:
            return self.target_linear, self.target_angular
        
    def stop(self):
        """Emergency stop"""
        self.set_velocity(0.0, 0.0)
        self.get_logger().info('🛑 STOP command sent')
        
    def reset_steering(self, maintain_speed=0.2, reset_duration=1.0, steps=8):
        """
        Gradually return steering to center while maintaining forward motion
        maintain_speed: speed to maintain during reset
        reset_duration: total time for reset in seconds
        steps: number of intermediate steps for smooth transition
        """
        _, current_angular = self.get_velocity()
        
        if abs(current_angular) < self.steering_center_threshold:
            self.get_logger().info('✓ Steering already centered')
            return
            
        self.get_logger().info(f'🔄 Resetting steering from {current_angular:.2f} to 0.0 over {reset_duration}s')
        
        # Calculate step size and delay
        step_delay = reset_duration / steps
        
        # Gradually reduce steering angle
        for step in range(steps + 1):
            # Linear interpolation from current_angular to 0
            progress = step / steps
            new_angular = current_angular * (1.0 - progress)
            
            self.set_velocity(maintain_speed, new_angular)
            self.get_logger().info(f'  Step {step + 1}/{steps + 1}: angular.z = {new_angular:.3f}')
            time.sleep(step_delay)
        
        # Final command to ensure centered
        self.set_velocity(maintain_speed, 0.0)
        self.get_logger().info('✓ Steering reset to center')
        
    def move(self, linear_speed, angular_speed):
        """
        Safe movement command with limits
        linear_speed: forward/backward speed in m/s (positive = forward)
        angular_speed: steering rate in rad/s (positive = left, negative = right)
        """
        # Apply safety limits
        linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
        
        # If trying to steer without moving, add minimum speed
        if abs(angular_speed) > 0.01 and abs(linear_speed) < self.min_linear_for_steering:
            self.get_logger().warn(f'⚠️  Steering requires movement. Setting minimum speed: {self.min_linear_for_steering}')
            linear_speed = self.min_linear_for_steering
        
        self.set_velocity(linear_speed, angular_speed)
        
        direction = "FORWARD" if linear_speed > 0 else "BACKWARD" if linear_speed < 0 else "STOPPED"
        turn = f"LEFT ({angular_speed:.2f})" if angular_speed > 0 else f"RIGHT ({angular_speed:.2f})" if angular_speed < 0 else "STRAIGHT"
        
        self.get_logger().info(f'🚗 Moving: {direction} | Steering: {turn} | Speed: {linear_speed:.2f} m/s')
        
    def move_forward(self, speed=0.2):
        """Move straight forward with centered steering"""
        self.move(speed, 0.0)
            
    def move_backward(self, speed=0.2):
        """Move straight backward"""
        self.move(-speed, 0.0)
        
    def turn_left(self, speed=0.2, turn_rate=0.5):
        """Turn left while moving forward"""
        self.move(speed, turn_rate)
        
    def turn_right(self, speed=0.2, turn_rate=0.5):
        """Turn right while moving forward"""
        self.move(speed, -turn_rate)
    
    def execute_maneuver(self, speed, angular, duration):
        """
        Execute a maneuver for a specific duration
        speed: linear speed (m/s)
        angular: angular velocity (rad/s)
        duration: how long to execute (seconds)
        """
        self.move(speed, angular)
        time.sleep(duration)


def main():
    rclpy.init()
    
    controller = SafeRoverController()
    
    # Spin in a separate thread so we can execute commands
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    try:
        print("\n" + "="*60)
        print("🤖 ROVER CONTROL TEST SEQUENCE")
        print("="*60)
        print("Publishing commands at 20 Hz continuously")
        print("⚠️  Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        time.sleep(1)
        
        # Test 1: Move forward
        print("Test 1: Moving forward (straight)...")
        controller.move_forward(speed=0.2)
        time.sleep(2)
        controller.stop()
        time.sleep(1)
        
        # Test 2: Sharp LEFT turn
        print("\nTest 2: Sharp LEFT turn...")
        controller.turn_left(speed=0.2, turn_rate=0.8)
        time.sleep(2)
        
        # Test 3: Reset steering gradually
        print("\nTest 3: Resetting steering to center...")
        controller.reset_steering(maintain_speed=0.2, reset_duration=1.5, steps=8)
        time.sleep(0.5)
        
        # Test 4: Move straight (should be centered)
        print("\nTest 4: Moving straight (wheels should be centered)...")
        controller.move_forward(speed=0.25)
        time.sleep(2)
        controller.stop()
        time.sleep(1)
        
        # Test 5: Sharp RIGHT turn
        print("\nTest 5: Sharp RIGHT turn...")
        controller.turn_right(speed=0.2, turn_rate=0.8)
        time.sleep(2)
        
        # Test 6: Reset steering
        print("\nTest 6: Resetting steering to center...")
        controller.reset_steering(maintain_speed=0.2, reset_duration=1.5, steps=8)
        time.sleep(0.5)
        
        # Test 7: Final straight
        print("\nTest 7: Final straight movement...")
        controller.move_forward(speed=0.3)
        time.sleep(2)
        
        # Stop
        print("\nStopping rover...")
        controller.stop()
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("✅ Test sequence completed successfully!")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n⚠️  Keyboard interrupt detected!")
        controller.stop()
        time.sleep(0.5)
        print("✅ Rover stopped safely")
        
    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        controller.stop()
        time.sleep(0.5)
        print("✅ Emergency stop executed")
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
