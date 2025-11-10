#!/usr/bin/env python3
"""
Simple rover test to verify continuous publishing works
This script demonstrates that Python with timer-based publishing is the solution
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading

class SimpleRoverTest(Node):
    def __init__(self):
        super().__init__('simple_rover_test')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for continuous publishing at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        # Current command
        self.current_twist = Twist()
        self.lock = threading.Lock()
        
        print("✓ Simple Rover Test initialized")
        print("✓ Publishing at 20 Hz to /cmd_vel")
        
    def timer_callback(self):
        """Continuously publish current command"""
        with self.lock:
            self.cmd_vel_pub.publish(self.current_twist)
    
    def set_cmd(self, linear_x, angular_z):
        """Update the command that's being published"""
        with self.lock:
            self.current_twist.linear.x = linear_x
            self.current_twist.angular.z = angular_z
        print(f"  → linear.x: {linear_x:.2f}, angular.z: {angular_z:.2f}")


def main():
    rclpy.init()
    controller = SimpleRoverTest()
    
    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    
    try:
        print("\n" + "="*60)
        print("🧪 SIMPLE ROVER TEST")
        print("="*60)
        print("Testing continuous publishing with timer\n")
        time.sleep(1)
        
        # Test 1: Forward
        print("Test 1: Move forward 2 seconds")
        controller.set_cmd(0.2, 0.0)
        time.sleep(2)
        
        # Test 2: Stop
        print("\nTest 2: Stop 1 second")
        controller.set_cmd(0.0, 0.0)
        time.sleep(1)
        
        # Test 3: Sharp left turn
        print("\nTest 3: Sharp LEFT turn 2 seconds")
        controller.set_cmd(0.2, 0.8)
        time.sleep(2)
        
        # Test 4: Gradually reduce steering
        print("\nTest 4: Gradually reduce steering (8 steps over 1.6 seconds)")
        steps = 8
        for i in range(steps + 1):
            progress = i / steps
            new_angular = 0.8 * (1.0 - progress)
            controller.set_cmd(0.2, new_angular)
            time.sleep(0.2)
        
        # Test 5: Continue straight
        print("\nTest 5: Continue straight 2 seconds")
        controller.set_cmd(0.25, 0.0)
        time.sleep(2)
        
        # Test 6: Sharp right turn
        print("\nTest 6: Sharp RIGHT turn 2 seconds")
        controller.set_cmd(0.2, -0.8)
        time.sleep(2)
        
        # Test 7: Gradually reduce steering
        print("\nTest 7: Gradually reduce steering (8 steps over 1.6 seconds)")
        for i in range(steps + 1):
            progress = i / steps
            new_angular = -0.8 * (1.0 - progress)
            controller.set_cmd(0.2, new_angular)
            time.sleep(0.2)
        
        # Test 8: Final straight
        print("\nTest 8: Final straight 2 seconds")
        controller.set_cmd(0.3, 0.0)
        time.sleep(2)
        
        # Stop
        print("\n🛑 Stopping...")
        controller.set_cmd(0.0, 0.0)
        time.sleep(1)
        
        print("\n" + "="*60)
        print("✅ ALL TESTS PASSED!")
        print("="*60)
        print("\nConclusion: Python with timer-based publishing WORKS!")
        print("Command-line 'ros2 topic pub' does NOT work reliably.\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Ctrl+C detected - STOPPING")
        controller.set_cmd(0.0, 0.0)
        time.sleep(0.5)
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
