#!/usr/bin/env python3

"""
Minimal GPS Command Line Interface
Simple CLI for testing GPS navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
import threading
import sys

class MinimalGPSCLI(Node):
    def __init__(self):
        super().__init__('minimal_gps_cli')
        
        # Publishers
        self.manual_cmd_pub = self.create_publisher(Twist, '/manual_cmd', 10)
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/gps_status', self.status_callback, 10
        )
        
        # Service clients
        self.manual_mode_client = self.create_client(Trigger, '/set_manual_mode')
        self.emergency_stop_client = self.create_client(Trigger, '/emergency_stop')
        
        self.current_gps = None
        self.current_status = "Waiting for status..."
        
        print("\n=== Minimal GPS Controller CLI ===")
        print("Commands:")
        print("  w - Forward")
        print("  s - Backward") 
        print("  a - Turn left")
        print("  d - Turn right")
        print("  x - Stop")
        print("  m - Manual mode")
        print("  e - Emergency stop")
        print("  g - Show GPS")
        print("  t - Set GPS target")
        print("  q - Quit")
        print("===================================\n")

    def gps_callback(self, msg):
        """Update GPS display"""
        if msg.status.status >= 0:
            self.current_gps = f"{msg.latitude:.6f}, {msg.longitude:.6f}"
        else:
            self.current_gps = "No GPS fix"

    def status_callback(self, msg):
        """Update status display"""
        self.current_status = msg.data

    def send_movement_command(self, linear_x, angular_z):
        """Send movement command"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.manual_cmd_pub.publish(twist)

    def stop_movement(self):
        """Stop all movement"""
        self.send_movement_command(0.0, 0.0)

    def set_manual_mode(self):
        """Switch to manual mode"""
        request = Trigger.Request()
        future = self.manual_mode_client.call_async(request)
        return future

    def emergency_stop(self):
        """Emergency stop"""
        request = Trigger.Request()
        future = self.emergency_stop_client.call_async(request)
        return future

    def show_status(self):
        """Display current status"""
        print(f"\nStatus: {self.current_status}")
        print(f"GPS: {self.current_gps if self.current_gps else 'Waiting...'}")
        print("Enter command: ", end="", flush=True)

def cli_thread(cli_node):
    """Command line interface thread"""
    speed = 0.3  # Movement speed
    turn_speed = 0.8  # Turn speed
    
    while rclpy.ok():
        try:
            cli_node.show_status()
            command = input().lower().strip()
            
            if command == 'q':
                print("Shutting down...")
                break
            elif command == 'w':
                print("Moving forward...")
                cli_node.send_movement_command(speed, 0.0)
            elif command == 's':
                print("Moving backward...")
                cli_node.send_movement_command(-speed, 0.0)
            elif command == 'a':
                print("Forward + Turning left...")
                cli_node.send_movement_command(speed, turn_speed)
            elif command == 'd':
                print("Forward + Turning right...")
                cli_node.send_movement_command(speed, -turn_speed)
            elif command == 'x':
                print("Stopping...")
                cli_node.stop_movement()
            elif command == 'm':
                print("Switching to manual mode...")
                cli_node.set_manual_mode()
            elif command == 'e':
                print("EMERGENCY STOP!")
                cli_node.emergency_stop()
            elif command == 'g':
                print(f"Current GPS: {cli_node.current_gps if cli_node.current_gps else 'No GPS data'}")
            elif command == 't':
                try:
                    lat = float(input("Enter target latitude: "))
                    lon = float(input("Enter target longitude: "))
                    print(f"GPS target would be set to: {lat:.6f}, {lon:.6f}")
                    print("(GPS target setting not implemented in minimal version)")
                except ValueError:
                    print("Invalid coordinates")
            else:
                print("Invalid command")
                
        except KeyboardInterrupt:
            break
        except EOFError:
            break

def main():
    rclpy.init()
    cli_node = MinimalGPSCLI()
    
    # Start CLI in separate thread
    cli_thread_handle = threading.Thread(target=cli_thread, args=(cli_node,))
    cli_thread_handle.daemon = True
    cli_thread_handle.start()
    
    try:
        rclpy.spin(cli_node)
    except KeyboardInterrupt:
        pass
    finally:
        cli_node.stop_movement()
        cli_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
