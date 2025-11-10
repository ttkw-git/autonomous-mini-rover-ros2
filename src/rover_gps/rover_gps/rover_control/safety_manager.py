#!/usr/bin/env python3
"""
Global safety manager for rover control
Ensures rover can be stopped from any thread in case of emergency
"""

from geometry_msgs.msg import Twist


class SafetyManager:
    """Global safety manager to ensure rover stops on any failure"""
    
    def __init__(self):
        """Initialize safety manager"""
        self.ros_node = None
        self.emergency_active = False
        
    def set_ros_node(self, node):
        """
        Register ROS node for emergency control
        
        Args:
            node: ROS2 node with cmd_vel publisher
        """
        self.ros_node = node
        
    def emergency_stop(self):
        """Emergency stop from any thread - sends zero velocity command"""
        self.emergency_active = True
        if self.ros_node:
            try:
                stop_msg = Twist()
                self.ros_node.cmd_vel_pub.publish(stop_msg)
                print("Emergency stop activated!")
            except Exception as e:
                print(f"Emergency stop error: {e}")
                
    def reset(self):
        """Reset emergency stop flag"""
        self.emergency_active = False
                
    def cleanup(self):
        """Cleanup on exit - stops rover and destroys node"""
        self.emergency_stop()
        if self.ros_node:
            try:
                self.ros_node.destroy_node()
            except Exception as e:
                print(f"Cleanup error: {e}")
