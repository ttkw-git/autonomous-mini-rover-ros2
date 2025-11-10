#!/usr/bin/env python3
"""
ROS2 interface for rover control - FIXED VERSION
Handles ROS2 publishers, subscribers, Ackermann constraints, and LiDAR safety
INTEGRATED: Proper LiDAR safety monitoring with fallback options
"""

import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu, LaserScan

from ackermann_handler import AckermannMovementHandler


class RoverROSInterface(Node):
    """ROS2 interface node for rover control with integrated LiDAR safety"""
    
    def __init__(self, enable_lidar_safety=True):
        super().__init__('rover_gui_interface')
        
        # Thread safety
        self._lock = threading.Lock()
        self.shutdown_requested = False
        
        # Ackermann handler (always enabled)
        self.ackermann = AckermannMovementHandler()
        
        # LiDAR safety monitor - import the FIXED version
        self.enable_lidar_safety = enable_lidar_safety
        self.lidar_safety = None
        self.lidar_subscription = None
        
        if enable_lidar_safety:
            self._setup_lidar_safety()
        else:
            self.get_logger().info('LiDAR safety monitoring DISABLED')
        
        # GPS data storage
        self.latest_gps_lat = 0.0
        self.latest_gps_lon = 0.0
        self.latest_gps_status = -1
        self.gps_data_available = False
        self._gps_lock = threading.Lock()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.get_logger().info('Created cmd_vel publisher on: /controller/cmd_vel')
        
        # GPS Subscribers - try multiple topics
        self.gps_topics_to_try = ['/fix', '/gps/fix', '/nmea_fix', '/gps_fix']
        self.gps_sub = None
        self.current_gps_topic = None
        self.setup_gps_subscription()
        
        # Safety timer
        self.last_command_time = time.time()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        # Mode
        self.manual_mode = True
        
        self.get_logger().info('Rover ROS Interface initialized with Ackermann constraints')
    
    def _setup_lidar_safety(self):
        """Setup LiDAR safety system with fallback options"""
        try:
            # Try to import the FIXED safety monitor
            try:
                from lidar_safety_monitor_fixed import LidarSafetyMonitor
                self.get_logger().info('Using FIXED LidarSafetyMonitor')
            except ImportError:
                # Fallback to original if fixed version not available
                try:
                    from lidar_safety_monitor import LidarSafetyMonitor
                    self.get_logger().warn('Using original LidarSafetyMonitor (may have issues)')
                except ImportError:
                    self.get_logger().error('LidarSafetyMonitor not found - disabling LiDAR safety')
                    self.enable_lidar_safety = False
                    return
            
            # Create safety monitor as a COMPONENT (not a separate node)
            self.lidar_safety = LidarSafetyMonitor(
                safety_distance=0.5, 
                warning_distance=1.0,
                logger=self.get_logger()  # Pass our logger
            )
            
            # Try to find and subscribe to LiDAR topic
            self._setup_lidar_subscription()
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup LiDAR safety: {e}')
            self.enable_lidar_safety = False
            self.lidar_safety = None
    
    def _setup_lidar_subscription(self):
        """Setup LiDAR topic subscription with automatic topic detection"""
        
        # List of common LiDAR topics to try
        lidar_topics_to_try = [
            '/scan_raw',
            '/scan',
            '/lidar/scan', 
            '/laser_scan',
            '/rplidar/scan',
            '/ldlidar_scan',
            '/robot/scan'
        ]
        
        # Try to find available LiDAR topics
        try:
            topic_list = self.get_topic_names_and_types()
            available_topics = {name: types for name, types in topic_list}
            
            self.get_logger().info(f'Searching for LiDAR topics in {len(available_topics)} available topics')
            
            # Try each potential LiDAR topic
            for topic in lidar_topics_to_try:
                if topic in available_topics:
                    topic_types = available_topics[topic]
                    
                    # Check if it's a LaserScan topic
                    if 'sensor_msgs/msg/LaserScan' in topic_types:
                        try:
                            self.lidar_subscription = self.create_subscription(
                                LaserScan,
                                topic,
                                self.lidar_callback,
                                10
                            )
                            self.get_logger().info(f'✅ Subscribed to LiDAR topic: {topic}')
                            return True
                            
                        except Exception as e:
                            self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
                    else:
                        self.get_logger().debug(f'Topic {topic} has wrong type: {topic_types}')
            
            # No LiDAR topic found
            self.get_logger().warn('❌ No LiDAR topics found - LiDAR safety will not function')
            self.get_logger().info('💡 Make sure LiDAR driver is running. Common commands:')
            self.get_logger().info('   ros2 launch peripherals lidar.launch.py')
            self.get_logger().info('   ros2 run rplidar_ros rplidar_composition')
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'Error setting up LiDAR subscription: {e}')
            return False
    
    def lidar_callback(self, msg):
        """Handle incoming LiDAR scan data"""
        if self.shutdown_requested or not self.lidar_safety:
            return
        
        try:
            # Pass the scan data to our safety monitor component
            self.lidar_safety.update_scan_data(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in LiDAR callback: {e}')
    
    def setup_gps_subscription(self):
        """Try to find and subscribe to available GPS topic"""
        if self.shutdown_requested:
            return
        
        try:
            topic_list = self.get_topic_names_and_types()
            available_topics = [topic_name for topic_name, _ in topic_list]
            
            self.get_logger().info(f'Searching for GPS topics in {len(available_topics)} topics')
            
            for topic in self.gps_topics_to_try:
                if topic in available_topics:
                    try:
                        self.gps_sub = self.create_subscription(
                            NavSatFix, topic, self.gps_callback, 10)
                        self.current_gps_topic = topic
                        self.get_logger().info(f'Subscribed to GPS topic: {topic}')
                        return
                    except Exception as e:
                        self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
            
            # No GPS topic found - using Direct GPS instead
            self.get_logger().info('No ROS GPS topic found. Using Direct GPS handler.')
            
        except Exception as e:
            self.get_logger().error(f'GPS setup error: {e}')
    
    def gps_callback(self, msg):
        """GPS callback with thread-safe data storage"""
        if self.shutdown_requested:
            return
        
        try:
            with self._gps_lock:
                if abs(msg.latitude) > 90 or abs(msg.longitude) > 180:
                    return
                
                self.latest_gps_lat = msg.latitude
                self.latest_gps_lon = msg.longitude
                self.latest_gps_status = msg.status.status
                self.gps_data_available = True
                
        except Exception as e:
            self.get_logger().error(f'GPS callback error: {e}')
    
    def get_gps_data(self):
        """Thread-safe GPS data retrieval"""
        with self._gps_lock:
            return {
                'lat': self.latest_gps_lat,
                'lon': self.latest_gps_lon,
                'status': self.latest_gps_status,
                'available': self.gps_data_available,
                'topic': self.current_gps_topic
            }
    
    def send_cmd_vel(self, linear, angular):
        """Send velocity command with Ackermann constraints and LiDAR safety (thread-safe)"""
        if self.shutdown_requested:
            return
        
        with self._lock:
            try:
                # Apply Ackermann constraints first
                linear, angular = self.ackermann.calculate_safe_command(linear, angular, debug=False)
                
                # Apply LiDAR safety if enabled and available
                if self.enable_lidar_safety and self.lidar_safety:
                    try:
                        safe, mod_linear, mod_angular = self.lidar_safety.is_safe_to_move(linear, angular)
                        
                        if not safe:
                            # Movement blocked by obstacle - EMERGENCY STOP
                            self.get_logger().warn('🚨 EMERGENCY STOP: LiDAR safety blocked movement')
                            linear = 0.0
                            angular = 0.0
                        else:
                            # Use modified commands (may be slowed down)
                            linear = mod_linear
                            angular = mod_angular
                            
                    except Exception as e:
                        self.get_logger().error(f'LiDAR safety check failed: {e}')
                        # Continue with original commands but log the error
                
                # Send the command
                msg = Twist()
                msg.linear.x = float(linear)
                msg.angular.z = float(angular)
                self.cmd_vel_pub.publish(msg)
                self.last_command_time = time.time()
                
                # Log if movement was modified
                if self.enable_lidar_safety and self.lidar_safety:
                    obstacle_info = self.lidar_safety.get_obstacle_info()
                    if obstacle_info['detected'] or obstacle_info['warning']:
                        self.get_logger().debug(f'LiDAR safety active: {obstacle_info}')
                
            except Exception as e:
                self.get_logger().error(f'Command send error: {e}')
    
    def safety_check(self):
        """Safety check - stop if no commands received"""
        if self.shutdown_requested:
            return
        
        try:
            if self.manual_mode:
                if time.time() - self.last_command_time > 2.0:
                    with self._lock:
                        stop_msg = Twist()
                        self.cmd_vel_pub.publish(stop_msg)
        except Exception as e:
            self.get_logger().error(f'Safety check error: {e}')
    
    def call_emergency_stop(self):
        """Call emergency stop (thread-safe)"""
        with self._lock:
            try:
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                self.get_logger().warn('🚨 Emergency stop command sent')
            except Exception as e:
                self.get_logger().error(f'Emergency stop error: {e}')
    
    def set_lidar_safety_distance(self, distance):
        """Update LiDAR safety distance"""
        if self.lidar_safety:
            self.lidar_safety.set_safety_distance(distance)
            self.get_logger().info(f'Safety distance updated to {distance:.2f}m')
        else:
            self.get_logger().warn('LiDAR safety not available')
    
    def set_lidar_safety_enabled(self, enabled):
        """Enable/disable LiDAR safety"""
        if self.lidar_safety:
            self.lidar_safety.set_enabled(enabled)
            self.enable_lidar_safety = enabled
            self.get_logger().info(f'LiDAR safety {"enabled" if enabled else "disabled"}')

            # Clear any pending safety states when disable
            if not enabled:
                self._clear_safety_state()

        else:
            self.get_logger().warn('LiDAR safety not available')
    
    def _clear_safety_state(self):
        """Clear safety state when disabling to prevent jerky movement"""
        if hasattr(self, '_last_safety_result'):
            delattr(self, '_last_safety_result')
        if hasattr(self, '_last_safety_check'):
            delattr(self, '_last_safety_check')
    
    def get_lidar_safety_info(self):
        """Get current LiDAR safety status"""
        if self.lidar_safety:
            try:
                return self.lidar_safety.get_obstacle_info()
            except Exception as e:
                self.get_logger().error(f'Error getting LiDAR info: {e}')
                return {
                    'detected': False,
                    'distance': float('inf'),
                    'angle_deg': 0.0,
                    'warning': False,
                    'enabled': False,
                    'data_age': float('inf'),
                    'data_stale': True
                }
        else:
            # Return safe defaults when LiDAR disabled
            return {
                'detected': False,
                'distance': float('inf'),
                'angle_deg': 0.0,
                'warning': False,
                'enabled': False,
                'data_age': float('inf'),
                'data_stale': True
            }
    
    def get_lidar_status_summary(self):
        """Get a summary of LiDAR system status for debugging"""
        status = {
            'safety_enabled': self.enable_lidar_safety,
            'monitor_available': self.lidar_safety is not None,
            'subscription_active': self.lidar_subscription is not None,
            'has_recent_data': False,
            'error_state': False
        }
        
        if self.lidar_safety:
            try:
                status['has_recent_data'] = self.lidar_safety.has_recent_data()
            except:
                status['error_state'] = True
        
        return status
    
    def cleanup(self):
        """Safe cleanup when shutting down"""
        self.get_logger().info('Cleaning up RoverROSInterface...')
        self.shutdown_requested = True
        
        with self._lock:
            try:
                # Send stop command
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                # Cancel timer
                if hasattr(self, 'safety_timer'):
                    self.safety_timer.cancel()
                
                # Cleanup LiDAR subscription
                if self.lidar_subscription:
                    self.destroy_subscription(self.lidar_subscription)
                
                # Small delay to let final message send
                time.sleep(0.1)
                
                self.get_logger().info('Cleanup completed')
            except Exception as e:
                self.get_logger().error(f'Cleanup error: {e}')
