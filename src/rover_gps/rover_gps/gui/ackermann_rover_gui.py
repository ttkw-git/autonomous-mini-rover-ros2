#!/usr/bin/env python3
"""
Complete Ackermann Rover Control GUI - Based on working rover_control_gui_fixed_v2
Integrates Ackermann steering constraints with ALL existing features
"""

import sys
import time
import math
from datetime import datetime, timezone
import threading
import signal
import atexit
import subprocess
import os
import serial
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# Import the GPS waypoint widget if available
try:
    from gps_waypoint_widget import GPSWaypointWidget
    GPS_WIDGET_AVAILABLE = True
except ImportError:
    print("GPS waypoint widget not available - creating placeholder")
    GPS_WIDGET_AVAILABLE = False

# ROS2 imports with error handling
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import NavSatFix, Imu
    from std_srvs.srv import Trigger
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"ROS2 imports failed: {e}")
    print("Please ensure ROS2 is properly sourced: source /opt/ros/humble/setup.bash")
    ROS_AVAILABLE = False


class AckermannMovementHandler:
    """Handles Ackermann steering constraints for proper rover control"""
    def __init__(self):
        # Ackermann chassis parameters
        self.wheelbase = 0.213  # meters (from HiWonder specs)
        self.max_steering_angle = math.radians(36)  # 36 degrees max
        self.max_linear = 0.6  # m/s (±0.6 range from docs)
        self.min_linear_for_steering = 0.15  # Minimum speed to steer effectively
        
    def calculate_safe_command(self, linear, angular):
        """
        Convert desired linear/angular to Ackermann-safe commands
        
        Args:
            linear: Desired linear velocity (m/s)
            angular: Desired angular velocity (rad/s)
            
        Returns:
            (safe_linear, safe_angular): Constrained velocities
        """
        # Clamp linear velocity
        safe_linear = max(-self.max_linear, min(self.max_linear, linear))
        
        # If not moving forward, don't allow turning
        if abs(safe_linear) < 0.01:
            return (0.0, 0.0)
        
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
            
        return (safe_linear, safe_angular)
    
    def get_steering_angle_deg(self, angular, linear):
        """Get current steering angle in degrees for display"""
        if abs(linear) < 0.01:
            return 0.0
        steering_angle = math.atan(angular * self.wheelbase / linear)
        return math.degrees(steering_angle)


class DirectGPSHandler:
    """Direct GPS handler that replaces ROS2 NMEA driver"""
    
    def __init__(self, port='/dev/ttyUSB0', baud=9600):
        self.port = port
        self.baud = baud
        self.running = False
        self.thread = None
        self.buffer = ""
        
        # GPS data - compatible with existing GUI code
        self.latest_gps_lat = 0.0
        self.latest_gps_lon = 0.0
        self.latest_gps_status = -1
        self.gps_data_available = False
        self.gps_fix_count = 0
        self._gps_lock = threading.Lock()
        
        # GUI callback
        self.gui_callback = None
    
    def set_gui_callback(self, callback):
        """Set callback to update GUI"""
        self.gui_callback = callback
    
    def start(self):
        """Start direct GPS reading"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._gps_loop, daemon=True)
            self.thread.start()
            print("Direct GPS handler started")
            return True
        return False
    
    def stop(self):
        """Stop GPS reading"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        print("Direct GPS handler stopped")
    
    def _gps_loop(self):
        """Main GPS reading loop - based on your working fixed_gps_reader.py"""
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                print(f"Connected to GPS on {self.port}")
                
                # Clear buffer and sync
                ser.reset_input_buffer()
                time.sleep(0.5)
                
                while self.running:
                    try:
                        # Read byte by byte for proper synchronization
                        char = ser.read(1).decode('ascii', errors='ignore')
                        
                        if char:
                            self.buffer += char
                            
                            # Complete sentence found
                            if char == '\n' or char == '\r':
                                sentence = self.buffer.strip()
                                
                                if sentence.startswith('$') and '*' in sentence:
                                    self._process_sentence(sentence)
                                elif sentence and not sentence.startswith('$'):
                                    # Discard fragments - this is what was causing NaN
                                    pass  # Just ignore fragments
                                
                                self.buffer = ""
                    except Exception as e:
                        if self.running:
                            print(f"GPS read error: {e}")
                        time.sleep(0.1)
                        
        except Exception as e:
            print(f"GPS connection error: {e}")
    
    def _process_sentence(self, sentence):
        """Process complete NMEA sentence"""
        try:
            # Validate checksum
            if '*' in sentence:
                data, checksum_received = sentence.split('*')
                checksum_received = checksum_received.strip()
                
                # Calculate checksum
                checksum_calc = 0
                for char in data[1:]:  # Skip $
                    checksum_calc ^= ord(char)
                checksum_expected = f"{checksum_calc:02X}"
                
                if checksum_received.upper() == checksum_expected:
                    self._parse_gps_data(sentence)
                else:
                    print(f"Checksum mismatch: {sentence[:50]}...")
            else:
                print(f"No checksum found: {sentence[:50]}...")
                
        except Exception as e:
            print(f"Sentence processing error: {e}")
    
    def _parse_gps_data(self, sentence):
        """Parse GPS coordinates from valid sentence"""
        try:
            fields = sentence.split(',')
            
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                if len(fields) >= 6:
                    lat_raw = fields[2]
                    lat_dir = fields[3]
                    lon_raw = fields[4]
                    lon_dir = fields[5]
                    quality = fields[6] if len(fields) > 6 else '0'
                    
                    if lat_raw and lon_raw and lat_raw != '0' and lon_raw != '0':
                        # Convert DDMM.MMMM to decimal degrees
                        lat_deg = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                        lon_deg = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                        
                        if lat_dir == 'S':
                            lat_deg = -lat_deg
                        if lon_dir == 'W':
                            lon_deg = -lon_deg
                        
                        # Store GPS data thread-safely
                        with self._gps_lock:
                            self.latest_gps_lat = lat_deg
                            self.latest_gps_lon = lon_deg
                            self.latest_gps_status = int(quality) if quality.isdigit() else 0
                            self.gps_data_available = True
                            self.gps_fix_count += 1
                        
                        # Update GUI
                        if self.gui_callback:
                            self.gui_callback(lat_deg, lon_deg, int(quality) if quality.isdigit() else 0)
                        
                        # Progress report
                        if self.gps_fix_count % 10 == 0:
                            print(f"GPS Fix #{self.gps_fix_count}: {lat_deg:.6f}, {lon_deg:.6f}")
                            
        except (ValueError, IndexError) as e:
            print(f"GPS parsing error: {e}")
    
    def get_gps_data(self):
        """Get current GPS data - thread safe"""
        with self._gps_lock:
            return {
                'lat': self.latest_gps_lat,
                'lon': self.latest_gps_lon,
                'status': self.latest_gps_status,
                'available': self.gps_data_available,
                'topic': 'direct_gps'
            }


class SafetyManager:
    """Global safety manager to ensure rover stops on any failure"""
    def __init__(self):
        self.ros_node = None
        self.emergency_active = False
        
    def set_ros_node(self, node):
        self.ros_node = node
        
    def emergency_stop(self):
        """Emergency stop from any thread"""
        self.emergency_active = True
        if self.ros_node:
            try:
                stop_msg = Twist()
                self.ros_node.cmd_vel_pub.publish(stop_msg)
                print("Emergency stop activated!")
            except Exception as e:
                print(f"Emergency stop error: {e}")
                
    def cleanup(self):
        """Cleanup on exit"""
        self.emergency_stop()
        if self.ros_node:
            try:
                self.ros_node.destroy_node()
            except Exception as e:
                print(f"Cleanup error: {e}")

# Global safety manager
safety_manager = SafetyManager()


class RoverROSInterface(Node):
    def __init__(self, use_ackermann=True):
        super().__init__('rover_gui_interface')
        
        # Ackermann handler
        self.use_ackermann = use_ackermann
        self.ackermann = AckermannMovementHandler()
        
        # GPS data storage attributes
        self.latest_gps_lat = 0.0
        self.latest_gps_lon = 0.0
        self.latest_gps_status = -1
        self.gps_data_available = False
        self._gps_lock = threading.Lock()  # Thread safety for GPS data
        
        # Publishers - using correct topic name from documentation
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.get_logger().info('Created cmd_vel publisher on topic: /controller/cmd_vel')
        
        # GPS Subscribers - Try multiple topics based on your project
        self.gps_topics_to_try = ['/fix', '/gps/fix', '/nmea_fix', '/gps_fix']
        self.gps_sub = None
        self.current_gps_topic = None
        
        # Try to find working GPS topic
        self.setup_gps_subscription()
        
        # Safety timer
        self.last_command_time = time.time()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        # GPS status tracking
        self.gps_fix_count = 0
        self.last_gps_time = 0
        self.manual_mode = True
        
        self.get_logger().info(f'Rover ROS Interface initialized (Ackermann mode: {use_ackermann})')
    
    def setup_gps_subscription(self):
        """Try to find and subscribe to available GPS topic"""
        try:
            # Get available topics
            topic_list = self.get_topic_names_and_types()
            available_topics = [topic_name for topic_name, _ in topic_list]
            
            self.get_logger().info(f'Found {len(available_topics)} available topics')
            
            # Try each GPS topic
            for topic in self.gps_topics_to_try:
                if topic in available_topics:
                    try:
                        self.gps_sub = self.create_subscription(
                            NavSatFix, topic, self.gps_callback, 10)
                        self.current_gps_topic = topic
                        self.get_logger().info(f'Successfully subscribed to GPS topic: {topic}')
                        return
                    except Exception as e:
                        self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
            
            # If no GPS topic found
            self.get_logger().warn('No GPS topic found. GPS functionality disabled.')
            
            # Create a timer to keep retrying GPS connection
            self.gps_retry_timer = self.create_timer(5.0, self.retry_gps_connection)
            
        except Exception as e:
            self.get_logger().error(f'GPS setup error: {e}')
    
    def retry_gps_connection(self):
        """Retry GPS connection periodically"""
        if self.gps_sub is None:
            self.get_logger().info('Retrying GPS connection...')
            self.setup_gps_subscription()
            
    def gps_callback(self, msg):
        """GPS callback with thread-safe data storage"""
        try:
            with self._gps_lock:  # Thread-safe GPS data access
                self.gps_fix_count += 1
                self.last_gps_time = time.time()
                
                # Validate GPS data
                if abs(msg.latitude) > 90 or abs(msg.longitude) > 180:
                    self.get_logger().warn(f'Invalid GPS coordinates: {msg.latitude}, {msg.longitude}')
                    return
                
                # Store GPS data
                self.latest_gps_lat = msg.latitude
                self.latest_gps_lon = msg.longitude  
                self.latest_gps_status = msg.status.status
                self.gps_data_available = True
                
                # Debug logging (every 10 fixes)
                if self.gps_fix_count % 10 == 0:
                    self.get_logger().info(
                        f'GPS Fix #{self.gps_fix_count}: '
                        f'Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, '
                        f'Status={msg.status.status}'
                    )
                    
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
        """Send velocity command to rover with optional Ackermann constraints"""
        try:
            # Apply Ackermann constraints if enabled
            if self.use_ackermann:
                linear, angular = self.ackermann.calculate_safe_command(linear, angular)
            
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.cmd_vel_pub.publish(msg)
            self.last_command_time = time.time()
            
            # Log significant commands
            if abs(linear) > 0.1 or abs(angular) > 0.1:
                self.get_logger().debug(f'Sent cmd_vel: linear={linear:.2f}, angular={angular:.2f}')
                
        except Exception as e:
            self.get_logger().error(f'Command send error: {e}')
            safety_manager.emergency_stop()
            
    def safety_check(self):
        """Safety check - stop if no commands received"""
        try:
            if self.manual_mode:
                if time.time() - self.last_command_time > 2.0:
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
        except Exception as e:
            self.get_logger().error(f'Safety check error: {e}')
            
    def call_emergency_stop(self):
        """Call emergency stop"""
        try:
            # Always send direct stop command
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().warn('Emergency stop command sent')
        except Exception as e:
            self.get_logger().error(f'Emergency stop error: {e}')


class VirtualJoystick(QWidget):
    position_changed = pyqtSignal(float, float)
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 200)
        self.center_x = 100
        self.center_y = 100
        self.knob_x = 100
        self.knob_y = 100
        self.dragging = False
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw outer circle
        painter.setBrush(QBrush(QColor(200, 200, 200)))
        painter.drawEllipse(20, 20, 160, 160)
        
        # Draw center point
        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.drawEllipse(95, 95, 10, 10)
        
        # Draw knob
        painter.setBrush(QBrush(QColor(50, 150, 250)))
        painter.drawEllipse(int(self.knob_x - 15), int(self.knob_y - 15), 30, 30)
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.update_knob_position(event.x(), event.y())
            
    def mouseMoveEvent(self, event):
        if self.dragging:
            self.update_knob_position(event.x(), event.y())
            
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self.knob_x = self.center_x
            self.knob_y = self.center_y
            self.update()
            self.position_changed.emit(0.0, 0.0)
            
    def update_knob_position(self, x, y):
        # Constrain to circle
        dx = x - self.center_x
        dy = y - self.center_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > 80:  # Limit to circle radius
            dx = dx / distance * 80
            dy = dy / distance * 80
            
        self.knob_x = self.center_x + dx
        self.knob_y = self.center_y + dy
        
        # Emit normalized position (-1 to 1)
        norm_x = dx / 80.0
        norm_y = -dy / 80.0  # Invert Y axis
        
        self.update()
        self.position_changed.emit(norm_x, norm_y)


class RoverControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover GPS Navigation Control - Ackermann Version")
        self.setGeometry(100, 100, 900, 700)
        
        # Check ROS availability
        if not ROS_AVAILABLE:
            QMessageBox.critical(self, "ROS2 Error", 
                               "ROS2 is not available. Please source ROS2 environment:\n"
                               "source /opt/ros/humble/setup.bash")
            sys.exit(1)
        
        # ROS2 node
        self.ros_node = None
        self.ros_thread = None
        self.ros_executor = None
        
        # Control state
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.emergency_stopped = False
        self.current_mode = 'MANUAL'
        
        # Movement parameters
        self.max_speed = 0.3
        self.max_turn_rate = 0.8
        
        # GPS status
        self.last_gps_lat = 0.0
        self.last_gps_lon = 0.0
        self.gps_status = -1
        
        # Direct GPS handler
        self.direct_gps = None
        
        # Safety measures
        self.setFocusPolicy(Qt.StrongFocus)
        
        self.init_ui()
        
        # Initialize ROS after UI
        self.init_ros()
        
        # GPS checking timer
        self.gps_check_timer = QTimer()
        self.gps_check_timer.timeout.connect(self.check_gps_data)
        self.gps_check_timer.start(1000)  # Check every second
        
        # Register safety cleanup
        atexit.register(self.cleanup_on_exit)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def init_direct_gps(self):
        """Initialize direct GPS handler"""
        self.direct_gps = DirectGPSHandler()
        self.direct_gps.set_gui_callback(self.on_direct_gps_update)

    def on_direct_gps_update(self, lat, lon, status):
        """Handle GPS update from direct GPS handler"""
        try:
            self.last_gps_lat = lat
            self.last_gps_lon = lon
            self.gps_status = status
        except Exception as e:
            print(f"GPS update error: {e}")

    def signal_handler(self, signum, frame):
        """Handle system signals"""
        print(f"Received signal {signum}, cleaning up...")
        self.cleanup_on_exit()
        sys.exit(0)
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left panel - Control
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, 1)
        
        # Right panel - Status
        status_panel = self.create_status_panel()
        main_layout.addWidget(status_panel, 1)
        
    def create_control_panel(self):
        group = QGroupBox("Rover Control")
        layout = QVBoxLayout()
        
        # Ackermann Mode Toggle
        ackermann_group = QGroupBox("Steering Mode")
        ackermann_layout = QVBoxLayout()
        self.ackermann_checkbox = QCheckBox("Enable Ackermann Constraints (±36° limit)")
        self.ackermann_checkbox.setChecked(True)
        self.ackermann_checkbox.stateChanged.connect(self.toggle_ackermann_mode)
        ackermann_layout.addWidget(self.ackermann_checkbox)
        ackermann_info = QLabel("When enabled: steering limited to ±36°, minimum speed for turns")
        ackermann_info.setStyleSheet("font-size: 10px; color: gray;")
        ackermann_layout.addWidget(ackermann_info)
        ackermann_group.setLayout(ackermann_layout)
        layout.addWidget(ackermann_group)
        
        # Emergency stop button
        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 15px;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #c82333;
            }
        """)
        self.emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_btn)
        
        self.resume_btn = QPushButton("Resume Control")
        self.resume_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 10px;
                border: none;
                border-radius: 8px;
            }
        """)
        self.resume_btn.clicked.connect(self.resume_control)
        self.resume_btn.setEnabled(False)
        layout.addWidget(self.resume_btn)

        # Speed control
        speed_group = QGroupBox("Speed Control")
        speed_layout = QVBoxLayout()
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.update_max_speed)
        
        self.speed_label = QLabel("Speed: 50%")
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        # Virtual joystick
        joystick_group = QGroupBox("Virtual Joystick")
        joystick_layout = QVBoxLayout()
        
        self.joystick = VirtualJoystick()
        self.joystick.position_changed.connect(self.joystick_moved)
        joystick_layout.addWidget(self.joystick)
        
        joystick_group.setLayout(joystick_layout)
        layout.addWidget(joystick_group)
        
        # GPS Control section
        gps_control_group = QGroupBox("GPS Control")
        gps_control_layout = QVBoxLayout()
        
        self.start_gps_btn = QPushButton("Start GPS Node")
        self.start_gps_btn.clicked.connect(self.start_gps_node)
        self.start_gps_btn.setStyleSheet("background-color: #007bff; color: white; padding: 8px; font-weight: bold;")
        gps_control_layout.addWidget(self.start_gps_btn)
        
        self.stop_gps_btn = QPushButton("Stop GPS Node")
        self.stop_gps_btn.clicked.connect(self.stop_gps_node)
        self.stop_gps_btn.setStyleSheet("background-color: #6c757d; color: white; padding: 8px;")
        gps_control_layout.addWidget(self.stop_gps_btn)
        
        self.gps_status_label = QLabel("GPS Node: Not started")
        self.gps_status_label.setStyleSheet("background-color: #fff3cd; padding: 5px; border-radius: 3px; border: 1px solid #ffeaa7;")
        gps_control_layout.addWidget(self.gps_status_label)
        
        gps_control_group.setLayout(gps_control_layout)
        layout.addWidget(gps_control_group)
        
        # ROS Status
        ros_status_group = QGroupBox("ROS2 Status")
        ros_status_layout = QVBoxLayout()
        
        self.ros_status_label = QLabel("ROS2: Initializing...")
        self.ros_status_label.setStyleSheet("background-color: #fff3cd; padding: 5px; border-radius: 3px;")
        ros_status_layout.addWidget(self.ros_status_label)
        
        ros_status_group.setLayout(ros_status_layout)
        layout.addWidget(ros_status_group)
        
        # Keyboard controls info
        keyboard_info = QLabel("""Keyboard Controls:
W/S - Forward/Backward  
A/D - Turn Left/Right
SPACE - Stop
ESC - Emergency Stop

SAFETY Features:
• Auto-stop if no commands for 2 seconds
• Emergency stop on GUI close
• Thread-safe GPS data handling
• Ackermann steering constraints
• Real-time status updates""")
        keyboard_info.setStyleSheet("background-color: #f0f8ff; padding: 10px; border-radius: 5px; font-size: 11px;")
        layout.addWidget(keyboard_info)
        
        group.setLayout(layout)
        return group
        
    def create_status_panel(self):
        group = QGroupBox("Status & GPS Information")
        layout = QVBoxLayout()
        
        # Current status
        self.status_label = QLabel("Status: Manual Mode - Initializing")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #fff3cd; border-radius: 5px;")
        layout.addWidget(self.status_label)
        
        # GPS Position section
        gps_group = QGroupBox("GPS Position & Status")
        gps_layout = QVBoxLayout()
        
        self.current_gps_label = QLabel("GPS: Waiting for connection...")
        self.current_gps_label.setStyleSheet("font-family: monospace; background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        gps_layout.addWidget(self.current_gps_label)
        
        # GPS connection status
        self.gps_connection_label = QLabel("Connection: Not connected")
        self.gps_connection_label.setStyleSheet("font-family: monospace; background-color: #ffebee; padding: 5px; border-radius: 3px; color: red;")
        gps_layout.addWidget(self.gps_connection_label)
        
        # GPS topic info
        self.gps_topic_label = QLabel("Topic: Searching...")
        self.gps_topic_label.setStyleSheet("font-family: monospace; font-size: 10px; color: gray;")
        gps_layout.addWidget(self.gps_topic_label)
        
        gps_group.setLayout(gps_layout)
        layout.addWidget(gps_group)

        # GPS Waypoint Recording Widget (only if available)
        if GPS_WIDGET_AVAILABLE:
            waypoint_group = QGroupBox("GPS Waypoint Recording")
            waypoint_layout = QVBoxLayout()

            self.waypoint_widget = GPSWaypointWidget(parent_gui=self)
            waypoint_layout.addWidget(self.waypoint_widget)

            self.waypoint_widget.parent_gui = self

            waypoint_group.setLayout(waypoint_layout)
            layout.addWidget(waypoint_group)
        else:
            # Create placeholder
            waypoint_group = QGroupBox("GPS Waypoint Recording")
            waypoint_layout = QVBoxLayout()
            placeholder = QLabel("GPS waypoint widget not available.\nCreate gps_waypoint_widget.py to enable this feature.")
            placeholder.setStyleSheet("background-color: #fff3cd; padding: 10px; border-radius: 5px;")
            waypoint_layout.addWidget(placeholder)
            waypoint_group.setLayout(waypoint_layout)
            layout.addWidget(waypoint_group)
        
        # Current movement display
        movement_group = QGroupBox("Current Movement")
        movement_layout = QVBoxLayout()
        
        self.movement_label = QLabel("Linear: 0.0 m/s\nAngular: 0.0 rad/s")
        self.movement_label.setStyleSheet("font-family: monospace; background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        movement_layout.addWidget(self.movement_label)
        
        movement_group.setLayout(movement_layout)
        layout.addWidget(movement_group)
        
        # GPS target input
        target_group = QGroupBox("GPS Navigation Target")
        target_layout = QVBoxLayout()
        
        # Quick location buttons
        location_buttons = QHBoxLayout()
        current_btn = QPushButton("Use Current")
        current_btn.clicked.connect(self.use_current_location)
        current_btn.setToolTip("Use current GPS coordinates as target")
        location_buttons.addWidget(current_btn)
        
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_target)
        location_buttons.addWidget(clear_btn)
        target_layout.addLayout(location_buttons)
        
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("Latitude (e.g., 49.891364)")
        self.lon_input = QLineEdit()
        self.lon_input.setPlaceholderText("Longitude (e.g., -97.153644)")
        
        self.set_target_btn = QPushButton("Navigate to GPS Coordinates")
        self.set_target_btn.clicked.connect(self.set_gps_target)
        self.set_target_btn.setEnabled(False)
        self.set_target_btn.setStyleSheet("background-color: #28a745; color: white; padding: 8px; font-weight: bold;")
        
        target_layout.addWidget(QLabel("Target Latitude:"))
        target_layout.addWidget(self.lat_input)
        target_layout.addWidget(QLabel("Target Longitude:"))
        target_layout.addWidget(self.lon_input)
        target_layout.addWidget(self.set_target_btn)
        
        load_waypoints_btn = QPushButton("Load Recorded Path")
        load_waypoints_btn.clicked.connect(self.load_waypoint_file_for_nav)
        load_waypoints_btn.setStyleSheet("background-color: #6f42c1; color: white; padding: 8px; font-weight: bold;")
        target_layout.addWidget(load_waypoints_btn) 

        # Target status
        self.target_status_label = QLabel("No target set")
        self.target_status_label.setStyleSheet("font-size: 10px; color: gray; padding: 5px;")
        target_layout.addWidget(self.target_status_label)
        
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        
        group.setLayout(layout)
        return group
    
    def toggle_ackermann_mode(self, state):
        """Toggle Ackermann mode on/off"""
        if self.ros_node:
            self.ros_node.use_ackermann = (state == Qt.Checked)
            mode_text = "enabled" if state == Qt.Checked else "disabled"
            print(f"Ackermann constraints {mode_text}")
        
    def init_ros(self):
        """Initialize ROS2 node in separate thread"""
        try:
            self.ros_thread = threading.Thread(target=self.ros_worker, daemon=True)
            self.ros_thread.start()
            print("ROS2 thread started")
        except Exception as e:
            print(f"Failed to start ROS thread: {e}")
            self.ros_status_label.setText("ROS2: Failed to initialize")
            self.ros_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;")
        
    def ros_worker(self):
        """ROS2 worker thread"""
        try:
            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # Create node with Ackermann enabled by default
            self.ros_node = RoverROSInterface(use_ackermann=True)
            
            # Set manual mode flag
            self.ros_node.manual_mode = True
            
            # Register with safety manager
            safety_manager.set_ros_node(self.ros_node)
            
            print("ROS2 node started successfully")
            
            # Update GUI status
            QTimer.singleShot(0, lambda: self.ros_status_label.setText("ROS2: Connected"))
            QTimer.singleShot(0, lambda: self.ros_status_label.setStyleSheet("background-color: #d4edda; padding: 5px; border-radius: 3px;"))
            QTimer.singleShot(0, lambda: self.status_label.setText("Status: Manual Mode - Ready"))
            QTimer.singleShot(0, lambda: self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #d4edda; border-radius: 5px;"))
            
            # Spin the node
            rclpy.spin(self.ros_node)
            
        except Exception as e:
            print(f"ROS thread error: {e}")
            QTimer.singleShot(0, lambda: self.ros_status_label.setText(f"ROS2: Error - {str(e)[:50]}"))
            QTimer.singleShot(0, lambda: self.ros_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;"))
            safety_manager.emergency_stop()
    
    def check_gps_data(self):
        """Timer method to check for GPS data updates"""
        try:
            # First check direct GPS if available
            if hasattr(self, 'direct_gps') and self.direct_gps:
                gps_data = self.direct_gps.get_gps_data()
                
                if gps_data['available'] and gps_data['lat'] != 0.0:
                    lat = gps_data['lat']
                    lon = gps_data['lon']
                    status = gps_data['status']
                    
                    # Validate data
                    if abs(lat) <= 90 and abs(lon) <= 180:
                        # Update GPS display
                        self.current_gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}")
                        self.current_gps_label.setStyleSheet(
                            "font-family: monospace; background-color: #d4edda; padding: 10px; border-radius: 5px;"
                        )
                        
                        # Update connection status
                        self.gps_connection_label.setText("Connection: DIRECT GPS ACTIVE")
                        self.gps_connection_label.setStyleSheet(
                            "font-family: monospace; background-color: #d4edda; padding: 5px; border-radius: 3px; color: green;"
                        )
                        self.set_target_btn.setEnabled(True)
                        
                        self.gps_topic_label.setText("Topic: Direct Serial GPS")
                        
                        # Store for waypoint recording
                        self.last_gps_lat = lat
                        self.last_gps_lon = lon
                        self.gps_status = status
                        return
            
            # Fallback to ROS GPS
            if not self.ros_node:
                return
            
            # Get GPS data thread-safely from ROS
            gps_data = self.ros_node.get_gps_data()
            
            if not gps_data['available']:
                return
            
            lat = gps_data['lat']
            lon = gps_data['lon']
            status = gps_data['status']
            topic = gps_data['topic']
            
            # Validate GPS data
            if lat == 0.0 and lon == 0.0:
                return
            
            if abs(lat) > 90 or abs(lon) > 180:
                return
            
            # Update GPS display
            self.current_gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}")
            self.current_gps_label.setStyleSheet(
                "font-family: monospace; background-color: #d4edda; padding: 10px; border-radius: 5px;"
            )
            
            # Update connection status
            if status >= 0:
                self.gps_connection_label.setText("Connection: ACTIVE (GPS FIX)")
                self.gps_connection_label.setStyleSheet(
                    "font-family: monospace; background-color: #d4edda; padding: 5px; border-radius: 3px; color: green;"
                )
                self.set_target_btn.setEnabled(True)
            else:
                self.gps_connection_label.setText("Connection: ACTIVE (No Fix)")
                self.gps_connection_label.setStyleSheet(
                    "font-family: monospace; background-color: #fff3cd; padding: 5px; border-radius: 3px; color: orange;"
                )
            
            # Update GPS topic info
            if topic:
                self.gps_topic_label.setText(f"Topic: {topic}")
            
            # Store coordinates
            self.last_gps_lat = lat
            self.last_gps_lon = lon
            self.gps_status = status
            
        except Exception as e:
            print(f"GPS check error: {e}")
    
    def update_max_speed(self, value):
        """Update maximum speed based on slider"""
        self.max_speed = (value / 100.0) * 1.0  # 0.1 to 1.0 m/s
        self.max_turn_rate = (value / 100.0) * 2.0  # 0.2 to 2.0 rad/s
        self.speed_label.setText(f"Speed: {value}% (Linear: {self.max_speed:.2f} m/s)")
        print(f"Speed updated: {value}% -> {self.max_speed:.2f} m/s, {self.max_turn_rate:.2f} rad/s")
    
    def keyPressEvent(self, event):
        """Handle keyboard input"""
        if self.emergency_stopped:
            return
            
        if event.key() == Qt.Key_W:
            self.send_movement_command(self.max_speed, 0.0)
        elif event.key() == Qt.Key_S:
            self.send_movement_command(-self.max_speed, 0.0)
        elif event.key() == Qt.Key_A:
            self.send_movement_command(0.0, self.max_turn_rate)
        elif event.key() == Qt.Key_D:
            self.send_movement_command(0.0, -self.max_turn_rate)
        elif event.key() == Qt.Key_Space:
            self.stop_movement()
        elif event.key() == Qt.Key_Escape:
            self.emergency_stop()
            
    def keyReleaseEvent(self, event):
        """Stop movement when key is released"""
        if event.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            self.stop_movement()
            
    def joystick_moved(self, x, y):
        """Handle virtual joystick movement"""
        if self.emergency_stopped:
            return
            
        # Convert joystick coordinates to movement commands
        linear = y * self.max_speed
        angular = -x * self.max_turn_rate

        if abs(x) < 0.1:
            angular = 0.0
        if abs(y) < 0.1:
            linear = 0.0
                
        self.send_movement_command(linear, angular)

    def send_movement_command(self, linear, angular):
        """Send movement command to rover"""
        if self.emergency_stopped:
            return
            
        self.current_linear = linear
        self.current_angular = angular
        
        if self.ros_node:
            try:
                self.ros_node.send_cmd_vel(linear, angular)
            except Exception as e:
                print(f"Command send error: {e}")
                self.emergency_stop()
                
        self.update_movement_display()
        
    def stop_movement(self):
        """Stop all movement"""
        self.send_movement_command(0.0, 0.0)
    
    def resume_control(self):
        """Resume control after emergency stop"""
        self.emergency_stopped = False

        if self.ros_node:
            self.ros_node.send_cmd_vel(0.0, 0.0)

        self.status_label.setText("Status: Manual Mode - Control Resumed")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #d4edda; border-radius: 5px;")
        self.resume_btn.setEnabled(False)
        self.emergency_btn.setEnabled(True)
    
    def emergency_stop(self):
        """Emergency stop"""
        self.emergency_stopped = True
        self.stop_movement()
        
        if self.ros_node:
            try:
                self.ros_node.call_emergency_stop()
            except:
                pass
                
        self.status_label.setText("Status: EMERGENCY STOP ACTIVATED")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #dc3545; color: white; border-radius: 5px;")
        self.resume_btn.setEnabled(True)
        self.emergency_btn.setEnabled(False)

    def start_gps_node(self):
        """Start GPS node with proper buffer clearing"""
        try:
            # Kill any existing GPS process first
            subprocess.run(["pkill", "-f", "nmea_serial_driver"], check=False)
            time.sleep(1)

            # Initialize direct GPS if not already done
            if not hasattr(self, 'direct_gps') or not self.direct_gps:
                self.init_direct_gps()
            
            # Start direct GPS reading
            if self.direct_gps.start():
                self.gps_status_label.setText("GPS Node: Direct GPS reader starting...")
                self.gps_status_label.setStyleSheet("background-color: #cce5ff; padding: 5px; border-radius: 3px;")
                self.start_gps_btn.setEnabled(False)
                
                # Check status after a delay
                QTimer.singleShot(3000, lambda: self.check_direct_gps_status())
                QTimer.singleShot(1000, lambda: self.stop_gps_btn.setEnabled(True))
            else:
                self.gps_status_label.setText("GPS Node: Failed to start direct GPS")
                self.gps_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;")
                
        except Exception as e:
            self.gps_status_label.setText(f"GPS Node: Error - {str(e)[:50]}")
            self.gps_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;")
            
    def check_direct_gps_status(self):
        """Check direct GPS status"""
        try:
            if hasattr(self, 'direct_gps') and self.direct_gps:
                gps_data = self.direct_gps.get_gps_data()
                if gps_data['available'] and gps_data['lat'] != 0.0:
                    self.gps_status_label.setText("GPS Node: Direct GPS - Active with fix")
                    self.gps_status_label.setStyleSheet("background-color: #d4edda; padding: 5px; border-radius: 3px;")
                    self.start_gps_btn.setEnabled(True)
                else:
                    self.gps_status_label.setText("GPS Node: Direct GPS - Waiting for fix...")
                    self.gps_status_label.setStyleSheet("background-color: #fff3cd; padding: 5px; border-radius: 3px;")
                    # Keep checking
                    QTimer.singleShot(5000, lambda: self.check_direct_gps_status())
        except Exception as e:
            print(f"GPS status check error: {e}")

    def stop_gps_node(self):
        """Stop GPS node"""
        try:
            # Stop direct GPS
            if hasattr(self, 'direct_gps') and self.direct_gps:
                self.direct_gps.stop()
            
            # Also kill ROS GPS processes
            subprocess.run(["pkill", "-f", "nmea_serial_driver"], check=False)
            
            self.gps_status_label.setText("GPS Node: Stopped")
            self.gps_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;")
            self.stop_gps_btn.setEnabled(False)
            self.start_gps_btn.setEnabled(True)
            
        except Exception as e:
            print(f"GPS stop error: {e}")
    
    def use_current_location(self):
        """Use current GPS location as target"""
        if self.gps_status >= 0 and self.last_gps_lat != 0.0 and self.last_gps_lon != 0.0:
            self.lat_input.setText(f"{self.last_gps_lat:.6f}")
            self.lon_input.setText(f"{self.last_gps_lon:.6f}")
            self.target_status_label.setText(f"Target: Current location ({self.last_gps_lat:.6f}, {self.last_gps_lon:.6f})")
        else:
            QMessageBox.warning(self, "No GPS Fix", "No valid GPS coordinates available to use as target.")
    
    def clear_target(self):
        """Clear GPS target"""
        self.lat_input.clear()
        self.lon_input.clear()
        self.target_status_label.setText("No target set")
    
    def update_movement_display(self):
        """Update movement display"""
        self.movement_label.setText(
            f"Linear: {self.current_linear:.2f} m/s\n"
            f"Angular: {self.current_angular:.2f} rad/s"
        )
    
    def set_gps_target(self):
        """Set GPS target coordinates"""
        try:
            lat = float(self.lat_input.text())
            lon = float(self.lon_input.text())
            
            if abs(lat) > 90 or abs(lon) > 180:
                QMessageBox.warning(self, "Invalid Coordinates", 
                                  "Please enter valid latitude (-90 to 90) and longitude (-180 to 180)")
                return
            
            QMessageBox.information(self, "GPS Target Set", 
                                  f"Target set to:\nLatitude: {lat:.6f}\nLongitude: {lon:.6f}\n\n"
                                  "Note: Autonomous navigation not implemented in this demo.")
            
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numeric coordinates")

    def load_waypoint_file_for_nav(self):
        """Load waypoint for autonomous navigation"""
        try:
            filename, _ = QFileDialog.getOpenFileName(self, "Load Waypoint for Navigation", "", "JSON files (*.json);;All files (*)")

            if not filename:
                return

            import json
            with open(filename, 'r') as f:
                data = json.load(f)

            waypoints = data['waypoints']
            origin = data['origin']

            # Show waypoint info
            QMessageBox.information(self, "Waypoints Loaded",
                                    f"Loaded {len(waypoints)} waypoints for navigation.\n\n"
                                    f"Origin: {origin['latitude']:.6f}, {origin['longitude']:.6f}\n"
                                    f"Total distance: ~{self.calculate_total_distance(waypoints):.1f}m\n\n"
                                    "You can now use these waypoints for autonomous navigation!")

        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to load waypoints: {e}")
    
    def calculate_total_distance(self, waypoints):
        """Calculate total distance of waypoint path"""
        if len(waypoints) < 2:
            return 0.0

        total_distance = 0
        for i in range(1, len(waypoints)):
            wp1 = waypoints[i-1]
            wp2 = waypoints[i]
            dist = ((wp2['x'] - wp1['x'])**2 + (wp2['y'] - wp1['y'])**2)**0.5
            total_distance += dist

        return total_distance

    def closeEvent(self, event):
        """Handle window close event"""
        self.cleanup_on_exit()
        event.accept()
        
    def cleanup_on_exit(self):
        """Cleanup on exit"""
        try:
            print("Cleaning up GUI...")
            self.emergency_stop()
            
            # Stop direct GPS
            if hasattr(self, 'direct_gps') and self.direct_gps:
                self.direct_gps.stop()
            
            # Stop timers
            if hasattr(self, 'gps_check_timer'):
                self.gps_check_timer.stop()
            
            # Kill GPS processes
            subprocess.run(["pkill", "-f", "nmea_serial_driver"], check=False)
            
            # Cleanup ROS
            if self.ros_node:
                try:
                    self.ros_node.destroy_node()
                except:
                    pass
            
            # Shutdown ROS
            if rclpy.ok():
                rclpy.shutdown()
                
            print("Cleanup completed")
            
        except Exception as e:
            print(f"Cleanup error: {e}")


def main():
    """Main function"""
    # Handle command line arguments
    app = QApplication(sys.argv)
    
    # Set application properties
    app.setApplicationName("Rover GPS Control - Ackermann")
    app.setApplicationVersion("2.0")
    app.setOrganizationName("Rover Control Systems")
    
    # Check ROS2 environment
    if not ROS_AVAILABLE:
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setWindowTitle("ROS2 Environment Error")
        msg.setText("ROS2 is not properly configured!")
        msg.setInformativeText(
            "Please run the following commands in your terminal:\n\n"
            "1. Source ROS2 environment:\n"
            "   source /opt/ros/humble/setup.bash\n\n"
            "2. Source your workspace:\n"
            "   source ~/ros2_ws/install/setup.bash\n\n"
            "3. Install required packages:\n"
            "   sudo apt install ros-humble-geometry-msgs\n"
            "   sudo apt install ros-humble-sensor-msgs\n"
            "   sudo apt install python3-pyqt5\n\n"
            "4. Then run this script again."
        )
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()
        sys.exit(1)
    
    try:
        # Create and show the main window
        window = RoverControlGUI()
        window.show()
        
        # Handle Ctrl+C gracefully
        def signal_handler(signum, frame):
            print(f"\nReceived signal {signum}, shutting down gracefully...")
            window.cleanup_on_exit()
            app.quit()
        
        signal.signal(signal.SIGINT, signal_handler)
        
        # Enable Ctrl+C to work
        timer = QTimer()
        timer.start(500)  # Check every 500ms
        timer.timeout.connect(lambda: None)  # Keep the event loop responsive
        
        print("Rover Control GUI started successfully!")
        print("Ackermann steering constraints enabled by default (±36° steering limit)")
        print("Use keyboard controls (W/A/S/D) or the virtual joystick to control the rover.")
        print("Press Ctrl+C or close the window to exit.")
        
        # Run the application
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"Application error: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()