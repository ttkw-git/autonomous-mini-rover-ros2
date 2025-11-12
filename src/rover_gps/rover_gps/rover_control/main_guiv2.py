#!/usr/bin/env python3
"""
Main GUI for Ackermann Rover Control - Auto-stops competing apps on startup
No need to run separate scripts!
"""

import sys
import time
import signal
import atexit
import subprocess
import threading
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

# Import modular components
from .ackermann_handler import AckermannMovementHandler
from .gps_handler import DirectGPSHandler
from .safety_manager import SafetyManager
from .virtual_joystick import VirtualJoystick
from .ros_interface import RoverROSInterface
from .waypoint_navigator import NavigationState, WaypointNavigator

# Import GPS waypoint widget if available
try:
    from .gps_waypoint_widget import GPSWaypointWidget
    GPS_WIDGET_AVAILABLE = True
except ImportError:
    print("GPS waypoint widget not available")
    GPS_WIDGET_AVAILABLE = False

# ROS2 imports
try:
    import rclpy
    from std_srvs.srv import SetBool, Trigger # ADD Trigger for exit services
    from sensor_msgs.msg import NavSatFix
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"ROS2 not available: {e}")
    ROS_AVAILABLE = False

# Global safety manager
safety_manager = SafetyManager()


def stop_competing_apps():
    """
    Stop competing apps automatically when GUI starts
    These apps will be stopped:
    - lidar_app
    - line_following
    - object_tracking
    - hand_gesture
    """
    print("\n" + "=" * 60)
    print("Stopping competing apps...")
    print("=" * 60)
    
    apps_to_stop = [
        'lidar_app',
        'line_following', 
        'object_tracking',
        'hand_gesture'
    ]
    
    # Method 1: Try calling exit services (clean shutdown)
    for app in apps_to_stop:
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', f'/{app}/exit', 'std_srvs/srv/Trigger'],
                capture_output=True,
                timeout=2
            )
            print(f"  Called exit service for {app}")
        except:
            pass  # Service might not exist, that's OK
    
    # Wait a moment for clean shutdown
    time.sleep(1)
    
    # Method 2: Force kill processes
    for app in apps_to_stop:
        try:
            subprocess.run(['pkill', '-9', '-f', app], capture_output=True)
            print(f"  ✓ Stopped {app}")
        except:
            pass
    
    # Wait for cleanup
    time.sleep(2)
    
    print("✓ Competing apps stopped")
    print("✓ Keeping joystick_control for manual override")
    print("=" * 60 + "\n")

class IndoorTestMode:
    """Indoor testing with simulated GPS coordinates"""
    def __init__(self, start_lat=49.8951, start_lon=-97.1384):
        self.current_lat = start_lat
        self.current_lon = start_lon
        self.enabled = False

        # Conversion factor (approximate)
        self.meters_per_deg_lat = 111000
        self.meters_per_deg_log = 70000

    def update_position_from_movement(self, linear_x, angular_z, dt=0.1):
        """
        Update fake GPS based on rover movement commands
        linear_x: forward speed (m/s)
        angular_z: turning rate (rad/s)
        dt: time step (seconds)
        """
        # Calculate distance mvoed
        distance = linear_x * dt

        # Update lat/lon based on heading
        delta_lat = distance / self.meters_per_deg_lat
        self.current_lat += delta_lat

    def get_position(self):
        return self.current_lat, self.current_lon

class RoverControlGUI(QMainWindow):
    """Main GUI window for rover control"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover GPS Navigation Control")
        self.setGeometry(100, 100, 900, 700)
        
        # Check ROS availability
        if not ROS_AVAILABLE:
            QMessageBox.critical(self, "ROS2 Error", 
                               "ROS2 is not available. Please source ROS2 environment")
            sys.exit(1)
        
        # Initialize components
        self.ros_node = None
        self.ros_thread = None
        self.direct_gps = None
        self.navigator = WaypointNavigator(waypoint_tolerance=0.3)

        # Indoor test Mode
        self.indoor_test_mode = False
        self.fake_gps_subscription = None
        
        # Control state
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.emergency_stopped = False
        self.max_speed = 0.3
        self.max_turn_rate = 0.8
        self.autonomous_mode = False
        
        # GPS status
        self.last_gps_lat = 0.0
        self.last_gps_lon = 0.0
        self.gps_status = -1
        
        # Keyboard state
        self.keys_pressed = set()
        self.keyboard_update_timer = QTimer()
        self.keyboard_update_timer.timeout.connect(self.process_keyboard_input)
        self.keyboard_update_timer.start(50)
        
        # Navigation update timer
        self.navigation_timer = QTimer()
        self.navigation_timer.timeout.connect(self.update_navigation)
        self.navigation_timer.start(100)
        
        # LiDAR safety update timer
        self.lidar_update_timer = QTimer()
        self.lidar_update_timer.timeout.connect(self.update_lidar_status)
        self.lidar_update_timer.start(2000)
        
        # Safety
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Initialize UI
        self.init_ui()
        
        # Initialize ROS
        self.init_ros()
        
        # NOTE: Servo enable service removed - servos work without it
        # If you need it later, uncomment the lines below:
        # self.servo_enable_timer = QTimer()
        # self.servo_enable_timer.setSingleShot(True)
        # self.servo_enable_timer.timeout.connect(self.enable_servo_reception)
        # self.servo_enable_timer.start(3000)
        
        # GPS checking timer
        self.gps_check_timer = QTimer()
        self.gps_check_timer.timeout.connect(self.check_gps_data)
        self.gps_check_timer.start(1000)
        
        # Register cleanup
        atexit.register(self.cleanup_on_exit)
        signal.signal(signal.SIGINT, self.signal_handler)

    def subscribe_to_fake_gps(self):
        """Subscribe to fake GPS topic for indoor testing"""
        if not self.ros_node:
            print("ROS node not ready")
            return False
        
        try:
            self.fake_gps_subscription = self.ros_node.create_subscription(
                    NavSatFix,
                    '/gps/fix',
                    self.fake_gps_callback,
                    10
            )
            print("Subscribe to fake GPS (/gps/fix)")
            return True
        except Exception as e:
            print(f" Failed to subscribe to fake GPS: {e}")
            return False

    def fake_gps_callback(self, msg):
        """Handle fake GPS message from indoor tester"""
        self.last_gps_lat = msg.latitude
        self.last_gps_lon = msg.longitude
        self.gps_status = msg.status.status
        
        # Update display withh Indoor test label
        self.current_gps_label.setText(f"GPS(Indoor Test): {self.last_gps_lat:.6f}, {self.last_gps_lon:.6f}")
        self.current_gps_label.setStyleSheet(
            "font-family: monospace; background-color: #d4edda; padding: 10px;"
            "border-radius: 5px; border: 2px solid #28a745;"
            )

    def toggle_indoor_mode(self, state):
        """Toggle indoor mode on/off"""
        if state == Qt.Checked:
            # Enable indoor mode
            self.indoor_test_mode = True

            # Subscribe to fake GPS
            QTimer.singleShot(1000, lambda: self.subscribe_to_fake_gps())

            # Disable real GPS buttons
            self.start_gps_btn.setEnabled(False)
            self.stop_gps_btn.setEnabled(False)
            
            # Update status
            self.gps_status_label.setText("Indoor Test Mode: Using Fake GPS")
            self.gps_status_label.setStyleSheet(
                    "background-color: #d1ecf1; color: #0c5460; padding: 5px; "
                    "border-radius: 3px; font-weight: bold;"
                    )

            print("\n" + "=" * 60)
            print(" INDOOR TEST MODE ENABLED")
            print("=" * 60)
            print("GPS data will come from indoor_nav_tester.py")
            print("Make sure indoor_nav_tester.py is running!")
            print("=" * 60 + "\n")

        else:
            # Disable indoor mode
            self.indoor_test_mode = False

            # Unsubscribe from fake GPS
            if self.fake_gps_subscription and self.ros_node:
                self.ros_node.destroy_subscription(self.fake_gps_subscription)
                self.fake_gps_subscription = None

            # Enable real GPS buttons
            self.start_gps_btn.setEnabled(True)
            self.stop_gps_btn.setEnabled(True)

            # Reset GPS data
            self.last_gps_lat = 0.0
            self.last_gps_lon = 0.0

            # Update status
            self.gps_status_label.setText("GPS: Not started")
            self.gps_status_label.setStyleSheet(
                "background-color: #fff3cd; padding: 5px; border-radius: 3px;")
            print("\n Outdoor mode enabled - Use real GPS\n")
        
    def enable_servo_reception(self):
        """
        Call the existing enable_reception service to enable servo motors
        """
        try:
            if not self.ros_node:
                print("⚠️  ROS node not ready, cannot enable servos")
                return
            
            # Create service client
            enable_client = self.ros_node.create_client(
                SetBool,
                '/ros_robot_controller/enable_reception'
            )
            
            # Wait for service
            print("Checking for servo enable service...")
            if not enable_client.wait_for_service(timeout_sec=5.0):
                print("ℹ️  Servo enable service not available (servos may already be enabled)")
                return
            
            # Call the service
            request = SetBool.Request()
            request.data = True
            
            future = enable_client.call_async(request)
            
            # Handle response
            def handle_response(future_obj):
                try:
                    response = future_obj.result()
                    if response.success:
                        print("✅ Servos ENABLED successfully!")
                        QTimer.singleShot(0, lambda: self.status_label.setText(
                            "Status: Ready - Servos Enabled"
                        ))
                    else:
                        print(f"⚠️  Servo enable failed: {response.message}")
                except Exception as e:
                    print(f"❌ Error calling servo service: {e}")
            
            future.add_done_callback(handle_response)
            
        except Exception as e:
            print(f"❌ Failed to enable servos: {e}")
    
    def signal_handler(self, signum, frame):
        """Handle system signals"""
        print(f"Received signal {signum}, cleaning up...")
        self.cleanup_on_exit()
        sys.exit(0)
    
    # ... [ALL YOUR OTHER EXISTING METHODS FROM main_gui.py] ...
    # (Keep all your existing init_ui, create_control_panel, etc.)
    
    def init_ui(self):
        """Initialize user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        control_panel = self.create_control_panel()
        status_panel = self.create_status_panel()
        
        main_layout.addWidget(control_panel, 1)
        main_layout.addWidget(status_panel, 1)
    
    def create_control_panel(self):
        """Create control panel (left side)"""
        group = QGroupBox("Rover Control")
        layout = QVBoxLayout()
        
        # Emergency stop
        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 15px;
                border-radius: 8px;
            }
        """)
        self.emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_btn)
        
        # Resume button
        self.resume_btn = QPushButton("Resume Control")
        self.resume_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 10px;
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
        
        # Indoor Test Mode Toggle
        indoor_mode_group = QGroupBox("Indoor Test Mode")
        indoor_mode_layout = QVBoxLayout()

        self.indoor_mode_checkbox = QCheckBox("Enable Indoor Test Mode (Fake GPS)")
        self.indoor_mode_checkbox.setToolTip("Use simulated GPS from indoor_nav_tester.py")
        self.indoor_mode_checkbox.stateChanged.connect(self.toggle_indoor_mode)
        indoor_mode_layout.addWidget(self.indoor_mode_checkbox)

        indoor_mode_info = QLabel("Check this to use fake GPS\nfrom indoor_nav_tester.py")
        indoor_mode_info.setStyleSheet("font-size: 10px; color: #666; padding: 5px;")
        indoor_mode_layout.addWidget(indoor_mode_info)

        indoor_mode_group.setLayout(indoor_mode_layout)
        layout.addWidget(indoor_mode_group)


        # GPS controls
        gps_control_group = QGroupBox("GPS Control")
        gps_control_layout = QVBoxLayout()
        
        self.start_gps_btn = QPushButton("Start GPS")
        self.start_gps_btn.clicked.connect(self.start_gps_node)
        self.start_gps_btn.setStyleSheet("background-color: #007bff; color: white; padding: 8px;")
        gps_control_layout.addWidget(self.start_gps_btn)
        
        self.stop_gps_btn = QPushButton("Stop GPS")
        self.stop_gps_btn.clicked.connect(self.stop_gps_node)
        self.stop_gps_btn.setStyleSheet("background-color: #6c757d; color: white; padding: 8px;")
        gps_control_layout.addWidget(self.stop_gps_btn)
        
        self.gps_status_label = QLabel("GPS: Not started")
        self.gps_status_label.setStyleSheet("background-color: #fff3cd; padding: 5px; border-radius: 3px;")
        gps_control_layout.addWidget(self.gps_status_label)
        
        gps_control_group.setLayout(gps_control_layout)
        layout.addWidget(gps_control_group)
        
        # LiDAR Safety controls
        lidar_group = QGroupBox("LiDAR Safety")
        lidar_layout = QVBoxLayout()
        
        self.lidar_safety_checkbox = QCheckBox("Enable LiDAR Safety")
        self.lidar_safety_checkbox.setChecked(True)
        self.lidar_safety_checkbox.setToolTip("Auto-stop when obstacles detected")
        lidar_layout.addWidget(self.lidar_safety_checkbox)
        
        distance_label = QLabel("Safety Distance: 0.5m")
        lidar_layout.addWidget(distance_label)
        
        self.safety_distance_slider = QSlider(Qt.Horizontal)
        self.safety_distance_slider.setRange(30, 150)
        self.safety_distance_slider.setValue(50)
        self.safety_distance_slider.valueChanged.connect(
            lambda v: self.update_safety_distance(v, distance_label)
        )
        lidar_layout.addWidget(self.safety_distance_slider)
        
        self.obstacle_status_label = QLabel("No obstacles detected")
        self.obstacle_status_label.setStyleSheet("background-color: #d4edda; padding: 5px; border-radius: 3px;")
        lidar_layout.addWidget(self.obstacle_status_label)
        
        lidar_group.setLayout(lidar_layout)
        layout.addWidget(lidar_group)
        
        # Info
        keyboard_info = QLabel("""Keyboard: W/A/S/D, Space=Stop
Ackermann: ±36° limit, 0.15m/s min
Safety: 2s auto-stop timeout""")
        keyboard_info.setStyleSheet("background-color: #f0f8ff; padding: 10px; border-radius: 5px; font-size: 11px;")
        layout.addWidget(keyboard_info)
        
        group.setLayout(layout)
        return group
    
    def create_status_panel(self):
        """Create status panel (right side)"""
        group = QGroupBox("Status & GPS")
        layout = QVBoxLayout()
        
        # Status
        self.status_label = QLabel("Status: Initializing")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #fff3cd; border-radius: 5px;")
        layout.addWidget(self.status_label)
        
        # GPS position
        gps_group = QGroupBox("GPS Position")
        gps_layout = QVBoxLayout()
        self.current_gps_label = QLabel("GPS: Waiting...")
        self.current_gps_label.setStyleSheet("font-family: monospace; padding: 10px;")
        gps_layout.addWidget(self.current_gps_label)
        gps_group.setLayout(gps_layout)
        layout.addWidget(gps_group)
        
        # Waypoint widget
        if GPS_WIDGET_AVAILABLE:
            waypoint_group = QGroupBox("GPS Waypoints")
            waypoint_layout = QVBoxLayout()
            self.waypoint_widget = GPSWaypointWidget(parent_gui=self)
            waypoint_layout.addWidget(self.waypoint_widget)
            waypoint_group.setLayout(waypoint_layout)
            layout.addWidget(waypoint_group)
        
        # Autonomous Navigation Control
        nav_group = QGroupBox("Autonomous Navigation")
        nav_layout = QVBoxLayout()
        
        self.load_nav_btn = QPushButton("Load Waypoint File")
        self.load_nav_btn.clicked.connect(self.load_waypoints_for_navigation)
        self.load_nav_btn.setStyleSheet("background-color: #6f42c1; color: white; padding: 8px;")
        nav_layout.addWidget(self.load_nav_btn)
        
        nav_buttons = QHBoxLayout()
        self.start_nav_btn = QPushButton("Start Navigation")
        self.start_nav_btn.clicked.connect(self.start_autonomous_navigation)
        self.start_nav_btn.setEnabled(False)
        self.start_nav_btn.setStyleSheet("background-color: #28a745; color: white; padding: 8px;")
        nav_buttons.addWidget(self.start_nav_btn)
        
        self.stop_nav_btn = QPushButton("Stop Navigation")
        self.stop_nav_btn.clicked.connect(self.stop_autonomous_navigation)
        self.stop_nav_btn.setEnabled(False)
        self.stop_nav_btn.setStyleSheet("background-color: #dc3545; color: white; padding: 8px;")
        nav_buttons.addWidget(self.stop_nav_btn)
        nav_layout.addLayout(nav_buttons)
        
        self.nav_status_label = QLabel("No waypoints loaded")
        self.nav_status_label.setStyleSheet("background-color: #f0f0f0; padding: 8px; border-radius: 3px; font-size: 10px;")
        nav_layout.addWidget(self.nav_status_label)
        
        self.nav_progress = QProgressBar()
        self.nav_progress.setValue(0)
        nav_layout.addWidget(self.nav_progress)
        
        nav_group.setLayout(nav_layout)
        layout.addWidget(nav_group)
        
        # Movement display
        movement_group = QGroupBox("Current Movement")
        movement_layout = QVBoxLayout()
        self.movement_label = QLabel("Linear: 0.0 m/s\nAngular: 0.0 rad/s")
        self.movement_label.setStyleSheet("font-family: monospace; padding: 10px;")
        movement_layout.addWidget(self.movement_label)
        movement_group.setLayout(movement_layout)
        layout.addWidget(movement_group)
        
        group.setLayout(layout)
        return group
    
    def init_ros(self):
        """Initialize ROS2 node in separate thread"""
        try:
            self.ros_thread = threading.Thread(target=self.ros_worker, daemon=True)
            self.ros_thread.start()
        except Exception as e:
            print(f"Failed to start ROS thread: {e}")
    
    def ros_worker(self):
        """ROS2 worker thread"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = RoverROSInterface(enable_lidar_safety=True)
            safety_manager.set_ros_node(self.ros_node)
            
            QTimer.singleShot(0, lambda: self.status_label.setText("Status: Ready"))
            
            rclpy.spin(self.ros_node)
        except Exception as e:
            print(f"ROS thread error: {e}")
    
    
    def process_keyboard_input(self):
        """Process keyboard input for simultaneous keys"""
        if self.emergency_stopped or not self.keys_pressed or self.autonomous_mode:
            return
        
        linear = 0.0
        angular = 0.0
        
        if Qt.Key_W in self.keys_pressed:
            linear = self.max_speed
        elif Qt.Key_S in self.keys_pressed:
            linear = -self.max_speed
        
        if Qt.Key_A in self.keys_pressed:
            angular = self.max_turn_rate
        elif Qt.Key_D in self.keys_pressed:
            angular = -self.max_turn_rate
        
        if linear != 0.0 or angular != 0.0:
            self.send_movement_command(linear, angular)
    
    def keyPressEvent(self, event):
        """Handle key press"""
        if self.emergency_stopped:
            return
        
        if event.key() == Qt.Key_Space:
            self.stop_movement()
            return
        elif event.key() == Qt.Key_Escape:
            self.emergency_stop()
            return
        
        if event.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            self.keys_pressed.add(event.key())
    
    def keyReleaseEvent(self, event):
        """Handle key release"""
        if event.key() in self.keys_pressed:
            self.keys_pressed.remove(event.key())
        
        if not self.keys_pressed:
            self.stop_movement()
    
    def joystick_moved(self, x, y):
        """Handle joystick movement"""
        if self.emergency_stopped:
            return
        
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
    
    def emergency_stop(self):
        """Emergency stop"""
        self.emergency_stopped = True
        self.stop_movement()
        
        if self.ros_node:
            self.ros_node.call_emergency_stop()
        
        self.status_label.setText("Status: EMERGENCY STOP")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #dc3545; color: white; border-radius: 5px;")
        self.resume_btn.setEnabled(True)
        self.emergency_btn.setEnabled(False)
    
    def resume_control(self):
        """Resume control after emergency stop"""
        self.emergency_stopped = False
        if self.ros_node:
            self.ros_node.send_cmd_vel(0.0, 0.0)
        
        self.status_label.setText("Status: Ready")
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #d4edda; border-radius: 5px;")
        self.resume_btn.setEnabled(False)
        self.emergency_btn.setEnabled(True)
    
    def update_max_speed(self, value):
        """Update maximum speed from slider"""
        self.max_speed = (value / 100.0) * 1.0
        self.max_turn_rate = (value / 100.0) * 2.0
        self.speed_label.setText(f"Speed: {value}%")
    
    def update_movement_display(self):
        """Update movement display"""
        self.movement_label.setText(f"Linear: {self.current_linear:.2f} m/s\nAngular: {self.current_angular:.2f} rad/s")
    
    def start_gps_node(self):
        """Start GPS node"""
        try:
            subprocess.run(["pkill", "-f", "nmea_serial_driver"], check=False)
            time.sleep(1)
            
            if not hasattr(self, 'direct_gps') or not self.direct_gps:
                self.init_direct_gps()
            
            if self.direct_gps.start():
                self.gps_status_label.setText("GPS: Starting...")
                self.gps_status_label.setStyleSheet("background-color: #cce5ff; padding: 5px; border-radius: 3px;")
                self.start_gps_btn.setEnabled(False)
                self.stop_gps_btn.setEnabled(True)
                QTimer.singleShot(3000, lambda: self.check_direct_gps_status())
            else:
                self.gps_status_label.setText("GPS: Failed to start")
                self.gps_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;")
        except Exception as e:
            print(f"GPS start error: {e}")
    
    def init_direct_gps(self):
        """Initialize direct GPS handler"""
        self.direct_gps = DirectGPSHandler(port='/dev/ttyUSB0', baud=9600)
        self.direct_gps.set_gui_callback(self.on_direct_gps_update)
        print("Direct GPS handler initialized")
    
    def on_direct_gps_update(self, lat, lon, status):
        """Handle GPS update"""
        self.last_gps_lat = lat
        self.last_gps_lon = lon
        self.gps_status = status
    
    def check_gps_data(self):
        """Check for GPS data updates"""
        if self.indoor_test_mode:
            return
        try:
            if hasattr(self, 'direct_gps') and self.direct_gps and self.direct_gps.running:
                gps_data = self.direct_gps.get_gps_data()
                
                if gps_data['available'] and gps_data['lat'] != 0.0:
                    lat = gps_data['lat']
                    lon = gps_data['lon']
                    status = gps_data['status']
                    
                    if abs(lat) <= 90 and abs(lon) <= 180:
                        self.current_gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}")
                        self.current_gps_label.setStyleSheet(
                            "font-family: monospace; background-color: #d4edda; padding: 10px; border-radius: 5px;"
                        )
                        self.last_gps_lat = lat
                        self.last_gps_lon = lon
                        self.gps_status = status
                        return
            
            if not hasattr(self, 'direct_gps') or not self.direct_gps or not self.direct_gps.running:
                self.current_gps_label.setText("GPS: Not started")
                self.current_gps_label.setStyleSheet(
                    "font-family: monospace; background-color: #f0f0f0; padding: 10px; border-radius: 5px;"
                )
        except Exception as e:
            print(f"GPS check error: {e}")
    
    def check_direct_gps_status(self):
        """Check direct GPS status"""
        try:
            if hasattr(self, 'direct_gps') and self.direct_gps:
                gps_data = self.direct_gps.get_gps_data()
                if gps_data['available'] and gps_data['lat'] != 0.0:
                    self.gps_status_label.setText("GPS: Active with fix")
                    self.gps_status_label.setStyleSheet("background-color: #d4edda; padding: 5px; border-radius: 3px;")
                else:
                    self.gps_status_label.setText("GPS: Active, waiting for fix...")
                    self.gps_status_label.setStyleSheet("background-color: #fff3cd; padding: 5px; border-radius: 3px;")
                    QTimer.singleShot(5000, lambda: self.check_direct_gps_status())
        except Exception as e:
            print(f"GPS status check error: {e}")
    
    def stop_gps_node(self):
        """Stop GPS node"""
        try:
            if hasattr(self, 'direct_gps') and self.direct_gps:
                self.direct_gps.stop()
            subprocess.run(["pkill", "-f", "nmea_serial_driver"], check=False)
            
            self.gps_status_label.setText("GPS: Stopped")
            self.gps_status_label.setStyleSheet("background-color: #f8d7da; padding: 5px; border-radius: 3px;")
            self.stop_gps_btn.setEnabled(False)
            self.start_gps_btn.setEnabled(True)
        except Exception as e:
            print(f"GPS stop error: {e}")
    
    def load_waypoints_for_navigation(self):
        """Load waypoint file for autonomous navigation"""
        try:
            filename, _ = QFileDialog.getOpenFileName(
                self, "Load Waypoint File", "", 
                "JSON files (*.json);;All files (*)"
            )
            
            if not filename:
                return
            
            if self.navigator.load_waypoints_from_file(filename):
                waypoint_count = len(self.navigator.waypoints)
                total_distance = self.navigator.total_distance
                
                self.nav_status_label.setText(
                    f"Loaded {waypoint_count} waypoints\n"
                    f"Total distance: {total_distance:.1f}m"
                )
                self.start_nav_btn.setEnabled(True)
                
                QMessageBox.information(
                    self, "Waypoints Loaded",
                    f"Successfully loaded {waypoint_count} waypoints\n"
                    f"Total path distance: {total_distance:.1f}m\n\n"
                    "Click 'Start Navigation' to begin autonomous driving."
                )
            else:
                QMessageBox.warning(self, "Load Failed", "Failed to load waypoint file")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error loading waypoints: {e}")
    
    def start_autonomous_navigation(self):
        """Start autonomous navigation"""
        if self.last_gps_lat == 0.0 or self.last_gps_lon == 0.0:
            QMessageBox.warning(
                self, "No GPS Data",
                "Cannot start navigation without valid GPS signal."
            )
            return
        
        if self.navigator.start_navigation():
            self.autonomous_mode = True
            self.start_nav_btn.setEnabled(False)
            self.stop_nav_btn.setEnabled(True)
            self.status_label.setText("Status: AUTONOMOUS NAVIGATION")
            self.status_label.setStyleSheet(
                "font-size: 14px; font-weight: bold; padding: 10px; "
                "background-color: #17a2b8; color: white; border-radius: 5px;"
            )
            print("Autonomous navigation started!")
        else:
            QMessageBox.warning(self, "Navigation Error", "Failed to start navigation")
    
    def stop_autonomous_navigation(self):
        """Stop autonomous navigation"""
        self.navigator.stop_navigation()
        self.autonomous_mode = False
        self.stop_movement()
        
        self.start_nav_btn.setEnabled(True)
        self.stop_nav_btn.setEnabled(False)
        self.status_label.setText("Status: Manual Mode - Ready")
        self.status_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; padding: 10px; "
            "background-color: #d4edda; border-radius: 5px;"
        )
        print("Autonomous navigation stopped")
    
    def update_navigation(self):
        """Update autonomous navigation (called by timer)"""
        if not self.autonomous_mode or not self.navigator.navigation_active:
            return
        
        if self.gps_status < 0 or self.last_gps_lat == 0.0:
            return
        
        print(f"\n[NAV] Current GPS: {self.last_gps_lat:.6f}, {self.last_gps_lon:.6f}")

        current_heading = None
        
        linear, angular, reached = self.navigator.calculate_control_command(
            self.last_gps_lat,
            self.last_gps_lon,
            current_heading
        )
        
        print(f"[NAV] Command: Linear={linear:.2f}, Angular={angular:.2f}")

        if self.ros_node and not reached:
            self.send_movement_command(linear, angular)
        elif reached:
            self.stop_autonomous_navigation()
            QMessageBox.information(
                self, "Navigation Complete",
                f"All {self.navigator.waypoints_reached} waypoints reached!\n"
                f"Total distance: {self.navigator.total_distance:.1f}m"
            )
        
        status = self.navigator.get_navigation_status()
        self.nav_status_label.setText(
            f"Waypoint: {status['current_waypoint']}/{status['total_waypoints']}\n"
            f"Progress: {status['progress_percent']:.1f}%"
        )
        self.nav_progress.setValue(int(status['progress_percent']))
    
    def update_safety_distance(self, value, label):
        """Update LiDAR safety distance from slider"""
        distance = value / 100.0
        label.setText(f"Safety Distance: {distance:.2f}m")
        
        if self.ros_node and hasattr(self.ros_node, 'set_lidar_safety_distance'):
            self.ros_node.set_lidar_safety_distance(distance)
    
    def update_lidar_status(self):
        """Update LiDAR obstacle status display"""
        if not self.ros_node:
            return
        
        if not hasattr(self.ros_node, 'get_lidar_safety_info'):
            return
        
        if hasattr(self.ros_node, 'set_lidar_safety_enabled'):
            self.ros_node.set_lidar_safety_enabled(self.lidar_safety_checkbox.isChecked())
        elif hasattr(self.ros_node, 'enable_lidar_safety'):
            self.ros_node.enable_lidar_safety = self.lidar_safety_checkbox.isChecked()
        
        try:
            obstacle_info = self.ros_node.get_lidar_safety_info()
            
            if obstacle_info['detected']:
                self.obstacle_status_label.setText(
                    f"⚠️ OBSTACLE! {obstacle_info['distance']:.2f}m at {obstacle_info['angle_deg']:.0f}°"
                )
                self.obstacle_status_label.setStyleSheet(
                    "background-color: #f8d7da; color: #721c24; padding: 5px; border-radius: 3px; font-weight: bold;"
                )
            elif obstacle_info['warning']:
                self.obstacle_status_label.setText(
                    f"⚠ Warning: {obstacle_info['distance']:.2f}m"
                )
                self.obstacle_status_label.setStyleSheet(
                    "background-color: #fff3cd; color: #856404; padding: 5px; border-radius: 3px;"
                )
            else:
                self.obstacle_status_label.setText(f"✓ Clear (min: {obstacle_info['distance']:.2f}m)")
                self.obstacle_status_label.setStyleSheet(
                    "background-color: #d4edda; color: #155724; padding: 5px; border-radius: 3px;"
                )
        except Exception as e:
            pass
    
    def closeEvent(self, event):
        """Handle window close"""
        self.cleanup_on_exit()
        event.accept()
    
    def cleanup_on_exit(self):
        """Cleanup on exit"""
        try:
            print("Cleaning up...")
            
            if self.autonomous_mode:
                self.stop_autonomous_navigation()
            
            self.emergency_stop()
            
            if hasattr(self, 'keyboard_update_timer'):
                self.keyboard_update_timer.stop()
            if hasattr(self, 'navigation_timer'):
                self.navigation_timer.stop()
            if hasattr(self, 'lidar_update_timer'):
                self.lidar_update_timer.stop()
            if hasattr(self, 'gps_check_timer'):
                self.gps_check_timer.stop()
            if hasattr(self, 'servo_enable_timer'):
                self.servo_enable_timer.stop()
            if hasattr(self, 'direct_gps') and self.direct_gps:
                self.direct_gps.stop()
            
            subprocess.run(["pkill", "-f", "nmea_serial_driver"], check=False)
            if self.fake_gps_subscription and self.ros_node:
                self.ros_node.destroy_subscription(self.fake_gps_subscription)
            if self.ros_node:
                try:
                    if hasattr(self.ros_node, 'cleanup'):
                        self.ros_node.destroy_node()

                    time.sleep(0.2)

                    self.ros_node.destroy_node()
                except Exception as e:
                    print(f"ROS node cleanup error: {e}")

            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception as e:
                    print(f"ROS shutdown error: {e}")
            
            print("Cleanup completed")

        except Exception as e:
            print(f"Cleanup error: {e}")


def main():
    """Main entry point - automatically stops competing apps"""
    
    # AUTOMATICALLY stop competing apps when GUI starts
    stop_competing_apps()
    
    # Start Qt application
    app = QApplication(sys.argv)
    
    if not ROS_AVAILABLE:
        QMessageBox.critical(None, "ROS2 Error", "ROS2 is not available")
        sys.exit(1)
    
    try:
        window = RoverControlGUI()
        window.show()
        
        # Enable Ctrl+C
        timer = QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)
        
        print("\n" + "=" * 60)
        print("Rover Control GUI Started")
        print("=" * 60)
        print("Controls:")
        print("  • W/A/S/D or joystick for manual control")
        print("  • Load waypoints for autonomous navigation")
        print("  • Physical joystick available for override")
        print("\nPress Ctrl+C or close window to exit")
        print("=" * 60 + "\n")
        
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Application error: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
