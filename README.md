# Autonomous Mini Rover - ROS2 Project

A comprehensive autonomous navigation system for a mini rover built on **ROS2 Humble in Docker**, featuring GPS waypoint navigation, LiDAR obstacle avoidance, and Ackermann steering control.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red.svg)

## 🚀 Project Overview

This project implements a complete autonomous rover system using the **Hiwonder MentorPi** platform with:
- **Hardware Platform**: Raspberry Pi 5 with comprehensive sensor suite
- **Software Stack**: ROS2 Humble in Docker environment
- **Navigation**: GPS waypoint navigation with indoor testing capabilities
- **Safety**: LiDAR-based obstacle avoidance with graduated response zones
- **Control**: Ackermann steering with proper geometric constraints

## 🎯 Key Features

### ✅ **Currently Implemented & Working**
- **Ackermann Steering Control**: Proper front-wheel steering with ±36° limit, 0.213m wheelbase
- **GPS Waypoint Navigation**: Autonomous navigation to GPS coordinates
- **LiDAR Safety System**: Real-time obstacle detection with graduated response zones:
  - Emergency stop: < 0.3m
  - Warning zone: 0.3-0.5m
  - Caution zone: 0.5-1.0m
- **Indoor Testing Mode**: Simulated GPS coordinates for development without outdoor access
- **PyQt5 GUI**: User-friendly control interface with virtual joystick
- **Multi-sensor Integration**: GPS, IMU, LiDAR, camera data streams
- **Safety Systems**: Emergency stops, timeout protection, manual override
- **VNC Remote Control**: With key debouncing for stable remote operation

### 🔄 **In Active Development**
- Outdoor GPS validation and accuracy testing
- IMU + GPS sensor fusion for improved straight-line navigation  
- Camera-based obstacle detection integration
- Exclusion zone configuration for false positive filtering

### 🎯 **Planned Enhancements**
- ROS2 Nav2 navigation stack integration
- Machine learning for terrain analysis
- Advanced path planning algorithms
- Multi-robot coordination capabilities

## 🏗️ Hardware Configuration

### **Rover Platform**
- **Base**: Hiwonder MentorPi (Raspberry Pi 5 based)
- **Steering**: Ackermann configuration with geometric constraints
- **Motor Control**: Differential drive with speed control

### **Sensor Suite**
- **LiDAR**: LD19 360° laser scanner (`/scan_raw`)
- **GPS**: External GPS module with AGNSS support (`/fix`, `/gps/fix`)
- **IMU**: External 9-axis inertial measurement unit (`/imu`) 
- **Camera**: USB camera for computer vision (`/ascamera/*`)

### **Hardware Connections**
- GPS and IMU: Daisy-chain configuration
- LiDAR: USB connection
- Camera: USB connection
- All devices have consistent mapping via udev rules

## 🐳 Docker Environment Setup

### **Container Configuration**
- **Base Image**: ROS2 Humble with Ubuntu 22.04
- **Workspace**: `/home/ubuntu/ros2_ws`
- **GUI Support**: X11 forwarding for PyQt5 applications
- **Device Access**: USB device passthrough for sensors

### **Running the System**
```bash
# Start Docker container with hardware access
docker run -it --privileged \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyUSB1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    your_ros2_image

# Inside container - source workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch main control system
ros2 launch rover_gps rover_control_launch.py

# Or run GUI directly
python3 ~/ros2_ws/src/rover_gps/rover_gps/rover_control/main_guiv2.py
```

## 📂 ROS2 Workspace Structure

```
ros2_ws/
├── src/
│   ├── rover_gps/              # Main rover GPS navigation package
│   │   ├── rover_gps/
│   │   │   └── rover_control/  # 🎯 Core rover control logic
│   │   │       ├── main_guiv2.py           # Main PyQt5 GUI
│   │   │       ├── ackermann_handler.py    # Steering mathematics
│   │   │       ├── ros_interface.py        # ROS2 integration
│   │   │       ├── gps_handler.py          # GPS processing
│   │   │       ├── waypoint_navigator.py   # Navigation logic
│   │   │       ├── safety_manager.py       # Safety coordination
│   │   │       └── virtual_joystick.py     # GUI controls
│   │   ├── launch/            # ROS2 launch files
│   │   └── config/            # Configuration files
│   ├── peripherals/           # Sensor driver packages
│   ├── controller/            # Hardware controller interfaces
│   ├── driver/               # Low-level hardware drivers
│   └── app/                  # Application-level nodes
├── build/                    # ROS2 build artifacts (excluded from git)
├── install/                  # ROS2 installation (excluded from git)
└── log/                     # ROS2 logs (excluded from git)
```

## 🛠️ Key Components

### **Core Control (`rover_gps/rover_control/`)**
- `ackermann_handler.py`: Ackermann steering mathematics and constraints
- `ros_interface.py`: ROS2 publishers/subscribers and safety integration  
- `safety_manager.py`: Emergency stop and safety coordination
- `main_guiv2.py`: Main PyQt5 control interface with joystick

### **Navigation System**
- `waypoint_navigator.py`: GPS waypoint navigation logic with tolerance settings
- `gps_handler.py`: GPS data processing and coordinate transformations

### **Safety & Monitoring**
- LiDAR safety monitoring with exclusion zones (filters GPS module false positives at -132°)
- Graduated safety responses based on obstacle proximity
- Emergency stop accessible via Space key or GUI button
- Automatic timeout protection with safe state management

## 📊 ROS2 Topics & Interfaces

### **Published Topics**
- `/controller/cmd_vel` (geometry_msgs/Twist): Motor control commands with Ackermann constraints
- `/rover/status` (custom): System status and diagnostics

### **Subscribed Topics**  
- `/scan_raw` (sensor_msgs/LaserScan): LiDAR data with safety zone processing
- `/fix` (sensor_msgs/NavSatFix): GPS position data with indoor test mode
- `/imu` (sensor_msgs/Imu): Inertial measurement data for future sensor fusion
- `/ascamera/image_raw` (sensor_msgs/Image): Camera image data for computer vision

### **Services**
- Emergency stop service for immediate safety shutdown
- Servo control services for hardware management

## 🧪 Testing & Development Features

### **Indoor Testing Capabilities**
- **Fake GPS Mode**: Simulated coordinates for development without outdoor access
- **Test Location**: Winnipeg area coordinates (49.8951°N, 97.1384°W)
- **Sensor Simulation**: Ability to test algorithms with simulated sensor data
- **VNC Remote Access**: Headless operation with connection debouncing

### **Safety-First Development**
- **Incremental Development**: Step-by-step feature addition with testing at each stage
- **Rollback Procedures**: Every major change includes detailed reversal instructions
- **Systematic Debugging**: Comprehensive diagnostic tools before implementing changes
- **Modular Architecture**: Independent, testable components with clear interfaces

## 🛡️ Safety Systems

### **LiDAR Safety Zones**
- **Emergency Stop Zone**: < 0.3m (immediate halt)
- **Warning Zone**: 0.3-0.5m (50% speed reduction)
- **Caution Zone**: 0.5-1.0m (25% speed reduction)
- **Exclusion Zones**: Filters false positives from GPS module at -132°

### **GPS Safety Features**
- **Indoor Test Mode**: Safe development without requiring outdoor GPS
- **Waypoint Validation**: Bounds checking and reasonable distance verification
- **Manual Override**: Always available emergency controls

### **System-Level Safety**
- **Emergency Stop**: Accessible via Space key, GUI button, or ROS2 service
- **Communication Timeout**: 500ms timeout with automatic safe state
- **Hardware Monitoring**: USB device connection verification
- **Safe Shutdown**: Proper cleanup on unexpected termination

## 🔧 Requirements & Dependencies

### **Hardware Requirements**
- Raspberry Pi 5 (or compatible ARM64 platform)
- LD19 LiDAR scanner with USB interface
- External GPS module with AGNSS capabilities
- External 9-axis IMU with serial interface
- USB camera for computer vision
- Ackermann steering rover chassis

### **Software Requirements**
- Docker with hardware device access
- ROS2 Humble desktop installation
- Python 3.8+ with pip package management
- PyQt5 for GUI applications
- Git for version control

### **ROS2 Package Dependencies**
```bash
# Core ROS2 packages
ros-humble-geometry-msgs
ros-humble-sensor-msgs  
ros-humble-nav-msgs
ros-humble-nmea-navsat-driver

# Python packages
pip install pynmea2 geopy pyserial numpy scipy
```

## 🚦 Current Development Status

### **Recent Achievements**
- ✅ **Ackermann Constraints**: Resolved steering geometry limitations with proper wheelbase calculations
- ✅ **LiDAR Safety Integration**: Implemented graduated safety zones with exclusion zone filtering
- ✅ **GPS Waypoint Navigation**: Successfully tested indoor navigation with simulated coordinates
- ✅ **VNC Stability**: Fixed remote control timing issues with key debouncing implementation
- ✅ **Multi-sensor Coordination**: Resolved competing ROS2 publisher conflicts through node management

### **Known Limitations & Solutions in Progress**
- **GPS-only Navigation**: Creates zigzag patterns due to lack of real-time heading
  - *Solution*: Implementing IMU + GPS sensor fusion for improved straight-line navigation
- **Single-sensor Obstacle Detection**: Currently relies on LiDAR only
  - *Solution*: Adding camera-based computer vision for enhanced detection
- **USB Device Conflicts**: Device identification issues with USB splitters
  - *Solution*: Direct connections or powered hubs for reliable multi-sensor operation

### **Next Development Milestones**
1. **Outdoor GPS Validation**: Real-world testing of waypoint navigation accuracy
2. **Sensor Fusion Implementation**: GPS + IMU integration for heading estimation
3. **Camera Integration**: Computer vision for obstacle detection and classification
4. **Nav2 Stack Integration**: Advanced path planning and navigation capabilities

## 🤝 Contributing & Development Guidelines

### **Development Philosophy**
1. **Safety-First Design**: All changes include comprehensive safety analysis
2. **Incremental Development**: Small, testable changes with rollback procedures
3. **Indoor-First Testing**: Validate algorithms before outdoor deployment
4. **Documentation-Driven**: Changes include updated documentation and examples

### **Contribution Process**
1. Create backups before any workspace modifications
2. Test changes thoroughly in indoor simulation mode
3. Include detailed rollback procedures for new features
4. Follow ROS2 best practices for node development and communication
5. Document hardware connections and dependency changes
6. Maintain compatibility with safety-critical systems

## 📚 Documentation & Resources

### **System Documentation**
- Hardware integration guides in `docs/hardware/`
- ROS2 topic and service documentation in `docs/api/`
- Safety system specifications in `docs/safety/`
- Development procedures in `docs/development/`

### **Hardware Integration Guides**
- GPS module setup and AGNSS configuration
- IMU calibration and coordinate frame alignment  
- LiDAR driver installation and parameter tuning
- Camera configuration and computer vision pipeline

## ⚠️ Safety Notice & Disclaimers

**This is an autonomous vehicle system operating with hardware access in a Docker environment.**

### **Safety Requirements**
- **Always test in controlled environments** before real-world deployment
- **Keep manual override (emergency stop) accessible** at all times during operation
- **Respect local regulations** for autonomous vehicle operation and testing
- **Never operate without human supervision** and situational awareness
- **Understand Docker container hardware access** implications and security considerations

### **Operational Guidelines**
- Conduct initial testing in enclosed areas with adequate safety margins
- Verify all sensor calibrations before autonomous operation
- Test emergency stop procedures regularly
- Monitor system logs for anomalies or safety warnings
- Maintain up-to-date backup procedures for system recovery

## 🏆 Project Achievements

This project demonstrates successful integration of:
- **Professional ROS2 Architecture**: Proper node organization and communication patterns
- **Multi-sensor Fusion**: Coordinated GPS, IMU, and LiDAR data processing
- **Safety-Critical Systems**: Reliable emergency stop and graduated safety responses
- **Docker-based Development**: Reproducible environment with hardware integration
- **Indoor/Outdoor Flexibility**: Development capabilities independent of location
- **User Interface Design**: Intuitive PyQt5 GUI with comprehensive controls

## 📞 Support & Contact

For questions, issues, or collaboration opportunities:
- **GitHub Issues**: Use the repository issue tracker for bugs and feature requests
- **Discussions**: GitHub Discussions for general questions and community interaction
- **Documentation**: Check the `docs/` directory for detailed guides and references

---

**Built with ROS2 Humble • Powered by Docker • Designed for Safety**

*Autonomous navigation through careful engineering and incremental development*
