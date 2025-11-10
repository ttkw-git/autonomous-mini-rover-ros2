# Installation & Setup Guide

This guide walks you through setting up the Autonomous Mini Rover ROS2 environment from scratch.

## 🎯 Prerequisites

### Hardware Requirements
- Raspberry Pi 5 (recommended) or compatible ARM64/x86_64 system
- Hiwonder MentorPi rover chassis with Ackermann steering
- LD19 LiDAR scanner
- External GPS module with AGNSS capability
- External 9-axis IMU
- USB camera
- MicroSD card (32GB+ recommended)

### Software Requirements
- Ubuntu 22.04 LTS (or compatible Docker host)
- Docker Engine with hardware device access
- Git version control
- Internet connection for package installation

## 🐳 Docker Setup (Recommended)

### 1. Install Docker
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Verify installation
docker --version
```

### 2. Clone Repository
```bash
# Clone the project
git clone https://github.com/ttkw-git/autonomous-mini-rover-ros2.git
cd autonomous-mini-rover-ros2
```

### 3. Build Docker Image
```bash
# Build the ROS2 development environment
docker build -t rover-ros2:humble .

# Or use docker-compose for easier management
docker-compose build
```

### 4. Run Docker Container
```bash
# Run with hardware access and GUI support
docker run -it --privileged \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyUSB1 \
    --device=/dev/ttyUSB2 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --name rover-development \
    rover-ros2:humble

# Alternative: Use docker-compose
docker-compose up -d
docker exec -it autonomous-mini-rover-ros2_rover-ros2_1 /bin/bash
```

## 🔧 Native Installation (Alternative)

### 1. Install ROS2 Humble
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop-full

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Dependencies
```bash
# System dependencies
sudo apt install \
    python3-pip \
    python3-pyqt5 \
    python3-serial \
    python3-numpy \
    python3-scipy \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-nmea-navsat-driver \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher

# Python packages
pip3 install pynmea2 geopy
```

### 3. Setup Workspace
```bash
# Create and setup workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone project (if not already done)
git clone https://github.com/ttkw-git/autonomous-mini-rover-ros2.git .

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash
```

## 🔌 Hardware Setup

### 1. USB Device Mapping
Create consistent device names using udev rules:

```bash
# Create udev rules file
sudo nano /etc/udev/rules.d/99-rover-sensors.rules
```

Add the following content (adjust VID/PID for your devices):
```bash
# GPS Module
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gps", MODE="0666"

# IMU Module  
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="imu", MODE="0666"

# LiDAR Scanner
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
```

Reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. Verify Hardware Connections
```bash
# List USB devices
lsusb

# Check device mappings
ls -la /dev/ | grep -E "(gps|imu|lidar|ttyUSB)"

# Test device communication
# GPS
sudo cat /dev/gps | head -10

# IMU (if available)
sudo cat /dev/imu | head -5
```

## 🚀 Running the System

### 1. Launch ROS2 Nodes
```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch main system (if launch files exist)
ros2 launch rover_gps rover_control.launch.py

# Or run individual components
cd ~/ros2_ws/src/rover_gps/rover_gps/rover_control/
python3 main_guiv2.py
```

### 2. Verify System Operation
```bash
# Check active nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /controller/cmd_vel
ros2 topic echo /scan_raw
ros2 topic echo /fix

# Check for errors
ros2 doctor
```

## 🧪 Testing

### 1. Indoor Testing Mode
The system supports indoor testing without requiring actual GPS signals:

```bash
# Run with indoor test mode (simulated GPS)
python3 main_guiv2.py --indoor-test

# Or set environment variable
export ROVER_INDOOR_TEST=true
python3 main_guiv2.py
```

### 2. Safety System Testing
```bash
# Test emergency stop
# - Press Space key in GUI
# - Click Emergency Stop button
# - Test ROS2 service: ros2 service call /emergency_stop std_srvs/srv/Trigger

# Test LiDAR safety zones
# - Place objects at different distances
# - Verify graduated speed responses
# - Check exclusion zones work correctly
```

## 🔍 Troubleshooting

### Common Issues

**1. USB Device Not Found**
```bash
# Check USB connections
lsusb
dmesg | tail -20

# Verify udev rules
udevadm info -a -n /dev/ttyUSB0
```

**2. Permission Denied**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER
newgrp dialout

# Check device permissions
ls -la /dev/ttyUSB*
```

**3. ROS2 Communication Issues**
```bash
# Check ROS2 environment
printenv | grep ROS

# Verify domain ID
echo $ROS_DOMAIN_ID

# Check network interfaces
ros2 daemon stop
ros2 daemon start
```

**4. GUI Display Issues (Docker)**
```bash
# Allow X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Verify X11 socket mount
ls -la /tmp/.X11-unix
```

## 📊 Performance Optimization

### 1. System Configuration
```bash
# Increase USB buffer sizes
echo 'SUBSYSTEM=="usb-serial", DRIVERS=="cp210x", ATTR{latency_timer}="1"' | sudo tee -a /etc/udev/rules.d/99-usb-serial.rules

# Optimize system for real-time performance
sudo apt install linux-headers-$(uname -r)
```

### 2. ROS2 Configuration
```bash
# Set ROS2 middleware configuration
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export ROS_DOMAIN_ID=42  # Use unique domain ID
```

## 🔒 Security Considerations

### Docker Security
- Run containers with minimal privileges when possible
- Limit device access to only required hardware
- Use read-only filesystem where appropriate
- Regular security updates

### Network Security
- Configure ROS2 domain ID for isolation
- Use VPN for remote access
- Monitor network traffic during operation

## 📈 Next Steps

After successful installation:

1. **Calibration**: Calibrate sensors using provided calibration procedures
2. **Testing**: Run comprehensive system tests in indoor mode
3. **Configuration**: Adjust parameters for your specific hardware
4. **Documentation**: Review component-specific documentation
5. **Development**: Begin customization for your specific use case

## 🤝 Getting Help

- **GitHub Issues**: Report bugs and request features
- **Documentation**: Check the `docs/` directory for detailed guides
- **Community**: Join discussions for general questions

---

**Installation complete!** Your autonomous mini rover development environment is ready for development and testing.
