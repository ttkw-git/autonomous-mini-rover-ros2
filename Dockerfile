# Dockerfile for Autonomous Mini Rover ROS2 Environment
# Based on ROS2 Humble with full desktop environment

FROM ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Python and development tools
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    vim \
    # GUI and display support
    python3-pyqt5 \
    python3-pyqt5.qtsvg \
    qt5-style-plugins \
    x11-apps \
    # Serial communication
    python3-serial \
    python3-usb \
    # Scientific computing
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    # ROS2 packages
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-nmea-navsat-driver \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    # Hardware interface packages
    ros-humble-usb-cam \
    ros-humble-v4l2-camera \
    # Development tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Networking tools
    net-tools \
    iputils-ping \
    ssh \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip && pip3 install \
    # GPS and navigation
    pynmea2>=1.18.0 \
    geopy>=2.2.0 \
    # GUI development
    PyQt5>=5.15.0 \
    # Scientific computing  
    numpy>=1.21.0 \
    scipy>=1.7.0 \
    # Development and testing
    pytest>=6.0.0 \
    black>=21.0.0 \
    pylint>=2.0.0 \
    # Hardware communication
    pyserial>=3.5 \
    # Optional computer vision (uncomment if needed)
    # opencv-python>=4.5.0

# Create workspace directory
WORKDIR /home/ubuntu
RUN mkdir -p ros2_ws/src

# Create non-root user for development
RUN useradd -ms /bin/bash ubuntu && \
    usermod -aG dialout ubuntu && \
    chown -R ubuntu:ubuntu /home/ubuntu

# Copy source files (when building from repository context)
COPY --chown=ubuntu:ubuntu . ros2_ws/src/

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc

# Build workspace (if source files are present)
USER ubuntu
WORKDIR /home/ubuntu/ros2_ws

# Initialize rosdep (as ubuntu user)
RUN sudo rosdep init || true
RUN rosdep update

# Install dependencies and build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

# Set up GUI environment
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-ubuntu
RUN mkdir -p /tmp/runtime-ubuntu && chmod 0700 /tmp/runtime-ubuntu

# Create entrypoint script
RUN echo '#!/bin/bash' > /home/ubuntu/entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /home/ubuntu/entrypoint.sh && \
    echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then' >> /home/ubuntu/entrypoint.sh && \
    echo '    source ~/ros2_ws/install/setup.bash' >> /home/ubuntu/entrypoint.sh && \
    echo 'fi' >> /home/ubuntu/entrypoint.sh && \
    echo 'exec "$@"' >> /home/ubuntu/entrypoint.sh && \
    chmod +x /home/ubuntu/entrypoint.sh

# Set working directory
WORKDIR /home/ubuntu

# Set default command
ENTRYPOINT ["/home/ubuntu/entrypoint.sh"]
CMD ["/bin/bash"]

# Metadata
LABEL maintainer="Autonomous Rover Development Team"
LABEL description="ROS2 Humble development environment for autonomous mini rover"
LABEL version="1.0"

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD ros2 node list || exit 1

# Expose commonly used ports (adjust as needed)
EXPOSE 11311 11345
