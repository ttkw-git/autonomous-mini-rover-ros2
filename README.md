# Autonomous Mini Rover ROS 2 Workspace

This repository contains the ROS 2 workspace that powers the autonomous mini rover platform. It
combines low-level motor control, perception pipelines, navigation, and higher level application
behaviours into a single `colcon` workspace that can be built for the physical rover or for
simulation.

## Repository layout

The workspace is organised as a typical ROS 2 overlay under `src/`:

| Package | Purpose |
| --- | --- |
| `app/` | Application nodes for interactive behaviours such as lidar obstacle avoidance, line following, object tracking, AR rendering, and hand gesture control. |
| `bringup/` | Launch files that start the core stack on the robot (controller, peripherals, app nodes, web bridge, joystick). |
| `calibration/` | Sensor calibration tools (camera, lidar, IMU) with supporting launch files and configurations. |
| `driver/` | Motor controller interface, odometry publisher, and SDK wrappers for the rover base. |
| `external_sensors/` | Integration of additional sensors (e.g., depth cameras) with parameterised launch files. |
| `interfaces/` | Custom message and service definitions used across the stack. |
| `multi/` | Multi-robot coordination utilities and RViz configurations. |
| `navigation/` | Navigation2 bringup with configurable localisation, path planning, and namespace support. |
| `peripherals/` | Drivers for lidar, cameras, joysticks, and auxiliary hardware. |
| `rover_gps/` & `simple_gps_ros2/` | GPS drivers and simple visualisers for outdoor localisation. |
| `rover_nav/` & `slam/` | Launch files for localisation/SLAM, prebuilt maps, and RViz setups. |
| `simulations/mentorpi_description/` | URDF description, meshes, and Gazebo support for simulation. |
| `wit_ros2_imu/` | WIT IMU driver and filter configuration. |
| `yolov5_ros2/` | Real-time object detection pipeline based on YOLOv5. |

Supporting resources (models, RViz layouts, parameters, and tests) are co-located inside each
package.

## Prerequisites

* ROS 2 Humble (or newer) on Ubuntu 22.04.
* Colcon build tools (`python3-colcon-common-extensions`).
* ROS 2 packages required by the workspace, including `robot_localization`, `rosbridge_server`,
  `web_video_server`, Navigation2, and sensor drivers such as `ldlidar` and camera drivers.
* Python dependencies used by application nodes (`numpy`, `opencv-python`, `pandas`, `scipy`,
  PyTorch for YOLOv5 if you plan to run detection, etc.). Install them in your ROS 2 environment or
  virtual environment as appropriate.

## Building the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
ln -s /path/to/autonomous-mini-rover-ros2/src src
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** several launch files expect environment variables to decide whether packages should be
> resolved from an installed underlay or from the source tree. Set `need_compile=True` when running
> from an installed environment. When developing inside this workspace, leave it unset or set it to
> `False` so the launch files use the local `~/ros2_ws/src` paths. Some application nodes also read
> `HOST`, `MASTER`, `LIDAR_TYPE`, and `MACHINE_TYPE` to configure networking and hardware-specific
> behaviour.

## Running key components

* **Full robot bringup** – boots the controller, sensors, joystick teleop, rosbridge, web video
  server, and application nodes:
  ```bash
  ros2 launch bringup bringup.launch.py need_compile:=False
  ```
* **Navigation stack** – brings up the robot base, SLAM/localisation, and Navigation2 stack. Provide
  a map name from `src/slam/maps` and make sure the required environment variables are set:
  ```bash
  export HOST=rover01
  export MASTER=rover-master
  ros2 launch navigation navigation.launch.py map:=map_01 sim:=false need_compile:=False
  ```
* **Application behaviours** – start individual features such as lidar obstacle avoidance or
  gesture control via their dedicated launch files in `src/app/launch/`, for example:
  ```bash
  ros2 launch app lidar_node.launch.py need_compile:=False
  ```
* **Simulation** – launch the URDF and Gazebo support contained in
  `src/simulations/mentorpi_description` to test the stack without hardware.

Refer to the launch files inside each package for additional arguments (namespace selection,
parameter overrides, etc.).

## Testing and linting

Many packages include `pytest`-based tests and ROS 2 linters. After building, run:

```bash
colcon test
colcon test-result --verbose
```

Fix any reported linting issues before committing changes.

## Contributing

1. Fork and clone the repository.
2. Create a new branch for your feature or fix.
3. Build and test with `colcon` as described above.
4. Submit a pull request describing your changes and testing steps.

Please also update package-specific documentation (launch instructions, parameter descriptions,
hardware requirements) when you make relevant changes so that other rover developers can stay in
sync.
