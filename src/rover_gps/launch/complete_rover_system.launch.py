#!/usr/bin/env python3

"""
Complete Rover System Launch File
Starts robot controller, GPS, and GUI together
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Robot controller
    robot_controller = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_robot_controller', 'ros_robot_controller.launch.py'],
        output='screen'
    )
    
    # GPS driver
    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud': 9600,
            'frame_id': 'gps_link',
        }],
        remappings=[
            ('/fix', '/gps/fix'),
        ],
        output='screen'
    )
    
    # GUI (with delay to let ROS nodes start first)
    gui_launcher = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 3 && python3 -m rover_gps.gui.rover_control_gui'],
        output='screen'
    )

    return LaunchDescription([
        robot_controller,
        gps_driver,
        gui_launcher,
    ])
