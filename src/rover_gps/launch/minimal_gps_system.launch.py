#!/usr/bin/env python3

"""
Minimal GPS System Launch File
Launches GPS + minimal controller
File: minimal_gps_system.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # GPS Driver (we already tested this works)
    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud': 9600,
            'frame_id': 'gps_link',
            'useRMC': False,
        }],
        remappings=[
            ('/fix', '/gps/fix'),
            ('/vel', '/gps/vel'),
        ],
        output='screen'
    )

    # Minimal GPS Controller
    gps_controller = Node(
        package='rover_gps',
        executable='minimal_gps_controller',
        name='minimal_gps_controller',
        output='screen'
    )

    return LaunchDescription([
        gps_driver,
        gps_controller,
    ])
