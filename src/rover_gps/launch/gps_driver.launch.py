#!/usr/bin/env python3

"""
GPS Driver Launch File for Rover
Launches NMEA GPS driver with configurable parameters
File: /home/ubuntu/ros2_ws/src/rover_gps/launch/gps_driver.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='GPS serial port'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='9600',
        description='GPS baud rate'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps',
        description='GPS frame ID'
    )

    # GPS driver node
    gps_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
            'time_ref_source': LaunchConfiguration('frame_id'),
            'useRMC': False,
        }],
        remappings=[
            ('/fix', '/gps/fix'),
            ('/vel', '/gps/vel'),
            ('/time_reference', '/gps/time_reference'),
        ],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_id_arg,
        gps_driver_node,
    ])
