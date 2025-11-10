#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for IMU'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='9600',
            description='Baud rate for IMU'
        ),
        
        Node(
            package='wit_ros2_imu',
            executable='wit_imu_node',
            name='wit_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': 'imu_link',
            }]
        ),
    ])
