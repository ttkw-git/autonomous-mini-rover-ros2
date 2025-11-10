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
            description='Serial port for GPS'
        ),
        
        Node(
            package='simple_gps_ros2',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': 9600,
                'frame_id': 'gps_link',
            }]
        ),
    ])
