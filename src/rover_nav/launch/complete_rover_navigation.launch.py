#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Complete rover navigation launch with SLAM and Nav2
    This will make your rover fully autonomous!
    """
    
    # Get directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # SLAM Toolbox node for real-time mapping
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan_raw',  # Your actual LiDAR topic
            'mode': 'mapping',
            'debug_logging': False,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True
        }]
    )
    
    # Navigation launch with your corrected config
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/navigation_launch.py']),
        launch_arguments={
            'params_file': '/home/ubuntu/ros2_ws/config/nav2_params_rover_corrected.yaml'
        }.items()
    )
    
    return LaunchDescription([
        slam_node,
        navigation_launch
    ])
