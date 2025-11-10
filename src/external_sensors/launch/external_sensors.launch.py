#!/usr/bin/env python3
"""
External Sensors Launch File
Launches GPS and external IMU publishers with TF transforms

This launch file starts both GPS and 10-axis IMU publishers safely
without interfering with existing ROS2 nodes.

Usage:
    ros2 launch external_sensors external_sensors.launch.py
    
Optional arguments:
    use_sim_time:=true/false (default: false)
    config_file:=path/to/config.yaml
    gps_enabled:=true/false (default: true)  
    imu_enabled:=true/false (default: true)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('external_sensors')
    
    # Configuration file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'external_sensors.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to sensor configuration file'
    )
    
    gps_enabled_arg = DeclareLaunchArgument(
        'gps_enabled',
        default_value='true',
        description='Enable GPS publisher'
    )
    
    imu_enabled_arg = DeclareLaunchArgument(
        'imu_enabled', 
        default_value='true',
        description='Enable external IMU publisher'
    )
    
    # GPS Publisher Node
    gps_publisher_node = Node(
        package='external_sensors',
        executable='gps_publisher',
        name='external_gps_publisher',
        namespace='external_sensors',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('gps_enabled'))
    )
    
    # External IMU Publisher Node  
    external_imu_publisher_node = Node(
        package='external_sensors',
        executable='external_imu_publisher',
        name='external_imu_publisher', 
        namespace='external_sensors',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('imu_enabled'))
    )
    
    # Static TF Publishers for sensor frames
    # GPS antenna transform (relative to base_link)
    gps_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_link_publisher',
        arguments=[
            '0.0', '0.0', '0.1',  # translation x, y, z (meters)
            '0.0', '0.0', '0.0', '1.0',  # rotation x, y, z, w (quaternion)
            'base_link',  # parent frame
            'gps_link'    # child frame
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gps_enabled'))
    )
    
    # External IMU transform (relative to base_link)
    external_imu_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='external_imu_link_publisher',
        arguments=[
            '0.0', '0.0', '0.05',  # translation x, y, z (meters)
            '0.0', '0.0', '0.0', '1.0',  # rotation x, y, z, w (quaternion)
            'base_link',           # parent frame
            'external_imu_link'    # child frame
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('imu_enabled'))
    )
    
    # Group all GPS-related nodes
    gps_group = GroupAction([
        gps_publisher_node,
        gps_tf_publisher
    ], condition=IfCondition(LaunchConfiguration('gps_enabled')))
    
    # Group all IMU-related nodes
    imu_group = GroupAction([
        external_imu_publisher_node,
        external_imu_tf_publisher  
    ], condition=IfCondition(LaunchConfiguration('imu_enabled')))
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        config_file_arg,
        gps_enabled_arg,
        imu_enabled_arg,
        
        # Sensor groups
        gps_group,
        imu_group
    ])

# Additional launch functions for individual sensors
def generate_gps_only_launch_description():
    """Launch only GPS publisher"""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('external_sensors'), 'config', 'external_sensors.yaml'
        ])),
        Node(
            package='external_sensors',
            executable='gps_publisher',
            name='external_gps_publisher',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_link_publisher',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', '1.0', 'base_link', 'gps_link']
        )
    ])

def generate_imu_only_launch_description():
    """Launch only external IMU publisher"""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('external_sensors'), 'config', 'external_sensors.yaml'
        ])),
        Node(
            package='external_sensors',
            executable='external_imu_publisher',
            name='external_imu_publisher',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='external_imu_link_publisher', 
            arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', '1.0', 'base_link', 'external_imu_link']
        )
    ])
