"""
External Sensors Package for ROS2
Provides GPS and 10-axis IMU sensor publishers for autonomous rover

This package adds external sensor capabilities without affecting existing sensors.
It creates new topics with /external_* prefixes to avoid conflicts.

Modules:
    gps_publisher: Publishes GPS position and velocity data
    external_imu_publisher: Publishes 10-axis IMU data (accel, gyro, mag, pressure)

Topics Created:
    GPS:
        /external_gps/fix - GPS position (NavSatFix)
        /external_gps/velocity - GPS velocity (TwistStamped)
    
    IMU (10-axis):
        /external_imu/data - Main IMU data (Imu)
        /external_imu/mag - Magnetometer (MagneticField)
        /external_imu/pressure - Barometric pressure (FluidPressure)
        /external_imu/temperature - Temperature (Temperature)
        /external_imu/rpy - Roll/Pitch/Yaw angles (Vector3Stamped)

Usage:
    ros2 launch external_sensors external_sensors.launch.py

Author: Rover Team
Version: 1.0.0
"""

__version__ = '1.0.0'
__author__ = 'Rover Team'
__email__ = 'rover@example.com'

# Package imports
try:
    from .gps_publisher import GPSPublisher, NMEAParser, BinaryGPSParser
    from .external_imu_publisher import ExternalIMUPublisher, WITProtocolParser
except ImportError:
    # Handle import errors gracefully during package building
    pass

__all__ = [
    'GPSPublisher',
    'NMEAParser', 
    'BinaryGPSParser',
    'ExternalIMUPublisher',
    'WITProtocolParser'
]

# Package information
PACKAGE_INFO = {
    'name': 'external_sensors',
    'version': __version__,
    'description': 'External GPS and 10-axis IMU sensor publishers for ROS2',
    'sensors_supported': {
        'gps': {
            'protocols': ['NMEA', 'UBX Binary'],
            'topics': ['/external_gps/fix', '/external_gps/velocity'],
            'device': '/dev/gps_serial'
        },
        'imu': {
            'protocol': 'WIT (0x55 headers)',
            'axes': 10,
            'sensors': ['3-axis accelerometer', '3-axis gyroscope', '3-axis magnetometer', 'barometer'],
            'topics': ['/external_imu/data', '/external_imu/mag', '/external_imu/pressure', '/external_imu/temperature', '/external_imu/rpy'],
            'device': '/dev/imu_usb'
        }
    },
    'safety': {
        'conflicts': 'None - uses /external_* namespace',
        'existing_topics': 'Preserved - no interference with /imu or other topics'
    }
}

def get_package_info():
    """Return package information dictionary"""
    return PACKAGE_INFO

def print_package_info():
    """Print formatted package information"""
    print("=" * 60)
    print(f"External Sensors Package v{__version__}")
    print("=" * 60)
    print("GPS Support:")
    print(f"  Device: {PACKAGE_INFO['sensors_supported']['gps']['device']}")
    print(f"  Protocols: {', '.join(PACKAGE_INFO['sensors_supported']['gps']['protocols'])}")
    print(f"  Topics: {', '.join(PACKAGE_INFO['sensors_supported']['gps']['topics'])}")
    print()
    print("IMU Support:")
    print(f"  Device: {PACKAGE_INFO['sensors_supported']['imu']['device']}")
    print(f"  Protocol: {PACKAGE_INFO['sensors_supported']['imu']['protocol']}")
    print(f"  Axes: {PACKAGE_INFO['sensors_supported']['imu']['axes']}")
    print(f"  Sensors: {', '.join(PACKAGE_INFO['sensors_supported']['imu']['sensors'])}")
    print(f"  Topics: {', '.join(PACKAGE_INFO['sensors_supported']['imu']['topics'])}")
    print()
    print("Safety:")
    print(f"  Conflicts: {PACKAGE_INFO['safety']['conflicts']}")
    print(f"  Existing Topics: {PACKAGE_INFO['safety']['existing_topics']}")
    print("=" * 60)
