#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import time

class SimpleGPSNode(Node):
    def __init__(self):
        super().__init__('simple_gps_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'gps_link')
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Publisher
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        
        # Timer for GPS reading
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        
        # Initialize serial connection
        self.setup_serial()
        
        self.get_logger().info(f'GPS node started on {self.port}')
    
    def setup_serial(self):
        """Setup serial connection to GPS"""
        possible_ports = [
            self.port,
            '/dev/gps_usb',
            '/dev/ttyUSB1',
            '/dev/ttyUSB2',
            '/dev/ttyACM1'
        ]
        
        self.serial_port = None
        for port in possible_ports:
            try:
                self.serial_port = serial.Serial(port, self.baud_rate, timeout=1)
                self.get_logger().info(f'GPS serial opened on {port}')
                break
            except Exception as e:
                self.get_logger().debug(f'Could not open {port}: {e}')
                continue
        
        if self.serial_port is None:
            self.get_logger().warn('Could not open GPS serial port, will publish dummy data')
    
    def publish_gps_data(self):
        """Publish GPS data (real or dummy)"""
        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = self.frame_id
        
        # Initialize status
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        
        if self.serial_port and self.serial_port.is_open:
            try:
                # Try to read real GPS data
                for _ in range(10):  # Try up to 10 lines
                    line = self.serial_port.readline().decode('ascii', errors='replace').strip()
                    
                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parts = line.split(',')
                        if len(parts) >= 15 and parts[2] and parts[4]:
                            try:
                                # Parse latitude
                                lat_deg = float(parts[2][:2])
                                lat_min = float(parts[2][2:])
                                latitude = lat_deg + lat_min / 60.0
                                if parts[3] == 'S':
                                    latitude = -latitude
                                
                                # Parse longitude
                                lon_deg = float(parts[4][:3])
                                lon_min = float(parts[4][3:])
                                longitude = lon_deg + lon_min / 60.0
                                if parts[5] == 'W':
                                    longitude = -longitude
                                
                                # Parse altitude
                                altitude = float(parts[9]) if parts[9] else 0.0
                                
                                # Set values
                                fix_msg.latitude = float(latitude)
                                fix_msg.longitude = float(longitude)
                                fix_msg.altitude = float(altitude)
                                
                                # Set status based on fix quality
                                fix_quality = int(parts[6]) if parts[6] else 0
                                if fix_quality >= 1:
                                    fix_msg.status.status = NavSatStatus.STATUS_FIX
                                else:
                                    fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
                                
                                self.fix_pub.publish(fix_msg)
                                self.get_logger().info(f'GPS: {latitude:.6f}, {longitude:.6f}, Alt: {altitude:.2f}m')
                                return
                            except (ValueError, IndexError) as e:
                                self.get_logger().debug(f'GPS parse error: {e}')
                                continue
                        
            except Exception as e:
                self.get_logger().error(f'GPS read error: {e}')
        
        # Publish dummy data if no real GPS or indoor
        fix_msg.latitude = 40.7128  # New York latitude as dummy
        fix_msg.longitude = -74.0060  # New York longitude as dummy
        fix_msg.altitude = 10.0
        fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        
        # Set covariance (optional)
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        self.fix_pub.publish(fix_msg)
        self.get_logger().info('GPS: Publishing dummy data (indoor/no fix)')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGPSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
