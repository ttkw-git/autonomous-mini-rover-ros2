#!/usr/bin/env python3
"""
External GPS Publisher Node
Publishes GPS data to /external_gps/fix topic
Handles NMEA protocol and binary GPS formats
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
import serial
import threading
import time
import math
import struct
from dataclasses import dataclass
from typing import Optional

@dataclass
class GPSData:
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    speed_knots: Optional[float] = None
    course: Optional[float] = None
    satellites: Optional[int] = None
    fix_quality: Optional[int] = None
    hdop: Optional[float] = None
    utc_time: Optional[str] = None

class NMEAParser:
    @staticmethod
    def parse_coordinate(coord_str: str, direction: str) -> Optional[float]:
        """Convert NMEA coordinate format to decimal degrees"""
        if not coord_str or not direction:
            return None
        
        try:
            if direction in ['N', 'S']:  # Latitude DDMM.MMMM
                degrees = int(coord_str[:2])
                minutes = float(coord_str[2:])
            else:  # Longitude DDDMM.MMMM
                degrees = int(coord_str[:3])
                minutes = float(coord_str[3:])
            
            decimal = degrees + minutes / 60.0
            
            if direction in ['S', 'W']:
                decimal = -decimal
            
            return decimal
        except (ValueError, IndexError):
            return None
    
    @staticmethod
    def parse_gga(sentence: str) -> GPSData:
        """Parse GGA sentence (Global Positioning System Fix Data)"""
        gps_data = GPSData()
        
        try:
            parts = sentence.split(',')
            if len(parts) >= 15 and parts[0] == '$GPGGA':
                gps_data.utc_time = parts[1]
                gps_data.latitude = NMEAParser.parse_coordinate(parts[2], parts[3])
                gps_data.longitude = NMEAParser.parse_coordinate(parts[4], parts[5])
                gps_data.fix_quality = int(parts[6]) if parts[6] else None
                gps_data.satellites = int(parts[7]) if parts[7] else None
                gps_data.hdop = float(parts[8]) if parts[8] else None
                gps_data.altitude = float(parts[9]) if parts[9] else None
        except (ValueError, IndexError):
            pass
        
        return gps_data
    
    @staticmethod
    def parse_rmc(sentence: str) -> GPSData:
        """Parse RMC sentence (Recommended Minimum Course)"""
        gps_data = GPSData()
        
        try:
            parts = sentence.split(',')
            if len(parts) >= 12 and parts[0] == '$GPRMC':
                gps_data.utc_time = parts[1]
                gps_data.latitude = NMEAParser.parse_coordinate(parts[3], parts[4])
                gps_data.longitude = NMEAParser.parse_coordinate(parts[5], parts[6])
                gps_data.speed_knots = float(parts[7]) if parts[7] else None
                gps_data.course = float(parts[8]) if parts[8] else None
        except (ValueError, IndexError):
            pass
        
        return gps_data

class BinaryGPSParser:
    @staticmethod
    def find_nmea_in_binary(data: bytes) -> Optional[str]:
        """Try to find NMEA sentences in binary data"""
        try:
            # Try different encodings to find NMEA sentences
            for encoding in ['utf-8', 'ascii', 'latin1', 'cp437']:
                try:
                    text = data.decode(encoding, errors='ignore')
                    lines = text.split('\n')
                    
                    for line in lines:
                        line = line.strip()
                        if line.startswith('$GP') and len(line) > 10 and '*' in line:
                            return line
                except:
                    continue
        except:
            pass
        
        return None
    
    @staticmethod
    def parse_ubx_nav_pvt(data: bytes) -> Optional[GPSData]:
        """Parse UBX NAV-PVT message (if GPS uses UBX protocol)"""
        if len(data) < 92 or data[0] != 0xB5 or data[1] != 0x62:
            return None
        
        try:
            msg_class = data[2]
            msg_id = data[3]
            length = struct.unpack('<H', data[4:6])[0]
            
            if msg_class == 0x01 and msg_id == 0x07 and length >= 84:  # NAV-PVT
                payload = data[6:6+length]
                
                gps_data = GPSData()
                
                # Parse NAV-PVT payload (UBX format)
                gps_data.longitude = struct.unpack('<l', payload[24:28])[0] * 1e-7
                gps_data.latitude = struct.unpack('<l', payload[28:32])[0] * 1e-7
                gps_data.altitude = struct.unpack('<l', payload[32:36])[0] * 1e-3
                gps_data.satellites = payload[23]
                gps_data.fix_quality = payload[20]
                
                # Calculate speed from velocity components
                vel_n = struct.unpack('<l', payload[48:52])[0] * 1e-3  # North velocity mm/s to m/s
                vel_e = struct.unpack('<l', payload[52:56])[0] * 1e-3  # East velocity mm/s to m/s
                speed_ms = math.sqrt(vel_n*vel_n + vel_e*vel_e)
                gps_data.speed_knots = speed_ms * 1.943844  # m/s to knots
                
                # Calculate course
                if vel_e != 0 or vel_n != 0:
                    gps_data.course = math.degrees(math.atan2(vel_e, vel_n))
                    if gps_data.course < 0:
                        gps_data.course += 360
                
                return gps_data
                
        except Exception as e:
            pass
        
        return None

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('external_gps_publisher')
        
        # Declare parameters
        self.declare_parameter('device_port', '/dev/gps_serial')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('topic_name', '/external_gps/fix')
        
        # Get parameters
        self.device_port = self.get_parameter('device_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.topic_name = self.get_parameter('topic_name').value
        
        # Publishers
        self.fix_publisher = self.create_publisher(
            NavSatFix, 
            self.topic_name, 
            10
        )
        
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            f"{self.topic_name.replace('/fix', '/velocity')}",
            10
        )
        
        # GPS data storage
        self.current_gps_data = GPSData()
        self.data_lock = threading.Lock()
        
        # Serial connection
        self.serial_connection = None
        self.running = False
        
        # Statistics
        self.nmea_count = 0
        self.valid_fix_count = 0
        
        # Start GPS reading thread
        self.start_gps_thread()
        
        # Publisher timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_gps_data
        )
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.log_status)
        
        self.get_logger().info(f'External GPS Publisher started on {self.topic_name}')
        self.get_logger().info(f'GPS device: {self.device_port} at {self.baud_rate} baud')
    
    def start_gps_thread(self):
        """Start GPS data reading thread"""
        self.running = True
        self.gps_thread = threading.Thread(target=self.gps_reader_thread)
        self.gps_thread.daemon = True
        self.gps_thread.start()
    
    def gps_reader_thread(self):
        """GPS data reading thread"""
        buffer = b''
        retry_count = 0
        
        while self.running:
            try:
                if self.serial_connection is None:
                    self.get_logger().info(f'Connecting to GPS at {self.device_port}')
                    self.serial_connection = serial.Serial(
                        self.device_port, 
                        self.baud_rate, 
                        timeout=1
                    )
                    
                    # Send GPS configuration commands
                    self.configure_gps()
                    retry_count = 0
                
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    chunk = self.serial_connection.read(self.serial_connection.in_waiting)
                    buffer += chunk
                    
                    # Try ASCII/NMEA parsing first
                    try:
                        text = buffer.decode('utf-8', errors='ignore')
                        lines = text.split('\n')
                        
                        for line in lines[:-1]:  # Process complete lines
                            line = line.strip()
                            if line:
                                self.process_nmea_line(line)
                        
                        # Keep last incomplete line in buffer
                        buffer = lines[-1].encode('utf-8')
                        
                    except UnicodeDecodeError:
                        # Try binary parsing for UBX or other binary protocols
                        self.process_binary_data(buffer)
                        buffer = b''
                
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
            except Exception as e:
                retry_count += 1
                self.get_logger().warning(f'GPS connection error (attempt {retry_count}): {e}')
                if self.serial_connection:
                    self.serial_connection.close()
                    self.serial_connection = None
                
                # Exponential backoff for retries
                time.sleep(min(retry_count * 0.5, 5.0))
    
    def process_nmea_line(self, line: str):
        """Process a single NMEA line"""
        self.nmea_count += 1
        
        if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
            gps_data = NMEAParser.parse_gga(line)
            if gps_data.latitude and gps_data.longitude:
                with self.data_lock:
                    # Update position data
                    self.current_gps_data.latitude = gps_data.latitude
                    self.current_gps_data.longitude = gps_data.longitude
                    self.current_gps_data.altitude = gps_data.altitude
                    self.current_gps_data.satellites = gps_data.satellites
                    self.current_gps_data.fix_quality = gps_data.fix_quality
                    self.current_gps_data.hdop = gps_data.hdop
                    self.current_gps_data.utc_time = gps_data.utc_time
                self.valid_fix_count += 1
        
        elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
            gps_data = NMEAParser.parse_rmc(line)
            if gps_data.speed_knots is not None:
                with self.data_lock:
                    self.current_gps_data.speed_knots = gps_data.speed_knots
                    self.current_gps_data.course = gps_data.course
    
    def process_binary_data(self, data: bytes):
        """Process binary GPS data"""
        # Look for NMEA in binary data
        nmea = BinaryGPSParser.find_nmea_in_binary(data)
        if nmea:
            self.process_nmea_line(nmea)
            return
        
        # Try UBX parsing
        if len(data) >= 8:
            for i in range(len(data) - 8):
                if data[i] == 0xB5 and data[i+1] == 0x62:
                    # Found potential UBX header
                    gps_data = BinaryGPSParser.parse_ubx_nav_pvt(data[i:])
                    if gps_data and gps_data.latitude and gps_data.longitude:
                        with self.data_lock:
                            self.current_gps_data = gps_data
                        self.valid_fix_count += 1
                        break
    
    def configure_gps(self):
        """Send configuration commands to GPS"""
        if not self.serial_connection:
            return
        
        try:
            # Common GPS configuration commands
            commands = [
                # Enable GGA and RMC sentences at 1Hz
                b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n",
                # Set update rate to 1Hz  
                b"$PMTK220,1000*1F\r\n",
                # Set to use GPS + GLONASS
                b"$PMTK353,1,1,0,0,0*2A\r\n",
            ]
            
            for cmd in commands:
                self.serial_connection.write(cmd)
                time.sleep(0.1)
                
            self.get_logger().info('GPS configuration commands sent')
            
        except Exception as e:
            self.get_logger().warning(f'GPS configuration failed: {e}')
    
    def publish_gps_data(self):
        """Publish GPS data to ROS topics"""
        with self.data_lock:
            if self.current_gps_data.latitude is None or self.current_gps_data.longitude is None:
                return
            
            current_time = self.get_clock().now().to_msg()
            
            # Create NavSatFix message
            fix_msg = NavSatFix()
            fix_msg.header.stamp = current_time
            fix_msg.header.frame_id = self.frame_id
            
            fix_msg.latitude = self.current_gps_data.latitude
            fix_msg.longitude = self.current_gps_data.longitude
            fix_msg.altitude = self.current_gps_data.altitude or 0.0
            
            # Set status based on fix quality
            if self.current_gps_data.fix_quality and self.current_gps_data.fix_quality > 0:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            else:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            
            # Set covariance based on HDOP
            hdop = self.current_gps_data.hdop or 5.0
            fix_msg.position_covariance[0] = hdop * hdop  # East
            fix_msg.position_covariance[4] = hdop * hdop  # North  
            fix_msg.position_covariance[8] = (hdop * 2) * (hdop * 2)  # Up
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            self.fix_publisher.publish(fix_msg)
            
            # Publish velocity if available
            if self.current_gps_data.speed_knots is not None:
                vel_msg = TwistStamped()
                vel_msg.header.stamp = current_time
                vel_msg.header.frame_id = self.frame_id
                
                # Convert speed and course to velocity components
                speed_ms = self.current_gps_data.speed_knots * 0.514444  # knots to m/s
                course_rad = math.radians(self.current_gps_data.course or 0.0)
                
                vel_msg.twist.linear.x = speed_ms * math.cos(course_rad)
                vel_msg.twist.linear.y = speed_ms * math.sin(course_rad)
                vel_msg.twist.linear.z = 0.0
                
                self.velocity_publisher.publish(vel_msg)
    
    def log_status(self):
        """Log GPS status periodically"""
        with self.data_lock:
            if self.current_gps_data.latitude and self.current_gps_data.longitude:
                self.get_logger().info(
                    f'GPS: {self.current_gps_data.latitude:.6f}, {self.current_gps_data.longitude:.6f} | '
                    f'Sats: {self.current_gps_data.satellites or 0} | '
                    f'Fix: {self.current_gps_data.fix_quality or 0} | '
                    f'NMEA: {self.nmea_count} | Valid: {self.valid_fix_count}'
                )
            else:
                self.get_logger().info(f'GPS: No fix | NMEA received: {self.nmea_count}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gps_publisher = GPSPublisher()
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gps_publisher' in locals():
            gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
