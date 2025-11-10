#!/usr/bin/env python3
"""
External 10-Axis IMU Publisher Node (Fixed - Always Publish RPY)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
import serial
import threading
import time
import struct
import math
from dataclasses import dataclass

@dataclass
class IMUData:
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    data_valid: bool = False

class WITProtocolParser:
    def __init__(self):
        self.key = 0
        self.buff = {}
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        
    def check_sum(self, list_data, check_data):
        return sum(list_data) & 0xff == check_data
    
    def hex_to_short(self, raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))
    
    def handle_serial_data(self, raw_data):
        imu_data = IMUData()
        
        self.buff[self.key] = raw_data
        self.key += 1
        
        if self.buff.get(0) != 0x55:
            self.key = 0
            return None
        
        if self.key < 11:
            return None
        
        data_buff = list(self.buff.values())
        packet_type = self.buff.get(1)
        
        if packet_type == 0x51:  # Acceleration
            if self.check_sum(data_buff[0:10], data_buff[10]):
                self.acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                imu_data.data_valid = True
                
        elif packet_type == 0x52:  # Angular velocity
            if self.check_sum(data_buff[0:10], data_buff[10]):
                self.angular_velocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
                imu_data.data_valid = True
                
        elif packet_type == 0x53:  # Angles
            if self.check_sum(data_buff[0:10], data_buff[10]):
                self.angle_degree = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                imu_data.data_valid = True
                
        elif packet_type == 0x54:  # Magnetometer
            if self.check_sum(data_buff[0:10], data_buff[10]):
                self.magnetometer = self.hex_to_short(data_buff[2:10])
                imu_data.data_valid = True
        
        self.buff = {}
        self.key = 0
        
        if imu_data.data_valid:
            # Always include current values
            imu_data.accel_x = float(self.acceleration[0])
            imu_data.accel_y = float(self.acceleration[1])
            imu_data.accel_z = float(self.acceleration[2])
            imu_data.gyro_x = float(self.angular_velocity[0])
            imu_data.gyro_y = float(self.angular_velocity[1])
            imu_data.gyro_z = float(self.angular_velocity[2])
            imu_data.roll = float(self.angle_degree[0])
            imu_data.pitch = float(self.angle_degree[1])
            imu_data.yaw = float(self.angle_degree[2])
            imu_data.mag_x = float(self.magnetometer[0])
            imu_data.mag_y = float(self.magnetometer[1])
            imu_data.mag_z = float(self.magnetometer[2])
            return imu_data
        
        return None

class ExternalIMUPublisher(Node):
    def __init__(self):
        super().__init__('external_imu_publisher')
        
        self.declare_parameter('device_port', '/dev/imu_usb')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'external_imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('topic_name', '/external_imu/data')
        
        self.device_port = self.get_parameter('device_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.topic_name = self.get_parameter('topic_name').value
        
        # Publishers
        self.imu_publisher = self.create_publisher(Imu, self.topic_name, 10)
        self.mag_publisher = self.create_publisher(MagneticField, f"{self.topic_name.replace('/data', '/mag')}", 10)
        self.rpy_publisher = self.create_publisher(Vector3Stamped, f"{self.topic_name.replace('/data', '/rpy')}", 10)
        
        self.current_imu_data = IMUData()
        self.data_lock = threading.Lock()
        self.wit_parser = WITProtocolParser()
        self.serial_connection = None
        self.running = False
        self.packet_count = 0
        self.valid_data_count = 0
        
        self.start_imu_thread()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)
        self.status_timer = self.create_timer(5.0, self.log_status)
        
        self.get_logger().info(f'External IMU Publisher started on {self.topic_name}')
    
    def start_imu_thread(self):
        self.running = True
        self.imu_thread = threading.Thread(target=self.imu_reader_thread)
        self.imu_thread.daemon = True
        self.imu_thread.start()
    
    def imu_reader_thread(self):
        while self.running:
            try:
                if self.serial_connection is None:
                    self.serial_connection = serial.Serial(self.device_port, self.baud_rate, timeout=0.5)
                
                if self.serial_connection:
                    buff_count = self.serial_connection.in_waiting
                    if buff_count > 0:
                        buff_data = self.serial_connection.read(buff_count)
                        self.packet_count += buff_count
                        
                        for i in range(buff_count):
                            imu_data = self.wit_parser.handle_serial_data(buff_data[i])
                            if imu_data and imu_data.data_valid:
                                with self.data_lock:
                                    self.current_imu_data = imu_data
                                self.valid_data_count += 1
                
                time.sleep(0.001)
                
            except Exception as e:
                if self.serial_connection:
                    self.serial_connection.close()
                    self.serial_connection = None
                time.sleep(1.0)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]
    
    def publish_imu_data(self):
        with self.data_lock:
            if not self.current_imu_data.data_valid:
                return
            
            current_time = self.get_clock().now().to_msg()
            
            # 1. Main IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = self.frame_id
            
            imu_msg.linear_acceleration.x = float(self.current_imu_data.accel_x)
            imu_msg.linear_acceleration.y = float(self.current_imu_data.accel_y)
            imu_msg.linear_acceleration.z = float(self.current_imu_data.accel_z)
            
            imu_msg.angular_velocity.x = float(self.current_imu_data.gyro_x)
            imu_msg.angular_velocity.y = float(self.current_imu_data.gyro_y)
            imu_msg.angular_velocity.z = float(self.current_imu_data.gyro_z)
            
            # Orientation
            roll_rad = math.radians(float(self.current_imu_data.roll))
            pitch_rad = math.radians(float(self.current_imu_data.pitch))
            yaw_rad = math.radians(float(self.current_imu_data.yaw))
            
            quat = self.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
            imu_msg.orientation.x = float(quat[0])
            imu_msg.orientation.y = float(quat[1])
            imu_msg.orientation.z = float(quat[2])
            imu_msg.orientation.w = float(quat[3])
            
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.angular_velocity_covariance[4] = 0.001
            imu_msg.angular_velocity_covariance[8] = 0.001
            
            self.imu_publisher.publish(imu_msg)
            
            # 2. Magnetometer message
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = float(self.current_imu_data.mag_x)
            mag_msg.magnetic_field.y = float(self.current_imu_data.mag_y)
            mag_msg.magnetic_field.z = float(self.current_imu_data.mag_z)
            mag_msg.magnetic_field_covariance[0] = 0.01
            mag_msg.magnetic_field_covariance[4] = 0.01
            mag_msg.magnetic_field_covariance[8] = 0.01
            self.mag_publisher.publish(mag_msg)
            
            # 3. RPY message (ALWAYS publish)
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = current_time
            rpy_msg.header.frame_id = self.frame_id
            rpy_msg.vector.x = float(self.current_imu_data.roll)
            rpy_msg.vector.y = float(self.current_imu_data.pitch)
            rpy_msg.vector.z = float(self.current_imu_data.yaw)
            self.rpy_publisher.publish(rpy_msg)
    
    def log_status(self):
        with self.data_lock:
            self.get_logger().info(
                f'IMU: Packets: {self.packet_count} | Valid: {self.valid_data_count} | '
                f'Accel: [{self.current_imu_data.accel_x:.2f}, {self.current_imu_data.accel_y:.2f}, {self.current_imu_data.accel_z:.2f}] | '
                f'RPY: [{self.current_imu_data.roll:.1f}, {self.current_imu_data.pitch:.1f}, {self.current_imu_data.yaw:.1f}]'
            )
    
    def destroy_node(self):
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        imu_publisher = ExternalIMUPublisher()
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'imu_publisher' in locals():
            imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
