#!/usr/bin/env python3

import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

# Global variables for sensor data
key = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]

def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def get_quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity
    angle_flag = False
    buff[key] = raw_data
    key += 1
    
    if buff[0] != 0x55:
        key = 0
        return False
        
    if key < 11:
        return False
    else:
        data_buff = list(buff.values())
        if buff[1] == 0x51:  # Acceleration
            if check_sum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
        elif buff[1] == 0x52:  # Angular velocity
            if check_sum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
        elif buff[1] == 0x53:  # Angle
            if check_sum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True
        elif buff[1] == 0x54:  # Magnetometer
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])

        buff = {}
        key = 0
        return angle_flag

class WitIMUNode(Node):
    def __init__(self):
        super().__init__('wit_imu_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Initialize messages
        self.setup_imu_message()
        
        # Start serial communication
        self.serial_thread = threading.Thread(target=self.serial_worker)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info(f'WIT IMU Node started on {self.port} at {self.baud_rate} baud')

    def setup_imu_message(self):
        """Setup IMU message with covariance matrices"""
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id
        
        # Set covariance matrices (9 elements each, as float64)
        # Format: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        self.imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        self.imu_msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        self.imu_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.mag_msg = MagneticField()
        self.mag_msg.header.frame_id = self.frame_id
        # Magnetometer covariance matrix
        self.mag_msg.magnetic_field_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

    def serial_worker(self):
        """Serial communication worker thread"""
        possible_ports = [
            self.port, 
            '/dev/imu_usb', 
            '/dev/ttyUSB0', 
            '/dev/ttyUSB1', 
            '/dev/ttyACM0',
            '/dev/ttyACM1'
        ]
        
        ser = None
        for port in possible_ports:
            try:
                ser = serial.Serial(port=port, baudrate=self.baud_rate, timeout=0.5)
                if ser.isOpen():
                    self.get_logger().info(f"Serial port opened successfully on {port}")
                    break
            except Exception as e:
                self.get_logger().debug(f"Could not open {port}: {e}")
                continue
        
        if ser is None:
            self.get_logger().warn("Could not open any serial port for IMU - will publish dummy data")
            # Start dummy data publisher instead
            self.dummy_timer = self.create_timer(0.1, self.publish_dummy_data)
            return
                
        try:
            while rclpy.ok():
                data = ser.read(1)
                if len(data) > 0:
                    if handle_serial_data(data[0]):
                        self.publish_imu_data()
                time.sleep(0.001)
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
        finally:
            if ser and ser.isOpen():
                ser.close()

    def publish_dummy_data(self):
        """Publish dummy IMU data when no hardware is connected"""
        now = self.get_clock().now()
        
        # Publish dummy IMU data
        self.imu_msg.header.stamp = now.to_msg()
        
        # Set dummy values
        self.imu_msg.linear_acceleration.x = 0.0
        self.imu_msg.linear_acceleration.y = 0.0
        self.imu_msg.linear_acceleration.z = 9.81  # Gravity
        
        self.imu_msg.angular_velocity.x = 0.0
        self.imu_msg.angular_velocity.y = 0.0
        self.imu_msg.angular_velocity.z = 0.0
        
        # Identity quaternion (no rotation)
        self.imu_msg.orientation.x = 0.0
        self.imu_msg.orientation.y = 0.0
        self.imu_msg.orientation.z = 0.0
        self.imu_msg.orientation.w = 1.0
        
        self.imu_pub.publish(self.imu_msg)
        
        # Publish dummy magnetometer data
        self.mag_msg.header.stamp = now.to_msg()
        self.mag_msg.magnetic_field.x = 0.0
        self.mag_msg.magnetic_field.y = 0.0
        self.mag_msg.magnetic_field.z = 0.0
        
        self.mag_pub.publish(self.mag_msg)

    def publish_imu_data(self):
        """Publish real IMU and magnetometer data"""
        global acceleration, angularVelocity, angle_degree, magnetometer
        
        now = self.get_clock().now()
        
        # Publish IMU data
        self.imu_msg.header.stamp = now.to_msg()
        
        # Linear acceleration (m/s^2)
        self.imu_msg.linear_acceleration.x = float(acceleration[0])
        self.imu_msg.linear_acceleration.y = float(acceleration[1])
        self.imu_msg.linear_acceleration.z = float(acceleration[2])
        
        # Angular velocity (rad/s)
        self.imu_msg.angular_velocity.x = float(angularVelocity[0])
        self.imu_msg.angular_velocity.y = float(angularVelocity[1])
        self.imu_msg.angular_velocity.z = float(angularVelocity[2])
        
        # Convert Euler angles to quaternion
        angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        quat = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
        
        self.imu_msg.orientation.x = float(quat[0])
        self.imu_msg.orientation.y = float(quat[1])
        self.imu_msg.orientation.z = float(quat[2])
        self.imu_msg.orientation.w = float(quat[3])
        
        self.imu_pub.publish(self.imu_msg)
        
        # Publish magnetometer data
        self.mag_msg.header.stamp = now.to_msg()
        self.mag_msg.magnetic_field.x = float(magnetometer[0] * 1e-6)  # Convert to Tesla
        self.mag_msg.magnetic_field.y = float(magnetometer[1] * 1e-6)
        self.mag_msg.magnetic_field.z = float(magnetometer[2] * 1e-6)
        
        self.mag_pub.publish(self.mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WitIMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
