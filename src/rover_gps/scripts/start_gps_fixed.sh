#!/bin/bash
echo "??? Starting GPS with buffer synchronization fix..."

# Kill any existing GPS processes
pkill -f nmea_serial_driver 2>/dev/null
sleep 1

# Clear GPS serial buffer using Python
python3 -c "
import serial
import time
try:
    with serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1) as ser:
        ser.reset_input_buffer()  # Clear input buffer
        ser.reset_output_buffer() # Clear output buffer
        time.sleep(0.5)
        print('?? GPS buffer cleared')
except Exception as e:
    print(f'Buffer clear warning: {e}')
"

# Wait a moment
sleep 1

# Start NMEA driver with better parameters
echo "?? Starting ROS2 NMEA driver..."
ros2 run nmea_navsat_driver nmea_serial_driver \
    --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p baud:=9600 \
    -p frame_id:=gps_link \
    -p timeout:=2.0 \
    -p time_ref_source:=gps \
    --log-level INFO
