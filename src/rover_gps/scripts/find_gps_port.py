#!/usr/bin/env python3
import serial.tools.list_ports
import sys

def find_gps_port():
    """Find GPS device automatically"""
    for port in serial.tools.list_ports.comports():
        try:
            # Try to open and read from each port
            with serial.Serial(port.device, 9600, timeout=1) as ser:
                data = ser.read(100).decode('ascii', errors='ignore')
                # Check for NMEA GPS sentences
                if any(gps_id in data for gps_id in ['$GNGGA', '$GPGGA', '$GNGLL']):
                    return port.device
        except:
            continue
    return None

if __name__ == '__main__':
    port = find_gps_port()
    if port:
        print(port)
        sys.exit(0)
    else:
        print('/dev/ttyUSB0')  # Fallback
        sys.exit(1)
