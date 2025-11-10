#!/usr/bin/env python3
"""
Direct GPS handler for serial NMEA GPS modules
Reads GPS data directly from serial port without ROS2 NMEA driver
"""

import time
import threading
import serial


class DirectGPSHandler:
    """Direct GPS handler that replaces ROS2 NMEA driver"""
    
    def __init__(self, port='/dev/ttyUSB0', baud=9600):
        """
        Initialize GPS handler
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            baud: Baud rate (default: 9600)
        """
        self.port = port
        self.baud = baud
        self.running = False
        self.thread = None
        self.buffer = ""
        
        # GPS data - thread-safe storage
        self.latest_gps_lat = 0.0
        self.latest_gps_lon = 0.0
        self.latest_gps_status = -1
        self.gps_data_available = False
        self.gps_fix_count = 0
        self._gps_lock = threading.Lock()
        
        # GUI callback
        self.gui_callback = None
    
    def set_gui_callback(self, callback):
        """
        Set callback function to update GUI
        
        Args:
            callback: Function(lat, lon, status) to call on GPS update
        """
        self.gui_callback = callback
    
    def start(self):
        """
        Start direct GPS reading thread
        
        Returns:
            True if started successfully, False otherwise
        """
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._gps_loop, daemon=True)
            self.thread.start()
            print("Direct GPS handler started")
            return True
        return False
    
    def stop(self):
        """Stop GPS reading thread"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        print("Direct GPS handler stopped")
    
    def _gps_loop(self):
        """Main GPS reading loop - reads NMEA sentences from serial port"""
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                print(f"Connected to GPS on {self.port}")
                
                # Clear buffer and sync
                ser.reset_input_buffer()
                time.sleep(0.5)
                
                while self.running:
                    try:
                        # Read byte by byte for proper synchronization
                        char = ser.read(1).decode('ascii', errors='ignore')
                        
                        if char:
                            self.buffer += char
                            
                            # Complete sentence found
                            if char == '\n' or char == '\r':
                                sentence = self.buffer.strip()
                                
                                if sentence.startswith('$') and '*' in sentence:
                                    self._process_sentence(sentence)
                                elif sentence and not sentence.startswith('$'):
                                    # Discard fragments
                                    pass
                                
                                self.buffer = ""
                    except Exception as e:
                        if self.running:
                            print(f"GPS read error: {e}")
                        time.sleep(0.1)
                        
        except Exception as e:
            print(f"GPS connection error: {e}")
    
    def _process_sentence(self, sentence):
        """
        Process complete NMEA sentence with checksum validation
        
        Args:
            sentence: Complete NMEA sentence string
        """
        try:
            # Validate checksum
            if '*' in sentence:
                data, checksum_received = sentence.split('*')
                checksum_received = checksum_received.strip()
                
                # Calculate checksum
                checksum_calc = 0
                for char in data[1:]:  # Skip $
                    checksum_calc ^= ord(char)
                checksum_expected = f"{checksum_calc:02X}"
                
                if checksum_received.upper() == checksum_expected:
                    self._parse_gps_data(sentence)
                    
        except Exception as e:
            print(f"Sentence processing error: {e}")
    
    def _parse_gps_data(self, sentence):
        """
        Parse GPS coordinates from validated NMEA sentence
        
        Args:
            sentence: Validated NMEA sentence (GGA format)
        """
        try:
            fields = sentence.split(',')
            
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                if len(fields) >= 6:
                    lat_raw = fields[2]
                    lat_dir = fields[3]
                    lon_raw = fields[4]
                    lon_dir = fields[5]
                    quality = fields[6] if len(fields) > 6 else '0'
                    
                    if lat_raw and lon_raw and lat_raw != '0' and lon_raw != '0':
                        # Convert DDMM.MMMM to decimal degrees
                        lat_deg = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                        lon_deg = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                        
                        if lat_dir == 'S':
                            lat_deg = -lat_deg
                        if lon_dir == 'W':
                            lon_deg = -lon_deg
                        
                        # Store GPS data thread-safely
                        with self._gps_lock:
                            self.latest_gps_lat = lat_deg
                            self.latest_gps_lon = lon_deg
                            self.latest_gps_status = int(quality) if quality.isdigit() else 0
                            self.gps_data_available = True
                            self.gps_fix_count += 1
                        
                        # Update GUI callback
                        if self.gui_callback:
                            self.gui_callback(lat_deg, lon_deg, int(quality) if quality.isdigit() else 0)
                        
                        # Progress report every 10 fixes
                        if self.gps_fix_count % 10 == 0:
                            print(f"GPS Fix #{self.gps_fix_count}: {lat_deg:.6f}, {lon_deg:.6f}")
                            
        except (ValueError, IndexError) as e:
            print(f"GPS parsing error: {e}")
    
    def get_gps_data(self):
        """
        Get current GPS data (thread-safe)
        
        Returns:
            Dictionary with keys: lat, lon, status, available, topic
        """
        with self._gps_lock:
            return {
                'lat': self.latest_gps_lat,
                'lon': self.latest_gps_lon,
                'status': self.latest_gps_status,
                'available': self.gps_data_available,
                'topic': 'direct_gps'
            }
