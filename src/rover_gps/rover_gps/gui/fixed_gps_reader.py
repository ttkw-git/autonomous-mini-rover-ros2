#!/usr/bin/env python3
import serial
import time
import threading

class FixedGPSReader:
    def __init__(self, port='/dev/ttyUSB0', baud=9600):
        self.port = port
        self.baud = baud
        self.running = False
        self.buffer = ""
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.start()
        
    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
    
    def _read_loop(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                print(f"??? Connected to GPS on {self.port}")
                
                # Clear buffer and sync to start of sentence
                ser.reset_input_buffer()
                time.sleep(0.5)
                
                while self.running:
                    try:
                        # Read one byte at a time to sync properly
                        char = ser.read(1).decode('ascii', errors='ignore')
                        
                        if char:
                            self.buffer += char
                            
                            # Look for complete sentence
                            if char == '\n' or char == '\r':
                                sentence = self.buffer.strip()
                                
                                if sentence.startswith('$') and '*' in sentence:
                                    self._process_sentence(sentence)
                                elif sentence and not sentence.startswith('$'):
                                    print(f"?? Discarding fragment: {sentence[:50]}...")
                                
                                self.buffer = ""  # Clear buffer for next sentence
                                
                    except Exception as e:
                        print(f"Read error: {e}")
                        time.sleep(0.1)
                        
        except Exception as e:
            print(f"? GPS connection error: {e}")
    
    def _process_sentence(self, sentence):
        """Process a complete NMEA sentence"""
        try:
            # Split sentence and checksum
            if '*' in sentence:
                data, checksum_str = sentence.split('*')
                checksum_received = checksum_str.strip()
            else:
                print(f"? No checksum in sentence: {sentence}")
                return
            
            # Calculate expected checksum
            checksum_calc = 0
            for char in data[1:]:  # Skip the $
                checksum_calc ^= ord(char)
            checksum_expected = f"{checksum_calc:02X}"
            
            if checksum_received.upper() == checksum_expected:
                print(f"? Valid: {sentence}")
                self._parse_gps_data(sentence)
            else:
                print(f"? Checksum mismatch: {sentence}")
                print(f"   Expected: {checksum_expected}, Got: {checksum_received}")
                
        except Exception as e:
            print(f"? Error processing sentence: {e}")
    
    def _parse_gps_data(self, sentence):
        """Parse GPS coordinates from valid sentence"""
        try:
            fields = sentence.split(',')
            
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                # GGA sentence: $GNGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,geoid,M,,checksum
                if len(fields) >= 6:
                    lat_raw = fields[2]
                    lat_dir = fields[3]
                    lon_raw = fields[4]
                    lon_dir = fields[5]
                    
                    if lat_raw and lon_raw:
                        # Convert from DDMM.MMMM to decimal degrees
                        lat_deg = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                        lon_deg = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                        
                        if lat_dir == 'S':
                            lat_deg = -lat_deg
                        if lon_dir == 'W':
                            lon_deg = -lon_deg
                        
                        print(f"?? GPS: {lat_deg:.6f}, {lon_deg:.6f}")
                        
        except Exception as e:
            print(f"Parse error: {e}")

# Test the fixed reader
if __name__ == '__main__':
    reader = FixedGPSReader()
    
    try:
        reader.start()
        print("Reading GPS data... Press Ctrl+C to stop")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n Stopping GPS reader...")
        reader.stop()
        print("GPS reader stopped")
