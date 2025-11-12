#!/usr/bin/env python3
"""
GPS Waypoint Recording Widget - Complete Fixed Version
Supports both Indoor Test Mode and Outdoor Real GPS
"""

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import json
import csv
import os
import math
from datetime import datetime, timezone

os.environ['TZ'] = 'America/Winnipeg'
import time
time.tzset()

from .path_utils import resolve_waypoint_path, waypoint_storage_dir

class GPSWaypointWidget(QWidget):
    """GPS waypoint recording widget with indoor test mode support"""
    
    def __init__(self, parent_gui=None):
        super().__init__()
        self.parent_gui = parent_gui  # Reference to main GUI
        self.recording = False
        self.waypoints = []
        self.recording_start_time = None
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_set = False
        
        self.init_ui()
        
        # Recording timer
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.record_waypoint)
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("GPS Waypoint Recording")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #333; padding: 5px;")
        layout.addWidget(title)
        
        # Recording status
        self.status_label = QLabel("Status: Ready to record")
        self.status_label.setStyleSheet("background-color: #f0f8ff; padding: 8px; border-radius: 5px;")
        layout.addWidget(self.status_label)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.start_recording)
        self.start_btn.setStyleSheet("background-color: #dc3545; color: white; padding: 6px; font-weight: bold;")
        
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_recording)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("background-color: #6c757d; color: white; padding: 6px;")
        
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.stop_btn)
        layout.addLayout(button_layout)
        
        # Waypoint counter
        self.counter_label = QLabel("Waypoints: 0")
        self.counter_label.setStyleSheet("background-color: #e9ecef; padding: 5px; border-radius: 3px;")
        layout.addWidget(self.counter_label)
        
        # File operations
        file_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("Save")
        self.save_btn.clicked.connect(self.save_waypoints)
        self.save_btn.setStyleSheet("background-color: #28a745; color: white; padding: 4px;")
        
        self.view_btn = QPushButton("View")
        self.view_btn.clicked.connect(self.view_waypoints)
        self.view_btn.setStyleSheet("background-color: #17a2b8; color: white; padding: 4px;")
        
        file_layout.addWidget(self.save_btn)
        file_layout.addWidget(self.view_btn)
        layout.addLayout(file_layout)
        
        self.setLayout(layout)
        
    def start_recording(self):
        """Start waypoint recording"""
        if self.recording:
            return

        # Check if we have GPS data from parent GUI
        if not self.parent_gui:
            QMessageBox.warning(self, "Error", "GPS not available.")
            return
         
        # Get GPS data
        try:
            if hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode:
                # Indoor mode: Use fake GPS coordinates from parent GUI
                lat = self.parent_gui.last_gps_lat
                lon = self.parent_gui.last_gps_lon

                if lat == 0.0 and lon == 0.0:
                    QMessageBox.warning(
                        self, "No GPS Data",
                        "Indoor Test Mode: No GPS data available.\n\n"
                        "Make sure:\n"
                        "1. indoor_nav_tester.py is running\n"
                        "2. You've driven the rover to generate GPS data"
                    )
                    return

                if not self.origin_set:
                    self.origin_lat = lat
                    self.origin_lon = lon
                    self.origin_set = True
                    print(f"Indoor test origin set: {lat:.6f}, {lon:.6f}")

            else:
                # Outdoor mode: Use real GPS
                if not self.parent_gui.direct_gps:
                    QMessageBox.warning(
                        self, "No GPS",
                        "GPS not initialized. Please start GPS first."
                    )
                    return

                gps_data = self.parent_gui.direct_gps.get_gps_data()

                if not gps_data['available'] or gps_data['lat'] == 0.0 or gps_data['lon'] == 0.0:
                    QMessageBox.warning(
                        self, "No GPS Fix", 
                        "No GPS fix available. Please wait for GPS signal."
                    )
                    return
                
                lat = gps_data['lat']
                lon = gps_data['lon']

                # Set GPS origin for outdoor mode
                if not self.origin_set:
                    self.origin_lat = lat
                    self.origin_lon = lon
                    self.origin_set = True
                    print(f"GPS origin set: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
            
            # Start recording (THIS CODE RUNS FOR BOTH MODES!)
            self.recording = True
            self.recording_start_time = datetime.now()
            self.waypoints.clear()
            
            # Start timer (record every 2 seconds)
            self.record_timer.start(2000)
            
            # Update UI
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            
            mode_text = "INDOOR TEST" if (hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode) else "OUTDOOR"
            self.status_label.setText(f"Status: RECORDING ({mode_text})")
            self.status_label.setStyleSheet("background-color: #dc3545; color: white; padding: 8px; border-radius: 5px;")
            
            print(f"Started GPS waypoint recording ({mode_text} mode)")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start recording: {e}")
            print(f"Recording start error: {e}")
    
    def stop_recording(self):
        """Stop waypoint recording"""
        self.recording = False
        self.record_timer.stop()
        
        duration = (datetime.now() - self.recording_start_time).total_seconds() if self.recording_start_time else 0
        
        # Update UI
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText(f"Status: Recorded {len(self.waypoints)} waypoints in {duration:.1f}s")
        self.status_label.setStyleSheet("background-color: #28a745; color: white; padding: 8px; border-radius: 5px;")
        
        # Auto-save if we have waypoints
        if len(self.waypoints) > 0:
            self.save_waypoints()
        
        print(f"Stopped recording. Total waypoints: {len(self.waypoints)}")
    
    def record_waypoint(self):
        """Record a single waypoint"""
        if not self.recording or not self.parent_gui:
            return
            
        try:
            # Get GPS data based on mode
            if hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode:
                # Indoor mode: Use fake GPS from parent GUI
                lat = self.parent_gui.last_gps_lat
                lon = self.parent_gui.last_gps_lon
                
                if lat == 0.0 and lon == 0.0:
                    return
                
                gps_status = self.parent_gui.gps_status if hasattr(self.parent_gui, 'gps_status') else 1
            else:
                # Outdoor mode: Use real GPS
                if not hasattr(self.parent_gui, 'direct_gps') or not self.parent_gui.direct_gps:
                    return
                    
                gps_data = self.parent_gui.direct_gps.get_gps_data()
                if not gps_data['available'] or gps_data['lat'] == 0.0 or gps_data['lon'] == 0.0:
                    return

                lat = gps_data['lat']
                lon = gps_data['lon']
                gps_status = gps_data['status']
            
            # Skip if too close to last waypoint (less than 0.5 meters)
            if len(self.waypoints) > 0:
                last_wp = self.waypoints[-1]
                distance = self.calculate_distance(lat, lon, last_wp['latitude'], last_wp['longitude'])
                if distance < 0.5:  # Less than 0.5 meters
                    return
            
            # Convert to local XY coordinates
            x, y = self.gps_to_xy(lat, lon)
            
            # Create waypoint
            current_time = datetime.now()
            waypoint = {
                'id': len(self.waypoints) + 1,
                'timestamp': current_time.timestamp(),
                'datetime': current_time.strftime('%Y-%m-%d %H:%M:%S'),
                'latitude': lat,
                'longitude': lon,
                'x': x,
                'y': y,
                'gps_status': gps_status
            }
            
            self.waypoints.append(waypoint)
            
            # Update counter
            self.counter_label.setText(f"Waypoints: {len(self.waypoints)}")
            
            print(f"Waypoint #{waypoint['id']}: GPS({lat:.6f}, {lon:.6f}) -> XY({x:.2f}, {y:.2f})")
            
        except Exception as e:
            print(f"Error recording waypoint: {e}")
    
    def gps_to_xy(self, lat, lon):
        """Convert GPS to local XY coordinates"""
        lat_diff = lat - self.origin_lat
        lon_diff = lon - self.origin_lon
        
        # Approximate conversion (meters per degree)
        x = lon_diff * 111320.0 * math.cos(math.radians(self.origin_lat))
        y = lat_diff * 110540.0
        
        return x, y
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between GPS points (Haversine formula)"""
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * \
            math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000  # Earth radius in meters
        
        return c * r
    
    def save_waypoints(self):
        """Save waypoints to file"""
        if len(self.waypoints) == 0:
            QMessageBox.information(self, "No Data", "No waypoints to save.")
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Determine mode for filename
        mode_prefix = "indoor_" if (hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode) else ""
        filename = f'{mode_prefix}rover_waypoints_{timestamp}.json'
        output_dir = waypoint_storage_dir()
        json_path = resolve_waypoint_path(filename)

        # Prepare data
        data = {
            'origin': {
                'latitude': self.origin_lat,
                'longitude': self.origin_lon
            },
            'waypoints': self.waypoints,
            'total_waypoints': len(self.waypoints),
            'recording_time': timestamp,
            'mode': 'indoor_test' if (hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode) else 'outdoor'
        }
        
        try:
            # Save JSON
            with open(json_path, 'w') as f:
                json.dump(data, f, indent=2)

            # Also save CSV for easy viewing
            csv_filename = f'{mode_prefix}rover_waypoints_{timestamp}.csv'
            csv_path = resolve_waypoint_path(csv_filename)
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=[
                    'id', 'datetime', 'latitude', 'longitude', 'x', 'y', 'gps_status'],
                    extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.waypoints)

            QMessageBox.information(
                self, "Saved",
                f"Saved {len(self.waypoints)} waypoints!\n\n"
                f"Location: {output_dir}\n"
                f"Files:\n• {json_path}\n• {csv_path}"
            )
            print(f"Saved waypoints to {json_path}")

        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save: {e}")
    
    def view_waypoints(self):
        """Show waypoints in a simple dialog"""
        if len(self.waypoints) == 0:
            QMessageBox.information(self, "No Data", "No waypoints recorded yet.")
            return
        
        # Create simple view dialog
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Recorded Waypoints ({len(self.waypoints)} points)")
        dialog.resize(600, 400)
        
        layout = QVBoxLayout()
        
        # Summary
        if len(self.waypoints) > 1:
            total_distance = 0
            for i in range(1, len(self.waypoints)):
                wp1 = self.waypoints[i-1]
                wp2 = self.waypoints[i]
                dist = ((wp2['x'] - wp1['x'])**2 + (wp2['y'] - wp1['y'])**2)**0.5
                total_distance += dist
            
            mode_text = "🏠 INDOOR TEST" if (hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode) else "🌍 OUTDOOR"
            summary = QLabel(
                f"Mode: {mode_text}\n"
                f"Total waypoints: {len(self.waypoints)}\n"
                f"Approximate path length: {total_distance:.1f} meters"
            )
        else:
            mode_text = "🏠 INDOOR TEST" if (hasattr(self.parent_gui, 'indoor_test_mode') and self.parent_gui.indoor_test_mode) else "🌍 OUTDOOR"
            summary = QLabel(f"Mode: {mode_text}\nTotal waypoints: {len(self.waypoints)}")
            
        summary.setStyleSheet("background-color: #f8f9fa; padding: 10px; border-radius: 5px;")
        layout.addWidget(summary)
        
        # Waypoint list
        list_widget = QListWidget()
        for wp in self.waypoints:
            item_text = (
                f"#{wp['id']}: {wp['datetime']} - "
                f"GPS({wp['latitude']:.6f}, {wp['longitude']:.6f}) - "
                f"XY({wp['x']:.1f}, {wp['y']:.1f})"
            )
            list_widget.addItem(item_text)
        
        layout.addWidget(list_widget)
        
        # Close button
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dialog.close)
        layout.addWidget(close_btn)
        
        dialog.setLayout(layout)
        dialog.exec_()
