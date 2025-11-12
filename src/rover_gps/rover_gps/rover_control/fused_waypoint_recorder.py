#!/usr/bin/env python3
"""
Enhanced Waypoint Recorder with IMU+GPS Sensor Fusion
Records waypoints with accurate heading information
"""

import json
import time
from datetime import datetime
from sensor_fusion import SensorFusion

from .path_utils import resolve_waypoint_path


class FusedWaypointRecorder:
    """Records waypoints using fused IMU+GPS data"""
    
    def __init__(self, sensor_fusion):
        """
        Initialize recorder
        
        Args:
            sensor_fusion: SensorFusion instance
        """
        self.fusion = sensor_fusion
        self.waypoints = []
        self.origin_lat = None
        self.origin_lon = None
        self.origin_heading = None
        self.recording = False
        self.waypoint_id = 0
        
    def start_recording(self):
        """Start recording waypoints"""
        pose = self.fusion.get_fused_pose()
        
        if not pose['valid']:
            print("❌ Cannot start - no valid sensor data!")
            return False
        
        # Set origin
        self.origin_lat = pose['latitude']
        self.origin_lon = pose['longitude']
        self.origin_heading = pose['heading']
        
        self.waypoints = []
        self.waypoint_id = 0
        self.recording = True
        
        print(f"✓ Recording started at: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
        print(f"  Initial heading: {self.origin_heading:.1f}°")
        
        return True
    
    def record_waypoint(self):
        """Record current position as waypoint"""
        if not self.recording:
            print("Not recording!")
            return False
        
        pose = self.fusion.get_fused_pose()
        
        if not pose['valid']:
            print("⚠ Skipping - invalid sensor data")
            return False
        
        self.waypoint_id += 1
        
        # Calculate relative position from origin
        x, y = self._gps_to_xy(
            pose['latitude'], pose['longitude'],
            self.origin_lat, self.origin_lon
        )
        
        waypoint = {
            'id': self.waypoint_id,
            'timestamp': time.time(),
            'datetime': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'latitude': pose['latitude'],
            'longitude': pose['longitude'],
            'heading': pose['heading'],  # ← NEW: Actual rover heading!
            'x': x,
            'y': y
        }
        
        self.waypoints.append(waypoint)
        
        print(f"✓ Waypoint {self.waypoint_id}: "
              f"({x:.2f}m, {y:.2f}m), heading={pose['heading']:.1f}°")
        
        return True
    
    def stop_recording(self):
        """Stop recording"""
        self.recording = False
        print(f"✓ Recording stopped. Total waypoints: {len(self.waypoints)}")
    
    def save_to_file(self, filename=None):
        """
        Save waypoints to JSON file
        
        Args:
            filename: Output filename (auto-generated if None)
        """
        if len(self.waypoints) == 0:
            print("No waypoints to save!")
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'fused_waypoints_{timestamp}.json'

        output_path = resolve_waypoint_path(filename)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Calculate total distance
        total_distance = 0.0
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i]['x'] - self.waypoints[i-1]['x']
            dy = self.waypoints[i]['y'] - self.waypoints[i-1]['y']
            total_distance += (dx**2 + dy**2)**0.5
        
        data = {
            'origin': {
                'latitude': self.origin_lat,
                'longitude': self.origin_lon,
                'heading': self.origin_heading
            },
            'waypoints': self.waypoints,
            'total_waypoints': len(self.waypoints),
            'total_distance_m': total_distance,
            'recording_time': datetime.now().strftime('%Y%m%d_%H%M%S'),
            'mode': 'fused_imu_gps'
        }
        
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"✓ Saved {len(self.waypoints)} waypoints to: {output_path}")
        print(f"  Total distance: {total_distance:.2f}m")

        return str(output_path)
    
    def _gps_to_xy(self, lat, lon, origin_lat, origin_lon):
        """
        Convert GPS to XY coordinates (meters from origin)
        
        Returns:
            (x, y) in meters
        """
        # Simple flat-earth approximation (good for short distances)
        # At 49.89° latitude:
        meters_per_deg_lat = 111132.92  # meters
        meters_per_deg_lon = 71000.0    # meters (approximate)
        
        x = (lat - origin_lat) * meters_per_deg_lat
        y = (lon - origin_lon) * meters_per_deg_lon
        
        return x, y


# Example usage
if __name__ == '__main__':
    # Create sensor fusion
    fusion = SensorFusion()
    
    # Create recorder
    recorder = FusedWaypointRecorder(fusion)
    
    # Simulate recording
    print("=== Simulating waypoint recording ===\n")
    
    # Update sensors
    fusion.update_gps(49.8951, -97.1384, status=1)
    fusion.update_imu(0.0)  # Facing North
    
    # Start recording
    recorder.start_recording()
    
    # Record waypoints
    for i in range(5):
        # Simulate movement
        new_lat = 49.8951 + (i * 0.00001)
        fusion.update_gps(new_lat, -97.1384, status=1)
        fusion.update_imu(5.0 * i)  # Gradually turning
        
        time.sleep(0.1)
        recorder.record_waypoint()
    
    # Stop and save
    recorder.stop_recording()
    filename = recorder.save_to_file()
    
    print(f"\n✓ Demo complete! Check {filename}")
