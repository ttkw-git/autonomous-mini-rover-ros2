#!/usr/bin/env python3
"""
Virtual joystick widget for rover control
"""

import math
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QBrush, QColor


class VirtualJoystick(QWidget):
    """Virtual joystick widget for touch/mouse control"""
    
    position_changed = pyqtSignal(float, float)  # (x, y) normalized position
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 200)
        self.center_x = 100
        self.center_y = 100
        self.knob_x = 100
        self.knob_y = 100
        self.dragging = False
        
    def paintEvent(self, event):
        """Draw the joystick"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw outer circle
        painter.setBrush(QBrush(QColor(200, 200, 200)))
        painter.drawEllipse(20, 20, 160, 160)
        
        # Draw center point
        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.drawEllipse(95, 95, 10, 10)
        
        # Draw knob
        painter.setBrush(QBrush(QColor(50, 150, 250)))
        painter.drawEllipse(int(self.knob_x - 15), int(self.knob_y - 15), 30, 30)
        
    def mousePressEvent(self, event):
        """Handle mouse press - start dragging"""
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.update_knob_position(event.x(), event.y())
            
    def mouseMoveEvent(self, event):
        """Handle mouse move - update position while dragging"""
        if self.dragging:
            self.update_knob_position(event.x(), event.y())
            
    def mouseReleaseEvent(self, event):
        """Handle mouse release - return to center"""
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self.knob_x = self.center_x
            self.knob_y = self.center_y
            self.update()
            self.position_changed.emit(0.0, 0.0)
            
    def update_knob_position(self, x, y):
        """
        Update joystick knob position with radius constraint
        
        Args:
            x: Mouse x coordinate
            y: Mouse y coordinate
        """
        # Calculate offset from center
        dx = x - self.center_x
        dy = y - self.center_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Limit to circle radius
        if distance > 80:
            dx = dx / distance * 80
            dy = dy / distance * 80
            
        self.knob_x = self.center_x + dx
        self.knob_y = self.center_y + dy
        
        # Emit normalized position (-1 to 1)
        norm_x = dx / 80.0
        norm_y = -dy / 80.0  # Invert Y axis (up = positive)
        
        self.update()
        self.position_changed.emit(norm_x, norm_y)
