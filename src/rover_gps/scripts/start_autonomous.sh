#!/bin/bash
# Rover Autonomous Navigation Startup Script
# Stops competing apps and prepares system for your GUI
# 
# ROLLBACK: Run ~/.start_ros.sh to restore all services

echo "=================================================="
echo "Rover Autonomous Navigation Startup"
echo "=================================================="
echo ""

# Step 1: Stop all competing ROS2 apps
echo "Step 1: Stopping competing apps..."
pkill -f lidar_app
pkill -f line_following
pkill -f object_tracking
pkill -f hand_gesture
# Keep joystick_control for manual override safety!
echo "✓ Stopped: lidar_app, line_following, object_tracking, hand_gesture"
echo "✓ Keeping: joystick_control (for manual override)"
echo ""

# Step 2: Verify motor controller is running
echo "Step 2: Checking motor controller status..."
if ros2 node list | grep -q "odom_publisher"; then
    echo "✓ Motor controller (odom_publisher) is running"
else
    echo "⚠ Warning: Motor controller not found!"
    echo "   Run: sudo systemctl restart start_node.service"
fi
echo ""

# Step 3: Check active publishers
echo "Step 3: Checking /controller/cmd_vel publishers..."
PUBLISHER_COUNT=$(ros2 topic info /controller/cmd_vel 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
if [ ! -z "$PUBLISHER_COUNT" ]; then
    echo "Active publishers: $PUBLISHER_COUNT"
    if [ "$PUBLISHER_COUNT" -le 2 ]; then
        echo "✓ Good! Only joystick and GUI will publish"
    else
        echo "⚠ Warning: Still $PUBLISHER_COUNT publishers active"
        echo "   Some apps may not have stopped completely"
    fi
else
    echo "⚠ Could not check publisher count (ROS might be restarting)"
fi
echo ""

# Step 4: Ready to launch
echo "=================================================="
echo "System Ready for Autonomous Navigation!"
echo "=================================================="
echo ""
echo "What's running:"
echo "  ✓ Hardware controller (motors, sensors)"
echo "  ✓ Motor controller (odom_publisher)"
echo "  ✓ Joystick control (manual override)"
echo ""
echo "What's stopped:"
echo "  ✗ LiDAR app (obstacle avoidance)"
echo "  ✗ Line following"
echo "  ✗ Object tracking"
echo "  ✗ Hand gesture control"
echo ""
echo "Now you can run:"
echo "  python3 main_gui.py"
echo ""
echo "Your GUI will be the PRIMARY controller"
echo "Joystick remains active for manual override safety"
echo ""
echo "=================================================="
echo "ROLLBACK: Run ~/.start_ros.sh to restore all apps"
echo "=================================================="
