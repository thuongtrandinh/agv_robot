#!/bin/bash

# Script to run RViz2 with mobile_robot configuration
# Author: Thuong Tran Dinh
# Date: November 30, 2025

# Source ROS 2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Get package share directory
PACKAGE_PATH=$(ros2 pkg prefix mobile_robot)/share/mobile_robot
RVIZ_CONFIG="${PACKAGE_PATH}/rviz/rviz2.rviz"

# Check if config file exists
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "❌ Error: RViz config file not found at: $RVIZ_CONFIG"
    exit 1
fi

echo "🚀 Starting RViz2 with config: $RVIZ_CONFIG"

# Run RViz2
rviz2 -d "$RVIZ_CONFIG"
