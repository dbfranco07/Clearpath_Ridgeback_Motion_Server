#!/bin/bash
# Ridgeback R100 - Web Controller Start Script
# Pulls latest code, builds, and runs web_controller

set -e

export ROS_DOMAIN_ID=8

echo "=========================================="
echo "Ridgeback R100 - Web Controller"
echo "=========================================="

# Navigate to workspace
cd ~/ridgeback

# Pull latest changes
echo ""
echo "[1/4] Pulling latest changes..."
git pull

# Build
echo ""
echo "[2/4] Building package..."
cd ~/ros2_ws
colcon build --packages-select ridgeback_image_motion

# Source
echo ""
echo "[3/4] Sourcing workspace..."
source install/setup.bash

# Run
echo ""
echo "[4/4] Starting web controller..."
echo "=========================================="
echo "Open in browser: http://$(hostname -I | awk '{print $1}'):8080"
echo "=========================================="
ros2 run ridgeback_image_motion web_controller.py
