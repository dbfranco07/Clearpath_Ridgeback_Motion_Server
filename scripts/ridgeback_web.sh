#!/bin/bash
# Ridgeback R100 - Web Controller Start Script
# Pulls latest code, builds, and runs web_controller

set -e

export ROS_DOMAIN_ID=0
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_FASTRTPS_USE_SHM=0

echo "=========================================="
echo "Ridgeback R100 - Web Controller"
echo "=========================================="

# Navigate to workspace
cd ~/ridgeback

# Pull latest changes
echo ""
echo "[1/5] Pulling latest changes..."
git pull

# Build
echo ""
echo "[2/5] Building package..."
colcon build --packages-select ridgeback_image_motion

# Source
echo ""
echo "[3/5] Sourcing workspace..."
source install/setup.bash

# Kill any previous instance on port 8080
echo ""
echo "[4/5] Clearing port 8080..."
kill $(lsof -t -i:8080) 2>/dev/null || true
sleep 1

# Run
echo ""
echo "[5/5] Starting web controller..."
echo "=========================================="
echo "Open in browser: http://$(hostname -I | awk '{print $1}'):8080"
echo "=========================================="
ros2 run ridgeback_image_motion web_controller.py
