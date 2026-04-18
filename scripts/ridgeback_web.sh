#!/bin/bash
# Ridgeback R100 - Web Controller Start Script
# Pulls latest code, builds, and runs web_controller

set -e

export ROS_DOMAIN_ID=0
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback/config/fastrtps_jetson.xml
export RIDGEBACK_ENV_FILE=~/ridgeback/ridgeback_image_motion/.env

echo "=========================================="
echo "Ridgeback R100 - Jetson Autonomy Dashboard"
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
# colcon build --packages-select ridgeback_image_motion
colcon build --packages-select ridgeback_image_motion --cmake-clean-cache


# Source
echo ""
echo "[3/5] Sourcing workspace..."
source install/setup.bash

# Kill any previous instance on port 8081
echo ""
echo "[4/5] Clearing port 8081..."
if pids=$(lsof -t -i:8081); then
	kill $pids 2>/dev/null || true
fi
sleep 1

# Run
echo ""
echo "[5/5] Starting autonomy dashboard..."
echo "=========================================="
echo "Open in browser: http://$(hostname -I | awk '{print $1}'):8081"
echo "=========================================="
ros2 run ridgeback_image_motion web_dashboard.py
