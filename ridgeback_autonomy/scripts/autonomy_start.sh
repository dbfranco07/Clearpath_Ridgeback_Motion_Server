#!/bin/bash
# Ridgeback Autonomy — Jetson Startup Script
# =============================================
# Builds the autonomy package and launches the full stack.
# Run on: Jetson Orin (after ridgeback_start.sh on the Ridgeback)

set -e

export ROS_DOMAIN_ID=0
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback/config/fastrtps_jetson.xml

# Optional staged-launch toggles (override via environment)
# Example: LAUNCH_VLM=false LAUNCH_DASHBOARD=false ./autonomy_start.sh
LAUNCH_SLAM=${LAUNCH_SLAM:-true}
LAUNCH_NAV2=${LAUNCH_NAV2:-true}
LAUNCH_VLM=${LAUNCH_VLM:-true}
LAUNCH_MEMORY=${LAUNCH_MEMORY:-true}
LAUNCH_MISSION=${LAUNCH_MISSION:-true}
LAUNCH_DASHBOARD=${LAUNCH_DASHBOARD:-true}

echo "=========================================="
echo "Ridgeback R100 — Autonomy Stack"
echo "=========================================="

# Navigate to workspace
cd ~/ridgeback

# Pull latest
echo "[1/5] Pulling latest..."
git pull

# Build both packages
echo "[2/5] Building packages..."
colcon build --packages-select ridgeback_image_motion ridgeback_autonomy

# Source
echo "[3/5] Sourcing workspace..."
source install/setup.bash

# Kill any previous instances
echo "[4/5] Clearing ports..."
kill $(lsof -t -i:8081) 2>/dev/null || true
sleep 1

# Launch
echo "[5/5] Launching autonomy stack..."
echo "=========================================="
echo "Launch toggles:"
echo "  slam=$LAUNCH_SLAM nav2=$LAUNCH_NAV2 vlm=$LAUNCH_VLM"
echo "  memory=$LAUNCH_MEMORY mission=$LAUNCH_MISSION dashboard=$LAUNCH_DASHBOARD"
echo "Web Dashboard: http://$(hostname -I | awk '{print $1}'):8081"
echo "Teleop UI:     http://$(hostname -I | awk '{print $1}'):8080 (if running)"
echo "=========================================="
ros2 launch ridgeback_autonomy autonomy.launch.py \
	launch_slam:=$LAUNCH_SLAM \
	launch_nav2:=$LAUNCH_NAV2 \
	launch_vlm:=$LAUNCH_VLM \
	launch_memory:=$LAUNCH_MEMORY \
	launch_mission:=$LAUNCH_MISSION \
	launch_dashboard:=$LAUNCH_DASHBOARD
