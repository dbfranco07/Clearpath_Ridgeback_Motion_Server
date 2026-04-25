#!/bin/bash
# Ridgeback R100 - Web Controller Start Script
# Pulls latest code, builds, and runs web_controller

set -e

export ROS_DOMAIN_ID=0
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback/config/fastrtps_jetson.xml
export RIDGEBACK_ENV_FILE=~/ridgeback/ridgeback_image_motion/.env

if [[ -n "${JETSON_IP:-}" && -n "${RIDGEBACK_IP:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_jetson_generated.xml
    python3 ~/ridgeback/scripts/generate_fastrtps_profile.py \
        --local-ip "$JETSON_IP" \
        --peer-ip "$RIDGEBACK_IP" \
        --output "$FASTRTPS_DEFAULT_PROFILES_FILE" >/dev/null
fi

echo "=========================================="
echo "Ridgeback R100 - Jetson Autonomy Dashboard"
echo "=========================================="

# Navigate to workspace
cd ~/ridgeback

# Pull latest changes
echo ""
echo "[1/5] Pulling latest changes..."
if [[ "${RIDGEBACK_SKIP_PULL:-0}" == "1" ]]; then
    echo "Skipping git pull (RIDGEBACK_SKIP_PULL=1)"
else
    git pull
fi

# Build
echo ""
echo "[2/5] Building package..."
if [[ "${RIDGEBACK_SKIP_BUILD:-0}" == "1" ]]; then
    echo "Skipping build (RIDGEBACK_SKIP_BUILD=1)"
elif [[ "${RIDGEBACK_CLEAN_BUILD:-0}" == "1" ]]; then
    colcon build --packages-select ridgeback_image_motion --cmake-clean-cache
else
    colcon build --packages-select ridgeback_image_motion
fi


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
echo "[5/5] Starting Jetson autonomy stack..."
echo "=========================================="
echo "Open in browser: http://$(hostname -I | awk '{print $1}'):8081"
echo "Profile: ${RIDGEBACK_PROFILE:-mission}"
echo "=========================================="
ros2 launch ridgeback_image_motion autonomy.launch.py host:=0.0.0.0 port:=8081 profile:=${RIDGEBACK_PROFILE:-mission}
