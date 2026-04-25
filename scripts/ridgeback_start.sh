#!/bin/bash
# Ridgeback R100 Start Script
# Builds package and runs motion_server + image_publisher
# (clearpath-robot.service handles platform bringup)

set -e

export ROS_DOMAIN_ID=0
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback99/config/fastrtps_ridgeback.xml

if [[ "${RIDGEBACK_DISABLE_FASTRTPS_PROFILE:-0}" == "1" ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
elif [[ -n "${RIDGEBACK_IP:-}" && -n "${JETSON_IP:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_ridgeback_generated.xml
    python3 ~/ridgeback99/scripts/generate_fastrtps_profile.py \
        --local-ip "$RIDGEBACK_IP" \
        --peer-ip "$JETSON_IP" \
        --output "$FASTRTPS_DEFAULT_PROFILES_FILE" >/dev/null
fi

echo "=========================================="
echo "Ridgeback R100 - Start Script"
echo "=========================================="

# Navigate to workspace
cd ~/ridgeback99

# Pull latest changes
echo ""
echo "[1/4] Pulling latest changes..."
git pull

# Build
echo ""
echo "[2/4] Building package..."
colcon build --packages-select ridgeback_image_motion

# Source
echo ""
echo "[3/4] Sourcing workspace..."
source install/setup.bash

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down..."
    if [ ! -z "$MOTION_PID" ]; then
        kill $MOTION_PID 2>/dev/null || true
    fi
    if [ ! -z "$IMAGE_PID" ]; then
        kill $IMAGE_PID 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

# Run motion server in background
echo ""
echo "[4/4] Starting services..."
echo "=========================================="
echo "Starting motion server..."
ros2 run ridgeback_image_motion motion_server.py &
MOTION_PID=$!

sleep 1

# Run image publisher in background
echo "Starting image publisher..."
ros2 run ridgeback_image_motion image_publisher.py &
IMAGE_PID=$!

echo ""
echo "=========================================="
echo "All services running!"
echo "  - Motion Server (PID: $MOTION_PID)"
echo "  - Image Publisher (PID: $IMAGE_PID)"
echo "Press Ctrl+C to stop all"
echo "=========================================="

wait
