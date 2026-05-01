#!/bin/bash
# Ridgeback R100 Start Script
# Builds package and runs motion_server + image_publisher
# (clearpath-robot.service handles platform bringup)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RIDGEBACK_WORKSPACE="${RIDGEBACK_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"

export ROS_DOMAIN_ID=0
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$RIDGEBACK_WORKSPACE/config/fastrtps_ridgeback.xml"

detect_local_ip() {
    hostname -I | tr ' ' '\n' | awk '
        /^[0-9]+\./ && $1 !~ /^127\./ && $1 != "192.168.131.1" { print; exit }
    '
}

resolve_ipv4() {
    local host="$1"
    if [[ -z "$host" ]]; then
        return 1
    fi
    getent ahostsv4 "$host" 2>/dev/null | awk '{ print $1; exit }'
}

RIDGEBACK_IP="${RIDGEBACK_IP:-$(detect_local_ip)}"
if [[ -z "${JETSON_IP:-}" ]]; then
    JETSON_IP="$(resolve_ipv4 "${JETSON_HOST:-jetson-ridgeback.local}")"
fi

if [[ "${RIDGEBACK_DISABLE_FASTRTPS_PROFILE:-0}" == "1" ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
elif [[ -n "${RIDGEBACK_IP:-}" && -n "${JETSON_IP:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_ridgeback_generated.xml
    python3 "$RIDGEBACK_WORKSPACE/scripts/generate_fastrtps_profile.py" \
        --local-ip "$RIDGEBACK_IP" \
        --peer-ip "$JETSON_IP" \
        --peer-ip "192.168.131.1" \
        --output "$FASTRTPS_DEFAULT_PROFILES_FILE" >/dev/null
else
    echo "WARN: using static FastDDS profile; could not infer RIDGEBACK_IP/JETSON_IP"
fi

echo "=========================================="
echo "Ridgeback R100 - Start Script"
echo "=========================================="
echo "FastDDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE:-disabled}"
echo "Ridgeback IP: ${RIDGEBACK_IP:-unknown}  Jetson IP: ${JETSON_IP:-unknown}"

# Navigate to workspace
cd "$RIDGEBACK_WORKSPACE"

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
    if [ ! -z "${MOTION_PID:-}" ]; then
        kill $MOTION_PID 2>/dev/null || true
    fi
    if [ ! -z "${IMAGE_PID:-}" ]; then
        kill $IMAGE_PID 2>/dev/null || true
    fi
    if [ ! -z "${TAIL_PID:-}" ]; then
        kill $TAIL_PID 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

# Run motion server in background, teeing output for post-mortem
echo ""
echo "[4/4] Starting services..."
echo "=========================================="
echo "Starting motion server..."
MOTION_LOG=/tmp/ridgeback_motion.log
IMAGE_LOG=/tmp/ridgeback_image.log
ros2 run ridgeback_image_motion motion_server.py >"$MOTION_LOG" 2>&1 &
MOTION_PID=$!

sleep 1

echo "Starting image publisher..."
ros2 run ridgeback_image_motion image_publisher.py >"$IMAGE_LOG" 2>&1 &
IMAGE_PID=$!

# Give nodes a moment to crash if they're going to crash on import.
sleep 3

check_alive() {
    local name="$1" pid="$2" log="$3" runcmd="$4"
    if ! kill -0 "$pid" 2>/dev/null; then
        echo ""
        echo "ERROR: $name (PID $pid) died within 3s of launch."
        echo "Last 40 lines of $log:"
        tail -n 40 "$log" 2>/dev/null || echo "(no log available)"
        echo ""
        echo "Reproduce in foreground for the full traceback:"
        echo "  $runcmd"
        cleanup
        exit 1
    fi
}

check_alive "motion_server"   "$MOTION_PID" "$MOTION_LOG" "ros2 run ridgeback_image_motion motion_server.py"
check_alive "image_publisher" "$IMAGE_PID"  "$IMAGE_LOG"  "ros2 run ridgeback_image_motion image_publisher.py"

echo ""
echo "=========================================="
echo "All services running!"
echo "  - Motion Server   (PID: $MOTION_PID, log: $MOTION_LOG)"
echo "  - Image Publisher (PID: $IMAGE_PID, log: $IMAGE_LOG)"
echo "Press Ctrl+C to stop all"
echo "=========================================="

# tail both logs into the foreground so the operator sees activity
tail -n +1 -F "$MOTION_LOG" "$IMAGE_LOG" &
TAIL_PID=$!

wait "$MOTION_PID" "$IMAGE_PID"
kill "$TAIL_PID" 2>/dev/null || true
