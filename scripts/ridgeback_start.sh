#!/bin/bash
# Ridgeback R100 Start Script
# Builds package and runs motion_server. The RealSense camera is now wired
# directly to the Jetson and driven by realsense2_camera there; the Ridgeback
# stack no longer needs image_publisher.
# (clearpath-robot.service handles platform bringup)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RIDGEBACK_WORKSPACE="${RIDGEBACK_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
source "$SCRIPT_DIR/ridgeback_clock_check.sh"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$RIDGEBACK_WORKSPACE/config/fastrtps_ridgeback.xml"

RIDGEBACK_PREFER_WIRED="${RIDGEBACK_PREFER_WIRED:-true}"
RIDGEBACK_REQUIRE_WIRED="${RIDGEBACK_REQUIRE_WIRED:-$RIDGEBACK_PREFER_WIRED}"
RIDGEBACK_WIRED_IP="${RIDGEBACK_WIRED_IP:-192.168.131.1}"
JETSON_WIRED_IP="${JETSON_WIRED_IP:-192.168.131.50}"

is_true() {
    [[ "$1" == "true" ]]
}

detect_wired_ridgeback_ip() {
    ip -4 addr show scope global 2>/dev/null | awk -v preferred="$RIDGEBACK_WIRED_IP" '
        $1 == "inet" {
            split($2, parts, "/")
            if (parts[1] == preferred) {
                print parts[1]
                found = 1
                exit
            }
            if (parts[1] ~ /^192\.168\.131\./ && fallback == "") {
                fallback = parts[1]
            }
        }
        END {
            if (!found && fallback != "") print fallback
        }
    '
}

detect_local_ip() {
    local wired_ip
    if is_true "$RIDGEBACK_PREFER_WIRED"; then
        wired_ip="$(detect_wired_ridgeback_ip)"
        if [[ -n "$wired_ip" ]]; then
            echo "$wired_ip"
            return
        fi
    fi

    hostname -I | tr ' ' '\n' | awk -v wired="$RIDGEBACK_WIRED_IP" '
        /^[0-9]+\./ && $1 !~ /^127\./ && $1 != wired { print; exit }
    '
}

resolve_ipv4() {
    local host="$1"
    if [[ -z "$host" ]]; then
        return 1
    fi
    getent ahostsv4 "$host" 2>/dev/null | awk '{ print $1; exit }'
}

if is_true "$RIDGEBACK_PREFER_WIRED"; then
    wired_ridgeback_ip="$(detect_wired_ridgeback_ip)"
    if [[ -n "$wired_ridgeback_ip" ]]; then
        if [[ -n "${RIDGEBACK_IP:-}" && "$RIDGEBACK_IP" != "$wired_ridgeback_ip" ]]; then
            echo "WARN: overriding RIDGEBACK_IP=$RIDGEBACK_IP with wired $wired_ridgeback_ip" >&2
        fi
        RIDGEBACK_IP="$wired_ridgeback_ip"
    elif is_true "$RIDGEBACK_REQUIRE_WIRED"; then
        echo "ERROR: wired Ridgeback IP is required but $RIDGEBACK_WIRED_IP is not present." >&2
        echo "       Check: ip -br addr; ip route get $JETSON_WIRED_IP; ping -c 3 $JETSON_WIRED_IP" >&2
        echo "       To intentionally debug over WiFi: RIDGEBACK_PREFER_WIRED=false goridge" >&2
        exit 1
    fi
fi

RIDGEBACK_IP="${RIDGEBACK_IP:-$(detect_local_ip)}"
if is_true "$RIDGEBACK_PREFER_WIRED" && [[ "${RIDGEBACK_IP:-}" == 192.168.131.* ]]; then
    if [[ -n "${JETSON_IP:-}" && "$JETSON_IP" != "$JETSON_WIRED_IP" ]]; then
        echo "WARN: overriding JETSON_IP=$JETSON_IP with wired $JETSON_WIRED_IP" >&2
    fi
    JETSON_IP="$JETSON_WIRED_IP"
elif [[ -z "${JETSON_IP:-}" ]]; then
    JETSON_IP="$(resolve_ipv4 "${JETSON_HOST:-jetson-ridgeback.local}")"
fi

if [[ "${RIDGEBACK_DISABLE_FASTRTPS_PROFILE:-0}" == "1" ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
elif [[ -n "${RIDGEBACK_IP:-}" && -n "${JETSON_IP:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_ridgeback_generated.xml
    python3 "$RIDGEBACK_WORKSPACE/scripts/generate_fastrtps_profile.py" \
        --local-ip "$RIDGEBACK_IP" \
        --peer-ip "$JETSON_IP" \
        --peer-ip "$RIDGEBACK_WIRED_IP" \
        --output "$FASTRTPS_DEFAULT_PROFILES_FILE" >/dev/null
else
    echo "WARN: using static FastDDS profile; could not infer RIDGEBACK_IP/JETSON_IP"
fi

echo "=========================================="
echo "Ridgeback R100 - Start Script"
echo "=========================================="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-unset}"
echo "FastDDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE:-disabled}"
echo "Ridgeback IP: ${RIDGEBACK_IP:-unknown}  Jetson IP: ${JETSON_IP:-unknown}"
echo "Network preference: wired=${RIDGEBACK_PREFER_WIRED} require_wired=${RIDGEBACK_REQUIRE_WIRED} ridgeback_wired=${RIDGEBACK_WIRED_IP} jetson_wired=${JETSON_WIRED_IP}"
echo "Workspace: $RIDGEBACK_WORKSPACE"

# Navigate to workspace
cd "$RIDGEBACK_WORKSPACE"

ridgeback_check_clock_for_build "$RIDGEBACK_WORKSPACE"

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

topic_status() {
    local topic="$1"
    local label="$2"
    local count
    count="$(timeout 3 ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ { print $3; exit }' || true)"
    if [[ -n "$count" && "$count" != "0" ]]; then
        echo "  OK   $label: $topic ($count publisher(s))"
    else
        echo "  WARN $label: $topic has no publishers visible yet"
    fi
}

echo ""
echo "Ridgeback platform diagnostics:"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "  RMW_FASTRTPS_USE_SHM=${RMW_FASTRTPS_USE_SHM:-unset}"
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-disabled}"
topic_status "/r100_0140/sensors/lidar2d_0/scan" "2D LiDAR"
topic_status "/r100_0140/platform/odom/filtered" "Filtered odom"
echo "  This script intentionally starts only motion_server.py."
echo "  RealSense RGB-D now publishes from the Jetson under /r100_0140/sensors/camera_0/*."

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down..."
    if [ ! -z "${MOTION_PID:-}" ]; then
        kill $MOTION_PID 2>/dev/null || true
    fi
    if [ ! -z "${TAIL_PID:-}" ]; then
        kill $TAIL_PID 2>/dev/null || true
    fi
    if [ ! -z "${POSTFLIGHT_PID:-}" ]; then
        kill $POSTFLIGHT_PID 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

# Postflight: confirm motion + image are publishing the expected topics.
# Runs in background ~8s after services start so we don't block the foreground tail.
#
# IMPORTANT: this Ridgeback uses a restrictive FastDDS profile that whitelists only
# the Jetson peer IP. Intra-machine DDS discovery (ros2 node list, ros2 topic info
# for locally-published topics) is blocked. So for processes WE launched we use
# their PIDs + log inspection. For external publishers (LiDAR / odom from the
# clearpath-platform service), we still use ros2 cli — those use the system DDS
# profile and discovery does work for them.
postflight_ridgeback() {
    local wait_s="${RIDGEBACK_POSTFLIGHT_WAIT_S:-8}"
    sleep "$wait_s"

    echo ""
    echo "=========================================="
    echo "[POSTFLIGHT] Ridgeback (after ${wait_s}s)"
    echo "=========================================="

    local errs=0

    check_owned_node() {
        local label="$1" pid="$2" log="$3" ready_pattern="$4"
        if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
            echo "  FAIL $label process (pid=${pid:-unset}) NOT alive" >&2
            errs=$((errs + 1))
            return
        fi
        if [[ -n "$ready_pattern" ]] && ! grep -qE "$ready_pattern" "$log" 2>/dev/null; then
            echo "  FAIL $label alive (pid=$pid) but ready signal not in $log" >&2
            errs=$((errs + 1))
            return
        fi
        echo "  OK   $label alive (pid=$pid) and ready"
    }

    check_owned_node "motion_server"   "$MOTION_PID" "$MOTION_LOG" "Motion Service Server started"

    # External publishers from the clearpath-platform/clearpath-sensors services
    # — these use the system DDS config so ros2 cli can see them.
    local external_pubs=(
        "/r100_0140/sensors/lidar2d_0/scan"
        "/r100_0140/platform/odom/filtered"
    )
    for topic in "${external_pubs[@]}"; do
        local pub_count
        pub_count="$(timeout 3 ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ { print $3; exit }')"
        if [[ -n "$pub_count" && "$pub_count" != "0" ]]; then
            echo "  OK   topic $topic ($pub_count publisher(s))"
        else
            echo "  FAIL topic $topic has NO publishers (clearpath-platform/sensors not running?)" >&2
            errs=$((errs + 1))
        fi
    done

    echo "------------------------------------------"
    if (( errs == 0 )); then
        echo "[POSTFLIGHT] PASS — Ridgeback is ready for the Jetson stack."
    else
        echo "[POSTFLIGHT] FAIL — ${errs} problem(s) above. Investigate before running 'goridge' on the Jetson." >&2
        echo "  Hints:" >&2
        echo "    - LiDAR/odom missing: 'sudo systemctl status clearpath-platform clearpath-sensors'." >&2
        echo "    - motion_server dead: check $MOTION_LOG for import or topic-publisher failures." >&2
    fi
    echo "=========================================="
}

# Run motion server in background, teeing output for post-mortem
echo ""
echo "[4/4] Starting services..."
echo "=========================================="
echo "Starting motion server..."
MOTION_LOG=/tmp/ridgeback_motion.log
ros2 run ridgeback_image_motion motion_server.py >"$MOTION_LOG" 2>&1 &
MOTION_PID=$!

# Give the node a moment to crash if it's going to crash on import.
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

echo ""
echo "=========================================="
echo "All services running!"
echo "  - Motion Server   (PID: $MOTION_PID, log: $MOTION_LOG)"
echo "Press Ctrl+C to stop all"
echo "=========================================="

# tail the log into the foreground so the operator sees activity
tail -n +1 -F "$MOTION_LOG" &
TAIL_PID=$!

postflight_ridgeback &
POSTFLIGHT_PID=$!

wait "$MOTION_PID"
kill "$TAIL_PID" 2>/dev/null || true
kill "$POSTFLIGHT_PID" 2>/dev/null || true
