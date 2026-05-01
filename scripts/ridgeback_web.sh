#!/bin/bash
# Ridgeback R100 - Web Controller Start Script
# Pulls latest code, builds, and runs web_controller

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RIDGEBACK_WORKSPACE="${RIDGEBACK_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --no-nav2                 Start the Jetson stack with Nav2 disabled
  --nav2                    Force Nav2 on
  --profile <profile>       Runtime profile: teleop, mapping, mission, debug
  -h, --help                Show this help

Environment overrides:
  RIDGEBACK_PROFILE         Runtime profile (default: mission)
  RIDGEBACK_LAUNCH_SLAM     auto|true|false (default: auto)
  RIDGEBACK_LAUNCH_NAV2     auto|true|false (default: auto)
  RIDGEBACK_LAUNCH_VLM      auto|true|false (default: auto)
  RIDGEBACK_LAUNCH_DASHBOARD auto|true|false (default: auto)
  RIDGEBACK_LAUNCH_VSLAM    true|false (default: false)
EOF
}

RIDGEBACK_PROFILE="${RIDGEBACK_PROFILE:-mission}"
RIDGEBACK_LAUNCH_SLAM="${RIDGEBACK_LAUNCH_SLAM:-auto}"
RIDGEBACK_LAUNCH_NAV2="${RIDGEBACK_LAUNCH_NAV2:-auto}"
RIDGEBACK_LAUNCH_VLM="${RIDGEBACK_LAUNCH_VLM:-auto}"
RIDGEBACK_LAUNCH_DASHBOARD="${RIDGEBACK_LAUNCH_DASHBOARD:-auto}"
RIDGEBACK_LAUNCH_VSLAM="${RIDGEBACK_LAUNCH_VSLAM:-false}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-nav2)
            RIDGEBACK_LAUNCH_NAV2=false
            shift
            ;;
        --nav2)
            RIDGEBACK_LAUNCH_NAV2=true
            shift
            ;;
        --profile)
            if [[ -z "${2:-}" ]]; then
                echo "ERROR: --profile requires one of: teleop, mapping, mission, debug" >&2
                usage >&2
                exit 2
            fi
            RIDGEBACK_PROFILE="$2"
            shift 2
            ;;
        --profile=*)
            RIDGEBACK_PROFILE="${1#*=}"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

case "$RIDGEBACK_PROFILE" in
    teleop|mapping|mission|debug) ;;
    *)
        echo "ERROR: invalid RIDGEBACK_PROFILE=$RIDGEBACK_PROFILE (expected teleop, mapping, mission, or debug)" >&2
        exit 2
        ;;
esac

validate_launch_toggle() {
    local name="$1"
    local value="$2"
    case "$value" in
        auto|true|false) ;;
        *)
            echo "ERROR: $name must be auto, true, or false (got: $value)" >&2
            exit 2
            ;;
    esac
}

validate_bool_toggle() {
    local name="$1"
    local value="$2"
    case "$value" in
        true|false) ;;
        *)
            echo "ERROR: $name must be true or false (got: $value)" >&2
            exit 2
            ;;
    esac
}

validate_launch_toggle RIDGEBACK_LAUNCH_SLAM "$RIDGEBACK_LAUNCH_SLAM"
validate_launch_toggle RIDGEBACK_LAUNCH_NAV2 "$RIDGEBACK_LAUNCH_NAV2"
validate_launch_toggle RIDGEBACK_LAUNCH_VLM "$RIDGEBACK_LAUNCH_VLM"
validate_launch_toggle RIDGEBACK_LAUNCH_DASHBOARD "$RIDGEBACK_LAUNCH_DASHBOARD"
validate_bool_toggle RIDGEBACK_LAUNCH_VSLAM "$RIDGEBACK_LAUNCH_VSLAM"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$RIDGEBACK_WORKSPACE/config/fastrtps_jetson.xml"
export RIDGEBACK_ENV_FILE="$RIDGEBACK_WORKSPACE/ridgeback_image_motion/.env"

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

JETSON_IP="${JETSON_IP:-$(detect_local_ip)}"
if [[ -z "${RIDGEBACK_IP:-}" ]]; then
    for candidate in \
        "${RIDGEBACK_HOST:-}" \
        "administrator.local" \
        "ridgeback.local" \
        "cpr-r100-0140.local" \
        "r100-0140.local"; do
        if [[ -z "$candidate" ]]; then
            continue
        fi
        RIDGEBACK_IP="$(resolve_ipv4 "$candidate")"
        if [[ -n "$RIDGEBACK_IP" ]]; then
            break
        fi
    done
fi

if [[ -n "${JETSON_IP:-}" && -n "${RIDGEBACK_IP:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_jetson_generated.xml
    python3 "$RIDGEBACK_WORKSPACE/scripts/generate_fastrtps_profile.py" \
        --local-ip "$JETSON_IP" \
        --peer-ip "$RIDGEBACK_IP" \
        --output "$FASTRTPS_DEFAULT_PROFILES_FILE" >/dev/null
else
    echo "WARN: using static FastDDS profile; could not infer JETSON_IP/RIDGEBACK_IP"
fi

echo "=========================================="
echo "Ridgeback R100 - Jetson Autonomy Dashboard"
echo "=========================================="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-unset}"
echo "FastDDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE:-disabled}"
echo "Jetson IP: ${JETSON_IP:-unknown}  Ridgeback IP: ${RIDGEBACK_IP:-unknown}"
echo "Workspace: $RIDGEBACK_WORKSPACE"

# Navigate to workspace
cd "$RIDGEBACK_WORKSPACE"

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

topic_publishers() {
    local topic="$1"
    timeout 4 ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ { print $3; exit }' || true
}

topic_status() {
    local topic="$1"
    local label="$2"
    local count
    count="$(topic_publishers "$topic")"
    if [[ -n "$count" && "$count" != "0" ]]; then
        echo "  OK   $label: $topic ($count publisher(s))"
    else
        echo "  WARN $label: $topic has no publishers visible yet"
    fi
}

wait_for_publishers() {
    local wait_s="${RIDGEBACK_PREFLIGHT_WAIT_S:-45}"
    local step_s=3
    local waited=0
    local required_topics=(
        "/r100_0140/sensors/lidar2d_0/scan"
        "/r100_0140/platform/odom/filtered"
        "/r100_0140/tf"
    )

    while (( waited < wait_s )); do
        local missing=0
        for topic in "${required_topics[@]}"; do
            count="$(topic_publishers "$topic")"
            if [[ -z "$count" || "$count" == "0" ]]; then
                missing=$((missing + 1))
            fi
        done

        if (( missing == 0 )); then
            return 0
        fi

        if (( waited == 0 )); then
            echo "  Waiting up to ${wait_s}s for Ridgeback ROS publishers..."
        fi
        sleep "$step_s"
        waited=$((waited + step_s))
    done

    return 1
}

sample_stamp_age() {
    local topic="$1"
    # Anchor sec: at the start of the line (after optional whitespace) so it
    # does not also capture the nanosec: line — the previous regex /sec:/
    # matched both, leaving sec holding the nanosec value (~10^8) and the
    # reported age off by ~30 years.
    timeout 6 ros2 topic echo "$topic" --once --field header.stamp 2>/dev/null | awk '
        /^[[:space:]]*sec:/ { sec=$2 }
        /^[[:space:]]*nanosec:/ { nsec=$2 }
        END {
            if (sec != "") {
                cmd = "date +%s"
                cmd | getline now
                close(cmd)
                if (nsec == "") nsec = 0
                printf "%.3f", now - sec - (nsec / 1000000000.0)
            }
        }'
}

echo ""
echo "Jetson autonomy preflight:"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "  RMW_FASTRTPS_USE_SHM=${RMW_FASTRTPS_USE_SHM:-unset}"
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-disabled}"
echo "  RIDGEBACK_PROFILE=$RIDGEBACK_PROFILE"
echo "  RIDGEBACK_LAUNCH_SLAM=$RIDGEBACK_LAUNCH_SLAM"
echo "  RIDGEBACK_LAUNCH_NAV2=$RIDGEBACK_LAUNCH_NAV2"
echo "  RIDGEBACK_LAUNCH_VLM=$RIDGEBACK_LAUNCH_VLM"
echo "  RIDGEBACK_LAUNCH_DASHBOARD=$RIDGEBACK_LAUNCH_DASHBOARD"
echo "  RIDGEBACK_LAUNCH_VSLAM=$RIDGEBACK_LAUNCH_VSLAM"
if ! wait_for_publishers; then
    echo "  WARN Ridgeback core ROS publishers are still not visible."
    echo "       Check that the Ridgeback terminal is already running:"
    echo "         bash ~/ridgeback99/scripts/ridgeback_start.sh"
    echo "       Also verify RIDGEBACK_IP/JETSON_IP and ROS_DOMAIN_ID match on both machines."
fi
topic_status "/r100_0140/sensors/lidar2d_0/scan" "2D LiDAR"
topic_status "/r100_0140/platform/odom/filtered" "Filtered odom"
topic_status "/r100_0140/image/compressed" "Compressed RGB"
topic_status "/r100_0140/image/depth_compressed" "Compressed depth"
topic_status "/r100_0140/tf" "TF"

scan_age="$(sample_stamp_age "/r100_0140/sensors/lidar2d_0/scan")"
if [[ -n "$scan_age" ]]; then
    echo "  LiDAR stamp age: ${scan_age}s"
    if awk "BEGIN { exit !($scan_age > 2.0 || $scan_age < -2.0) }"; then
        echo "  WARN LiDAR timestamps differ from Jetson wall clock by more than 2s."
        echo "       Check chrony/NTP on both Ridgeback and Jetson before trusting SLAM/Nav2."
    fi
else
    echo "  WARN Could not sample LiDAR timestamp for clock-sync check."
fi

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
echo "Profile: $RIDGEBACK_PROFILE"
echo "Launch: SLAM=$RIDGEBACK_LAUNCH_SLAM Nav2=$RIDGEBACK_LAUNCH_NAV2 VLM=$RIDGEBACK_LAUNCH_VLM Dashboard=$RIDGEBACK_LAUNCH_DASHBOARD vSLAM=$RIDGEBACK_LAUNCH_VSLAM"
echo "Manual teleop while Jetson stack is running:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop"
echo "=========================================="
ros2 launch ridgeback_image_motion autonomy.launch.py \
    host:=0.0.0.0 \
    port:=8081 \
    profile:="$RIDGEBACK_PROFILE" \
    launch_slam:="$RIDGEBACK_LAUNCH_SLAM" \
    launch_nav2:="$RIDGEBACK_LAUNCH_NAV2" \
    launch_vlm:="$RIDGEBACK_LAUNCH_VLM" \
    launch_dashboard:="$RIDGEBACK_LAUNCH_DASHBOARD" \
    launch_vslam:="$RIDGEBACK_LAUNCH_VSLAM"
