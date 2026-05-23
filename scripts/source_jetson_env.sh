#!/bin/bash
# Source this in a Jetson diagnostic shell before running ros2 CLI commands:
#   source ~/ridgeback99/scripts/source_jetson_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RIDGEBACK_WORKSPACE="${RIDGEBACK_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/humble/setup.bash
if [[ -f "$RIDGEBACK_WORKSPACE/install/setup.bash" ]]; then
    source "$RIDGEBACK_WORKSPACE/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RIDGEBACK_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "$RMW_IMPLEMENTATION" == "rmw_fastrtps_cpp" ]]; then
    export RMW_FASTRTPS_USE_SHM=1
else
    unset RMW_FASTRTPS_USE_SHM
fi

RIDGEBACK_WIRED_IP="${RIDGEBACK_WIRED_IP:-192.168.131.1}"
JETSON_WIRED_IP="${JETSON_WIRED_IP:-192.168.131.50}"

detect_jetson_wired_ip() {
    ip -4 addr show scope global 2>/dev/null | awk -v fallback="$JETSON_WIRED_IP" '
        $1 == "inet" {
            split($2, parts, "/")
            if (parts[1] == fallback) {
                print parts[1]
                found = 1
                exit
            }
            if (parts[1] ~ /^192\.168\.131\./ && candidate == "") {
                candidate = parts[1]
            }
        }
        END {
            if (!found && candidate != "") print candidate
        }
    '
}

JETSON_IP="${JETSON_IP:-$(detect_jetson_wired_ip)}"
JETSON_IP="${JETSON_IP:-$JETSON_WIRED_IP}"
RIDGEBACK_IP="${RIDGEBACK_IP:-$RIDGEBACK_WIRED_IP}"

if [[ "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
elif [[ "${RIDGEBACK_DISABLE_FASTRTPS_PROFILE:-0}" == "1" ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
else
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_jetson_generated.xml
    python3 "$RIDGEBACK_WORKSPACE/scripts/generate_fastrtps_profile.py" \
        --local-ip "$JETSON_IP" \
        --peer-ip "$RIDGEBACK_IP" \
        --output "$FASTRTPS_DEFAULT_PROFILES_FILE" >/dev/null
fi

if [[ "${RIDGEBACK_KEEP_ROS_DAEMON:-0}" != "1" ]]; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "Ridgeback Jetson ROS env:"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-disabled}"
echo "  Jetson IP=$JETSON_IP  Ridgeback IP=$RIDGEBACK_IP"
