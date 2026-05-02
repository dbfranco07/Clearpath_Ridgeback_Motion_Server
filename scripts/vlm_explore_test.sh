#!/bin/bash
# Time-boxed VLM-driven exploration test.
#
# Runs on the Jetson alongside an already-running autonomy stack
# (e.g. `RIDGEBACK_PROFILE=mapping RIDGEBACK_LAUNCH_VLM=true RIDGEBACK_LAUNCH_NAV2=true goridge`).
# Sends a start command to frontier_explorer, waits N minutes, sends stop, then saves the SLAM map.

set -e

MINUTES=5
MAP_NAME="vlm_explore_$(date +%Y%m%d_%H%M%S)"
MAP_DIR="$HOME/maps"
COMMAND_TOPIC="/ridgeback/exploration/command"
STATUS_TOPIC="/ridgeback/exploration/status"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --minutes N        Run exploration for N minutes (default: 5)
  --map-name NAME    Output map basename (default: vlm_explore_<timestamp>)
  --map-dir DIR      Output directory (default: ~/maps)
  -h, --help         Show this help

The script publishes start/stop on $COMMAND_TOPIC and calls
slam_toolbox save_map + serialize_map at the end.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --minutes)
            MINUTES="$2"; shift 2 ;;
        --minutes=*)
            MINUTES="${1#*=}"; shift ;;
        --map-name)
            MAP_NAME="$2"; shift 2 ;;
        --map-name=*)
            MAP_NAME="${1#*=}"; shift ;;
        --map-dir)
            MAP_DIR="$2"; shift 2 ;;
        --map-dir=*)
            MAP_DIR="${1#*=}"; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "ERROR: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if ! [[ "$MINUTES" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    echo "ERROR: --minutes must be a number (got: $MINUTES)" >&2
    exit 2
fi

mkdir -p "$MAP_DIR"
SECONDS_TOTAL=$(awk "BEGIN { printf \"%.0f\", $MINUTES * 60 }")
MAP_PATH="$MAP_DIR/$MAP_NAME"

echo "=========================================="
echo "VLM exploration test"
echo "=========================================="
echo "Duration: ${MINUTES} min (${SECONDS_TOTAL}s)"
echo "Map output: $MAP_PATH.{pgm,yaml,posegraph,data}"
echo

# Pre-flight: make sure the autonomy stack is up.
required=( "$COMMAND_TOPIC" "/map" "/ridgeback/exploration/status" )
for topic in "${required[@]}"; do
    if ! timeout 4 ros2 topic info "$topic" >/dev/null 2>&1; then
        echo "ERROR: topic $topic not found. Is the Jetson autonomy stack (goridge) running?" >&2
        exit 1
    fi
done

if ! timeout 4 ros2 service list 2>/dev/null | grep -q "^/slam_toolbox/save_map$"; then
    echo "ERROR: /slam_toolbox/save_map service not found. Is SLAM running?" >&2
    exit 1
fi

stop_exploration() {
    echo "[stop] sending stop command..."
    ros2 topic pub --once "$COMMAND_TOPIC" std_msgs/msg/String \
        '{data: "{\"action\": \"stop\"}"}' >/dev/null 2>&1 || true
}

trap stop_exploration EXIT

echo "[start] sending start command..."
ros2 topic pub --once "$COMMAND_TOPIC" std_msgs/msg/String \
    '{data: "{\"action\": \"start\"}"}' >/dev/null

echo "[run] exploring for ${SECONDS_TOTAL}s..."
START_T=$(date +%s)
END_T=$(( START_T + SECONDS_TOTAL ))
while (( $(date +%s) < END_T )); do
    sleep 5
    REMAINING=$(( END_T - $(date +%s) ))
    if (( REMAINING < 0 )); then REMAINING=0; fi
    LATEST=$(timeout 2 ros2 topic echo --once "$STATUS_TOPIC" --field data 2>/dev/null | tr -d '\0' | head -c 240 || true)
    echo "  t-${REMAINING}s | status: ${LATEST:-<no status>}"
done

stop_exploration
trap - EXIT
sleep 2

echo
echo "[save] calling /slam_toolbox/save_map -> $MAP_PATH.{pgm,yaml}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$MAP_PATH'}}"

echo
echo "[serialize] calling /slam_toolbox/serialize_map -> $MAP_PATH.{posegraph,data}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$MAP_PATH'}" || echo "WARN: serialize_map failed (non-fatal)"

echo
echo "=========================================="
echo "Done. Map files in $MAP_DIR:"
ls -la "$MAP_DIR" | grep "$MAP_NAME" || echo "(no files matched — check service output above)"
