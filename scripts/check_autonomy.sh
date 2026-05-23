#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_USER="${CHECK_JETSON_USER:-dbfranco}"
JETSON_HOST="${CHECK_JETSON_HOST:-jetson-ridgeback.local}"
RIDGEBACK_USER="${CHECK_RIDGEBACK_USER:-administrator}"
RIDGEBACK_HOST="${CHECK_RIDGEBACK_HOST:-192.168.131.1}"
JETSON_SETUP="${CHECK_JETSON_SETUP:-~/ridgeback/install/setup.bash}"
RIDGEBACK_SETUP="${CHECK_RIDGEBACK_SETUP:-~/ridgeback99/install/setup.bash}"
ROBOT_NS="${CHECK_ROBOT_NS:-r100_0140}"
VLM_URL="${CHECK_VLM_URL:-http://202.92.159.240:8000/v1}"
VLM_CHAT_URL="${CHECK_VLM_CHAT_URL:-}"
VLM_CHAT_PATH="${CHECK_VLM_CHAT_PATH:-}"

SSH_OPTS=(
    -o BatchMode=yes
    -o ConnectTimeout=5
    -o StrictHostKeyChecking=accept-new
)

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RESET='\033[0m'

ok=1

section() {
    printf '\n== %s ==\n' "$1"
}

pass() {
    printf "%bOK%b %s\n" "$GREEN" "$RESET" "$1"
}

warn() {
    printf "%bWARN%b %s\n" "$YELLOW" "$RESET" "$1"
}

fail() {
    printf "%bFAIL%b %s\n" "$RED" "$RESET" "$1" >&2
    ok=0
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        fail "missing command: $1"
        return 1
    fi
}

run_ssh() {
    local label="$1"
    local user="$2"
    local host="$3"
    local remote_script="$4"

    printf '\n-- %s (%s@%s) --\n' "$label" "$user" "$host"
    if ! ssh "${SSH_OPTS[@]}" "$user@$host" "bash -lc $(printf '%q' "$remote_script")"; then
        fail "$label check failed"
    else
        pass "$label check passed"
    fi
}

jetson_script=$(cat <<EOF
set -euo pipefail
source /opt/ros/humble/setup.bash
source $JETSON_SETUP
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=1

check_topic() {
    local topic="\$1"
    local count
    count="\$(timeout 5 ros2 topic info "\$topic" 2>/dev/null | awk '/Publisher count:/ { print \$3; exit }' || true)"
    if [[ -n "\$count" && "\$count" != "0" ]]; then
        printf '  OK   %s (%s publisher(s))\n' "\$topic" "\$count"
    else
        printf '  FAIL %s has no publishers\n' "\$topic"
        exit 1
    fi
}

echo "ROS_DOMAIN_ID=\${ROS_DOMAIN_ID}"
check_topic "/cmd_vel/teleop"
check_topic "/cmd_vel/mux_out"
check_topic "/safety/latched"
check_topic "/operator/heartbeat"
check_topic "/vlm/observation"

vlm_models_code="\$(curl -o /dev/null -sS -w '%{http_code}' '$VLM_URL/models' || true)"
vlm_chat_code="000"
if [[ -n '$VLM_CHAT_URL' ]]; then
    vlm_chat_code="\$(curl -o /dev/null -sS -w '%{http_code}' -X POST '$VLM_CHAT_URL' -H 'Content-Type: application/json' -d '{}' || true)"
else
    for path in '${VLM_CHAT_PATH:-/chat/completions}' '/chat/completions'; do
        chat_url="${VLM_URL%/}/\${path#/}"
        vlm_chat_code="\$(curl -o /dev/null -sS -w '%{http_code}' -X POST "\$chat_url" -H 'Content-Type: application/json' -d '{}' || true)"
        if [[ "\$vlm_chat_code" != "404" && "\$vlm_chat_code" != "000" ]]; then
            break
        fi
        if [[ '$VLM_URL' == */v1 ]]; then
            chat_url="${VLM_URL%/v1}/\${path#/}"
            vlm_chat_code="\$(curl -o /dev/null -sS -w '%{http_code}' -X POST "\$chat_url" -H 'Content-Type: application/json' -d '{}' || true)"
            if [[ "\$vlm_chat_code" != "404" && "\$vlm_chat_code" != "000" ]]; then
                break
            fi
        fi
    done
fi
echo "VLM /models HTTP \${vlm_models_code}"
echo "VLM /chat/completions HTTP \${vlm_chat_code}"
if [[ "\$vlm_chat_code" == "404" ]]; then
    echo "  FAIL VLM endpoint mismatch: /chat/completions is missing"
    exit 1
fi
EOF
)

ridgeback_script=$(cat <<EOF
set -euo pipefail
source /opt/ros/humble/setup.bash
source $RIDGEBACK_SETUP
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

check_topic() {
    local topic="\$1"
    local count
    count="\$(timeout 5 ros2 topic info "\$topic" 2>/dev/null | awk '/Publisher count:/ { print \$3; exit }' || true)"
    if [[ -n "\$count" && "\$count" != "0" ]]; then
        printf '  OK   %s (%s publisher(s))\n' "\$topic" "\$count"
    else
        printf '  FAIL %s has no publishers\n' "\$topic"
        exit 1
    fi
}

echo "ROS_DOMAIN_ID=\${ROS_DOMAIN_ID}"
check_topic "/$ROBOT_NS/platform/cmd_vel_unstamped"
check_topic "/$ROBOT_NS/cmd_vel"
check_topic "/$ROBOT_NS/platform/odom/filtered"
check_topic "/$ROBOT_NS/sensors/lidar2d_0/scan"
EOF
)

section "Prerequisites"
require_cmd ssh
require_cmd curl
require_cmd awk
require_cmd timeout
pass "local tooling present"

section "Jetson"
run_ssh "Jetson autonomy stack" "$JETSON_USER" "$JETSON_HOST" "$jetson_script"

section "Ridgeback"
run_ssh "Ridgeback platform bridge" "$RIDGEBACK_USER" "$RIDGEBACK_HOST" "$ridgeback_script"

section "Summary"
if [[ "$ok" == "1" ]]; then
    pass "all autonomy checks passed"
else
    fail "one or more checks failed"
fi

exit "$((ok == 1 ? 0 : 1))"
