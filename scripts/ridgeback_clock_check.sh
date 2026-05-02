#!/bin/bash

ridgeback_clock_sync_hint() {
    cat >&2 <<'EOF'
Fix the host clock before building/running ROS:
  date -u
  timedatectl status
  sudo timedatectl set-ntp true
  sudo systemctl restart systemd-timesyncd

If this host uses chrony instead of systemd-timesyncd:
  sudo systemctl restart chrony
  chronyc tracking

After the clock is correct, rebuild:
  rm -rf build/ridgeback_image_motion install/ridgeback_image_motion log
  colcon build --packages-select ridgeback_image_motion

To bypass only for emergency debugging:
  RIDGEBACK_SKIP_CLOCK_CHECK=1 <script>
EOF
}

ridgeback_check_clock_for_build() {
    if [[ "${RIDGEBACK_SKIP_CLOCK_CHECK:-0}" == "1" ]]; then
        echo "WARN: skipping clock check (RIDGEBACK_SKIP_CLOCK_CHECK=1)" >&2
        return 0
    fi

    local workspace="${1:-$PWD}"
    local now
    now="$(date +%s 2>/dev/null || true)"
    if [[ -z "$now" || ! "$now" =~ ^[0-9]+$ ]]; then
        echo "ERROR: could not read system clock." >&2
        ridgeback_clock_sync_hint
        return 1
    fi

    # Jan 1 2024. A Ridgeback/Jetson booted near the Unix epoch will make
    # ROS, TLS, and make dependency checks behave unpredictably.
    local min_epoch=1704067200
    if (( now < min_epoch )); then
        echo "ERROR: system clock is not synchronized." >&2
        echo "Current UTC: $(date -u 2>/dev/null || date)" >&2
        ridgeback_clock_sync_hint
        return 1
    fi

    local max_future_s="${RIDGEBACK_CLOCK_SKEW_MAX_S:-300}"
    if [[ ! "$max_future_s" =~ ^[0-9]+$ ]]; then
        max_future_s=300
    fi

    local cutoff=$((now + max_future_s))
    local candidates=()
    [[ -e "$workspace/CMakeLists.txt" ]] && candidates+=("$workspace/CMakeLists.txt")
    [[ -e "$workspace/package.xml" ]] && candidates+=("$workspace/package.xml")
    [[ -d "$workspace/srv" ]] && candidates+=("$workspace/srv")
    [[ -d /opt/ros/humble/lib ]] && candidates+=("/opt/ros/humble/lib")
    [[ -e /usr/lib/aarch64-linux-gnu/libpython3.10.so ]] && candidates+=("/usr/lib/aarch64-linux-gnu/libpython3.10.so")
    [[ -e /usr/lib/x86_64-linux-gnu/libpython3.10.so ]] && candidates+=("/usr/lib/x86_64-linux-gnu/libpython3.10.so")

    local future_hits
    future_hits="$(
        for path in "${candidates[@]}"; do
            if [[ -d "$path" ]]; then
                find "$path" -maxdepth 4 -type f -newermt "@$cutoff" -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null
            elif [[ -f "$path" ]]; then
                find "$path" -maxdepth 0 -type f -newermt "@$cutoff" -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null
            fi
        done | head -n 12
    )"

    if [[ -n "$future_hits" ]]; then
        echo "ERROR: system clock is older than files required by the ROS build." >&2
        echo "Current UTC: $(date -u 2>/dev/null || date)" >&2
        echo "Files more than ${max_future_s}s in the future:" >&2
        while IFS= read -r line; do
            echo "  $line" >&2
        done <<< "$future_hits"
        ridgeback_clock_sync_hint
        return 1
    fi
}
