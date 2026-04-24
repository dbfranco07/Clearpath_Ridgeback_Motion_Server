#!/bin/bash
# sim_mission.sh — run a single autonomous mission in sim and collect metrics.
#
# Usage:
#   ./sim/scripts/sim_mission.sh --world corridor --target-room 305 --strategy vlm_guided_frontier
#   ./sim/scripts/sim_mission.sh --target-room 303   # uses .env defaults for world/strategy
#
# Output: results saved to sim/results/<timestamp>/mission.json

set -euo pipefail

SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RESULTS_DIR="$SIM_DIR/results"

# ── Args ─────────────────────────────────────────────────────────────
WORLD="${SIM_WORLD:-small_room}"
TARGET_ROOM=""
STRATEGY="${STRATEGY:-frontier_nearest}"

while [[ $# -gt 0 ]]; do
    case $1 in
        --world) WORLD="$2"; shift 2 ;;
        --target-room) TARGET_ROOM="$2"; shift 2 ;;
        --strategy) STRATEGY="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

if [ -z "$TARGET_ROOM" ]; then
    echo "Usage: sim_mission.sh --target-room <room_number> [--world <world>] [--strategy <strategy>]"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUT_DIR="$RESULTS_DIR/${TIMESTAMP}_room${TARGET_ROOM}_${STRATEGY}"
mkdir -p "$OUT_DIR"

echo "======================================================"
echo " Mission: find room $TARGET_ROOM"
echo " World:   $WORLD  |  Strategy: $STRATEGY"
echo " Output:  $OUT_DIR"
echo "======================================================"

# Send mission via dashboard REST API (autonomy container must be running)
RESPONSE=$(curl -sf -X POST http://localhost:8081/api/mission \
    -H "Content-Type: application/json" \
    -d "{\"command\": \"go to room $TARGET_ROOM and come back\", \"strategy\": \"$STRATEGY\"}" \
    2>/dev/null || echo '{"error": "dashboard not reachable"}')

echo "$RESPONSE" | tee "$OUT_DIR/mission_start.json"

# Poll for completion
echo ""
echo "Mission running... (Ctrl+C to abort)"
START_S=$SECONDS
while true; do
    sleep 5
    STATUS=$(curl -sf http://localhost:8081/api/mission/status 2>/dev/null || echo '{"state":"unknown"}')
    STATE=$(echo "$STATUS" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('state','?'))" 2>/dev/null || echo "?")
    ELAPSED=$((SECONDS - START_S))
    echo "  [${ELAPSED}s] State: $STATE"

    if [[ "$STATE" =~ ^(DONE|FAILED|ERROR)$ ]]; then
        echo "$STATUS" > "$OUT_DIR/mission_result.json"
        break
    fi

    if [ $ELAPSED -gt 600 ]; then
        echo "Mission timed out after 600s."
        echo '{"state": "TIMEOUT"}' > "$OUT_DIR/mission_result.json"
        break
    fi
done

# Collect metrics from dashboard
curl -sf http://localhost:8081/api/metrics 2>/dev/null > "$OUT_DIR/metrics.json" || true
curl -sf http://localhost:8081/api/map/snapshot 2>/dev/null > "$OUT_DIR/map_snapshot.pgm" || true

echo ""
echo "Results saved to: $OUT_DIR"
