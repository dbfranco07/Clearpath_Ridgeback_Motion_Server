#!/bin/bash
# sim_up.sh — build images (first time) and start the full sim stack.
#
# Usage:
#   ./sim/scripts/sim_up.sh              # real DGX VLM (default)
#   ./sim/scripts/sim_up.sh --mock       # offline mock VLM
#   ./sim/scripts/sim_up.sh --world corridor   # different world
#   ./sim/scripts/sim_up.sh --detach     # background (no log streaming)
#
# Opens dashboard at http://localhost:8081 when ready.
# Foxglove: open Foxglove Studio → connect to ws://localhost:8765

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM_DIR="$REPO_ROOT/sim"
COMPOSE_FILE="$SIM_DIR/docker/docker-compose.yml"
ENV_FILE="$SIM_DIR/.env"

# ── Parse args ────────────────────────────────────────────────────────
MOCK=false
WORLD=""
DETACH=false
for arg in "$@"; do
    case $arg in
        --mock) MOCK=true ;;
        --world=*) WORLD="${arg#*=}" ;;
        --world) shift; WORLD="$1" ;;
        --detach|-d) DETACH=true ;;
    esac
done

# ── Load .env ─────────────────────────────────────────────────────────
if [ ! -f "$ENV_FILE" ]; then
    echo "No sim/.env found. Creating from example..."
    cp "$SIM_DIR/.env.example" "$ENV_FILE"
    echo "Edit sim/.env (at minimum, check VLM_ENDPOINT) then rerun."
    exit 1
fi
set -a; source "$ENV_FILE"; set +a

# Override from flags
[ -n "$WORLD" ] && export SIM_WORLD="$WORLD"
$MOCK && export VLM_BACKEND=mock

# ── Summary ───────────────────────────────────────────────────────────
echo "======================================================"
echo " Ridgeback Sim — starting up"
echo "======================================================"
echo "  World:       ${SIM_WORLD:-small_room}"
echo "  VLM backend: ${VLM_BACKEND:-dgx}"
if [ "${VLM_BACKEND:-dgx}" = "dgx" ]; then
    echo "  VLM endpoint:${VLM_ENDPOINT:-not set}"
fi
echo "  Strategy:    ${STRATEGY:-frontier_nearest}"
echo ""

# ── Build and start ───────────────────────────────────────────────────
cd "$SIM_DIR"

PROFILES=""
$MOCK && PROFILES="--profile mock"

COMPOSE_OPTS="$PROFILES -f docker/docker-compose.yml --env-file .env"

echo "Building images (cached after first run)..."
docker compose $COMPOSE_OPTS build --quiet

echo ""
echo "Starting containers..."

if $DETACH; then
    docker compose $COMPOSE_OPTS up -d
    echo ""
    echo "Containers started in background."
else
    docker compose $COMPOSE_OPTS up &
    COMPOSE_PID=$!
fi

# ── Wait for dashboard ────────────────────────────────────────────────
echo ""
echo "Waiting for dashboard to come up..."
MAX_WAIT=120
ELAPSED=0
while ! curl -sf http://localhost:8081/health &>/dev/null; do
    sleep 3
    ELAPSED=$((ELAPSED+3))
    if [ $ELAPSED -ge $MAX_WAIT ]; then
        echo "Dashboard did not start within ${MAX_WAIT}s. Check logs:"
        echo "  docker compose -f sim/docker/docker-compose.yml logs autonomy"
        break
    fi
    echo -n "."
done

echo ""
echo "======================================================"
echo " Sim stack is ready!"
echo "======================================================"
echo "  Dashboard:   http://localhost:8081"
echo "  Foxglove:    ws://localhost:8765  (open Foxglove Studio)"
if $MOCK; then
    echo "  Mock VLM:    http://localhost:9000 (local)"
else
    echo "  VLM:         $VLM_ENDPOINT (DGX)"
fi
echo ""
echo "  Type your mission at the dashboard — e.g.: 'go to room 305 and come back'"
echo ""
echo "  To stop: ./sim/scripts/sim_down.sh"
echo "======================================================"

if ! $DETACH; then
    wait $COMPOSE_PID
fi
