#!/bin/bash
# sim_down.sh — stop all sim containers.
#
# Usage:
#   ./sim/scripts/sim_down.sh          # stop, keep volumes (preserves build cache)
#   ./sim/scripts/sim_down.sh --clean  # stop + remove volumes (full reset)

set -euo pipefail

SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
COMPOSE_FILE="$SIM_DIR/docker/docker-compose.yml"

CLEAN=false
for arg in "$@"; do
    [ "$arg" = "--clean" ] && CLEAN=true
done

cd "$SIM_DIR"

if $CLEAN; then
    echo "Stopping and removing all containers + volumes..."
    docker compose -f docker/docker-compose.yml --profile mock down -v --remove-orphans
    echo "Full clean done. Next sim_up.sh will rebuild from scratch."
else
    echo "Stopping sim containers (build cache preserved)..."
    docker compose -f docker/docker-compose.yml --profile mock down --remove-orphans
    echo "Done. Run sim_up.sh to restart."
fi
