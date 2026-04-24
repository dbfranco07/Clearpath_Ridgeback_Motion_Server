#!/bin/bash
# mac_setup.sh — one-time prerequisite check for the MacBook sim environment.
# Run: bash sim/scripts/mac_setup.sh
#
# Checks: Docker/OrbStack, architecture, disk space, DGX reachability.
# Does NOT install anything — just tells you what's missing.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM_DIR="$REPO_ROOT/sim"
ENV_FILE="$SIM_DIR/.env"
EXAMPLE_ENV="$SIM_DIR/.env.example"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; ERRORS=$((ERRORS+1)); }

ERRORS=0

echo "=================================================="
echo " Ridgeback Sim — MacBook prerequisite check"
echo "=================================================="
echo ""

# ── Architecture ───────────────────────────────────────────────────────
ARCH=$(uname -m)
if [ "$ARCH" = "arm64" ]; then
    ok "Architecture: Apple Silicon (arm64) — Docker images are native, no emulation."
elif [ "$ARCH" = "x86_64" ]; then
    ok "Architecture: Intel x86_64 — Docker images are native."
else
    warn "Architecture: $ARCH — untested. May need manual image adjustments."
fi

# ── Docker / OrbStack ──────────────────────────────────────────────────
if command -v docker &>/dev/null; then
    DOCKER_VERSION=$(docker --version 2>/dev/null | head -1)
    if docker info &>/dev/null 2>&1; then
        ok "Docker: running — $DOCKER_VERSION"
        # Recommend OrbStack for Apple Silicon
        if [ "$ARCH" = "arm64" ] && ! pgrep -q "OrbStack" 2>/dev/null; then
            warn "Consider OrbStack instead of Docker Desktop on Apple Silicon — faster, lighter."
            warn "  brew install --cask orbstack"
        fi
    else
        fail "Docker daemon is not running. Start Docker Desktop or OrbStack."
    fi
else
    fail "Docker not found. Install OrbStack (recommended on Apple Silicon) or Docker Desktop."
    echo "    brew install --cask orbstack"
    echo "    # or: brew install --cask docker"
fi

# ── docker compose ─────────────────────────────────────────────────────
if docker compose version &>/dev/null 2>&1; then
    ok "docker compose: available"
else
    fail "docker compose plugin not found. Update Docker/OrbStack to a recent version."
fi

# ── Disk space ─────────────────────────────────────────────────────────
FREE_GB=$(df -g / | awk 'NR==2 {print $4}')
if [ "$FREE_GB" -ge 20 ]; then
    ok "Disk space: ${FREE_GB} GB free (need ~15 GB for Docker images)"
else
    warn "Disk space: ${FREE_GB} GB free — may be tight. Images need ~15 GB."
fi

# ── Docker memory allocation ───────────────────────────────────────────
DOCKER_MEM_BYTES=$(docker info --format '{{.MemTotal}}' 2>/dev/null || echo 0)
DOCKER_MEM_GB=$(( DOCKER_MEM_BYTES / 1024 / 1024 / 1024 ))
if [ "$DOCKER_MEM_GB" -ge 6 ]; then
    ok "Docker memory: ${DOCKER_MEM_GB} GB"
else
    warn "Docker memory: ${DOCKER_MEM_GB} GB — recommend ≥ 8 GB. Adjust in Docker/OrbStack settings."
fi

# ── .env file ──────────────────────────────────────────────────────────
if [ -f "$ENV_FILE" ]; then
    ok ".env: found at sim/.env"
else
    warn ".env not found. Creating from example..."
    cp "$EXAMPLE_ENV" "$ENV_FILE"
    echo "    Edit sim/.env and set VLM_ENDPOINT to the DGX URL."
fi

# ── DGX reachability ───────────────────────────────────────────────────
if [ -f "$ENV_FILE" ]; then
    VLM_ENDPOINT=$(grep '^VLM_ENDPOINT=' "$ENV_FILE" | cut -d= -f2- | tr -d '"' | tr -d "'")
    VLM_HOST=$(echo "$VLM_ENDPOINT" | sed -E 's|https?://([^/:]+).*|\1|')
    VLM_PORT=$(echo "$VLM_ENDPOINT" | sed -E 's|.*:([0-9]+).*|\1|')
    VLM_PORT="${VLM_PORT:-8000}"

    if nc -z -w3 "$VLM_HOST" "$VLM_PORT" 2>/dev/null; then
        ok "DGX VLM reachable: $VLM_HOST:$VLM_PORT"
    else
        warn "DGX VLM not reachable at $VLM_HOST:$VLM_PORT"
        warn "  If off-campus: use VPN or switch to VLM_BACKEND=mock in sim/.env"
    fi
fi

# ── Foxglove Studio (optional) ─────────────────────────────────────────
if [ -d "/Applications/Foxglove Studio.app" ] || command -v foxglove-studio &>/dev/null; then
    ok "Foxglove Studio: installed"
else
    warn "Foxglove Studio not found (optional, but recommended for sensor/TF visualization)."
    warn "  brew install --cask foxglove-studio"
fi

# ── Summary ────────────────────────────────────────────────────────────
echo ""
echo "=================================================="
if [ "$ERRORS" -eq 0 ]; then
    echo -e "${GREEN}All checks passed! Ready to start the sim.${NC}"
    echo ""
    echo "  cd $(realpath --relative-to="$PWD" "$SIM_DIR" 2>/dev/null || echo "$SIM_DIR")"
    echo "  ./scripts/sim_up.sh"
else
    echo -e "${RED}$ERRORS check(s) failed. Fix the issues above before running sim_up.sh.${NC}"
fi
echo "=================================================="
