#!/bin/bash
# sim_benchmark.sh — sweep exploration strategies × worlds × N trials.
#
# Usage:
#   ./sim/scripts/sim_benchmark.sh \
#       --worlds small_room,corridor \
#       --strategies frontier_nearest,vlm_guided_frontier \
#       --target-room 305 \
#       --trials 5
#
# Output: sim/results/benchmark_<timestamp>/results.csv
#         (also per-trial JSON in subdirs)
#
# The sim stack must already be running (sim_up.sh) before calling this.

set -euo pipefail

SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RESULTS_DIR="$SIM_DIR/results"

# ── Defaults ─────────────────────────────────────────────────────────
WORLDS="small_room"
STRATEGIES="frontier_nearest"
TARGET_ROOM=""
TRIALS=3

while [[ $# -gt 0 ]]; do
    case $1 in
        --worlds) WORLDS="$2"; shift 2 ;;
        --strategies) STRATEGIES="$2"; shift 2 ;;
        --target-room) TARGET_ROOM="$2"; shift 2 ;;
        --trials) TRIALS="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

if [ -z "$TARGET_ROOM" ]; then
    echo "Usage: sim_benchmark.sh --target-room <room> [options]"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BENCH_DIR="$RESULTS_DIR/benchmark_${TIMESTAMP}"
mkdir -p "$BENCH_DIR"

CSV="$BENCH_DIR/results.csv"
echo "world,strategy,trial,result,elapsed_s,path_length_m,vlm_calls,safety_interventions" > "$CSV"

IFS=',' read -ra WORLD_LIST <<< "$WORLDS"
IFS=',' read -ra STRAT_LIST <<< "$STRATEGIES"

TOTAL=$(( ${#WORLD_LIST[@]} * ${#STRAT_LIST[@]} * TRIALS ))
RUN=0

echo "======================================================"
echo " Benchmark: $TOTAL runs  (${#WORLD_LIST[@]} worlds × ${#STRAT_LIST[@]} strategies × $TRIALS trials)"
echo " Target room: $TARGET_ROOM"
echo " Output: $BENCH_DIR"
echo "======================================================"

for WORLD in "${WORLD_LIST[@]}"; do
  for STRATEGY in "${STRAT_LIST[@]}"; do
    for TRIAL in $(seq 1 $TRIALS); do
      RUN=$((RUN+1))
      echo ""
      echo "[$RUN/$TOTAL] World=$WORLD  Strategy=$STRATEGY  Trial=$TRIAL"

      # Restart sim with new world/strategy (send reset command)
      curl -sf -X POST http://localhost:8081/api/sim/reset \
          -H "Content-Type: application/json" \
          -d "{\"world\": \"$WORLD\", \"strategy\": \"$STRATEGY\"}" \
          > /dev/null 2>&1 || { echo "  Reset failed — is sim running?"; continue; }

      sleep 5  # let Gazebo reload

      # Run mission via sim_mission.sh (waits for completion)
      TRIAL_DIR="$BENCH_DIR/${WORLD}__${STRATEGY}__trial${TRIAL}"
      SIM_WORLD="$WORLD" STRATEGY="$STRATEGY" \
        "$SIM_DIR/scripts/sim_mission.sh" \
          --target-room "$TARGET_ROOM" \
          --world "$WORLD" \
          --strategy "$STRATEGY" 2>&1 | tee "$TRIAL_DIR/run.log" || true

      # Extract metrics
      METRICS_FILE="$TRIAL_DIR/metrics.json"
      if [ -f "$METRICS_FILE" ]; then
          RESULT=$(python3 -c "import json,sys; d=json.load(open('$METRICS_FILE')); \
              print(d.get('result','unknown'),d.get('elapsed_s',0),d.get('path_length_m',0), \
              d.get('vlm_calls',0),d.get('safety_interventions',0))" 2>/dev/null \
              || echo "unknown 0 0 0 0")
          echo "$WORLD,$STRATEGY,$TRIAL,$RESULT" | tr ' ' ',' >> "$CSV"
      else
          echo "$WORLD,$STRATEGY,$TRIAL,no_data,0,0,0,0" >> "$CSV"
      fi
    done
  done
done

echo ""
echo "======================================================"
echo " Benchmark complete!"
echo " Results: $CSV"
echo ""
echo " Quick summary:"
python3 - <<'PYEOF' "$CSV"
import sys, csv
from collections import defaultdict

data = defaultdict(lambda: {"results": [], "elapsed": [], "vlm": []})
with open(sys.argv[1]) as f:
    for row in csv.DictReader(f):
        key = (row["world"], row["strategy"])
        data[key]["results"].append(row["result"])
        try:
            data[key]["elapsed"].append(float(row["elapsed_s"]))
            data[key]["vlm"].append(int(row["vlm_calls"]))
        except:
            pass

print(f"{'World':<15} {'Strategy':<25} {'Success':>8} {'Avg time':>10} {'Avg VLM':>9}")
print("-" * 75)
for (world, strat), v in sorted(data.items()):
    n = len(v["results"])
    succ = sum(1 for r in v["results"] if "DONE" in r or "success" in r.lower())
    avg_t = sum(v["elapsed"]) / n if v["elapsed"] else 0
    avg_v = sum(v["vlm"]) / n if v["vlm"] else 0
    print(f"{world:<15} {strat:<25} {succ}/{n:>5}  {avg_t:>8.0f}s  {avg_v:>7.1f}")
PYEOF
echo "======================================================"
