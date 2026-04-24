# Ridgeback Sim — MacBook Dev Environment

Full simulation of the autonomous room-finder stack running on macOS via
Docker. **No ROS2, Gazebo, or Python installation required on the host** —
everything runs inside containers.

Hardware (Jetson Orin Nano + Ridgeback) is available ~once/week, so this
sim is the primary dev loop. Key design rule: **what passes in sim must
pass on hardware**. See [Sim ↔ HW parity](#sim--hw-parity) below.

---

## Architecture

```
MacBook (host)
│
├── Docker bridge network: ros_net (172.30.0.0/24)
│   │
│   ├── gazebo    (172.30.0.10) — Ignition Fortress + Ridgeback model
│   │                             Publishes /r100_0140/* sensor topics
│   ├── bridge    (172.30.0.11) — ros-ign-bridge (Gazebo ↔ ROS2)
│   ├── autonomy  (172.30.0.12) — Our ROS2 nodes (slam, nav2, dashboard)
│   │                             Source bind-mounted from repo
│   └── mock_vlm  (172.30.0.20) — (optional) local DGX mock
│
├── http://localhost:8081        — Mission dashboard
└── ws://localhost:8765          — Foxglove WebSocket bridge

VLM (default): real DGX endpoint (over WiFi/VPN from MacBook)
VLM (offline): local mock_vlm container (--mock flag)
```

---

## One-time setup

### 1. Install prerequisites on macOS

```bash
# OrbStack (recommended on Apple Silicon — faster than Docker Desktop)
brew install --cask orbstack

# Or Docker Desktop (works on both Intel and Apple Silicon)
brew install --cask docker

# Foxglove Studio — optional but recommended for sensor/TF visualization
brew install --cask foxglove-studio
```

### 2. Create your .env file

```bash
cd ~/projects/ridgeback
cp sim/.env.example sim/.env
# Edit sim/.env:
#   - VLM_ENDPOINT: update if DGX IP/port changes
#   - SIM_WORLD: small_room | corridor | floor
#   - STRATEGY: frontier_nearest | vlm_guided_frontier | ...
```

### 3. Run the prerequisite check

```bash
bash sim/scripts/mac_setup.sh
```

This verifies Docker, disk space, DGX reachability, etc. Fix any
reported issues before proceeding.

---

## Every session

### Start the sim

```bash
# Real DGX VLM (default — highest fidelity)
bash sim/scripts/sim_up.sh

# Offline mock VLM (no DGX needed — for CI / off-campus)
bash sim/scripts/sim_up.sh --mock

# Different world
bash sim/scripts/sim_up.sh --world corridor

# Background (no log streaming)
bash sim/scripts/sim_up.sh --detach
```

First run builds Docker images (~10–15 min). Subsequent runs use cached
layers and start in ~1–2 min.

### Open the dashboards

```
http://localhost:8081       ← Mission dashboard (type goal here)
Foxglove Studio → ws://localhost:8765  ← Sensor/TF/map visualization
```

### Run a mission

Type in the dashboard, or via CLI:

```bash
bash sim/scripts/sim_mission.sh \
    --world corridor \
    --target-room 305 \
    --strategy vlm_guided_frontier
```

### Benchmark strategies

```bash
bash sim/scripts/sim_benchmark.sh \
    --worlds small_room,corridor \
    --strategies frontier_nearest,frontier_info_gain,vlm_guided_frontier \
    --target-room 305 \
    --trials 5
# Results: sim/results/benchmark_<timestamp>/results.csv
```

### Stop the sim

```bash
bash sim/scripts/sim_down.sh          # stop, keep build cache
bash sim/scripts/sim_down.sh --clean  # full reset (clears volumes)
```

---

## Development workflow

The autonomy source is **bind-mounted** into the `autonomy` container at
`/ros2_ws/src/ridgeback_autonomy`. Edit code locally in your IDE; rebuild
inside the container:

```bash
# Open a shell in the autonomy container
docker exec -it ridgeback-sim-autonomy-1 bash

# Inside container — rebuild and relaunch
colcon build --packages-select ridgeback_autonomy --symlink-install
source install/setup.bash
ros2 launch ridgeback_autonomy sim.launch.py strategy:=vlm_guided_frontier
```

Or just restart the container to get a clean rebuild:

```bash
docker compose -f sim/docker/docker-compose.yml restart autonomy
```

---

## Worlds

| File | Mirrors hardware phase | Description |
|---|---|---|
| `small_room.world` | Phase 2 | Single 5 m × 5 m room, one "Room 301" sign |
| `corridor.world` | Phase 3 | 14 m corridor, 4 rooms (301, 303, 305, 307) |
| `floor.world` | Phase 4 | L-shaped floor, 8 rooms, lobby start area |

---

## VLM backend

| `VLM_BACKEND` | Used when | Tradeoff |
|---|---|---|
| `dgx` (default) | Normal dev | Highest fidelity — real model, real latency, real prompts |
| `mock` | Offline / CI | Deterministic, fast, no DGX needed — but synthetic responses |

Switch via `.env` or `--mock` flag. The autonomy code is identical either
way; only the HTTP endpoint URL changes.

### Mock VLM fixtures

Edit `sim/config/mock_vlm_fixtures.yaml` to change which pose regions
return which room detections. Useful for testing:
- Strategy behavior in different room layouts
- Confidence thresholds
- Direction-hint following

---

## Sim ↔ HW parity

These rules are enforced to prevent sim-only bugs:

1. **Same namespace**: all topics are `/r100_0140/*` in both sim and hw.
2. **Same message types and rates**: see `sim/worlds/*.world` sensor configs.
3. **Shared launch file**: `sim.launch.py` and `hw.launch.py` both include
   `autonomy_core.launch.py`. Only sensor sourcing differs.
4. **Shared params**: `config/nav2_params.yaml`, `slam_params.yaml` are used
   verbatim. Sim overrides go in a separate overlay file.
5. **No mocks inside autonomy**: the only mock is `mock_vlm_server`.

**Fidelity gaps to keep in mind:**

- Gazebo LiDAR is ideal (no beam dropouts, no reflections). Tune
  costmap inflation after Phase 2 hardware bringup.
- Sim odom is near-perfect; EKF drift behavior differs on hardware.
- MacBook runs faster than Orin Nano. Always measure `tegrastats` on
  hardware weekly.

---

## CI

```bash
# Run unit tests against mock VLM (no DGX, no hardware needed)
cd sim
VLM_BACKEND=mock pytest tests/ -v -k "not integration"

# Run full integration test (requires sim_up.sh --mock to be running)
VLM_BACKEND=mock pytest tests/ -v --tb=short
```

---

## Troubleshooting

**Gazebo won't start**
```bash
docker logs ridgeback-sim-gazebo-1
# Common: shared memory issue on macOS — add SHM_SIZE=512m to docker-compose
```

**Dashboard not reachable at :8081**
```bash
docker logs ridgeback-sim-autonomy-1 | tail -30
# If 'ridgeback_autonomy not found': colcon build failed — check Python deps
```

**DGX not reachable**
```bash
# Test from host
nc -z -w3 10.158.36.90 8000 && echo "DGX reachable" || echo "use --mock"
# On campus WiFi or VPN needed for DGX access
```

**ros-ign-bridge fails**
```bash
docker logs ridgeback-sim-bridge-1
# Topic name mismatches: update the bridge command in docker-compose.yml
# to match your Ignition model's actual topic names
```

**Build very slow (first run)**  
Normal — Docker images include a full ROS2 desktop + Gazebo (~3 GB).
Subsequent builds use the layer cache and finish in seconds.
