# Ridgeback R100 Autonomous Navigation — Experiment Plan

**Date:** 2026-04-17
**Robot:** Clearpath Ridgeback R100 (r100-0140)
**Duration:** ~5 hours
**Location:** 2nd Floor
**Team:** Deo Franco

---

## Table of Contents

1. [Pre-Flight Checklist](#pre-flight-checklist)
2. [Phase 1 — Safety + Teleop via Nav2 (Small Room)](#phase-1--safety--teleop-via-nav2-small-room)
3. [Phase 2 — VLM Perception (Small Room)](#phase-2--vlm-perception-small-room)
4. [Phase 3 — Full Pipeline / Mission (Small Room)](#phase-3--full-pipeline--mission-small-room)
5. [Phase 4 — Hallway Test (Larger Scale)](#phase-4--hallway-test-larger-scale)
6. [Phase 5 — Full 2nd Floor Deployment](#phase-5--full-2nd-floor-deployment)
7. [Code Explanations](#code-explanations)
8. [Troubleshooting](#troubleshooting)

---

## Pre-Flight Checklist

Do this **every time** before powering on the robot.

### Physical Inspection
- [ ] Clear the testing area of loose cables, bags, and fragile items
- [ ] Verify E-Stop button is accessible and functional (press it, release it, press Stop Reset)
- [ ] Check battery level (lights should not be yellow/low)
- [ ] Confirm wheels are not obstructed
- [ ] Verify the LiDAR (front Hokuyo) lens is clean and unobstructed
- [ ] Verify the RealSense camera is mounted and connected via USB

### Network Setup
Open **3 terminal windows** on your laptop. You will use them throughout:

| Terminal | Label | Purpose |
|----------|-------|---------|
| T1 | **Ridgeback** | SSH into the robot |
| T2 | **Jetson** | SSH into the Jetson Orin |
| T3 | **Local** | Monitoring from your laptop |

```bash
# T1 — SSH into the Ridgeback
ssh administrator@192.168.131.1
# Password: clearpath
source /opt/ros/humble/setup.bash

# T2 — SSH into the Jetson
ssh dbfranco@10.158.36.90
source /opt/ros/humble/setup.bash

# T3 — Local (your laptop)
source /opt/ros/humble/setup.bash
```

### Verify Robot Services Are Running (T1)

```bash
# Check the main clearpath services
sudo systemctl status clearpath-robot.service
sudo systemctl status clearpath-platform
sudo systemctl status clearpath-sensors
```

**What to look for:** All three should say `active (running)`. If not:
```bash
sudo systemctl restart clearpath-robot.service
# Wait 10 seconds, then check again
```

### Verify Sensor Topics (T1)

```bash
# Check that key topics are publishing
ros2 topic hz /r100_0140/sensors/lidar2d_0/scan
# Expected: ~25 Hz

ros2 topic hz /r100_0140/sensors/imu_0/data
# Expected: ~50 Hz

ros2 topic hz /r100_0140/platform/odom/filtered
# Expected: ~50 Hz
```

Press `Ctrl+C` after each one once you see the rate.

```bash
# Check E-Stop is released
ros2 topic echo /r100_0140/platform/mcu/status/stop --once
# stop_power_status should be false
```

**If `stop_power_status: true`:** Release the E-Stop mushroom button, then press the Stop Reset button on the robot.

### Start Image Publisher on Ridgeback (T1)

```bash
cd ~/projects/ridgeback
source install/setup.bash

# Start the image publisher (compresses RealSense images for streaming)
ros2 run ridgeback_image_motion image_publisher
```

**What this does:** The RealSense camera publishes raw images (~10 MB/frame). `image_publisher` compresses them to JPEG (~50 KB/frame) and republishes on `/r100_0140/image/compressed`. This is essential because the Jetson receives images over WiFi — raw images would saturate the link.

Leave this running. Open a **new terminal** (T1b) if you need to run more commands on the Ridgeback.

---

## Phase 1 — Safety + Teleop via Nav2 (Small Room)

**Goal:** Verify the safety controller stops the robot before hitting walls, and that SLAM + Nav2 can build a map and accept navigation goals.
**Duration:** ~60 minutes
**Risk Level:** Low (robot stays in one room, safety layer is the first thing we test)

### Step 1.1 — Launch Safety Controller Only (T2)

```bash
cd ~/projects/ridgeback
source install/setup.bash

# Launch ONLY the safety controller + cmd_vel_mux
ros2 launch ridgeback_image_motion autonomy.launch.py profile:=teleop
```

**What this does:**
- Starts `safety_controller` — monitors LiDAR for obstacles within 0.45m (danger zone) and 0.8m (warning zone). If anything enters the danger zone while the robot is moving forward, it publishes a zero-velocity override.
- Starts `cmd_vel_mux` — a priority multiplexer that decides which velocity command reaches the robot. Priority order: safety override (highest) > Nav2 > teleop (lowest). This ensures the safety controller can always stop the robot.

### Step 1.2 — Verify Safety Controller Is Working (T3)

```bash
# Watch the safety status topic
ros2 topic echo /safety/status
```

**Expected output:**
```
stamp:
  sec: ...
  nanosec: ...
is_safe: true
closest_obstacle_m: 2.35      # distance to nearest wall
danger_zone_m: 0.45
status_text: "OK"
lidar_active: true
emergency_stop_active: false
---
```

**Key fields to watch:**
- `is_safe`: `true` means nothing in the danger zone
- `closest_obstacle_m`: should decrease as you push something toward the LiDAR
- `lidar_active`: must be `true` — if `false`, the safety controller will emergency-stop

### Step 1.3 — Test Safety by Walking Toward the Robot (T3)

**IMPORTANT: Keep your hand on the E-Stop during this test.**

1. While watching `/safety/status` in T3, slowly walk toward the front of the robot
2. Watch `closest_obstacle_m` decrease
3. When you enter the warning zone (< 0.8m), `status_text` should change to `"WARNING: obstacle at X.XXm"`
4. When you enter the danger zone (< 0.45m), `is_safe` should flip to `false` and `emergency_stop_active` to `true`

**Pass criteria:** `is_safe` becomes `false` before you touch the robot.

### Step 1.4 — Test Teleop Through the Mux (T3, new terminal)

```bash
# Send a teleop command through the mux
ros2 topic pub /cmd_vel_teleop geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
```

**What this does:** Publishes a slow forward command (0.1 m/s) to the **teleop input** of the mux. The mux should pass it through to `/r100_0140/cmd_vel` since no higher-priority source is active.

**Expected:** The robot creeps forward slowly. Press `Ctrl+C` to stop publishing (robot will stop after the mux timeout of 0.3s).

Now test the safety override:

1. Start the teleop command again (same command above)
2. Walk in front of the robot
3. The robot should stop even though teleop is still publishing
4. Walk away — the robot should resume moving

**Pass criteria:** Safety controller overrides teleop commands when obstacles are detected.

### Step 1.5 — Stop Safety-Only, Launch Full SLAM + Nav2 (T2)

Press `Ctrl+C` in T2 to stop the safety-only launch. Then:

```bash
# Launch the full autonomy stack (but without VLM for now)
ros2 launch ridgeback_image_motion autonomy.launch.py launch_vlm:=false
```

**What this does:** Launches the tiered startup:
- **t=0s:** safety_controller + cmd_vel_mux (same as before)
- **t=2s:** SLAM Toolbox (online async) + Nav2 (planner, controller, costmaps, recoveries)
- **t=5s:** spatial_memory (database, but no VLM feeding it yet)
- **t=8s:** mission_orchestrator (state machine, but we won't use it yet)
- **t=10s:** web_dashboard (port 8081)

The `launch_vlm:=false` flag skips the VLM perception node since we're testing navigation first.

### Step 1.6 — Open the Web Dashboard

Open a browser on your laptop and go to:

```
http://10.158.36.90:8081
```

(Replace with the Jetson's current IP if it changed.)

**What you should see:**
- Camera feed (MJPEG stream from the RealSense)
- SLAM map (starts as a small area around the robot)
- Safety status indicator (green = safe)
- Teleop controls (arrow buttons or WASD keys)
- Mission control panel (we'll use this later)

### Step 1.7 — Build a Map with Teleop (Dashboard)

Use the dashboard's teleop controls (WASD keys or arrow buttons) to slowly drive the robot around the room.

**What is happening behind the scenes:**
1. You press a key on the dashboard → the dashboard publishes to `/cmd_vel_teleop`
2. `cmd_vel_mux` receives it as priority 3 (teleop) and forwards to `/r100_0140/cmd_vel`
3. The robot moves; odometry updates on `/r100_0140/platform/odom/filtered`
4. SLAM Toolbox receives LiDAR scans + odometry and builds an occupancy grid map
5. The map is published on `/map` and rendered live in the dashboard

**Tips for good mapping:**
- Drive slowly (the dashboard limits speed to 0.3 m/s)
- Make sure you revisit areas so SLAM can close loops
- Watch the map in the dashboard — walls should appear as solid lines
- If the map looks distorted, drive back to a known area for loop closure

### Step 1.8 — Send a Nav2 Goal via CLI

Once you have a small map of the room:

```bash
# T3 — Send a navigation goal to Nav2
# First, check the robot's current pose
ros2 topic echo /r100_0140/platform/odom/filtered --once | head -20
```

Note the current `position.x` and `position.y`. Then pick a point ~1-2 meters away that you've already mapped:

```bash
# Send Nav2 goal (replace X and Y with a mapped location)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**What this does:**
1. Nav2 receives the goal via the `NavigateToPose` action server
2. The global planner (NavFn) computes a path on the map
3. The local planner (DWB, holonomic-enabled) generates velocity commands
4. Commands go to `/cmd_vel_nav2` → `cmd_vel_mux` (priority 2) → `/r100_0140/cmd_vel`
5. The safety controller monitors throughout — if the robot gets too close to a wall, it overrides

**Keep your hand on E-Stop!** This is the first time Nav2 drives the robot.

**Expected:** The robot navigates to the goal, avoiding obstacles. It may spin, strafe sideways, or back up — this is normal for holonomic navigation.

### Step 1.9 — Test Nav2 Recovery Behaviors

Intentionally place a chair or box in the robot's path while it's navigating to a goal.

**Expected:** Nav2 should:
1. Detect the obstacle in the local costmap
2. Try to replan around it
3. If stuck, execute recovery behaviors (back up, spin)
4. If completely blocked, abort the goal

**Pass criteria for Phase 1:**
- [ ] Safety controller detects obstacles and stops the robot
- [ ] Safety override takes priority over teleop
- [ ] LiDAR heartbeat watchdog works (test by briefly blocking LiDAR — robot should stop)
- [ ] SLAM builds a coherent map of the room
- [ ] Nav2 navigates to goals within the mapped area
- [ ] Nav2 recovery behaviors work when path is blocked
- [ ] Web dashboard shows map, camera, and safety status

---

## Phase 2 — VLM Perception (Small Room)

**Goal:** Verify the Vision-Language Model can detect room numbers from the camera feed.
**Duration:** ~45 minutes
**Risk Level:** Low (robot can stay stationary for most of this)

### Step 2.1 — Start the VLLM Server on the Jetson (T2, new terminal)

Open a new terminal on the Jetson (T2b):

```bash
# Check if the VLLM server is already running
curl http://localhost:8000/v1/models 2>/dev/null
```

If it responds with a JSON listing the Qwen model, skip ahead. Otherwise:

```bash
# Start the VLLM server (this takes ~2-3 minutes to load the model)
python3 -m vllm.entrypoints.openai.api_server \
  --model Qwen/Qwen2.5-VL-7B-Instruct \
  --port 8000 \
  --max-model-len 4096 \
  --gpu-memory-utilization 0.85
```

**What this does:** Launches a local HTTP server that serves the Qwen 2.5 VL 7B model using the OpenAI-compatible API format. The VLM node will POST images to `http://10.158.36.90:8000/v1/chat/completions` as base64-encoded JPEGs.

Wait until you see `INFO: Application startup complete` before proceeding.

### Step 2.2 — Verify VLM Server Manually

```bash
# Quick test — send a dummy request to verify the server responds
curl http://localhost:8000/v1/models
```

**Expected:** JSON with model info including `"id": "Qwen/Qwen2.5-VL-7B-Instruct"`.

### Step 2.3 — Relaunch with VLM Enabled (T2)

Go back to the main Jetson terminal and stop the current launch (`Ctrl+C`), then:

```bash
# Launch with VLM enabled
ros2 launch ridgeback_image_motion autonomy.launch.py
```

(VLM is enabled by default, so no extra flags needed.)

### Step 2.4 — Point the Robot at a Room Number Sign

Manually teleop (via the dashboard at `:8081`) so the camera faces a room number sign on a door.

### Step 2.5 — Monitor VLM Detections (T3)

```bash
# Watch VLM perception output
ros2 topic echo /vlm/perception
```

**Expected output when facing a room sign:**
```
stamp:
  sec: ...
description: "Hallway with tiled floor, fluorescent lighting..."
detected_rooms:
- "301"
room_confidence:
- 0.85
environment_type: "hallway"
has_obstacles: false
robot_x: 1.23
robot_y: 0.45
robot_yaw: 0.1
---
```

**What is happening:**
1. `vlm_perception` captures a frame from `/r100_0140/image/compressed` every 3 seconds
2. It resizes the image to max 640px width and encodes as base64 JPEG
3. It sends an HTTP POST to the VLLM server with a system prompt asking to identify room numbers, environment type, and obstacles
4. The model's text response is parsed with regex to extract structured data
5. The result is published as a `Perception` message

**Key things to verify:**
- `detected_rooms` contains the correct room number
- `room_confidence` is above 0.6 (the auto-store threshold)
- `environment_type` makes sense (hallway, room, intersection, etc.)
- The VLM doesn't hallucinate room numbers when none are visible

### Step 2.6 — Verify Spatial Memory Storage (T3)

```bash
# Check what spatial memory has stored in the dashboard Mission+Mem panel.
# The current ridgeback_image_motion stack stores this in ~/ridgeback_memory.db.
sqlite3 ~/ridgeback_memory.db 'select label, room_number, x, y, confidence, created_at from locations order by id desc limit 20;'
```

**Expected:** Should list the start position and any rooms the VLM detected with confidence ≥ 0.6.

```bash
# Query a specific room
sqlite3 ~/ridgeback_memory.db "select label, x, y, confidence, created_at from locations where room_number='301' order by confidence desc, id desc limit 5;"
```

**What is happening:**
- `spatial_memory` listens to `/vlm/perception` messages
- When a room is detected with confidence ≥ 0.6, it stores the room ID along with the robot's current map-frame coordinates (from `/r100_0140/platform/odom/filtered`) into a SQLite database at `~/ridgeback_memory.db`
- If the same room is detected again, it updates the stored coordinates using a weighted average (higher confidence = more influence)
- The database persists across restarts — the robot "remembers" where rooms are

### Step 2.7 — Test VLM with No Room Number Visible

Face the robot toward a blank wall or open space.

**Expected:** `detected_rooms` should be empty (`[]`) and `room_confidence` empty (`[]`).

**Pass criteria for Phase 2:**
- [ ] VLLM server is running and responding
- [ ] VLM correctly identifies room numbers from camera images
- [ ] VLM does not hallucinate room numbers when none are visible
- [ ] Spatial memory auto-stores detected rooms with correct coordinates
- [ ] Memory persists (stop and restart the stack, rooms are still stored)
- [ ] Dashboard shows VLM perception log with detections

---

## Phase 3 — Full Pipeline / Mission (Small Room)

**Goal:** Test the complete natural language mission: "Go to room X" → explore → find room → arrive → "Come back" → return to start.
**Duration:** ~60 minutes
**Risk Level:** Medium (robot navigates autonomously, but confined to a small area)

### Step 3.1 — Reset Spatial Memory

```bash
# T3 — Clear the memory database so the robot starts fresh
ros2 service call /memory/reset std_srvs/srv/Trigger '{}'
```

**Why:** We want to test the full flow from "no knowledge" — the robot should explore, discover rooms via VLM, store them in memory, and navigate to them.

### Step 3.2 — Issue a Natural Language Command via Dashboard

Open the dashboard at `http://10.158.36.90:8081`.

In the **Mission Control** panel, type:

```
Go to room 301
```

Press **Send**.

### Step 3.3 — Monitor the Mission State Machine (T3)

```bash
# Watch mission status
ros2 topic echo /mission/status
```

**What happens step by step:**

1. **IDLE → CHECKING_MEMORY:** The mission orchestrator receives "Go to room 301". Its NL parser extracts intent=`GO_TO_ROOM`, room=`301`. It queries spatial memory for room 301.

2. **CHECKING_MEMORY → EXPLORING** (if room is unknown): Room 301 is not in memory. The orchestrator enters exploration mode:
   - It reads the current `/map` (occupancy grid from SLAM)
   - `exploration.py` finds **frontiers** — boundaries between explored (free) and unexplored (unknown) cells
   - Frontiers are clustered and sorted by distance
   - The closest frontier centroid becomes a Nav2 goal
   - While exploring, the VLM scans for room numbers every 3 seconds

3. **EXPLORING → ARRIVED** (when room is found): When the VLM detects room 301 with confidence ≥ 0.6:
   - The detection is auto-stored in spatial memory with the current coordinates
   - The orchestrator cancels the current exploration goal
   - It sends a Nav2 goal to the stored coordinates of room 301
   - When the robot is within `arrive_tolerance_m` (1.0m) of the target, state becomes ARRIVED

4. **ARRIVED** (waiting for next command): The robot stops and waits.

### Step 3.4 — Observe Frontier Exploration

While the robot is in EXPLORING state, watch:

```bash
# T3 — Monitor mission status for exploration details
ros2 topic echo /mission/status
```

**Key fields:**
- `state`: should show `EXPLORING`
- `frontiers_explored`: increments as the robot visits frontiers
- `rooms_discovered`: list of rooms the VLM has found so far
- `status_text`: describes what the robot is doing

**Watch the SLAM map on the dashboard** — you should see the map growing as the robot explores new areas.

### Step 3.5 — Command the Robot to Return

Once the robot arrives at room 301, type in the Mission Control panel:

```
Come back
```

or

```
Return to start
```

**What happens:**
1. The NL parser extracts intent=`RETURN_TO_START`
2. The orchestrator queries spatial memory for `__start__` (the position saved at launch)
3. It sends a Nav2 goal to the start coordinates
4. State: ARRIVED → RETURNING → RETURNED → IDLE

### Step 3.6 — Test Edge Cases

**a) Command while already on a mission:**
```
Go to room 305
```
While the robot is exploring for 301. **Expected:** The orchestrator should reject or queue the command.

**b) Stop command:**
```
Stop
```
**Expected:** The robot stops immediately, cancels the current Nav2 goal, and returns to IDLE.

**c) Unknown command:**
```
Do a backflip
```
**Expected:** The NL parser returns `UNKNOWN` intent, and the robot replies with a help message.

**d) Room already known (second request):**
After completing phase 3.5, reset memory and re-run. Then after room 301 is discovered once, issue the same command again. This time it should skip EXPLORING and go directly to NAVIGATING (since 301 is already in memory).

### Step 3.7 — Test Safety During Autonomous Navigation

**With the robot exploring:**

1. Walk in front of it → it should stop (safety override)
2. Move away → it should resume exploring
3. Place a chair in its planned path → Nav2 should replan
4. Block all paths → Nav2 should execute recovery behaviors, then abort

**Pass criteria for Phase 3:**
- [ ] NL parser correctly extracts room numbers and intents
- [ ] Exploration discovers new frontiers and systematically covers the area
- [ ] VLM detects room numbers during exploration
- [ ] Robot navigates to discovered rooms
- [ ] "Return to start" brings robot back to launch position
- [ ] Safety override works during autonomous navigation
- [ ] Exploration timeout (10 min) works if room is never found
- [ ] Dashboard shows mission progress in real-time

---

## Phase 4 — Hallway Test (Larger Scale)

**Goal:** Test the system in a real hallway with multiple rooms. Verify SLAM handles larger environments and the robot can find and return from rooms further away.
**Duration:** ~75 minutes
**Risk Level:** Medium-High (hallway has pedestrians, the robot must be monitored at all times)

### Step 4.1 — Safety Preparation

- [ ] **Designate a safety spotter** — one person walks alongside the robot at all times with hand near E-Stop
- [ ] **Announce testing** — let people on the floor know a robot will be moving in the hallway
- [ ] **Clear the hallway** — remove trash cans, chairs, and other obstacles that could confuse the robot
- [ ] **Identify safe zones** — spots where you can quickly pull the robot aside if needed

### Step 4.2 — Move Robot to Hallway Starting Position

Use teleop (dashboard) to drive the robot to the hallway entrance. Or physically push it (with E-Stop engaged, the wheels should be free).

### Step 4.3 — Fresh Start

```bash
# T2 — Stop and restart the full stack
# Ctrl+C in T2 to stop, then:
ros2 launch ridgeback_image_motion autonomy.launch.py

# T3 — Reset memory for a clean start
ros2 service call /memory/reset std_srvs/srv/Trigger '{}'
```

### Step 4.4 — Build a Baseline Map

Before autonomous exploration, manually teleop down the hallway and back to build a baseline map. This ensures SLAM has a good initial map with loop closures.

**Why this is important:** SLAM in a long hallway without loop closure can accumulate drift. By manually driving the full length first, we give the SLAM algorithm reference points.

Watch the map on the dashboard. The hallway should appear as two parallel walls.

### Step 4.5 — Test Room Finding in Hallway

```
Go to room 301
```

**What to watch for:**
- Does the robot systematically explore side corridors and doorways?
- Does the VLM detect room numbers on doors?
- Does frontier exploration prioritize unexplored areas?
- Does the map remain consistent (no "ghosting" of walls)?

### Step 4.6 — Test Return Over Longer Distance

Once arrived at room 301:

```
Come back
```

**What to watch for:**
- Does the robot navigate the full hallway back to start?
- Does it use the SLAM map for efficient path planning?
- Does it handle pedestrians (safety controller should slow/stop)?

### Step 4.7 — Test Multiple Rooms

Repeat for 2-3 different rooms:

```
Go to room 303
```
Then return. Then:
```
Go to room 305
```

**The second and third runs should be faster** because rooms are now in spatial memory — the robot skips exploration and navigates directly.

### Step 4.8 — Stress Test: Revisit Known Room

```
Go to room 301
```

**Expected:** The robot should remember room 301's location from earlier and navigate there directly without exploring. This validates the spatial memory persistence.

**Pass criteria for Phase 4:**
- [ ] SLAM handles hallway-scale environment (20+ meters)
- [ ] Robot navigates safely past pedestrians
- [ ] VLM detects room numbers on actual door signs
- [ ] Spatial memory correctly maps multiple rooms
- [ ] Return-to-start works over long distances
- [ ] Second visit to a known room skips exploration
- [ ] Map quality remains good (no major drift or artifacts)

---

## Phase 5 — Full 2nd Floor Deployment

**Goal:** Full end-to-end demonstration. A person gives a natural language command, the robot finds the room and returns. This is the "demo run" for the professor.
**Duration:** ~60 minutes
**Risk Level:** Medium-High

### Step 5.1 — Pre-Demo Setup

```bash
# T2 — Fresh launch
ros2 launch ridgeback_image_motion autonomy.launch.py

# T3 — Optional: keep some known rooms in memory from Phase 4
# Or reset for a full tabula rasa demo:
ros2 service call /memory/reset std_srvs/srv/Trigger '{}'
```

### Step 5.2 — Demo Script

**Demo 1: Explore and find a room (tabula rasa)**

```
Go to room 303
```

Show the professor:
- The SLAM map building in real-time on the dashboard
- The VLM perception log showing room detections
- The mission status showing exploration progress
- The safety controller stopping for obstacles

**Demo 2: Return to start**

```
Come back
```

Show:
- The robot navigating back using the map it built
- The spatial memory showing start position and discovered rooms

**Demo 3: Use memory to revisit**

```
Go to room 303
```

Show:
- The robot going directly to the room (no exploration needed)
- Much faster than the first time

**Demo 4: Find a new room**

```
Go to room 305
```

Show:
- Exploration resumes but only for unmapped areas
- Previously mapped areas are skipped

### Step 5.3 — Key Talking Points for the Professor

**Q: How does the robot know where to go?**
A: It doesn't start with any map. It uses SLAM (Simultaneous Localization And Mapping) with the front-facing LiDAR to build a map in real-time. It explores by finding "frontiers" — boundaries between known and unknown space — and systematically visiting them.

**Q: How does it recognize room numbers?**
A: The onboard camera captures frames every 3 seconds and sends them to a Vision-Language Model (Qwen 2.5 VL 7B) running on the Jetson Orin. The model reads text in the image and identifies room numbers. When a room is found, its map coordinates are stored in a SQLite database.

**Q: What if it loses network connection?**
A: The safety controller runs on the Jetson Orin, which is a separate computer from the Ridgeback. If the WiFi link drops, the safety controller detects the LiDAR heartbeat timeout (no scans for 2+ seconds) and immediately stops the robot. The safety controller is the highest-priority input to the velocity mux — nothing can override it.

**Q: How does it avoid obstacles?**
A: Three layers of protection:
1. **Safety controller** — LiDAR-based hard stop at 0.45m
2. **Nav2 local costmap** — inflates obstacles and plans around them
3. **Physical E-Stop** — always available to the operator

**Q: How does it find its way back?**
A: At launch, the robot saves its starting position in the spatial memory database. When you say "come back," the mission orchestrator queries this `__start__` position and sends it as a Nav2 goal. Nav2 plans a path on the SLAM map.

**Q: Is the map pre-loaded?**
A: No. The map starts completely empty (tabula rasa). Every map is built from scratch using SLAM at runtime. This is a project requirement.

---

## Code Explanations

### `safety_controller.py` — The Guardian

The safety controller is the most critical node. It runs independently and cannot be overridden.

**How it works:**
```
LiDAR scan arrives (25 Hz)
  → Filter to front cone (±60° from forward direction)
  → Find minimum range in the front cone
  → If min_range < danger_zone (0.45m) AND robot is moving forward:
       Publish zero velocity on /safety/cmd_vel_override
       Set emergency_stop_active = true
  → If min_range < critical_zone (0.225m) from ANY direction:
       Publish zero velocity (regardless of movement direction)
  → Else:
       Set is_safe = true, no override published
```

**LiDAR heartbeat:** A separate timer checks if LiDAR data has arrived in the last 2 seconds. If not, it assumes the sensor or network has failed and triggers emergency stop. This is the "Jetson safety controller" requirement — even if the WiFi to the Ridgeback drops, the safety controller on the Jetson stops sending commands.

**Key parameter tuning:**
- `danger_zone_m: 0.45` — the robot is ~0.96m × 0.79m, so 0.45m gives enough stopping distance at max speed (0.4 m/s)
- `front_cone_deg: 60.0` — only checks the front ±30° for forward movement stops (a wider cone would prevent rotation near walls)

---

### `cmd_vel_mux.py` — The Traffic Controller

This node ensures only one velocity source controls the robot at a time.

**Priority system:**
```
Priority 1 (highest): /safety/cmd_vel_override    — safety controller
Priority 2:           /cmd_vel_nav2               — Nav2 autonomous navigation
Priority 3 (lowest):  /cmd_vel_teleop             — human teleop from dashboard
```

**How it works:**
- Runs at 20 Hz
- Each input has a timeout (safety: 1.0s, nav2: 0.5s, teleop: 0.3s)
- On each tick, it publishes the command from the highest-priority source that has sent a message within its timeout
- If no source is active, it publishes zero velocity (robot stops)

**Why this matters:** Without the mux, if you send a teleop command while Nav2 is navigating, both would fight for control of the robot. The mux ensures Nav2 takes priority over teleop, and safety takes priority over everything.

---

### `vlm_perception.py` — The Eyes

This node gives the robot the ability to "see" and understand its environment using AI.

**Flow:**
```
1. Capture frame from /r100_0140/image/compressed (every 3 seconds)
2. Resize to max 640px width (reduce bandwidth)
3. Encode as base64 JPEG string
4. Build HTTP POST request:
   {
     "model": "Qwen/Qwen2.5-VL-7B-Instruct",
     "messages": [
       {"role": "system", "content": "You are a robot perception system..."},
       {"role": "user", "content": [
         {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}}
         {"type": "text", "text": "Describe what you see. Report room numbers..."}
       ]}
     ]
   }
5. Send to VLLM at http://10.158.36.90:8000/v1/chat/completions
6. Parse response with regex:
   - ROOM_NUMBERS: 301(0.85), 303(0.70)
   - ENVIRONMENT: hallway
   - OBSTACLES: chair on left
7. Publish as Perception message
```

**Rate limiting:** If the VLM is still processing the previous frame (typical response time: 2-5 seconds), new frames are dropped. This prevents a backlog.

---

### `spatial_memory.py` — The Memory

This node gives the robot persistent spatial memory using SQLite.

**Database schema:**
```sql
room_locations:
  room_id TEXT PRIMARY KEY,    -- e.g., "301" or "__start__"
  map_x REAL,                  -- map-frame X coordinate
  map_y REAL,                  -- map-frame Y coordinate
  map_yaw REAL,                -- robot heading when detected
  confidence REAL,             -- VLM confidence (0.0 - 1.0)
  detection_count INTEGER,     -- how many times detected
  first_seen TEXT,             -- ISO timestamp
  last_seen TEXT,              -- ISO timestamp
  session_id TEXT              -- launch session UUID
```

**Auto-store logic:**
- Subscribes to `/vlm/perception`
- When a room is detected with confidence ≥ 0.6:
  - If room is new → INSERT with current odometry coordinates
  - If room exists → UPDATE coordinates using weighted average:
    `new_x = (old_x * old_count + current_x) / (old_count + 1)`
  - Increment `detection_count`
- The `__start__` position is stored automatically at node startup

**Why weighted average?** The robot may detect the same room from different positions. Averaging gives the centroid of all detection positions, which is usually near the door — a good navigation target.

---

### `exploration.py` — The Explorer (Helper Module)

Not a ROS node — a pure Python module imported by `mission_orchestrator.py`.

**Frontier-based exploration algorithm:**
```
1. Receive occupancy grid (from SLAM /map topic)
   - Each cell is: 0 = free, 100 = occupied, -1 = unknown

2. Find frontier cells:
   - A frontier cell is a FREE cell that has at least one UNKNOWN neighbor
   - Uses 8-connected adjacency (including diagonals)

3. Cluster frontiers (BFS):
   - Group adjacent frontier cells into clusters
   - Discard small clusters (< 5 cells) — these are noise

4. Compute cluster centroids:
   - Convert from grid indices to map-frame coordinates
   - Sort by distance from robot (closest first)

5. Return list of frontier waypoints
```

**Why frontiers?** A frontier represents the boundary between what the robot has mapped and what it hasn't. By navigating to frontiers, the robot systematically explores the entire environment. When no more frontiers remain, the environment is fully mapped.

---

### `mission_orchestrator.py` — The Brain

The state machine that ties everything together.

**State diagram:**
```
IDLE
 │
 ├─ "Go to room 301" ──→ CHECKING_MEMORY
 │                            │
 │                   ┌────────┴────────┐
 │                   │                 │
 │              Room known        Room unknown
 │                   │                 │
 │                   ▼                 ▼
 │              NAVIGATING        EXPLORING
 │                   │                 │
 │                   │          (VLM finds room)
 │                   │                 │
 │                   └────────┬────────┘
 │                            ▼
 │                        ARRIVED
 │                            │
 ├─ "Come back" ──────→ RETURNING
 │                            │
 │                            ▼
 │                        RETURNED
 │                            │
 │                            ▼
 └────────────────────────── IDLE
```

**Natural Language Parser:**
The NL parser is intentionally simple — regex-based, not an LLM:
- `"go to room 301"` → intent: `GO_TO_ROOM`, room: `"301"`
- `"take me to 305"` → intent: `GO_TO_ROOM`, room: `"305"`
- `"come back"` / `"return"` → intent: `RETURN_TO_START`
- `"stop"` / `"halt"` → intent: `STOP`
- Anything else → intent: `UNKNOWN`

**Exploration strategy:**
When a room is unknown, the orchestrator:
1. Gets frontiers from `exploration.py`
2. Sends the closest frontier as a Nav2 goal
3. Waits for Nav2 to reach the goal (or fail/abort)
4. Checks if the VLM has found the target room during transit
5. If not found, gets new frontiers and repeats
6. Times out after 10 minutes if room is never found

---

### `web_dashboard.py` — The Interface

A FastAPI web application that serves as the unified control center.

**Architecture:**
```
Browser (your laptop)
  ↕ HTTP
FastAPI server (port 8081 on Jetson)
  ↕ ROS2 topics/services
All other nodes
```

**Key endpoints:**
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Full HTML dashboard page |
| `/video_feed` | GET | MJPEG stream (camera) |
| `/map` | GET | SLAM map as PNG image |
| `/status` | GET | JSON: pose, velocity, battery |
| `/safety` | GET | JSON: safety status |
| `/mission_status` | GET | JSON: mission state, progress |
| `/perception_log` | GET | JSON: recent VLM detections |
| `/memory` | GET | JSON: all stored room locations |
| `/teleop` | POST | Send velocity command |
| `/mission` | POST | Send NL mission command |
| `/stop` | POST | Emergency stop all movement |

The dashboard page embeds JavaScript that polls these endpoints every 200-500ms to update the UI in real-time.

---

### `image_publisher.py` — The Compressor (Runs on Ridgeback)

**Why this exists:** The RealSense camera publishes raw images at ~30 FPS. Each 640×480 RGB frame is ~900 KB. At 30 FPS, that is ~27 MB/s — far too much for WiFi. This node:

1. Subscribes to raw images from the RealSense
2. Rate-limits to 15 FPS
3. Compresses each frame to JPEG (quality 75) — ~50 KB per frame
4. Publishes on `/r100_0140/image/compressed`

Result: bandwidth drops from ~27 MB/s to ~0.75 MB/s, which WiFi can handle.

---

### `sim_topic_bridge.py` — The Translator (Simulation Only)

Bridges TurtleBot3 Gazebo topics to Ridgeback's namespaced topics so all autonomy nodes work unchanged in simulation:

| Gazebo Topic | → | Ridgeback Topic |
|---|---|---|
| `/scan` | → | `/r100_0140/sensors/lidar2d_0/scan` |
| `/odom` | → | `/r100_0140/platform/odom/filtered` |
| `/r100_0140/cmd_vel` | → | `/cmd_vel` |
| `/camera/image_raw` | → | `/r100_0140/image/compressed` (converts raw → JPEG) |

This way you never need to modify any autonomy code for simulation vs real robot.

---

## Troubleshooting

### Robot Won't Move

```bash
# 1. Check E-Stop
ros2 topic echo /r100_0140/platform/mcu/status/stop --once
# If stop_power_status: true → release E-Stop + press Stop Reset

# 2. Check safety controller
ros2 topic echo /safety/status --once
# If emergency_stop_active: true → clear the obstacle in front of the LiDAR

# 3. Check cmd_vel_mux output
ros2 topic echo /r100_0140/cmd_vel
# Should show non-zero velocities when teleop/Nav2 is active

# 4. Check if any node crashed
ros2 node list | grep -E "safety|mux|nav2|mission"
```

### SLAM Map Looks Wrong

```bash
# Check LiDAR data
ros2 topic echo /r100_0140/sensors/lidar2d_0/scan --once
# ranges should be reasonable (0.1m - 10m for indoor)

# Check odometry
ros2 topic echo /r100_0140/platform/odom/filtered --once
# position should change as robot moves

# Check TF tree
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link
```

**Common SLAM issues:**
- Map "jumping" = loop closure correction (normal, but jarring)
- Walls appearing doubled = odometry drift (drive more slowly)
- Map not updating = SLAM node crashed (check T2 terminal for errors)

### VLM Not Detecting Rooms

```bash
# 1. Verify VLLM server is running
curl http://10.158.36.90:8000/v1/models

# 2. Check if images are arriving
ros2 topic hz /r100_0140/image/compressed
# Should be ~15 Hz

# 3. Check VLM node is alive
ros2 node list | grep vlm

# 4. Check for VLM errors in launch output (T2)
# Look for HTTP timeout errors or JSON parse errors
```

### Nav2 Fails to Plan a Path

```bash
# 1. Check if the map has the destination
# Open dashboard → look at SLAM map → is the target area mapped?

# 2. Check Nav2 costmap
ros2 topic echo /local_costmap/costmap --once
# Should have data

# 3. Check Nav2 controller
ros2 action list
# Should include /navigate_to_pose
```

### Network Issues (Jetson ↔ Ridgeback)

```bash
# From Jetson, check DDS communication
ros2 topic list
# Should show /r100_0140/* topics

# If topics are missing, check DDS config
echo $FASTRTPS_DEFAULT_PROFILES_FILE
# Should point to fastrtps_jetson.xml

# Ping the Ridgeback
ping 10.158.39.203 -c 3
```

---

## Time Budget Summary

| Phase | Duration | What We're Testing |
|-------|----------|--------------------|
| Pre-Flight | 15 min | Physical inspection, network, sensors |
| Phase 1 | 60 min | Safety controller, SLAM, Nav2 teleop + goals |
| Phase 2 | 45 min | VLM perception, spatial memory auto-store |
| Phase 3 | 60 min | Full mission pipeline (NL → explore → find → return) |
| Phase 4 | 75 min | Hallway-scale navigation with multiple rooms |
| Phase 5 | 60 min | Full demo run, professor Q&A prep |
| **Total** | **~5 hr 15 min** | |

---

## Emergency Procedures

1. **E-Stop:** Press the red mushroom button on the robot. This cuts motor power immediately.
2. **Software stop:** In any terminal: `ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once`
3. **Dashboard stop:** Press the red STOP button on the web dashboard.
4. **Kill all nodes:** `Ctrl+C` in the Jetson terminal (T2) to stop all autonomy nodes.
5. **If robot is unresponsive:** Use the physical E-Stop. If E-Stop doesn't work, disconnect the battery.
