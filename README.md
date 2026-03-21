# 🤖 Clearpath Ridgeback R100 — Motion Server & Web Controller

A ROS2 Humble package for the **Clearpath Ridgeback R100** omnidirectional robot featuring a motion service server, compressed image publisher, and a web-based teleop dashboard.

> 🔄 Adapted from the [TurtleBot3 AI361-MEX2](https://github.com/SuperMadee/AI361-MEX2) project for holonomic (omnidirectional) control.

---

## 📦 Package Contents

| File | Description |
|---|---|
| 🎯 `srv/Motion.srv` | Custom ROS2 service — supports `linear`, `lateral` (strafe), and `angular` velocity |
| 🏎️ `ridgeback_image_motion/motion_server.py` | Receives service calls → publishes to `/r100_0140/cmd_vel` |
| 📷 `ridgeback_image_motion/image_publisher.py` | Subscribes to RealSense raw images → re-publishes as compressed JPEG |
| 🌐 `ridgeback_image_motion/web_controller.py` | FastAPI web dashboard with MJPEG streaming, LiDAR map, omnidirectional teleop, and live status |
| 🚀 `scripts/ridgeback_start.sh` | Builds & runs motion_server + image_publisher |
| 🖥️ `scripts/ridgeback_web.sh` | Builds & runs the web controller |

---

## 🔧 Hardware

| Component | Model |
|---|---|
| 🤖 Platform | Clearpath Ridgeback R100 (omnidirectional) |
| 📡 Front LiDAR | Hokuyo UST-10LX (270°, 25Hz) |
| 📷 RGB-D Camera | Intel RealSense D435 |
| 🧭 IMU | Built-in (50Hz) |
| 🎮 Controller | PS4 Bluetooth |

---

## 🆚 Key Differences from TurtleBot Version

| Feature | TurtleBot3 🐢 | Ridgeback 🤖 |
|---|---|---|
| Drive type | Differential (2-wheel) | Omnidirectional (holonomic) |
| Motion service | `linear` + `angular` | `linear` + `lateral` + `angular` |
| Diagonal movement | Rotate → then move forward | True diagonal (strafe + forward simultaneously) |
| Camera source | OpenCV direct capture (RPi Camera) | ROS2 subscriber (RealSense raw → JPEG compress) |
| Teleop left/right | Rotate in place | Strafe sideways |
| Extra controls | — | Separate CCW/CW rotation buttons |
| Keyboard | — | WASD (move) + QE (rotate) + Space (stop) |
| LiDAR visualization | — | Live top-down 2D map with distance coloring |

---

## 🚀 How to Run

This system uses **two machines** — the Ridgeback's onboard PC runs the motion server and image publisher, while the Jetson runs the web controller. Both must be on the same network and use the same `ROS_DOMAIN_ID`.

### 📋 Prerequisites

Both machines need:
- ROS2 Humble sourced (`source /opt/ros/humble/setup.bash`)
- This package cloned and built in a ROS2 workspace

```bash
cd ~
git clone git@github.com:SuperMadee/Clearpath_Ridgeback_Motion_Server.git ridgeback
cd ~/ridgeback
colcon build --packages-select ridgeback_image_motion
source install/setup.bash
```

---

### 🤖 Step 1: Start the Ridgeback (onboard PC)

SSH into the Ridgeback:
```bash
ssh administrator@192.168.131.1
# Password: clearpath
```

Run the start script — this pulls the latest code, builds, and launches **motion_server** + **image_publisher**:
```bash
bash ~/ridgeback/scripts/ridgeback_start.sh
```

This starts:
- `/motion_server` — listens for `motion_service` calls and publishes to `/r100_0140/cmd_vel`
- `/image_publisher` — subscribes to RealSense raw images and publishes compressed JPEG

> 💡 The Clearpath platform nodes (motors, LiDAR, IMU, camera driver) are already running via `clearpath-robot.service` on boot.

---

### 🖥️ Step 2: Start the Jetson (web controller)

On the Jetson, run the web controller script:
```bash
bash ~/ridgeback/scripts/ridgeback_web.sh
```

This starts:
- `web_controller.py` — FastAPI server that subscribes to compressed images, odometry, and LiDAR, and serves the web dashboard

---

### 🌐 Step 3: Open the Dashboard

Open a browser on any device connected to the same network:

```
http://<jetson-ip>:8080
```

You should see:
- 📷 Live camera feed (MJPEG from RealSense)
- 🗺️ Live LiDAR top-down map
- 🎮 Omnidirectional teleop controls
- 📊 Real-time velocity, pose, and latency

---

### ✅ Verify Everything Is Connected

On either machine, check that all nodes can see each other:
```bash
# List all active nodes
ros2 node list

# Check image stream is flowing
ros2 topic hz /r100_0140/image/compressed

# Check motion service is available
ros2 service list | grep motion
```

---

## 🕹️ Basic Teleop (Without the Web Controller)

You can drive the Ridgeback directly from the terminal without running the motion server or web controller.

### Keyboard Teleop

```bash
# Install if not already installed
sudo apt-get install ros-humble-teleop-twist-keyboard

# Run keyboard teleop (remapped to the Ridgeback's cmd_vel topic)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/r100_0140/cmd_vel
```

Use `i` to go forward, `j`/`l` to rotate, `u`/`o` for arcs, and `k` to stop.

### Manual Drive Commands (ros2 topic pub)

```bash
# Forward at 0.2 m/s
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10

# Strafe left at 0.1 m/s (holonomic)
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10

# Rotate at 0.5 rad/s
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' -r 10

# Stop
ros2 topic pub /r100_0140/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once
```

### PS4 Controller

The PS4 controller works automatically via `clearpath-robot.service` — no extra setup needed. Just pair via Bluetooth and use the joysticks.

---

## 🎮 Web Dashboard Teleop Controls

### 🖱️ Click Controls (Omnidirectional Pad)

```
  ↖  ▲  ↗       ← Forward + Strafe diagonals
  ◄  ■  ►       ← Strafe left / Stop / Strafe right
  ↙  ▼  ↘       ← Backward + Strafe diagonals

  ↺ CCW   CW ↻  ← Rotation buttons
```

### ⌨️ Keyboard Controls

| Key | Action |
|---|---|
| `W` / `↑` | Forward |
| `S` / `↓` | Backward |
| `A` / `←` | Strafe Left |
| `D` / `→` | Strafe Right |
| `Q` | Rotate CCW |
| `E` | Rotate CW |
| `Space` | 🛑 Emergency Stop |

---

## 📡 ROS2 Topics Used

```
/r100_0140/cmd_vel                          → Drive commands (Twist)
/r100_0140/platform/odom/filtered           → EKF-filtered odometry
/r100_0140/sensors/camera_0/color/image     → RealSense raw RGB
/r100_0140/image/compressed                 → Compressed JPEG (published by image_publisher)
/r100_0140/sensors/lidar2d_0/scan           → Front LiDAR scan (LaserScan, 25Hz)
```

---

## 🗺️ LiDAR Map

The web dashboard includes a **live top-down LiDAR visualization** below the camera feed:

- 🟠 **Robot** shown as orange dot at center
- 🔴🟡🟢 **Scan points** color-coded by distance (red = close, yellow = mid, green = far)
- ⭕ **Range rings** at 1m, 2m, 3m, 4m for scale
- ⬆️ **Forward arrow** shows robot heading
- 📊 **Info bar** with closest obstacle distance and point count
- Updates at **5 Hz** for smooth visualization

---

## 🧩 ROS2 Node Architecture

All nodes running on the system, grouped by function.

### 🔗 Custom Nodes (Image & Motion Pipeline)

These are the custom nodes that bridge the **Ridgeback** and the **Jetson controller**:

| Node | Runs On | Role |
|---|---|---|
| `/image_publisher` | Ridgeback | Subscribes to raw RealSense images (`/r100_0140/sensors/camera_0/color/image`) and re-publishes as JPEG CompressedImage (`/r100_0140/image/compressed`) at up to 15 FPS |
| `/motion_server` | Ridgeback | Provides the `motion_service` ROS2 service — receives holonomic motion commands (`linear`, `lateral`, `angular`) and publishes Twist to `/r100_0140/cmd_vel` |
| `/launch_ros_1219` | Ridgeback | ROS2 launch daemon process that started the custom nodes above |
| Web Controller *(runs as FastAPI, not a ROS2 node)* | Jetson | Subscribes to compressed images, odometry, and LiDAR — calls `motion_service` on the Ridgeback — serves the web UI |

### 🏎️ Drive & Control Nodes

| Node | Role |
|---|---|
| `/r100_0140/controller_manager` | ros2_control manager — loads and manages hardware interfaces and controllers |
| `/r100_0140/platform_velocity_controller` | Converts `cmd_vel` Twist into individual wheel velocity commands for omnidirectional drive |
| `/r100_0140/twist_mux` | Multiplexes multiple Twist sources (joystick, teleop, autonomy) by priority |
| `/r100_0140/twist_server_node` | Clearpath internal twist relay between the mux and the controller |
| `/r100_0140/puma_control` | Manages PUMA motor controller state |
| `/r100_0140/puma_hardware_interface` | ros2_control hardware plugin — talks directly to PUMA motors over CAN bus |
| `/r100_0140/r100_node` | Platform node — communicates with the MCU firmware over CAN (e-stop, lighting, status) |
| `/r100_0140/vcan0_socket_can_receiver` | CAN bus bridge — receives CAN frames from the MCU into ROS2 |
| `/r100_0140/vcan0_socket_can_sender` | CAN bus bridge — sends CAN frames from ROS2 to the MCU |

### 🧭 Localization & State Estimation

| Node | Role |
|---|---|
| `/r100_0140/ekf_node` | Extended Kalman Filter — fuses wheel odometry + IMU → publishes filtered odometry (`/r100_0140/platform/odom/filtered`) |
| `/r100_0140/imu_filter_madgwick` | Madgwick orientation filter — produces stable orientation from raw IMU data |
| `/r100_0140/joint_state_broadcaster` | ros2_control broadcaster — publishes `/joint_states` from the hardware interface |
| `/r100_0140/robot_state_publisher` | Reads URDF + joint states → publishes all TF transforms (`/tf`, `/tf_static`) |

### 📡 Sensor Nodes

| Node | Role |
|---|---|
| `/r100_0140/sensors/camera_0/intel_realsense` | RealSense D435 driver — publishes raw RGB, depth, and pointcloud topics |
| `/r100_0140/sensors/camera_0/image_processing_container` | Composable node container — hosts image processing nodelets (rectification, debayering) |
| `/r100_0140/sensors/lidar2d_0/urg_node` | Hokuyo UST-10LX driver — publishes LaserScan at 25 Hz, 270° FOV |

### 🎮 Teleop & HID Nodes

| Node | Role |
|---|---|
| `/r100_0140/joy_node` | Reads PS4 controller over Bluetooth → publishes `sensor_msgs/Joy` |
| `/r100_0140/teleop_twist_joy_node` | Converts Joy messages into Twist velocity commands for `twist_mux` |

### 🩺 Diagnostics & Status Nodes

| Node | Role |
|---|---|
| `/r100_0140/analyzers` | diagnostic_aggregator — collects diagnostics from all subsystems |
| `/r100_0140/clearpath_diagnostics_updater` | Publishes periodic diagnostic updates for the platform |
| `/r100_0140/battery_state_estimator` | Estimates battery state of charge |
| `/r100_0140/battery_state_control` | Battery monitoring and control |
| `/r100_0140/lighting_node` | Controls the LED light ring (color patterns for status) |
| `/r100_0140/wireless_watcher` | Monitors WiFi connection status and publishes diagnostics |

### 📊 Data Flow Diagram

```
                         RIDGEBACK R100
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  RealSense ──► /intel_realsense ──► raw Image            │
│                                        │                 │
│                                   /image_publisher       │
│                                        │                 │
│                                  CompressedImage         │
│                                        │                 │
│                                        ▼                 │
│              ┌─────────────────────────────────┐         │
│              │        ROS2 DDS Network         │         │
│              │       (Domain ID = 0)           │         │
│              └─────────────────────────────────┘         │
│                                        ▲                 │
│  /motion_server ◄── motion_service ────┘                 │
│       │                                                  │
│       ▼                                                  │
│  /cmd_vel ──► twist_mux ──► velocity_ctrl ──► motors     │
│                                                          │
└──────────────────────────────────────────────────────────┘
                           │
                    ROS2 DDS (Domain ID 0)
                           │
┌──────────────────────────┴───────────────────────────────┐
│               JETSON / WEB CONTROLLER                    │
│                                                          │
│  web_controller.py (FastAPI)                             │
│    ├── subscribes: CompressedImage, Odometry, LaserScan  │
│    ├── calls: motion_service (client)                    │
│    └── serves: Web UI at :8080 (MJPEG + teleop)          │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

#### How It Works

The system runs on **two computers** that communicate over WiFi using **ROS2 DDS** (Domain ID 0) — no broker or central server needed, nodes discover each other automatically.

**Image Pipeline (Ridgeback → Jetson → Browser):**
1. The **RealSense D435** camera captures raw frames
2. The `/intel_realsense` driver node publishes them as `sensor_msgs/Image`
3. `/image_publisher` subscribes, compresses each frame to JPEG, and publishes as `CompressedImage` — this reduces bandwidth for WiFi streaming
4. The **web controller** on the Jetson subscribes to the compressed images and streams them as MJPEG video to the browser

**Motion Pipeline (Browser → Jetson → Ridgeback → Wheels):**
1. User presses a key (e.g. `W` for forward) on the web dashboard
2. The **web controller** sends a `motion_service` request over ROS2 to the Ridgeback
3. `/motion_server` receives the request with `linear`, `lateral`, and `angular` values and publishes a `Twist` message to `/cmd_vel`
4. `twist_mux` selects the highest-priority velocity source (web controller vs. PS4 joystick vs. autonomy)
5. `velocity_ctrl` converts the Twist into individual wheel speeds
6. **PUMA motor drivers** spin the 4 omnidirectional wheels

**The Closed Loop:** You see what the camera sees → you send a command → the robot moves → the next frame shows the result.

---

## 🛠️ Dependencies

- ROS2 Humble 🐝
- Python 3 🐍
- OpenCV (`python3-opencv`)
- cv_bridge
- FastAPI + Uvicorn (`pip install fastapi uvicorn`)
- NumPy

---

## 📄 License

Apache-2.0
