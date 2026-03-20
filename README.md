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
| 🌐 `ridgeback_image_motion/web_controller.py` | FastAPI web dashboard with MJPEG streaming, omnidirectional teleop, and live status |
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

---

## 🚀 Quick Start

### 1️⃣ SSH into the Ridgeback

```bash
ssh administrator@10.158.39.184
# Password: clearpath
source /opt/ros/humble/setup.bash
```

### 2️⃣ Clone into ROS2 workspace

```bash
cd ~/ros2_ws/src
git clone git@github.com:SuperMadee/Clearpath_Ridgeback_Motion_Server.git ridgeback_image_motion
```

### 3️⃣ Build & run

**Terminal 1** — Motion server + Image publisher:
```bash
bash ~/ros2_ws/src/ridgeback_image_motion/scripts/ridgeback_start.sh
```

**Terminal 2** — Web controller:
```bash
bash ~/ros2_ws/src/ridgeback_image_motion/scripts/ridgeback_web.sh
```

### 4️⃣ Open the dashboard

🌐 **http://10.158.39.184:8080**

---

## 🎮 Teleop Controls

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
```

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
