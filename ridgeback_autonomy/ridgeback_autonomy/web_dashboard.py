#!/usr/bin/env python3
"""
Autonomous Operations Web Dashboard — Runs on Jetson Orin
==========================================================
Extended web dashboard for the full autonomous navigation stack.
Serves on port 8081 (8080 is reserved for the legacy teleop web_controller).

New capabilities over the base web_controller:
  - Live SLAM map visualization (OccupancyGrid → canvas)
  - VLM perception log with thumbnails
  - Mission control panel (natural language commands)
  - Spatial memory viewer
  - Safety status panel

Retains all teleop features from the original web_controller:
  - MJPEG live camera feed
  - Keyboard + button-based holonomic teleop (via /cmd_vel_teleop)
  - LiDAR visualization
  - Pose + velocity display
  - Battery status

Run alongside the original web_controller (different port) or as a replacement.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, BatteryState
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool
from ridgeback_autonomy.msg import SafetyStatus, MissionStatus, Perception
from ridgeback_autonomy.srv import MissionCommand, QueryLocation, GetAllLocations

import cv2
import numpy as np
import threading
import time
import math
import json
import base64
from contextlib import asynccontextmanager
from datetime import datetime

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from pydantic import BaseModel
import uvicorn


# ── Pydantic models ────────────────────────────────────────────────────────

class TeleopCmd(BaseModel):
    linear: float = 0.0
    lateral: float = 0.0
    angular: float = 0.0


class MissionCmd(BaseModel):
    command: str


class RearOverrideCmd(BaseModel):
    enabled: bool


# ── ROS2 Node ──────────────────────────────────────────────────────────────

class AutonomousDashboard(Node):
    def __init__(self):
        super().__init__('autonomous_dashboard')
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('port', 8081)
        self.declare_parameter('host', '0.0.0.0')

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage, '/r100_0140/image/compressed',
            self._image_cb, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/r100_0140/platform/odom/filtered',
            self._odom_cb, sensor_qos
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/r100_0140/sensors/lidar2d_0/scan',
            self._lidar_cb, sensor_qos
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/r100_0140/platform/bms/state',
            self._battery_cb, sensor_qos
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, latch_qos
        )
        self.safety_sub = self.create_subscription(
            SafetyStatus, '/safety/status', self._safety_cb, reliable_qos
        )
        self.mission_sub = self.create_subscription(
            MissionStatus, '/mission/status', self._mission_cb, reliable_qos
        )
        self.perception_sub = self.create_subscription(
            Perception, '/vlm/perception', self._perception_cb, reliable_qos
        )

        # Publishers (teleop goes through cmd_vel_mux)
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', cmd_qos)
        # Latched so safety_controller gets the current state even after restart
        self.rear_override_pub = self.create_publisher(Bool, '/safety/rear_override', latch_qos)

        # Service clients
        self.mission_client = self.create_client(
            MissionCommand, 'mission/command', callback_group=self.cb_group
        )
        self.get_all_client = self.create_client(
            GetAllLocations, 'memory/get_all_locations', callback_group=self.cb_group
        )

        # ── State ──────────────────────────────────────────────────────────

        # Camera
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()

        # Odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.vel_linear = 0.0
        self.vel_lateral = 0.0
        self.vel_angular = 0.0
        self.pose_offset_x = 0.0
        self.pose_offset_y = 0.0

        # LiDAR
        self.lidar_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.lidar_range_max = 10.0
        self.lidar_lock = threading.Lock()

        # Battery
        self.battery_voltage = 0.0
        self.battery_pct = 0.0

        # Map (SLAM OccupancyGrid)
        self.latest_map_image_b64 = None   # PNG base64 rendered map
        self.map_info = None               # OccupancyGrid.info
        self.map_lock = threading.Lock()
        self.map_robot_px = (0, 0)         # Robot pixel position on map image
        self.map_rooms = []                # [{room_id, px, py}, ...] for overlay

        # Safety
        self.safety_status = {
            'is_safe': True, 'closest_m': 99.0,
            'stop_active': False, 'status_text': 'No data', 'lidar_active': False,
            'block_pos_x': False, 'block_neg_x': False,
            'block_pos_y': False, 'block_neg_y': False,
            'block_pos_yaw': False, 'block_neg_yaw': False,
            'clearance_fwd': -1.0, 'clearance_rev': -1.0,
            'clearance_left': -1.0, 'clearance_right': -1.0,
        }
        self.rear_override_active = False

        # Mission
        self.mission_status = {
            'state': 'IDLE', 'target_room': '', 'status_text': 'Ready',
            'progress': 0.0, 'rooms_discovered': 0, 'known_rooms': []
        }

        # VLM perception log (recent entries from /vlm/perception)
        self.perception_log = []
        self.perception_lock = threading.Lock()
        self.max_perception_log = 50

        self.get_logger().info('Autonomous Dashboard node started')

    # ── Subscribers ────────────────────────────────────────────────────────

    def _image_cb(self, msg: CompressedImage):
        with self.frame_lock:
            self.latest_frame = bytes(msg.data)
        self.frame_event.set()

    def _odom_cb(self, msg: Odometry):
        self.vel_linear = msg.twist.twist.linear.x
        self.vel_lateral = msg.twist.twist.linear.y
        self.vel_angular = msg.twist.twist.angular.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        # Update robot pixel on map if available
        self._update_robot_map_pos()

    def _lidar_cb(self, msg: LaserScan):
        with self.lidar_lock:
            self.lidar_ranges = list(msg.ranges)
            self.lidar_angle_min = msg.angle_min
            self.lidar_angle_increment = msg.angle_increment
            self.lidar_range_max = msg.range_max

    def _battery_cb(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.battery_pct = msg.percentage

    def _map_cb(self, msg: OccupancyGrid):
        """Convert OccupancyGrid to a PNG image for the web UI."""
        try:
            w = msg.info.width
            h = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))

            # Create RGB image
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[data == -1] = [40, 40, 60]      # Unknown = dark blue-grey
            img[data == 0] = [240, 240, 240]    # Free = light grey
            img[data > 0] = [30, 30, 30]        # Occupied = dark

            # Flip vertically (ROS map origin is bottom-left, image is top-left)
            img = np.flipud(img)

            # Scale up for visibility (each cell = 4px)
            scale = 4
            img_scaled = cv2.resize(
                img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST
            )

            # Encode to PNG base64
            _, buf = cv2.imencode('.png', img_scaled)
            b64 = base64.b64encode(buf.tobytes()).decode('utf-8')

            with self.map_lock:
                self.latest_map_image_b64 = b64
                self.map_info = {
                    'width': w,
                    'height': h,
                    'resolution': msg.info.resolution,
                    'origin_x': msg.info.origin.position.x,
                    'origin_y': msg.info.origin.position.y,
                    'scale': scale
                }
                self._update_robot_map_pos()

        except Exception as e:
            self.get_logger().error(f'Map render failed: {e}')

    def _update_robot_map_pos(self):
        """Convert robot odom pose to pixel position on the map image."""
        if self.map_info is None:
            return
        info = self.map_info
        scale = info['scale']
        res = info['resolution']
        ox = info['origin_x']
        oy = info['origin_y']
        h = info['height']

        # Convert map coordinates to pixel (accounting for vertical flip)
        px = int((self.odom_x - ox) / res * scale)
        py = int(h * scale - (self.odom_y - oy) / res * scale)
        self.map_robot_px = (px, py)

    def _safety_cb(self, msg: SafetyStatus):
        self.safety_status = {
            'is_safe': msg.is_safe,
            'closest_m': msg.closest_obstacle_m,
            'stop_active': msg.emergency_stop_active,
            'status_text': msg.status_text,
            'lidar_active': msg.lidar_active,
            'block_pos_x': msg.block_pos_x,
            'block_neg_x': msg.block_neg_x,
            'block_pos_y': msg.block_pos_y,
            'block_neg_y': msg.block_neg_y,
            'block_pos_yaw': msg.block_pos_yaw,
            'block_neg_yaw': msg.block_neg_yaw,
            'clearance_fwd': msg.clearance_forward_m,
            'clearance_rev': msg.clearance_reverse_m,
            'clearance_left': msg.clearance_left_m,
            'clearance_right': msg.clearance_right_m,
        }

    def set_rear_override(self, enabled: bool):
        self.rear_override_active = enabled
        msg = Bool()
        msg.data = enabled
        self.rear_override_pub.publish(msg)

    def _mission_cb(self, msg: MissionStatus):
        self.mission_status = {
            'state': msg.state,
            'target_room': msg.target_room,
            'status_text': msg.status_text,
            'progress': msg.progress,
            'rooms_discovered': msg.rooms_discovered,
            'known_rooms': list(msg.known_rooms),
            'goal_x': msg.goal_x,
            'goal_y': msg.goal_y,
            'frontiers_explored': msg.frontiers_explored
        }

    def _perception_cb(self, msg: Perception):
        entry = {
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'environment': msg.environment_type,
            'rooms': list(msg.detected_rooms),
            'has_obstacles': msg.has_obstacles,
            'description': msg.description,
            'image_id': msg.image_id
        }
        with self.perception_lock:
            self.perception_log.append(entry)
            if len(self.perception_log) > self.max_perception_log:
                self.perception_log.pop(0)

    # ── Teleop ─────────────────────────────────────────────────────────────

    def teleop(self, linear: float, lateral: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = float(lateral)
        msg.angular.z = float(angular)
        self.teleop_pub.publish(msg)

    def stop_teleop(self):
        self.teleop(0.0, 0.0, 0.0)

    def reset_pose(self):
        self.pose_offset_x = self.odom_x
        self.pose_offset_y = self.odom_y

    # ── MJPEG Streaming ────────────────────────────────────────────────────

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def mjpeg_generator(self):
        while True:
            self.frame_event.wait(timeout=2.0)
            self.frame_event.clear()
            frame = self.get_frame()
            if frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                )

    # ── Data Accessors ─────────────────────────────────────────────────────

    def get_status(self) -> dict:
        rel_x = self.odom_x - self.pose_offset_x
        rel_y = self.odom_y - self.pose_offset_y
        return {
            'x': round(rel_x, 3),
            'y': round(rel_y, 3),
            'yaw_deg': round(math.degrees(self.odom_yaw), 1),
            'vel_linear': round(self.vel_linear, 3),
            'vel_lateral': round(self.vel_lateral, 3),
            'vel_angular': round(self.vel_angular, 3),
            'battery_v': round(self.battery_voltage, 2),
            'battery_pct': round(self.battery_pct * 100, 1)
        }

    def get_lidar_data(self) -> dict | None:
        with self.lidar_lock:
            if not self.lidar_ranges:
                return None
            step = 3
            return {
                'ranges': self.lidar_ranges[::step],
                'angle_min': self.lidar_angle_min,
                'angle_increment': self.lidar_angle_increment * step,
                'range_max': self.lidar_range_max
            }

    def get_map_data(self) -> dict:
        with self.map_lock:
            return {
                'image_b64': self.latest_map_image_b64,
                'info': self.map_info,
                'robot_px': list(self.map_robot_px),
                'rooms': self.map_rooms
            }

    def get_perception_log(self, n: int = 20) -> list:
        with self.perception_lock:
            return list(self.perception_log[-n:])

    async def send_mission_command(self, command: str) -> dict:
        if not self.mission_client.wait_for_service(timeout_sec=2.0):
            return {'accepted': False, 'message': 'Mission orchestrator not available'}
        req = MissionCommand.Request()
        req.command = command
        future = self.mission_client.call_async(req)
        # Use a short busy-wait (acceptable in FastAPI async handler)
        for _ in range(50):
            if future.done():
                break
            await asyncio.sleep(0.05)
        if future.done():
            r = future.result()
            return {
                'accepted': r.accepted,
                'message': r.message,
                'parsed_intent': r.parsed_intent,
                'parsed_room': r.parsed_room
            }
        return {'accepted': False, 'message': 'Timeout waiting for orchestrator'}

    async def get_memory_locations(self) -> list:
        if not self.get_all_client.wait_for_service(timeout_sec=2.0):
            return []
        req = GetAllLocations.Request()
        req.include_start_position = True
        future = self.get_all_client.call_async(req)
        for _ in range(50):
            if future.done():
                break
            await asyncio.sleep(0.05)
        if not future.done():
            return []
        r = future.result()
        rooms = []
        for i, rid in enumerate(r.room_ids):
            rooms.append({
                'room_id': rid,
                'x': r.map_xs[i],
                'y': r.map_ys[i],
                'confidence': r.confidences[i],
                'last_seen': r.last_seen_times[i],
                'count': r.detection_counts[i]
            })
        return rooms


# ── FastAPI Application ────────────────────────────────────────────────────

import asyncio

ros_node: AutonomousDashboard = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros_node
    rclpy.init()
    ros_node = AutonomousDashboard()

    # Spin ROS2 in background thread
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield

    ros_node.destroy_node()
    rclpy.shutdown()


app = FastAPI(title='Ridgeback Autonomous Dashboard', lifespan=lifespan)


# ── Routes ─────────────────────────────────────────────────────────────────

@app.get('/video_feed')
def video_feed():
    return StreamingResponse(
        ros_node.mjpeg_generator(),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )


@app.get('/status')
def status():
    return JSONResponse(ros_node.get_status())


@app.get('/lidar')
def lidar():
    data = ros_node.get_lidar_data()
    return JSONResponse(data or {})


@app.get('/map')
def map_data():
    return JSONResponse(ros_node.get_map_data())


@app.get('/safety')
def safety():
    return JSONResponse(ros_node.safety_status)


@app.get('/mission_status')
def mission_status():
    return JSONResponse(ros_node.mission_status)


@app.get('/perception_log')
def perception_log():
    return JSONResponse(ros_node.get_perception_log(n=30))


@app.get('/memory')
async def memory():
    locations = await ros_node.get_memory_locations()
    return JSONResponse(locations)


@app.post('/teleop')
def teleop(cmd: TeleopCmd):
    ros_node.teleop(cmd.linear, cmd.lateral, cmd.angular)
    return {'ok': True}


@app.post('/stop')
def stop():
    ros_node.stop_teleop()
    return {'ok': True}


@app.post('/rear_override')
def rear_override(cmd: RearOverrideCmd):
    ros_node.set_rear_override(cmd.enabled)
    return {'ok': True, 'enabled': cmd.enabled}


@app.post('/reset_pose')
def reset_pose():
    ros_node.reset_pose()
    return {'ok': True}


@app.post('/mission')
async def mission(cmd: MissionCmd):
    result = await ros_node.send_mission_command(cmd.command)
    return JSONResponse(result)


@app.get('/', response_class=HTMLResponse)
def index():
    return HTMLResponse(DASHBOARD_HTML)


# ── HTML Dashboard ─────────────────────────────────────────────────────────

DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Ridgeback — Autonomous Dashboard</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: linear-gradient(135deg, #0d0f1a 0%, #1a1e2e 100%); color: #e0e0e0; font-family: 'Courier New', monospace; min-height: 100vh; }
.header { background: rgba(0,0,0,0.5); border-bottom: 2px solid #f7941d; padding: 10px 20px; display: flex; align-items: center; justify-content: space-between; }
.header h1 { color: #f7941d; font-size: 1.2rem; letter-spacing: 2px; }
.header .sub { color: #888; font-size: 0.75rem; }
.grid { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: auto auto; gap: 12px; padding: 12px; }
.panel { background: rgba(255,255,255,0.05); border: 1px solid rgba(255,255,255,0.1); border-radius: 8px; padding: 12px; }
.panel h2 { color: #f7941d; font-size: 0.85rem; letter-spacing: 1px; margin-bottom: 10px; border-bottom: 1px solid rgba(247,148,29,0.3); padding-bottom: 6px; }
/* Camera */
#camera-panel { grid-column: 1; grid-row: 1; }
#camera-feed { width: 100%; border-radius: 4px; display: block; }
/* Mission Control */
#mission-panel { grid-column: 2; grid-row: 1; }
.mission-input { display: flex; gap: 8px; margin-bottom: 10px; }
.mission-input input { flex: 1; background: rgba(255,255,255,0.1); border: 1px solid rgba(247,148,29,0.5); color: #e0e0e0; padding: 8px; border-radius: 4px; font-family: monospace; }
.mission-input input::placeholder { color: #666; }
.btn { padding: 8px 14px; border: none; border-radius: 4px; cursor: pointer; font-family: monospace; font-size: 0.8rem; transition: all 0.2s; }
.btn-primary { background: #f7941d; color: #000; }
.btn-primary:hover { background: #ffc107; }
.btn-stop { background: #ed1c24; color: #fff; }
.btn-stop:hover { background: #ff4444; }
.btn-secondary { background: rgba(255,255,255,0.15); color: #e0e0e0; }
.btn-secondary:hover { background: rgba(255,255,255,0.25); }
.btn-return { background: #4fc3f7; color: #000; }
.btn-return:hover { background: #81d4fa; }
.mission-state { padding: 8px; border-radius: 4px; margin-bottom: 8px; font-size: 0.85rem; }
.state-IDLE { background: rgba(100,100,100,0.3); border-left: 3px solid #888; }
.state-EXPLORING { background: rgba(255,193,7,0.2); border-left: 3px solid #ffc107; }
.state-NAVIGATING { background: rgba(33,150,243,0.2); border-left: 3px solid #2196f3; }
.state-ARRIVED { background: rgba(76,175,80,0.2); border-left: 3px solid #4caf50; }
.state-RETURNING { background: rgba(156,39,176,0.2); border-left: 3px solid #9c27b0; }
.state-RETURNED { background: rgba(76,175,80,0.2); border-left: 3px solid #4caf50; }
.state-FAILED { background: rgba(244,67,54,0.2); border-left: 3px solid #f44336; }
.state-CANCELLED { background: rgba(100,100,100,0.3); border-left: 3px solid #888; }
.progress-bar { height: 6px; background: rgba(255,255,255,0.1); border-radius: 3px; margin-top: 6px; }
.progress-fill { height: 100%; background: #f7941d; border-radius: 3px; transition: width 0.5s; }
.quick-btns { display: flex; gap: 6px; flex-wrap: wrap; margin-top: 8px; }
/* Map Panel */
#map-panel { grid-column: 3; grid-row: 1 / 3; }
#map-canvas { width: 100%; border-radius: 4px; border: 1px solid rgba(255,255,255,0.1); background: #1a1a2e; }
.map-legend { display: flex; gap: 12px; margin-top: 6px; font-size: 0.7rem; }
.legend-item { display: flex; align-items: center; gap: 4px; }
.legend-dot { width: 10px; height: 10px; border-radius: 50%; }
/* Teleop Panel */
#teleop-panel { grid-column: 1; grid-row: 2; }
.dpad { display: grid; grid-template-columns: repeat(3, 44px); grid-template-rows: repeat(3, 44px); gap: 4px; margin: 8px auto; width: fit-content; }
.dpad-btn { width: 44px; height: 44px; background: rgba(255,255,255,0.1); border: 1px solid rgba(255,255,255,0.2); border-radius: 6px; color: #e0e0e0; cursor: pointer; font-size: 1rem; display: flex; align-items: center; justify-content: center; transition: all 0.1s; user-select: none; }
.dpad-btn:hover { background: rgba(247,148,29,0.3); border-color: #f7941d; }
.dpad-btn.active { background: #f7941d; color: #000; border-color: #ffc107; }
.dpad-stop { background: rgba(237,28,36,0.3); border-color: #ed1c24; }
.speed-sliders { display: flex; gap: 10px; margin-top: 8px; font-size: 0.75rem; }
.speed-sliders label { display: flex; flex-direction: column; gap: 2px; }
.speed-sliders input[type=range] { width: 100px; }
.pose-row { display: flex; gap: 16px; margin-top: 8px; }
.pose-val { font-size: 0.75rem; }
.pose-val span { color: #4fc3f7; font-size: 0.9rem; }
/* VLM Log Panel */
#vlm-panel { grid-column: 2; grid-row: 2; }
#vlm-log { height: 260px; overflow-y: auto; font-size: 0.72rem; }
.vlm-entry { padding: 6px; border-bottom: 1px solid rgba(255,255,255,0.05); margin-bottom: 4px; }
.vlm-time { color: #888; }
.vlm-env { color: #4fc3f7; }
.vlm-room { background: rgba(247,148,29,0.3); border: 1px solid #f7941d; border-radius: 3px; padding: 1px 5px; color: #f7941d; font-weight: bold; }
.vlm-obstacle { color: #ed1c24; }
.vlm-desc { color: #aaa; margin-top: 2px; line-height: 1.3; }
/* Safety + Status bar */
.status-bar { grid-column: 1 / 4; display: flex; gap: 12px; padding: 8px 12px; background: rgba(0,0,0,0.3); border-top: 1px solid rgba(255,255,255,0.1); margin: 0 -12px -12px; border-radius: 0 0 8px 8px; font-size: 0.75rem; }
.safety-ok { color: #4caf50; }
.safety-warn { color: #ffc107; }
.safety-stop { color: #ed1c24; font-weight: bold; }
.stat { color: #888; }
.stat span { color: #4fc3f7; }
/* Directional block indicators */
.block-grid { display: grid; grid-template-columns: repeat(3, 28px); grid-template-rows: repeat(3, 28px); gap: 2px; margin: 6px auto; width: fit-content; }
.block-cell { width: 28px; height: 28px; border-radius: 4px; display: flex; align-items: center; justify-content: center; font-size: 0.65rem; font-weight: bold; }
.block-free { background: rgba(76,175,80,0.2); border: 1px solid #4caf50; color: #4caf50; }
.block-blocked { background: rgba(237,28,36,0.4); border: 1px solid #ed1c24; color: #ed1c24; }
.block-na { background: rgba(100,100,100,0.1); border: 1px solid rgba(255,255,255,0.1); color: #555; }
.btn-rear-override { font-size: 0.7rem; padding: 3px 8px; }
.btn-rear-override.active { background: #ed1c24; color: #fff; }
.clearance-row { display: flex; gap: 8px; font-size: 0.7rem; color: #888; flex-wrap: wrap; margin-top: 4px; }
.clearance-row span { color: #4fc3f7; }
/* LiDAR mini canvas in teleop panel */
#lidar-mini { display: block; margin: 8px auto 0; }
/* Memory panel (inside mission panel) */
#memory-list { max-height: 80px; overflow-y: auto; margin-top: 6px; font-size: 0.72rem; }
.memory-row { display: flex; justify-content: space-between; padding: 2px 0; border-bottom: 1px solid rgba(255,255,255,0.05); }
.memory-room { color: #f7941d; }
.memory-pos { color: #888; }
</style>
</head>
<body>
<div class="header">
  <div>
    <div class="header h1">RIDGEBACK R100 — AUTONOMOUS DASHBOARD</div>
    <div class="sub">r100-0140 | ROS2 Humble | SLAM + Nav2 + VLM</div>
  </div>
  <div id="header-safety" class="safety-ok">● SAFETY OK</div>
</div>

<div class="grid">
  <!-- Camera -->
  <div class="panel" id="camera-panel">
    <h2>CAMERA FEED</h2>
    <img id="camera-feed" src="/video_feed" alt="Camera Feed">
  </div>

  <!-- Mission Control -->
  <div class="panel" id="mission-panel">
    <h2>MISSION CONTROL</h2>
    <div class="mission-input">
      <input id="mission-input" type="text" placeholder='e.g. "go to room 305" or "come back"' />
      <button class="btn btn-primary" onclick="sendMission()">GO</button>
    </div>
    <div class="quick-btns">
      <button class="btn btn-return" onclick="sendMissionCmd('come back')">⟵ Return</button>
      <button class="btn btn-stop" onclick="sendMissionCmd('stop')">■ STOP</button>
      <button class="btn btn-secondary" onclick="sendMissionCmd('where am i')">? Status</button>
    </div>
    <div id="mission-state" class="mission-state state-IDLE" style="margin-top:10px">
      <div id="mission-state-label">IDLE</div>
      <div id="mission-status-text" style="font-size:0.75rem; color:#aaa; margin-top:3px">Ready</div>
      <div class="progress-bar"><div id="mission-progress" class="progress-fill" style="width:0%"></div></div>
    </div>
    <div id="mission-response" style="font-size:0.75rem; color:#4fc3f7; margin-top:6px; min-height:20px"></div>
    <!-- Memory -->
    <div style="margin-top:10px">
      <div style="display:flex; justify-content:space-between; align-items:center">
        <span style="font-size:0.75rem; color:#888">SPATIAL MEMORY</span>
        <button class="btn btn-secondary" style="padding:2px 8px; font-size:0.7rem" onclick="loadMemory()">⟳ Refresh</button>
      </div>
      <div id="memory-list"></div>
    </div>
  </div>

  <!-- SLAM Map -->
  <div class="panel" id="map-panel">
    <h2>SLAM MAP</h2>
    <canvas id="map-canvas" width="400" height="400"></canvas>
    <div class="map-legend">
      <div class="legend-item"><div class="legend-dot" style="background:#f0f0f0"></div> Free</div>
      <div class="legend-item"><div class="legend-dot" style="background:#28283c"></div> Unknown</div>
      <div class="legend-item"><div class="legend-dot" style="background:#1e1e1e"></div> Obstacle</div>
      <div class="legend-item"><div class="legend-dot" style="background:#f7941d"></div> Robot</div>
      <div class="legend-item"><div class="legend-dot" style="background:#4caf50"></div> Room</div>
    </div>
  </div>

  <!-- Teleop -->
  <div class="panel" id="teleop-panel">
    <h2>MANUAL CONTROL</h2>
    <div style="font-size:0.7rem; color:#888; margin-bottom:4px">WASD/Arrows=move Q/E=rotate Space=stop | Click dir=start, click again=stop</div>
    <div class="dpad">
      <button class="dpad-btn" id="btn-fl"  data-lin="1"  data-lat="1"  data-ang="0">↖</button>
      <button class="dpad-btn" id="btn-f"   data-lin="1"  data-lat="0"  data-ang="0">↑</button>
      <button class="dpad-btn" id="btn-fr"  data-lin="1"  data-lat="-1" data-ang="0">↗</button>
      <button class="dpad-btn" id="btn-sl"  data-lin="0"  data-lat="1"  data-ang="0">←</button>
      <button class="dpad-btn dpad-stop" id="btn-stop" onclick="stopTeleop()">■</button>
      <button class="dpad-btn" id="btn-sr"  data-lin="0"  data-lat="-1" data-ang="0">→</button>
      <button class="dpad-btn" id="btn-bl"  data-lin="-1" data-lat="1"  data-ang="0">↙</button>
      <button class="dpad-btn" id="btn-b"   data-lin="-1" data-lat="0"  data-ang="0">↓</button>
      <button class="dpad-btn" id="btn-br"  data-lin="-1" data-lat="-1" data-ang="0">↘</button>
    </div>
    <div style="display:flex; gap:8px; justify-content:center; margin-top:4px">
      <button class="dpad-btn" id="btn-ccw" data-lin="0" data-lat="0" data-ang="1">↺</button>
      <button class="dpad-btn" id="btn-cw"  data-lin="0" data-lat="0" data-ang="-1">↻</button>
    </div>
    <div class="speed-sliders">
      <label>Linear <span id="lbl-lin">0.20</span>m/s
        <input type="range" id="spd-linear" min="0.05" max="0.5" step="0.05" value="0.2"
               oninput="document.getElementById('lbl-lin').textContent=parseFloat(this.value).toFixed(2)">
      </label>
      <label>Angular <span id="lbl-ang">0.50</span>r/s
        <input type="range" id="spd-angular" min="0.1" max="1.5" step="0.1" value="0.5"
               oninput="document.getElementById('lbl-ang').textContent=parseFloat(this.value).toFixed(2)">
      </label>
    </div>
    <div class="pose-row">
      <div class="pose-val">X:<span id="val-x">0.00</span>m</div>
      <div class="pose-val">Y:<span id="val-y">0.00</span>m</div>
      <div class="pose-val">Yaw:<span id="val-yaw">0.0</span>°</div>
      <button class="btn btn-secondary" style="padding:2px 6px; font-size:0.7rem" onclick="resetPose()">⊙</button>
    </div>
    <canvas id="lidar-mini" width="200" height="200"></canvas>
    <!-- Directional block indicators -->
    <div style="margin-top:8px; font-size:0.7rem; color:#888">AXIS BLOCKS</div>
    <div class="block-grid">
      <div class="block-cell block-na" id="blk-fl">↖</div>
      <div class="block-cell block-free" id="blk-f">+X</div>
      <div class="block-cell block-na" id="blk-fr">↗</div>
      <div class="block-cell block-free" id="blk-l">+Y</div>
      <div class="block-cell block-na" style="font-size:0.5rem">BOT</div>
      <div class="block-cell block-free" id="blk-r">-Y</div>
      <div class="block-cell block-na" id="blk-rl">↙</div>
      <div class="block-cell block-free" id="blk-b">-X</div>
      <div class="block-cell block-na" id="blk-rr">↘</div>
    </div>
    <div style="display:flex; gap:6px; align-items:center; margin-top:4px">
      <div class="block-cell block-free" id="blk-ccw" style="width:auto; padding:0 6px">↺ CCW</div>
      <div class="block-cell block-free" id="blk-cw"  style="width:auto; padding:0 6px">↻ CW</div>
      <button class="btn btn-secondary btn-rear-override" id="btn-rear-override"
              onclick="toggleRearOverride()">REV UNLOCK</button>
    </div>
    <div class="clearance-row">
      Fwd:<span id="clr-fwd">--</span>m
      Rev:<span id="clr-rev">--</span>m
      L:<span id="clr-l">--</span>m
      R:<span id="clr-r">--</span>m
    </div>
  </div>

  <!-- VLM Perception Log -->
  <div class="panel" id="vlm-panel">
    <h2>VLM PERCEPTION LOG</h2>
    <div id="vlm-log"></div>
  </div>

</div>

<script>
// ── State ────────────────────────────────────────────────────────────────
let activeBtn = null;
let teleopInterval = null;
let keysDown = {};
let keyTeleopActive = false;

// ── Mission Control ───────────────────────────────────────────────────────
function sendMission() {
  const input = document.getElementById('mission-input');
  const cmd = input.value.trim();
  if (!cmd) return;
  sendMissionCmd(cmd);
  input.value = '';
}

document.getElementById('mission-input').addEventListener('keydown', e => {
  if (e.key === 'Enter') sendMission();
});

async function sendMissionCmd(cmd) {
  try {
    const r = await fetch('/mission', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command: cmd})
    });
    const data = await r.json();
    document.getElementById('mission-response').textContent =
      `→ ${data.message || (data.accepted ? 'OK' : 'Not accepted')}`;
  } catch(e) {
    document.getElementById('mission-response').textContent = `Error: ${e}`;
  }
}

// ── Teleop Buttons ────────────────────────────────────────────────────────
document.querySelectorAll('.dpad-btn[data-lin]').forEach(btn => {
  btn.addEventListener('click', () => toggleBtnTeleop(btn));
});

function toggleBtnTeleop(btn) {
  if (keyTeleopActive) return;
  if (activeBtn === btn) { stopTeleop(); return; }
  if (activeBtn) { activeBtn.classList.remove('active'); }
  activeBtn = btn;
  btn.classList.add('active');
  clearInterval(teleopInterval);
  sendTeleopFromBtn(btn);
  teleopInterval = setInterval(() => sendTeleopFromBtn(btn), 80);
}

function sendTeleopFromBtn(btn) {
  const lin = parseFloat(btn.dataset.lin);
  const lat = parseFloat(btn.dataset.lat);
  const ang = parseFloat(btn.dataset.ang);
  const linSpd = parseFloat(document.getElementById('spd-linear').value);
  const angSpd = parseFloat(document.getElementById('spd-angular').value);
  sendTeleop(lin * linSpd, lat * linSpd, ang * angSpd);
}

function stopTeleop() {
  if (keyTeleopActive) return;
  clearInterval(teleopInterval);
  if (activeBtn) { activeBtn.classList.remove('active'); activeBtn = null; }
  sendStop();
}

async function sendTeleop(lin, lat, ang) {
  fetch('/teleop', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({linear: lin, lateral: lat, angular: ang})
  }).catch(() => {});
}

async function sendStop() {
  fetch('/stop', {method: 'POST'}).catch(() => {});
}

async function resetPose() {
  fetch('/reset_pose', {method: 'POST'}).catch(() => {});
}

// ── Keyboard Teleop ───────────────────────────────────────────────────────
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if (keysDown[e.key]) return;
  keysDown[e.key] = true;
  if (['w','a','s','d','q','e','ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key)) {
    e.preventDefault();
    keyTeleopActive = true;
    sendKeyTeleop();
  }
});
document.addEventListener('keyup', e => {
  delete keysDown[e.key];
  if (!Object.keys(keysDown).some(k =>
    ['w','a','s','d','q','e','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(k))) {
    keyTeleopActive = false;
    sendStop();
  } else {
    sendKeyTeleop();
  }
});

function sendKeyTeleop() {
  const linSpd = parseFloat(document.getElementById('spd-linear').value);
  const angSpd = parseFloat(document.getElementById('spd-angular').value);
  let lin = 0, lat = 0, ang = 0;
  if (keysDown['w'] || keysDown['ArrowUp'])    lin  =  1;
  if (keysDown['s'] || keysDown['ArrowDown'])  lin  = -1;
  if (keysDown['a'] || keysDown['ArrowLeft'])  lat  =  1;
  if (keysDown['d'] || keysDown['ArrowRight']) lat  = -1;
  if (keysDown['q']) ang =  1;
  if (keysDown['e']) ang = -1;
  if (keysDown[' ']) { sendStop(); return; }
  sendTeleop(lin * linSpd, lat * linSpd, ang * angSpd);
}

// ── Status Polling ────────────────────────────────────────────────────────
async function updateStatus() {
  try {
    const [status, safety, mission] = await Promise.all([
      fetch('/status').then(r => r.json()),
      fetch('/safety').then(r => r.json()),
      fetch('/mission_status').then(r => r.json())
    ]);

    // Odometry
    document.getElementById('val-x').textContent = status.x.toFixed(2);
    document.getElementById('val-y').textContent = status.y.toFixed(2);
    document.getElementById('val-yaw').textContent = status.yaw_deg.toFixed(1);

    // Safety header
    const safetyEl = document.getElementById('header-safety');
    if (safety.stop_active) {
      safetyEl.className = 'safety-stop';
      safetyEl.textContent = `▲ SAFETY STOP: ${safety.status_text}`;
    } else if (!safety.lidar_active) {
      safetyEl.className = 'safety-warn';
      safetyEl.textContent = `⚠ LIDAR INACTIVE`;
    } else if (safety.closest_m < 0.8) {
      safetyEl.className = 'safety-warn';
      safetyEl.textContent = `⚠ OBSTACLE ${safety.closest_m.toFixed(2)}m`;
    } else {
      safetyEl.className = 'safety-ok';
      safetyEl.textContent = `● SAFETY OK  ${safety.closest_m > 0 ? safety.closest_m.toFixed(1)+'m' : ''}`;
    }

    // Per-axis block indicators
    function setBlock(id, blocked) {
      const el = document.getElementById(id);
      if (!el) return;
      el.className = el.className.replace(/block-(free|blocked)/g, blocked ? 'block-blocked' : 'block-free');
    }
    setBlock('blk-f',   safety.block_pos_x);
    setBlock('blk-b',   safety.block_neg_x);
    setBlock('blk-l',   safety.block_pos_y);
    setBlock('blk-r',   safety.block_neg_y);
    setBlock('blk-ccw', safety.block_pos_yaw);
    setBlock('blk-cw',  safety.block_neg_yaw);
    // Diagonal indicators: blocked if either contributing axis is blocked
    setBlock('blk-fl', safety.block_pos_x || safety.block_pos_y);
    setBlock('blk-fr', safety.block_pos_x || safety.block_neg_y);
    setBlock('blk-rl', safety.block_neg_x || safety.block_pos_y);
    setBlock('blk-rr', safety.block_neg_x || safety.block_neg_y);

    // Clearance display
    function fmtClr(v) { return (v >= 0) ? v.toFixed(2) : 'blind'; }
    const cFwd = document.getElementById('clr-fwd');
    const cRev = document.getElementById('clr-rev');
    const cL   = document.getElementById('clr-l');
    const cR   = document.getElementById('clr-r');
    if (cFwd) cFwd.textContent = fmtClr(safety.clearance_fwd ?? -1);
    if (cRev) cRev.textContent = fmtClr(safety.clearance_rev ?? -1);
    if (cL)   cL.textContent   = fmtClr(safety.clearance_left ?? -1);
    if (cR)   cR.textContent   = fmtClr(safety.clearance_right ?? -1);

    // Mission state
    const stateEl = document.getElementById('mission-state');
    stateEl.className = `mission-state state-${mission.state}`;
    document.getElementById('mission-state-label').textContent =
      `${mission.state}${mission.target_room ? ' → Room ' + mission.target_room : ''}`;
    document.getElementById('mission-status-text').textContent = mission.status_text;
    document.getElementById('mission-progress').style.width =
      (mission.progress * 100).toFixed(0) + '%';

  } catch(e) {}
}

// ── LiDAR Mini Canvas ────────────────────────────────────────────────────
async function updateLidar() {
  try {
    const data = await fetch('/lidar').then(r => r.json());
    if (!data.ranges) return;
    drawLidarMini(data);
  } catch(e) {}
}

function drawLidarMini(data) {
  const canvas = document.getElementById('lidar-mini');
  const ctx = canvas.getContext('2d');
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const maxPx = canvas.width / 2 - 4;
  const maxRange = data.range_max || 10;

  ctx.fillStyle = '#0d0f1a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Range rings
  ctx.strokeStyle = 'rgba(255,255,255,0.1)';
  ctx.lineWidth = 1;
  for (let r = 1; r <= 4; r++) {
    ctx.beginPath();
    ctx.arc(cx, cy, r / maxRange * maxPx, 0, 2 * Math.PI);
    ctx.stroke();
  }

  // Scan points
  data.ranges.forEach((range, i) => {
    if (!isFinite(range) || range <= 0.05) return;
    const angle = data.angle_min + i * data.angle_increment - Math.PI / 2;
    const px = cx + Math.cos(angle) * (range / maxRange * maxPx);
    const py = cy + Math.sin(angle) * (range / maxRange * maxPx);
    const t = Math.min(1, range / maxRange);
    ctx.fillStyle = `hsl(${120 * t}, 90%, 55%)`;
    ctx.fillRect(px - 1, py - 1, 2, 2);
  });

  // Robot dot
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
  ctx.fillStyle = '#f7941d';
  ctx.fill();
}

// ── SLAM Map Canvas ───────────────────────────────────────────────────────
async function updateMap() {
  try {
    const data = await fetch('/map').then(r => r.json());
    if (!data.image_b64) return;
    const canvas = document.getElementById('map-canvas');
    const ctx = canvas.getContext('2d');
    const img = new Image();
    img.onload = () => {
      canvas.width = img.width;
      canvas.height = img.height;
      ctx.drawImage(img, 0, 0);
      // Draw robot position
      const [rx, ry] = data.robot_px || [0, 0];
      ctx.beginPath();
      ctx.arc(rx, ry, 6, 0, 2 * Math.PI);
      ctx.fillStyle = '#f7941d';
      ctx.fill();
      ctx.strokeStyle = '#ffc107';
      ctx.lineWidth = 2;
      ctx.stroke();
      // Draw room markers
      (data.rooms || []).forEach(room => {
        ctx.beginPath();
        ctx.arc(room.px, room.py, 5, 0, 2 * Math.PI);
        ctx.fillStyle = '#4caf50';
        ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.font = '10px monospace';
        ctx.fillText(room.room_id, room.px + 8, room.py + 4);
      });
    };
    img.src = 'data:image/png;base64,' + data.image_b64;
  } catch(e) {}
}

// ── VLM Perception Log ────────────────────────────────────────────────────
let lastPerceptionCount = 0;

async function updatePerceptionLog() {
  try {
    const log = await fetch('/perception_log').then(r => r.json());
    if (log.length === lastPerceptionCount) return;
    lastPerceptionCount = log.length;

    const container = document.getElementById('vlm-log');
    const wasAtBottom = container.scrollTop + container.clientHeight >= container.scrollHeight - 20;

    container.innerHTML = [...log].reverse().map(e => {
      const roomTags = e.rooms.map(r => `<span class="vlm-room">ROOM ${r}</span>`).join(' ');
      const obstTag = e.has_obstacles ? '<span class="vlm-obstacle">⚠ OBSTACLES</span>' : '';
      return `<div class="vlm-entry">
        <span class="vlm-time">[${e.timestamp}]</span>
        <span class="vlm-env"> ${e.environment}</span>
        ${roomTags} ${obstTag}
        <div class="vlm-desc">${e.description}</div>
      </div>`;
    }).join('');

    if (wasAtBottom) container.scrollTop = 0;
  } catch(e) {}
}

// ── Memory ────────────────────────────────────────────────────────────────
async function loadMemory() {
  try {
    const rooms = await fetch('/memory').then(r => r.json());
    const el = document.getElementById('memory-list');
    if (!rooms.length) {
      el.innerHTML = '<div style="color:#666;font-size:0.72rem">No rooms in memory</div>';
      return;
    }
    el.innerHTML = rooms.map(r =>
      `<div class="memory-row">
        <span class="memory-room">${r.room_id}</span>
        <span class="memory-pos">(${r.x.toFixed(1)}, ${r.y.toFixed(1)}) ×${r.count}</span>
       </div>`
    ).join('');
  } catch(e) {}
}

// ── Rear Override Toggle ──────────────────────────────────────────────────
let rearOverrideEnabled = false;

async function toggleRearOverride() {
  rearOverrideEnabled = !rearOverrideEnabled;
  const btn = document.getElementById('btn-rear-override');
  btn.textContent = rearOverrideEnabled ? 'REV LOCKED' : 'REV UNLOCK';
  btn.classList.toggle('active', rearOverrideEnabled);
  try {
    await fetch('/rear_override', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({enabled: rearOverrideEnabled})
    });
  } catch(e) {}
}

// ── Polling Intervals ─────────────────────────────────────────────────────
setInterval(updateStatus, 1500);
setInterval(updateLidar, 250);
setInterval(updateMap, 2000);
setInterval(updatePerceptionLog, 1000);
setInterval(loadMemory, 5000);

// Initial loads
updateStatus();
updateLidar();
updateMap();
updatePerceptionLog();
loadMemory();
</script>
</body>
</html>
"""


def main(args=None):
    import sys
    print('=' * 55)
    print('Ridgeback Autonomy — Autonomous Web Dashboard')
    print('=' * 55)

    # Port from ROS2 param or default
    port = 8081
    host = '0.0.0.0'

    uvicorn.run(
        app,
        host=host,
        port=port,
        log_level='warning'
    )


if __name__ == '__main__':
    main()
