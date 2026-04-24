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
from sensor_msgs.msg import CompressedImage, Image, LaserScan, BatteryState
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool
from ridgeback_autonomy.msg import SafetyStatus, MissionStatus, Perception
from ridgeback_autonomy.srv import MissionCommand, QueryLocation, GetAllLocations
from ridgeback_image_motion.srv import Motion

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


class SoftBumperCmd(BaseModel):
    enabled: bool


class SafetyEnabledCmd(BaseModel):
    enabled: bool


class VlmPromptCmd(BaseModel):
    prompt: str


class MoveRequest(BaseModel):
    distance: float = 0.5
    speed: float = 0.2


class RotateRequest(BaseModel):
    angle: float = 90.0
    speed: float = 0.5


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
        self.depth_sub = self.create_subscription(
            Image, '/r100_0140/sensors/camera_0/depth/image',
            self._depth_image_cb, sensor_qos
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

        # Publishers — fallback if motion_service unavailable
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', cmd_qos)
        # Latched so safety_controller gets the current state even after restart
        self.rear_override_pub = self.create_publisher(Bool, '/safety/rear_override', latch_qos)
        self.soft_bumper_pub = self.create_publisher(Bool, '/safety/soft_bumper_enabled', latch_qos)
        self.safety_enabled_pub = self.create_publisher(Bool, '/safety/enabled', latch_qos)

        # Service clients
        self.motion_client = self.create_client(
            Motion, 'motion_service', callback_group=self.cb_group
        )
        self.mission_client = self.create_client(
            MissionCommand, 'mission/command', callback_group=self.cb_group
        )
        self.get_all_client = self.create_client(
            GetAllLocations, 'memory/get_all_locations', callback_group=self.cb_group
        )

        # ── State ──────────────────────────────────────────────────────────

        # Safety toggle — seed latched topic so nodes get the default on subscribe
        _safety_init = Bool()
        _safety_init.data = True
        self.safety_enabled_pub.publish(_safety_init)

        # Camera
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.last_frame_time = 0.0

        # Depth camera
        self.bridge = CvBridge()
        self.latest_depth_frame = None
        self.depth_frame_lock = threading.Lock()
        self.depth_frame_event = threading.Event()
        self.last_depth_frame_time = 0.0

        # Odometry
        self.last_odom_time = 0.0
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
            'soft_bumper_enabled': True,
            'safety_enabled': True,
        }
        self.rear_override_active = False
        self.soft_bumper_enabled = True
        self.safety_enabled = True

        # Mission
        self.mission_status = {
            'state': 'IDLE', 'target_room': '', 'status_text': 'Ready',
            'progress': 0.0, 'rooms_discovered': 0, 'known_rooms': []
        }

        # VLM perception log (recent entries from /vlm/perception)
        self.perception_log = []
        self.perception_lock = threading.Lock()
        self.max_perception_log = 50

        # Motion log + latency (ported from web_controller)
        self.log_buffer: list = []
        self.max_logs = 30
        self.image_latency_ms = 0.0
        self.motion_latency_ms = 0.0
        self.odom_latency_ms = 0.0

        # Move/rotate state
        self.is_moving = False
        self.motion_status = 'Ready'
        self.stop_motion = threading.Event()
        self.motion_thread = None
        self.max_linear_accel = 1.0
        self.max_angular_accel = 2.0

        self.get_logger().info('Autonomous Dashboard node started')

    # ── Subscribers ────────────────────────────────────────────────────────

    def _image_cb(self, msg: CompressedImage):
        try:
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.image_latency_ms = (now - stamp).nanoseconds / 1e6
        except Exception:
            pass
        with self.frame_lock:
            self.latest_frame = bytes(msg.data)
        self.last_frame_time = time.time()
        self.frame_event.set()

    def _depth_image_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # uint16 mm
            depth_clipped = np.clip(depth, 0, 5000).astype(np.float32)
            depth_norm = (depth_clipped / 5000.0 * 255).astype(np.uint8)
            colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
            colored[depth == 0] = [30, 30, 30]  # invalid pixels → dark grey
            ok, buf = cv2.imencode('.jpg', colored, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self.depth_frame_lock:
                    self.latest_depth_frame = buf.tobytes()
                self.last_depth_frame_time = time.time()
                self.depth_frame_event.set()
        except Exception as e:
            self.get_logger().error(f'Depth image error: {e}')

    def _odom_cb(self, msg: Odometry):
        self.last_odom_time = time.time()
        try:
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.odom_latency_ms = (now - stamp).nanoseconds / 1e6
        except Exception:
            pass
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
            'soft_bumper_enabled': msg.soft_bumper_enabled,
            'safety_enabled': msg.safety_enabled,
        }

    def set_rear_override(self, enabled: bool):
        self.rear_override_active = enabled
        msg = Bool()
        msg.data = enabled
        self.rear_override_pub.publish(msg)

    def set_soft_bumper(self, enabled: bool):
        self.soft_bumper_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.soft_bumper_pub.publish(msg)

    def set_safety_enabled(self, enabled: bool):
        self.safety_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.safety_enabled_pub.publish(msg)

    # ── Motion log ─────────────────────────────────────────────────────────

    def add_log(self, entry: str):
        ts = time.strftime('%H:%M:%S')
        self.log_buffer.append(f'[{ts}] {entry}')
        if len(self.log_buffer) > self.max_logs:
            self.log_buffer.pop(0)

    # ── Move / Rotate (closed-loop, ported from web_controller) ────────────

    def ramp_velocity(self, current, target, max_accel, dt):
        if abs(target - current) < 0.01:
            return target
        max_change = max_accel * dt
        return min(current + max_change, target) if target > current else max(current - max_change, target)

    def move_straight(self, distance: float, speed: float):
        if self.is_moving:
            self.stop_motion.set()
        self.stop_motion.clear()
        self.is_moving = True
        self.motion_status = f'Moving {distance:.2f}m at {speed:.2f}m/s'
        direction = 1.0 if distance >= 0 else -1.0
        initial_velocity = min(abs(speed), 0.15) * direction
        self.add_log(f'Starting move: {distance}m at {speed}m/s')
        self.teleop(initial_velocity, 0.0, 0.0)

        def _task():
            start_x, start_y = self.odom_x, self.odom_y
            target_dist = abs(distance)
            target_vel = abs(speed) * direction
            cur_vel = initial_velocity
            dt = 0.02
            while not self.stop_motion.is_set():
                traveled = math.sqrt((self.odom_x - start_x) ** 2 + (self.odom_y - start_y) ** 2)
                remaining = target_dist - traveled
                if traveled >= target_dist:
                    break
                decel_vel = max(0.05, remaining * 2) * direction
                tv = decel_vel if remaining < 0.1 and abs(decel_vel) < abs(target_vel) else target_vel
                cur_vel = self.ramp_velocity(cur_vel, tv, self.max_linear_accel, dt)
                self.teleop(cur_vel, 0.0, 0.0)
                self.motion_status = f'Moving {traveled:.2f}/{target_dist:.2f}m'
                time.sleep(dt)
            # Ramp to zero
            while abs(cur_vel) > 0.01 and not self.stop_motion.is_set():
                cur_vel = self.ramp_velocity(cur_vel, 0.0, self.max_linear_accel * 2, dt)
                self.teleop(cur_vel, 0.0, 0.0)
                time.sleep(dt)
            self.teleop(0.0, 0.0, 0.0)
            self.is_moving = False
            if not self.stop_motion.is_set():
                self.motion_status = f'Moved {target_dist:.2f}m'
                self.add_log(f'MOTION complete: moved {target_dist:.2f}m')

        self.motion_thread = threading.Thread(target=_task, daemon=True)
        self.motion_thread.start()
        return True

    def rotate(self, angle_deg: float, speed: float):
        if self.is_moving:
            self.stop_motion.set()
        self.stop_motion.clear()
        self.is_moving = True
        self.motion_status = f'Rotating {angle_deg:.1f}° at {speed:.2f}r/s'
        direction = 1.0 if angle_deg >= 0 else -1.0
        initial_velocity = min(abs(speed), 0.3) * direction
        self.add_log(f'Starting rotate: {angle_deg}° at {speed}r/s')
        self.teleop(0.0, 0.0, initial_velocity)

        def _task():
            target_angle = abs(math.radians(angle_deg))
            target_vel = abs(speed) * direction
            cur_vel = initial_velocity
            dt = 0.02
            total_rotated = 0.0
            last_yaw = self.odom_yaw
            while not self.stop_motion.is_set():
                delta = self.odom_yaw - last_yaw
                if delta > math.pi:
                    delta -= 2 * math.pi
                elif delta < -math.pi:
                    delta += 2 * math.pi
                total_rotated += abs(delta)
                last_yaw = self.odom_yaw
                remaining = target_angle - total_rotated
                if total_rotated >= target_angle:
                    break
                decel_vel = max(0.1, remaining * 2) * direction
                tv = decel_vel if remaining < 0.1 and abs(decel_vel) < abs(target_vel) else target_vel
                cur_vel = self.ramp_velocity(cur_vel, tv, self.max_angular_accel, dt)
                self.teleop(0.0, 0.0, cur_vel)
                self.motion_status = f'Rotating {math.degrees(total_rotated):.1f}/{abs(angle_deg):.1f}°'
                time.sleep(dt)
            while abs(cur_vel) > 0.01 and not self.stop_motion.is_set():
                cur_vel = self.ramp_velocity(cur_vel, 0.0, self.max_angular_accel * 2, dt)
                self.teleop(0.0, 0.0, cur_vel)
                time.sleep(dt)
            self.teleop(0.0, 0.0, 0.0)
            self.is_moving = False
            if not self.stop_motion.is_set():
                self.motion_status = f'Rotated {abs(angle_deg):.1f}°'
                self.add_log(f'MOTION complete: rotated {abs(angle_deg):.1f}°')

        self.motion_thread = threading.Thread(target=_task, daemon=True)
        self.motion_thread.start()
        return True

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
        is_stop = linear == 0.0 and lateral == 0.0 and angular == 0.0
        if self.motion_client.service_is_ready():
            req = Motion.Request()
            req.linear = float(linear)
            req.lateral = float(lateral)
            req.angular = float(angular)
            send_time = time.time()
            future = self.motion_client.call_async(req)
            future.add_done_callback(
                lambda f, t=send_time: setattr(self, 'motion_latency_ms', (time.time() - t) * 1000)
            )
        else:
            # Fallback: publish through cmd_vel_mux
            msg = Twist()
            msg.linear.x = float(linear)
            msg.linear.y = float(lateral)
            msg.angular.z = float(angular)
            self.teleop_pub.publish(msg)
        if is_stop:
            self.add_log('STOP')
        elif not self.is_moving:
            self.add_log(f'MOTION lin={linear:.2f} lat={lateral:.2f} ang={angular:.2f}')

    def stop_teleop(self):
        if self.is_moving:
            self.stop_motion.set()
            self.is_moving = False
        self.teleop(0.0, 0.0, 0.0)

    def reset_pose(self):
        self.pose_offset_x = self.odom_x
        self.pose_offset_y = self.odom_y

    # ── MJPEG Streaming ────────────────────────────────────────────────────

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def get_camera_status(self) -> dict:
        age = time.time() - self.last_frame_time if self.last_frame_time > 0 else -1.0
        return {
            'has_frame': self.last_frame_time > 0,
            'last_frame_age_s': round(age, 1) if age >= 0 else -1.0
        }

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

    def depth_mjpeg_generator(self):
        while True:
            self.depth_frame_event.wait(timeout=2.0)
            self.depth_frame_event.clear()
            with self.depth_frame_lock:
                frame = self.latest_depth_frame
            if frame:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'

    def get_depth_camera_status(self) -> dict:
        age = time.time() - self.last_depth_frame_time if self.last_depth_frame_time > 0 else -1.0
        return {
            'has_frame': self.last_depth_frame_time > 0,
            'last_frame_age_s': round(age, 1) if age >= 0 else -1.0
        }

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
            'battery_pct': round(self.battery_pct * 100, 1),
            'logs': self.log_buffer[-5:],
            'latency': {
                'image_ms': round(self.image_latency_ms, 1),
                'motion_ms': round(self.motion_latency_ms, 1),
                'odom_ms': round(self.odom_latency_ms, 1),
            },
            'motion_status': self.motion_status,
            'is_moving': self.is_moving,
            'safety': self.safety_status,
            'rear_override_active': self.rear_override_active,
            'soft_bumper_enabled': self.soft_bumper_enabled,
            'safety_enabled': self.safety_enabled,
            'camera_connected': (time.time() - self.last_frame_time < 5.0) if self.last_frame_time > 0 else False,
            'odom_connected': (time.time() - self.last_odom_time < 2.0) if self.last_odom_time > 0 else False,
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


@app.get('/camera_status')
def camera_status():
    return JSONResponse(ros_node.get_camera_status())


@app.get('/depth_feed')
def depth_feed():
    return StreamingResponse(
        ros_node.depth_mjpeg_generator(),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )


@app.get('/depth_status')
def depth_status():
    return JSONResponse(ros_node.get_depth_camera_status())


@app.post('/vlm_prompt')
async def vlm_prompt(cmd: VlmPromptCmd):
    result = await ros_node.send_mission_command(cmd.prompt)
    result['prompt'] = cmd.prompt
    result['timestamp'] = datetime.now().strftime('%H:%M:%S')
    return JSONResponse(result)


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


@app.post('/soft_bumper')
def soft_bumper(cmd: SoftBumperCmd):
    ros_node.set_soft_bumper(cmd.enabled)
    return {'ok': True, 'enabled': cmd.enabled}


@app.post('/safety_enabled')
def safety_enabled(cmd: SafetyEnabledCmd):
    ros_node.set_safety_enabled(cmd.enabled)
    return {'ok': True, 'enabled': cmd.enabled}


@app.post('/move')
def move(req: MoveRequest):
    success = ros_node.move_straight(req.distance, req.speed)
    return {'ok': success, 'status': ros_node.motion_status}


@app.post('/rotate')
def rotate_endpoint(req: RotateRequest):
    success = ros_node.rotate(req.angle, req.speed)
    return {'ok': success, 'status': ros_node.motion_status}


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
body { background: linear-gradient(135deg, #0d0f1a 0%, #1a1e2e 100%); color: #e0e0e0; font-family: 'Courier New', monospace; height: 100vh; overflow: hidden; display: flex; flex-direction: column; }
.header { background: rgba(0,0,0,0.5); border-bottom: 2px solid #f7941d; padding: 8px 20px; display: flex; align-items: center; justify-content: space-between; flex-shrink: 0; }
.header h1 { color: #f7941d; font-size: 1.1rem; letter-spacing: 2px; }
.header .sub { color: #888; font-size: 0.72rem; }
/* 3×2 grid */
.grid { flex: 1; min-height: 0; display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); grid-template-rows: minmax(0, 1fr) minmax(0, 1fr); gap: 8px; padding: 8px; overflow: hidden; }
.panel { background: rgba(255,255,255,0.05); border: 1px solid rgba(255,255,255,0.1); border-radius: 8px; padding: 10px; min-height: 0; display: flex; flex-direction: column; overflow: hidden; }
.panel h2 { color: #f7941d; font-size: 0.82rem; letter-spacing: 1px; margin-bottom: 8px; border-bottom: 1px solid rgba(247,148,29,0.3); padding-bottom: 5px; flex-shrink: 0; }
/* Panel placement */
#camera-panel    { grid-column: 1; grid-row: 1; position: relative; }
#depth-panel     { grid-column: 2; grid-row: 1; position: relative; }
#lidar-panel     { grid-column: 3; grid-row: 1; }
#status-panel    { grid-column: 1; grid-row: 2; overflow-y: auto; }
#vlm-panel       { grid-column: 2; grid-row: 2; }
#switchable-panel { grid-column: 3; grid-row: 2; display: flex; flex-direction: column; }
/* Camera / depth feeds */
#camera-feed, #depth-feed { width: 100%; flex: 1; min-height: 0; object-fit: contain; border-radius: 4px; display: block; max-height: calc(100% - 60px); }
.feed-overlay { display: none; position: absolute; top: 36px; left: 12px; right: 12px; background: rgba(237,28,36,0.85); color: #fff; font-size: 0.75rem; padding: 6px 10px; border-radius: 4px; border: 1px solid #ed1c24; text-align: center; z-index: 2; }
.depth-legend { display: flex; align-items: center; gap: 6px; font-size: 0.66rem; margin-top: 4px; flex-shrink: 0; }
.depth-gradient-bar { flex: 1; height: 6px; border-radius: 3px; background: linear-gradient(to right, #4fc3f7, #00ff7f, #ffff00, #ff5252); }
#depth-age { color: #666; }
/* LiDAR */
#lidar-canvas { display: block; width: 100%; aspect-ratio: 1 / 1; border-radius: 4px; border: 1px solid rgba(255,255,255,0.1); background: #0a0a0a; max-height: calc(100% - 56px); }
.lidar-info { display: flex; gap: 12px; margin-top: 6px; font-size: 0.72rem; color: #888; flex-shrink: 0; }
.lidar-info span { color: #4fc3f7; }
/* Buttons */
.btn { padding: 6px 12px; border: none; border-radius: 4px; cursor: pointer; font-family: monospace; font-size: 0.78rem; transition: all 0.2s; }
.btn-primary { background: #f7941d; color: #000; }
.btn-primary:hover { background: #ffc107; }
.btn-stop { background: #ed1c24; color: #fff; }
.btn-stop:hover { background: #ff4444; }
.btn-secondary { background: rgba(255,255,255,0.15); color: #e0e0e0; }
.btn-secondary:hover { background: rgba(255,255,255,0.25); }
.btn-xs { padding: 1px 5px; font-size: 0.65rem; }
/* Status panel (box 4) */
.status-section { border-bottom: 1px solid rgba(255,255,255,0.07); padding: 5px 0; }
.status-row { display: flex; align-items: center; gap: 5px; font-size: 0.70rem; flex-wrap: wrap; margin-bottom: 2px; }
.stat-label { color: #888; font-size: 0.63rem; text-transform: uppercase; }
.stat-val   { color: #4fc3f7; font-size: 0.75rem; }
.lat-val    { color: #f7941d; font-size: 0.72rem; }
.battery-bar-wrap { flex: 1; height: 8px; background: rgba(255,255,255,0.1); border-radius: 4px; overflow: hidden; }
.battery-bar { height: 100%; border-radius: 4px; transition: width 1s, background 1s; }
.safety-ok   { color: #4caf50; }
.safety-warn { color: #ffc107; }
.safety-stop { color: #ed1c24; font-weight: bold; }
.dir-blocks { display: flex; gap: 8px; margin-top: 4px; align-items: flex-start; }
.dir-grid { display: grid; grid-template-columns: repeat(3, 30px); grid-template-rows: repeat(3, 22px); gap: 3px; }
.clearance-grid { display: grid; grid-template-columns: auto auto; gap: 2px 6px; font-size: 0.65rem; align-content: start; }
.clr-label { color: #888; } .clr-val { color: #4fc3f7; }
.block-cell { border-radius: 3px; display: flex; align-items: center; justify-content: center; font-size: 0.55rem; font-weight: bold; }
.block-free    { background: rgba(76,175,80,0.2); border: 1px solid #4caf50; color: #4caf50; }
.block-blocked { background: rgba(237,28,36,0.4); border: 1px solid #ed1c24; color: #ed1c24; }
.motion-strip { overflow-x: auto; white-space: nowrap; font-size: 0.65rem; color: #9ad2ff; border: 1px solid rgba(79,195,247,0.3); border-radius: 4px; padding: 3px 5px; margin-top: 3px; }
/* VLM perception log (box 5) */
#vlm-panel { overflow: hidden; }
#vlm-log { flex: 1; min-height: 0; overflow-y: auto; font-size: 0.70rem; }
.vlm-entry { padding: 5px; border-bottom: 1px solid rgba(255,255,255,0.05); margin-bottom: 3px; }
.vlm-time { color: #888; }
.vlm-env { color: #4fc3f7; }
.vlm-room { background: rgba(247,148,29,0.3); border: 1px solid #f7941d; border-radius: 3px; padding: 1px 4px; color: #f7941d; font-weight: bold; }
.vlm-obstacle { color: #ed1c24; }
.vlm-desc { color: #aaa; margin-top: 2px; line-height: 1.3; }
/* Switchable panel (box 6) */
.panel-header-tabs { display: flex; align-items: center; gap: 6px; margin-bottom: 8px; border-bottom: 1px solid rgba(247,148,29,0.3); padding-bottom: 5px; flex-shrink: 0; }
.panel-header-tabs h2 { flex: 1; border: none; margin: 0; padding: 0; }
.tab-btn { padding: 3px 9px; font-family: monospace; font-size: 0.68rem; background: rgba(255,255,255,0.08); border: 1px solid rgba(255,255,255,0.2); border-radius: 4px; color: #888; cursor: pointer; transition: all 0.15s; }
.tab-btn:hover { background: rgba(247,148,29,0.2); color: #f7941d; }
.tab-btn.tab-active { background: rgba(247,148,29,0.25); border-color: #f7941d; color: #f7941d; }
#pane-teleop, #pane-vlmprompt { flex: 1; min-height: 0; display: flex; flex-direction: column; overflow: hidden; }
.kbd-hint { font-size: 0.65rem; color: #888; margin-bottom: 5px; flex-shrink: 0; }
/* D-pad */
.dpad { display: grid; grid-template-columns: repeat(3, clamp(40px, 5.5vh, 56px)); grid-template-rows: repeat(3, clamp(40px, 5.5vh, 56px)); gap: 4px; margin: 4px auto; width: fit-content; flex-shrink: 0; }
.dpad-btn { width: clamp(40px, 5.5vh, 56px); height: clamp(40px, 5.5vh, 56px); background: linear-gradient(160deg, rgba(247,148,29,0.18), rgba(255,255,255,0.08)); border: 1px solid rgba(255,255,255,0.25); border-radius: 10px; color: #e0e0e0; cursor: pointer; font-size: 1.1rem; display: flex; align-items: center; justify-content: center; transition: all 0.1s; user-select: none; }
.dpad-btn:hover { background: rgba(247,148,29,0.3); border-color: #f7941d; }
.dpad-btn.active { background: #f7941d; color: #000; border-color: #ffc107; }
.dpad-stop { background: rgba(237,28,36,0.3); border-color: #ed1c24; }
.speed-sliders { display: flex; gap: 10px; margin-top: 6px; font-size: 0.72rem; flex-shrink: 0; }
.speed-sliders label { display: flex; flex-direction: column; gap: 2px; }
.speed-sliders input[type=range] { width: 90px; }
.toggle-row { display: flex; gap: 4px; flex-wrap: wrap; margin-top: 5px; flex-shrink: 0; }
.toggle-sm { font-size: 0.63rem; padding: 3px 6px; }
.btn-rear-override.active { background: #ed1c24; color: #fff; border-color: #ed1c24; }
/* VLM prompt pane */
.vlm-chat-history { flex: 1; min-height: 0; overflow-y: auto; font-size: 0.70rem; border: 1px solid rgba(255,255,255,0.08); border-radius: 4px; padding: 6px; margin-bottom: 7px; background: rgba(0,0,0,0.2); }
.chat-user  { color: #f7941d; margin-bottom: 2px; }
.chat-robot { color: #4fc3f7; margin-bottom: 7px; }
.chat-ts    { color: #666; font-size: 0.63rem; }
.vlm-prompt-row { display: flex; gap: 6px; flex-shrink: 0; }
.vlm-prompt-row input { flex: 1; background: rgba(255,255,255,0.1); border: 1px solid rgba(247,148,29,0.5); color: #e0e0e0; padding: 6px; border-radius: 4px; font-family: monospace; font-size: 0.76rem; }
.vlm-prompt-row input::placeholder { color: #555; }
/* Responsive collapse */
@media (max-width: 900px) {
  body { overflow: auto; height: auto; }
  .grid { grid-template-columns: 1fr; grid-template-rows: auto; overflow: visible; }
  #camera-panel, #depth-panel, #lidar-panel, #status-panel, #vlm-panel, #switchable-panel { grid-column: 1; grid-row: auto; min-height: 300px; }
  #camera-feed, #depth-feed { max-height: 220px; }
  #lidar-canvas { max-height: 220px; }
}
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

  <!-- Box 1: RGB Camera -->
  <div class="panel" id="camera-panel">
    <h2>RGB CAMERA</h2>
    <img id="camera-feed" src="/video_feed" alt="Camera Feed">
    <div id="camera-overlay" class="feed-overlay">NO CAMERA SIGNAL</div>
  </div>

  <!-- Box 2: Depth Camera -->
  <div class="panel" id="depth-panel">
    <h2>DEPTH CAMERA</h2>
    <img id="depth-feed" src="/depth_feed" alt="Depth Feed">
    <div id="depth-overlay" class="feed-overlay">NO DEPTH SIGNAL</div>
    <div class="depth-legend">
      <span style="color:#4fc3f7">NEAR</span>
      <div class="depth-gradient-bar"></div>
      <span style="color:#ff5252">FAR (5m)</span>
      <span id="depth-age"></span>
    </div>
  </div>

  <!-- Box 3: LiDAR Scan -->
  <div class="panel" id="lidar-panel">
    <h2>LIDAR SCAN</h2>
    <canvas id="lidar-canvas" width="600" height="600"></canvas>
    <div class="lidar-info">
      Closest: <span id="lidar-closest">--</span>m &nbsp; Points: <span id="lidar-points">--</span>
    </div>
  </div>

  <!-- Box 4: System Status -->
  <div class="panel" id="status-panel">
    <h2>SYSTEM STATUS</h2>
    <div class="status-section">
      <div class="status-row">
        <span class="stat-label">BATTERY</span>
        <div class="battery-bar-wrap"><div id="batt-bar" class="battery-bar" style="width:0%;background:#4caf50"></div></div>
        <span id="stat-batt-pct" class="stat-val">--%</span>
        <span id="stat-batt-v" class="stat-val">--V</span>
      </div>
    </div>
    <div class="status-section">
      <div class="status-row">
        X:<span id="val-x" class="stat-val">--</span>m
        Y:<span id="val-y" class="stat-val">--</span>m
        YAW:<span id="val-yaw" class="stat-val">--</span>°
        <button class="btn btn-secondary btn-xs" onclick="resetPose()">⊙RST</button>
      </div>
      <div class="status-row">
        Vlin:<span id="val-vlin" class="stat-val">--</span>
        Vlat:<span id="val-vlat" class="stat-val">--</span>
        Vang:<span id="val-vang" class="stat-val">--</span>
      </div>
    </div>
    <div class="status-section">
      <div class="status-row">
        <span class="stat-label">LATENCY</span>
        IMG<span id="lat-image" class="lat-val">--</span>
        MOT<span id="lat-motion" class="lat-val">--</span>
        ODO<span id="lat-odom" class="lat-val">--</span>
      </div>
    </div>
    <div class="status-section">
      <div id="strip-safety" class="status-row safety-ok">● SAFETY OK</div>
      <div class="dir-blocks">
        <div class="dir-grid">
          <div></div>
          <div class="block-cell block-free" id="blk-f">FWD</div>
          <div></div>
          <div class="block-cell block-free" id="blk-l">L</div>
          <div class="block-cell block-free" id="blk-ccw">CCW</div>
          <div class="block-cell block-free" id="blk-r">R</div>
          <div></div>
          <div class="block-cell block-free" id="blk-b">REV</div>
          <div></div>
        </div>
        <div class="clearance-grid">
          <span class="clr-label">FWD</span><span id="clr-fwd" class="clr-val">--</span>
          <span class="clr-label">REV</span><span id="clr-rev" class="clr-val">--</span>
          <span class="clr-label">L</span>  <span id="clr-l"   class="clr-val">--</span>
          <span class="clr-label">R</span>  <span id="clr-r"   class="clr-val">--</span>
        </div>
      </div>
    </div>
    <div class="status-section">
      <div class="status-row">
        <span class="stat-label">ESTOP</span><span id="val-stop-loop" class="stat-val">--</span>
        <span class="stat-label">CAM</span><span id="conn-cam" class="stat-val">--</span>
        <span class="stat-label">ODOM</span><span id="conn-odom" class="stat-val">--</span>
      </div>
      <div id="motion-log" class="motion-strip">--</div>
    </div>
  </div>

  <!-- Box 5: VLM Perception Log -->
  <div class="panel" id="vlm-panel">
    <h2>VLM PERCEPTION LOG</h2>
    <div id="vlm-log"></div>
  </div>

  <!-- Box 6: Switchable Panel -->
  <div class="panel" id="switchable-panel">
    <div class="panel-header-tabs">
      <h2>CONTROL</h2>
      <button id="tab-teleop"    class="tab-btn tab-active" onclick="switchTab('teleop')">TELEOP</button>
      <button id="tab-vlmprompt" class="tab-btn"            onclick="switchTab('vlmprompt')">VLM PROMPT</button>
    </div>

    <!-- Mode A: Teleop -->
    <div id="pane-teleop">
      <div class="kbd-hint">WASD / Arrows = move &nbsp; Q/E = rotate &nbsp; Space = stop</div>
      <div class="dpad">
        <button class="dpad-btn" id="btn-fl"  data-lin="1"  data-lat="1"  data-ang="0">↖</button>
        <button class="dpad-btn" id="btn-f"   data-lin="1"  data-lat="0"  data-ang="0">↑</button>
        <button class="dpad-btn" id="btn-fr"  data-lin="1"  data-lat="-1" data-ang="0">↗</button>
        <button class="dpad-btn" id="btn-sl"  data-lin="0"  data-lat="1"  data-ang="0">←</button>
        <button class="dpad-btn dpad-stop" id="btn-stop" onclick="stopRobot()">■</button>
        <button class="dpad-btn" id="btn-sr"  data-lin="0"  data-lat="-1" data-ang="0">→</button>
        <button class="dpad-btn" id="btn-bl"  data-lin="-1" data-lat="1"  data-ang="0">↙</button>
        <button class="dpad-btn" id="btn-b"   data-lin="-1" data-lat="0"  data-ang="0">↓</button>
        <button class="dpad-btn" id="btn-br"  data-lin="-1" data-lat="-1" data-ang="0">↘</button>
      </div>
      <div style="display:flex;gap:8px;justify-content:center;margin-top:4px">
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
      <div class="toggle-row">
        <button class="btn btn-secondary toggle-sm" id="btn-safety-enabled" onclick="toggleSafetyEnabled()">SAFETY: ON</button>
        <button class="btn btn-secondary toggle-sm" id="btn-soft-bumper"    onclick="toggleSoftBumper()">SOFT BUMPER: ON</button>
        <button class="btn btn-secondary toggle-sm btn-rear-override" id="btn-rear-override" onclick="toggleRearOverride()">REV UNLOCK</button>
      </div>
      <button class="btn btn-stop" style="width:100%;margin-top:6px" onclick="stopRobot()">■ STOP</button>
    </div>

    <!-- Mode B: VLM Prompt -->
    <div id="pane-vlmprompt" style="display:none">
      <div id="vlm-chat" class="vlm-chat-history"></div>
      <div class="vlm-prompt-row">
        <input id="vlm-prompt-input" type="text" placeholder='"what do you see?" or "go to room 202"'>
        <button class="btn btn-primary" onclick="sendVlmPrompt()">SEND</button>
      </div>
    </div>
  </div>

</div>

<script>
// ── State ────────────────────────────────────────────────────────────────
let activeBtn = null;
let teleopInterval = null;
let keyboardInterval = null;
const keyState = {};
let safetyEnabled = true;
let softBumperEnabled = true;
let rearOverrideEnabled = false;
const vlmChatHistory = [];

// ── Tab Switch ────────────────────────────────────────────────────────────
function switchTab(tab) {
  ['teleop', 'vlmprompt'].forEach(p => {
    const pane = document.getElementById('pane-' + p);
    const btn  = document.getElementById('tab-'  + p);
    if (pane) pane.style.display = (p === tab) ? 'flex' : 'none';
    if (btn)  btn.classList.toggle('tab-active', p === tab);
  });
}

// ── Teleop Buttons ────────────────────────────────────────────────────────
document.querySelectorAll('.dpad-btn[data-lin]').forEach(btn => {
  btn.addEventListener('click', () => toggleBtnTeleop(btn));
});

function toggleBtnTeleop(btn) {
  if (keyboardInterval) return;
  if (activeBtn === btn) { stopRobot(); return; }
  if (activeBtn) { activeBtn.classList.remove('active'); }
  activeBtn = btn;
  btn.classList.add('active');
  clearInterval(teleopInterval);
  sendTeleopFromBtn(btn);
  teleopInterval = setInterval(() => sendTeleopFromBtn(btn), 50);
}

function sendTeleopFromBtn(btn) {
  const lin = parseFloat(btn.dataset.lin);
  const lat = parseFloat(btn.dataset.lat);
  const ang = parseFloat(btn.dataset.ang);
  const linSpd = parseFloat(document.getElementById('spd-linear').value);
  const angSpd = parseFloat(document.getElementById('spd-angular').value);
  sendTeleop(lin * linSpd, lat * linSpd, ang * angSpd);
}

function stopRobot() {
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

// ── Keyboard Teleop — 20 Hz repeat while keys held ────────────────────────
const MOVE_KEYS = ['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright'];

function updateFromKeyboard() {
  const linSpd = parseFloat(document.getElementById('spd-linear').value);
  const angSpd = parseFloat(document.getElementById('spd-angular').value);
  let lin = 0, lat = 0, ang = 0;
  if (keyState['w'] || keyState['arrowup'])    lin  =  1;
  if (keyState['s'] || keyState['arrowdown'])  lin  = -1;
  if (keyState['a'] || keyState['arrowleft'])  lat  =  1;
  if (keyState['d'] || keyState['arrowright']) lat  = -1;
  if (keyState['q']) ang =  1;
  if (keyState['e']) ang = -1;
  sendTeleop(lin * linSpd, lat * linSpd, ang * angSpd);
}

document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  const key = e.key.toLowerCase();
  if (key === ' ') { e.preventDefault(); sendStop(); return; }
  if (!MOVE_KEYS.includes(key)) return;
  e.preventDefault();
  if (keyState[key]) return;
  keyState[key] = true;
  if (teleopInterval) { clearInterval(teleopInterval); teleopInterval = null; }
  if (activeBtn) { activeBtn.classList.remove('active'); activeBtn = null; }
  if (!keyboardInterval) {
    updateFromKeyboard();
    keyboardInterval = setInterval(updateFromKeyboard, 50);
  }
});

document.addEventListener('keyup', e => {
  const key = e.key.toLowerCase();
  delete keyState[key];
  if (Object.keys(keyState).length === 0 && keyboardInterval) {
    clearInterval(keyboardInterval);
    keyboardInterval = null;
    sendTeleop(0, 0, 0);
  }
});

// ── Status Polling ────────────────────────────────────────────────────────
function setBlock(id, blocked) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = el.className.replace(/block-(free|blocked)/g, blocked ? 'block-blocked' : 'block-free');
}

function syncToggleButton(id, on, labelOn, labelOff) {
  const btn = document.getElementById(id);
  if (!btn) return;
  btn.textContent    = on ? labelOn : labelOff;
  btn.style.background   = on ? 'rgba(76,175,80,0.3)'  : 'rgba(237,28,36,0.3)';
  btn.style.borderColor  = on ? '#4caf50' : '#ed1c24';
  btn.style.color        = on ? '#4caf50' : '#ed1c24';
}

async function updateStatus() {
  try {
    const [status, camStatus, depthStatus] = await Promise.all([
      fetch('/status').then(r => r.json()),
      fetch('/camera_status').then(r => r.json()),
      fetch('/depth_status').then(r => r.json())
    ]);
    const safety = status.safety || {};

    // Battery
    const pct = status.battery_pct ?? 0;
    const battBar = document.getElementById('batt-bar');
    if (battBar) {
      battBar.style.width = pct.toFixed(0) + '%';
      battBar.style.background = pct < 20 ? '#ed1c24' : pct < 40 ? '#ffc107' : '#4caf50';
    }
    const e = id => document.getElementById(id);
    if (e('stat-batt-pct')) e('stat-batt-pct').textContent = pct.toFixed(1) + '%';
    if (e('stat-batt-v'))   e('stat-batt-v').textContent   = (status.battery_v ?? 0).toFixed(1) + 'V';

    // Pose
    if (e('val-x'))    e('val-x').textContent    = (status.x   ?? 0).toFixed(2);
    if (e('val-y'))    e('val-y').textContent     = (status.y   ?? 0).toFixed(2);
    if (e('val-yaw'))  e('val-yaw').textContent   = (status.yaw_deg ?? 0).toFixed(1);
    if (e('val-vlin')) e('val-vlin').textContent  = (status.vel_linear  ?? 0).toFixed(2);
    if (e('val-vlat')) e('val-vlat').textContent  = (status.vel_lateral ?? 0).toFixed(2);
    if (e('val-vang')) e('val-vang').textContent  = (status.vel_angular ?? 0).toFixed(2);

    // Latency
    if (e('lat-image'))  e('lat-image').textContent  = (status.latency?.image_ms  ?? 0).toFixed(0) + 'ms';
    if (e('lat-motion')) e('lat-motion').textContent = (status.latency?.motion_ms ?? 0).toFixed(0) + 'ms';
    if (e('lat-odom'))   e('lat-odom').textContent   = (status.latency?.odom_ms   ?? 0).toFixed(0) + 'ms';

    // Safety header + strip
    const safetyEl    = document.getElementById('header-safety');
    const stripEl     = document.getElementById('strip-safety');
    let safeText, safeCls;
    if (safety.stop_active) {
      safeText = `▲ STOP: ${safety.status_text}`; safeCls = 'safety-stop';
    } else if (!safety.lidar_active) {
      safeText = '⚠ LIDAR INACTIVE'; safeCls = 'safety-warn';
    } else if (safety.closest_m < 0.8) {
      safeText = `⚠ OBSTACLE ${(safety.closest_m ?? 0).toFixed(2)}m`; safeCls = 'safety-warn';
    } else {
      safeText = `● SAFETY OK ${safety.closest_m > 0 ? safety.closest_m.toFixed(1) + 'm' : ''}`; safeCls = 'safety-ok';
    }
    [safetyEl, stripEl].forEach(el => { if (el) { el.className = safeCls + (el === stripEl ? ' status-row' : ''); el.textContent = safeText; }});

    // Directional blocks
    setBlock('blk-f',   safety.block_pos_x);
    setBlock('blk-b',   safety.block_neg_x);
    setBlock('blk-l',   safety.block_pos_y);
    setBlock('blk-r',   safety.block_neg_y);
    setBlock('blk-ccw', safety.block_pos_yaw);
    setBlock('blk-cw',  safety.block_neg_yaw);

    // Clearances
    function fmtClr(v) { return (v >= 0) ? v.toFixed(2) + 'm' : 'blind'; }
    if (e('clr-fwd')) e('clr-fwd').textContent = fmtClr(safety.clearance_fwd   ?? -1);
    if (e('clr-rev')) e('clr-rev').textContent = fmtClr(safety.clearance_rev   ?? -1);
    if (e('clr-l'))   e('clr-l').textContent   = fmtClr(safety.clearance_left  ?? -1);
    if (e('clr-r'))   e('clr-r').textContent   = fmtClr(safety.clearance_right ?? -1);

    // E-Stop
    if (e('val-stop-loop')) {
      const active = safety.stop_active ?? false;
      e('val-stop-loop').textContent = active ? 'ACTIVE' : 'CLEAR';
      e('val-stop-loop').style.color = active ? '#ed1c24' : '#4caf50';
    }

    // Connection
    if (e('conn-cam')) {
      e('conn-cam').textContent = status.camera_connected ? 'OK' : 'LOST';
      e('conn-cam').style.color = status.camera_connected ? '#4caf50' : '#ed1c24';
    }
    if (e('conn-odom')) {
      e('conn-odom').textContent = status.odom_connected ? 'OK' : 'LOST';
      e('conn-odom').style.color = status.odom_connected ? '#4caf50' : '#ed1c24';
    }

    // Motion log
    if (e('motion-log')) {
      const parts = status.logs && status.logs.length ? status.logs : [status.motion_status || 'Ready'];
      e('motion-log').textContent = parts.join(' | ');
    }

    // Toggle buttons
    softBumperEnabled = safety.soft_bumper_enabled ?? softBumperEnabled;
    safetyEnabled     = safety.safety_enabled      ?? safetyEnabled;
    syncToggleButton('btn-soft-bumper',    softBumperEnabled, 'SOFT BUMPER: ON', 'SOFT BUMPER: OFF');
    syncToggleButton('btn-safety-enabled', safetyEnabled,     'SAFETY: ON',      'SAFETY: OFF');

    // Camera feed overlays
    const camOverlay   = document.getElementById('camera-overlay');
    const depthOverlay = document.getElementById('depth-overlay');
    if (camOverlay)   camOverlay.style.display   = (!camStatus.has_frame   || camStatus.last_frame_age_s   > 3.0) ? 'block' : 'none';
    if (depthOverlay) depthOverlay.style.display = (!depthStatus.has_frame || depthStatus.last_frame_age_s > 3.0) ? 'block' : 'none';
    if (e('depth-age') && depthStatus.last_frame_age_s >= 0)
      e('depth-age').textContent = depthStatus.last_frame_age_s.toFixed(1) + 's ago';

  } catch(err) {}
}

// ── LiDAR Canvas ─────────────────────────────────────────────────────────
const LIDAR_MAX_RANGE = 4.0;

function syncCanvasToDisplaySize(canvas) {
  const rect = canvas.getBoundingClientRect();
  const size = Math.max(200, Math.floor(Math.min(rect.width, rect.height || rect.width)));
  if (canvas.width !== size || canvas.height !== size) {
    canvas.width = size;
    canvas.height = size;
  }
}

async function updateLidar() {
  try {
    const data = await fetch('/lidar').then(r => r.json());
    if (!data.ranges) return;
    drawLidar(data);
  } catch(e) {}
}

function drawLidar(data) {
  const canvas = document.getElementById('lidar-canvas');
  if (!canvas) return;
  syncCanvasToDisplaySize(canvas);
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  const cx = w / 2, cy = h / 2;
  const rangeMax = Math.min(data.range_max || 10, LIDAR_MAX_RANGE);
  const lidarScale = Math.min(w, h) / (2 * (rangeMax + 0.25));

  ctx.fillStyle = '#0a0a0a';
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = '#1a2a1a';
  ctx.lineWidth = 1;
  ctx.font = '11px monospace';
  ctx.fillStyle = '#2a4a2a';
  for (let r = 1; r <= rangeMax; r++) {
    const pr = r * lidarScale;
    ctx.beginPath(); ctx.arc(cx, cy, pr, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(r + 'm', cx + pr + 3, cy - 3);
  }

  ctx.strokeStyle = '#1a3a1a';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cx, cy - rangeMax * lidarScale - 10); ctx.lineTo(cx, cy + rangeMax * lidarScale + 10); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx - rangeMax * lidarScale - 10, cy); ctx.lineTo(cx + rangeMax * lidarScale + 10, cy); ctx.stroke();

  ctx.fillStyle = '#2a6a2a';
  ctx.font = '10px monospace';
  ctx.fillText('FWD', cx + 4, cy - rangeMax * lidarScale - 2);

  let validCount = 0, closestRange = Infinity;
  const angleMin = data.angle_min, angleInc = data.angle_increment;

  for (let i = 0; i < data.ranges.length; i++) {
    const range = data.ranges[i];
    if (!isFinite(range) || range < 0.05 || range > rangeMax) continue;
    validCount++;
    if (range < closestRange) closestRange = range;
    const angle = angleMin + i * angleInc;
    const px = cx - range * Math.sin(angle) * lidarScale;
    const py = cy - range * Math.cos(angle) * lidarScale;
    const t = Math.min(range / rangeMax, 1.0);
    let r, g, b;
    if (t < 0.33) { r = 255; g = Math.floor(t * 3 * 255); b = 0; }
    else if (t < 0.66) { r = Math.floor((1 - (t - 0.33) * 3) * 255); g = 255; b = 0; }
    else { r = 0; g = 255; b = Math.floor((t - 0.66) * 3 * 255); }
    ctx.fillStyle = `rgb(${r},${g},${b})`;
    ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2); ctx.fill();
  }

  ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI * 2);
  ctx.fillStyle = '#f7941d'; ctx.fill();
  ctx.strokeStyle = '#ffc107'; ctx.lineWidth = 2; ctx.stroke();

  ctx.beginPath(); ctx.moveTo(cx, cy - 26); ctx.lineTo(cx - 7, cy - 14); ctx.lineTo(cx + 7, cy - 14);
  ctx.closePath(); ctx.fillStyle = '#ffc107'; ctx.fill();

  const elClosest = document.getElementById('lidar-closest');
  const elPoints  = document.getElementById('lidar-points');
  if (elClosest) elClosest.textContent = isFinite(closestRange) ? closestRange.toFixed(2) : '--';
  if (elPoints)  elPoints.textContent  = validCount;
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
      const obstTag  = e.has_obstacles ? '<span class="vlm-obstacle">⚠ OBSTACLES</span>' : '';
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

// ── VLM Prompt (Box 6 Mode B) ─────────────────────────────────────────────
async function sendVlmPrompt() {
  const input = document.getElementById('vlm-prompt-input');
  const prompt = input.value.trim();
  if (!prompt) return;
  input.value = '';
  const ts = new Date().toLocaleTimeString('en-US', {hour12: false});
  vlmChatHistory.push({role: 'user', text: prompt, ts});
  renderVlmChat();
  try {
    const data = await fetch('/vlm_prompt', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({prompt})
    }).then(r => r.json());
    const reply = data.message || (data.accepted ? 'Command accepted' : 'Not accepted');
    vlmChatHistory.push({role: 'robot', text: reply, ts: data.timestamp || ts});
  } catch(err) {
    vlmChatHistory.push({role: 'robot', text: `Error: ${err}`, ts});
  }
  renderVlmChat();
}

function renderVlmChat() {
  const c = document.getElementById('vlm-chat');
  if (!c) return;
  c.innerHTML = vlmChatHistory.map(m =>
    `<div class="chat-${m.role}"><span class="chat-ts">[${m.ts}] ${m.role === 'user' ? 'YOU' : 'ROBOT'}:</span> ${m.text}</div>`
  ).join('');
  c.scrollTop = c.scrollHeight;
}

document.addEventListener('DOMContentLoaded', () => {
  const pi = document.getElementById('vlm-prompt-input');
  if (pi) pi.addEventListener('keydown', e => { if (e.key === 'Enter') sendVlmPrompt(); });
});

// ── Safety Toggles ────────────────────────────────────────────────────────
async function toggleSoftBumper() {
  softBumperEnabled = !softBumperEnabled;
  syncToggleButton('btn-soft-bumper', softBumperEnabled, 'SOFT BUMPER: ON', 'SOFT BUMPER: OFF');
  fetch('/soft_bumper', {method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({enabled: softBumperEnabled})}).catch(() => {});
}

async function toggleSafetyEnabled() {
  safetyEnabled = !safetyEnabled;
  syncToggleButton('btn-safety-enabled', safetyEnabled, 'SAFETY: ON', 'SAFETY: OFF');
  fetch('/safety_enabled', {method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({enabled: safetyEnabled})}).catch(() => {});
}

async function toggleRearOverride() {
  rearOverrideEnabled = !rearOverrideEnabled;
  const btn = document.getElementById('btn-rear-override');
  if (btn) {
    btn.textContent = rearOverrideEnabled ? 'REV LOCKED' : 'REV UNLOCK';
    btn.classList.toggle('active', rearOverrideEnabled);
  }
  fetch('/rear_override', {method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({enabled: rearOverrideEnabled})}).catch(() => {});
}

// ── Polling Intervals ─────────────────────────────────────────────────────
setInterval(updateStatus, 1500);
setInterval(updateLidar, 200);
setInterval(updatePerceptionLog, 1000);

updateStatus();
updateLidar();
updatePerceptionLog();
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
