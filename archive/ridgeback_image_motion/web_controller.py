#!/usr/bin/env python3
"""
Ridgeback R100 Web Controller (FastAPI)
- Smooth MJPEG video streaming from RealSense
- Motion control via Motion Service (Client)
- Omnidirectional teleop (holonomic: forward, strafe, rotate)
- Live velocity, pose, and latency display
Runs on: Ridgeback R100
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CompressedImage, LaserScan, BatteryState
from nav_msgs.msg import Odometry
from ridgeback_image_motion.srv import Motion

import cv2
import numpy as np
import threading
import time
import math
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from pydantic import BaseModel
import uvicorn


class RidgebackController(Node):
    def __init__(self):
        super().__init__('web_controller')

        # Parameters
        self.declare_parameter('image_topic', '/r100_0140/image/compressed')
        self.declare_parameter('motion_service', 'motion_service')
        self.declare_parameter('odom_topic', '/r100_0140/platform/odom/filtered')
        self.declare_parameter('lidar_topic', '/r100_0140/sensors/lidar2d_0/scan')
        self.declare_parameter('battery_topic', '/r100_0140/platform/bms/state')
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 2.0)

        image_topic = self.get_parameter('image_topic').value
        motion_service = self.get_parameter('motion_service').value
        odom_topic = self.get_parameter('odom_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        battery_topic = self.get_parameter('battery_topic').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value

        # QoS profiles
        reliable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, best_effort_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, best_effort_qos
        )

        # LiDAR subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan, lidar_topic, self.lidar_callback, best_effort_qos
        )

        # Battery subscriber
        self.battery_sub = self.create_subscription(
            BatteryState, battery_topic, self.battery_callback, best_effort_qos
        )

        # Battery state
        self.battery_voltage = 0.0
        self.battery_percentage = 0.0

        # LiDAR state
        self.lidar_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_max = 0.0
        self.lidar_angle_increment = 0.0
        self.lidar_range_max = 10.0
        self.lidar_lock = threading.Lock()

        # Motion Service Client
        self.motion_client = self.create_client(Motion, motion_service)
        self.service_ready = False

        # State
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.current_linear_vel = 0.0
        self.current_lateral_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.status = "Ready"
        self.is_moving = False

        # Motion control
        self.motion_thread = None
        self.stop_motion = threading.Event()
        self.current_cmd_linear = 0.0
        self.current_cmd_lateral = 0.0
        self.current_cmd_angular = 0.0

        # Log buffer for UI
        self.log_buffer = []
        self.max_logs = 50

        # Pose offset for reset functionality
        self.pose_offset_x = 0.0
        self.pose_offset_y = 0.0
        self.pose_offset_yaw = 0.0

        # Latency tracking (milliseconds)
        self.image_latency_ms = 0.0
        self.motion_latency_ms = 0.0
        self.odom_latency_ms = 0.0

        self.get_logger().info('Ridgeback Web Controller started')
        self.get_logger().info(f'  Image: {image_topic}')
        self.get_logger().info(f'  Motion Service: {motion_service}')
        self.get_logger().info(f'  Odom: {odom_topic}')
        self.get_logger().info(f'  LiDAR: {lidar_topic}')

    def image_callback(self, msg):
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.image_latency_ms = (now - stamp).nanoseconds / 1e6

        with self.frame_lock:
            self.latest_frame = bytes(msg.data)
        self.frame_event.set()

    def odom_callback(self, msg):
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.odom_latency_ms = (now - stamp).nanoseconds / 1e6

        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_lateral_vel = msg.twist.twist.linear.y
        self.current_angular_vel = msg.twist.twist.angular.z
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        with self.lidar_lock:
            self.lidar_ranges = list(msg.ranges)
            self.lidar_angle_min = msg.angle_min
            self.lidar_angle_max = msg.angle_max
            self.lidar_angle_increment = msg.angle_increment
            self.lidar_range_max = msg.range_max

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage

    def get_lidar_data(self):
        with self.lidar_lock:
            if not self.lidar_ranges:
                return None
            # Downsample to every 3rd point for performance
            step = 3
            sampled = self.lidar_ranges[::step]
            return {
                "ranges": sampled,
                "angle_min": self.lidar_angle_min,
                "angle_increment": self.lidar_angle_increment * step,
                "range_max": self.lidar_range_max
            }

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def add_log(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log_buffer.append(f"[{timestamp}] {msg}")
        if len(self.log_buffer) > self.max_logs:
            self.log_buffer.pop(0)

    def get_logs(self):
        return self.log_buffer[-5:]

    def send_motion_command(self, linear, lateral, angular, force_log=False):
        """Send holonomic motion command via service."""
        request = Motion.Request()
        request.linear = float(linear)
        request.lateral = float(lateral)
        request.angular = float(angular)

        if self.motion_client.service_is_ready():
            send_time = time.time()
            future = self.motion_client.call_async(request)
            future.add_done_callback(
                lambda f, t=send_time: self._motion_latency_cb(f, t)
            )
            self.service_ready = True
        else:
            if self.service_ready:
                self.get_logger().warn('Motion service not available')
                self.add_log('Motion service disconnected!')
            self.service_ready = False

        self.current_cmd_linear = linear
        self.current_cmd_lateral = lateral
        self.current_cmd_angular = angular

        is_stop = linear == 0.0 and lateral == 0.0 and angular == 0.0
        if force_log or is_stop:
            self.add_log(
                f'MOTION: lin={linear:.3f}, lat={lateral:.3f}, ang={angular:.3f}'
            )

    def _motion_latency_cb(self, future, send_time):
        self.motion_latency_ms = (time.time() - send_time) * 1000.0

    def ramp_velocity(self, current, target, max_accel, dt):
        if abs(target - current) < 0.01:
            return target
        max_change = max_accel * dt
        if target > current:
            return min(current + max_change, target)
        else:
            return max(current - max_change, target)

    def stop(self):
        self.add_log('STOP command received!')
        self.get_logger().info('STOP command received!')
        self.stop_motion.set()
        self.is_moving = False

        for _ in range(5):
            self.send_motion_command(0.0, 0.0, 0.0)
            time.sleep(0.02)

        self.status = "Stopped"
        self.add_log('Robot stopped')

    def move_straight(self, distance, speed):
        """Move straight by distance (meters) at speed (m/s)."""
        if self.is_moving:
            self.stop_motion.set()

        self.stop_motion.clear()
        self.is_moving = True
        self.status = f"Moving {distance:.2f}m at {speed:.2f}m/s"

        direction = 1.0 if distance >= 0 else -1.0
        initial_velocity = min(abs(speed), 0.15) * direction
        self.add_log(f'Starting move: {distance}m at {speed}m/s')
        self.send_motion_command(initial_velocity, 0.0, 0.0, force_log=True)

        def motion_task():
            start_x = self.current_x
            start_y = self.current_y
            target_distance = abs(distance)
            target_velocity = abs(speed) * direction
            current_vel = initial_velocity
            dt = 0.02

            while not self.stop_motion.is_set():
                dx = self.current_x - start_x
                dy = self.current_y - start_y
                traveled = math.sqrt(dx*dx + dy*dy)
                remaining = target_distance - traveled

                if traveled >= target_distance:
                    break

                if remaining < 0.1:
                    decel_vel = max(0.05, remaining * 2) * direction
                    target_vel_now = decel_vel if abs(decel_vel) < abs(target_velocity) else target_velocity
                else:
                    target_vel_now = target_velocity

                current_vel = self.ramp_velocity(current_vel, target_vel_now, self.max_linear_accel, dt)
                self.send_motion_command(current_vel, 0.0, 0.0)
                self.status = f"Moving... {traveled:.2f}/{target_distance:.2f}m"
                time.sleep(dt)

            while abs(current_vel) > 0.01 and not self.stop_motion.is_set():
                current_vel = self.ramp_velocity(current_vel, 0.0, self.max_linear_accel * 2, dt)
                self.send_motion_command(current_vel, 0.0, 0.0)
                time.sleep(dt)

            self.send_motion_command(0.0, 0.0, 0.0)
            self.is_moving = False
            if not self.stop_motion.is_set():
                self.status = f"Moved {target_distance:.2f}m"

        self.motion_thread = threading.Thread(target=motion_task, daemon=True)
        self.motion_thread.start()
        return True

    def rotate(self, angle_deg, speed):
        """Rotate by angle (degrees) at speed (rad/s)."""
        if self.is_moving:
            self.stop_motion.set()

        self.stop_motion.clear()
        self.is_moving = True
        self.status = f"Rotating {angle_deg:.1f} deg at {speed:.2f}rad/s"

        direction = 1.0 if angle_deg >= 0 else -1.0
        initial_velocity = min(abs(speed), 0.3) * direction
        self.add_log(f'Starting rotate: {angle_deg} deg at {speed}rad/s')
        self.send_motion_command(0.0, 0.0, initial_velocity, force_log=True)

        def motion_task():
            target_angle = abs(math.radians(angle_deg))
            target_velocity = abs(speed) * direction
            current_vel = initial_velocity
            dt = 0.02

            total_rotated = 0.0
            last_yaw = self.current_yaw

            while not self.stop_motion.is_set():
                delta = self.current_yaw - last_yaw
                if delta > math.pi:
                    delta -= 2 * math.pi
                elif delta < -math.pi:
                    delta += 2 * math.pi

                total_rotated += abs(delta)
                last_yaw = self.current_yaw
                remaining = target_angle - total_rotated

                if total_rotated >= target_angle:
                    break

                if remaining < 0.1:
                    decel_vel = max(0.1, remaining * 2) * direction
                    target_vel_now = decel_vel if abs(decel_vel) < abs(target_velocity) else target_velocity
                else:
                    target_vel_now = target_velocity

                current_vel = self.ramp_velocity(current_vel, target_vel_now, self.max_angular_accel, dt)
                self.send_motion_command(0.0, 0.0, current_vel)
                rotated_deg = math.degrees(total_rotated)
                self.status = f"Rotating... {rotated_deg:.1f}/{abs(angle_deg):.1f} deg"
                time.sleep(dt)

            while abs(current_vel) > 0.01 and not self.stop_motion.is_set():
                current_vel = self.ramp_velocity(current_vel, 0.0, self.max_angular_accel * 2, dt)
                self.send_motion_command(0.0, 0.0, current_vel)
                time.sleep(dt)

            self.send_motion_command(0.0, 0.0, 0.0)
            self.is_moving = False
            if not self.stop_motion.is_set():
                self.status = f"Rotated {abs(angle_deg):.1f} deg"

        self.motion_thread = threading.Thread(target=motion_task, daemon=True)
        self.motion_thread.start()
        return True

    def reset_pose(self):
        self.pose_offset_x = self.current_x
        self.pose_offset_y = self.current_y
        self.pose_offset_yaw = self.current_yaw
        self.add_log('Pose reset to (0, 0, 0)')

    def get_status(self):
        rel_x = self.current_x - self.pose_offset_x
        rel_y = self.current_y - self.pose_offset_y
        rel_yaw = self.current_yaw - self.pose_offset_yaw
        while rel_yaw > math.pi:
            rel_yaw -= 2 * math.pi
        while rel_yaw < -math.pi:
            rel_yaw += 2 * math.pi

        return {
            "status": self.status,
            "is_moving": self.is_moving,
            "velocity": {
                "linear": round(self.current_linear_vel, 4),
                "lateral": round(self.current_lateral_vel, 4),
                "angular": round(self.current_angular_vel, 4)
            },
            "commanded": {
                "linear": round(self.current_cmd_linear, 4),
                "lateral": round(self.current_cmd_lateral, 4),
                "angular": round(self.current_cmd_angular, 4)
            },
            "pose": {
                "x": round(rel_x, 4),
                "y": round(rel_y, 4),
                "yaw_deg": round(math.degrees(rel_yaw), 2)
            },
            "logs": self.get_logs(),
            "latency": {
                "image_ms": round(self.image_latency_ms, 2),
                "motion_ms": round(self.motion_latency_ms, 2),
                "odom_ms": round(self.odom_latency_ms, 2)
            },
            "battery": {
                "voltage": round(self.battery_voltage, 2),
                "percentage": round(self.battery_percentage, 1)
            }
        }


# Global controller
controller = None
ros_thread = None


def ros_spin():
    global controller
    rclpy.spin(controller)


def init_ros():
    global controller, ros_thread
    rclpy.init()
    controller = RidgebackController()
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    time.sleep(2)
    controller.reset_pose()
    controller.add_log('Web controller started - pose auto-reset')


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_ros()
    yield
    if controller:
        controller.stop()
        controller.destroy_node()
    rclpy.shutdown()


app = FastAPI(title="Ridgeback R100 Controller", lifespan=lifespan)


class MoveRequest(BaseModel):
    distance: float = 0.5
    speed: float = 0.2


class RotateRequest(BaseModel):
    angle: float = 90.0
    speed: float = 0.5


class TeleopRequest(BaseModel):
    linear: float = 0.0
    lateral: float = 0.0
    angular: float = 0.0


def generate_mjpeg():
    while True:
        if controller:
            controller.frame_event.wait(timeout=0.5)
            controller.frame_event.clear()
            frame = controller.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.1)


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(
        generate_mjpeg(),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )


@app.get("/status")
async def get_status():
    if controller:
        return JSONResponse(controller.get_status())
    return JSONResponse({"error": "Not connected"})


@app.post("/move")
async def move(req: MoveRequest):
    if controller:
        success = controller.move_straight(req.distance, req.speed)
        return JSONResponse({"success": success, "status": controller.status})
    return JSONResponse({"success": False, "error": "Not connected"})


@app.post("/rotate")
async def rotate(req: RotateRequest):
    if controller:
        success = controller.rotate(req.angle, req.speed)
        return JSONResponse({"success": success, "status": controller.status})
    return JSONResponse({"success": False, "error": "Not connected"})


@app.post("/stop")
async def stop():
    if controller:
        controller.stop()
        return JSONResponse({"success": True, "status": "Stopped"})
    return JSONResponse({"success": False, "error": "Not connected"})


@app.post("/reset_pose")
async def reset_pose():
    if controller:
        controller.reset_pose()
        return JSONResponse({"success": True, "status": "Pose reset"})
    return JSONResponse({"success": False, "error": "Not connected"})


@app.post("/teleop")
async def teleop(req: TeleopRequest):
    if controller:
        controller.send_motion_command(req.linear, req.lateral, req.angular, force_log=True)
        is_moving = req.linear != 0 or req.lateral != 0 or req.angular != 0
        controller.status = "Teleop" if is_moving else "Ready"
        return JSONResponse({"success": True})
    return JSONResponse({"success": False, "error": "Not connected"})


@app.get("/lidar")
async def lidar_data():
    if controller:
        data = controller.get_lidar_data()
        if data:
            return JSONResponse(data)
        return JSONResponse({"error": "No LiDAR data"})
    return JSONResponse({"error": "Not connected"})


HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Ridgeback R100 Controller</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #0d0f1a 0%, #1a1e2e 50%, #0f111a 100%);
            color: #fff;
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        h1 {
            text-align: center;
            margin-bottom: 20px;
            font-size: 2em;
            background: linear-gradient(90deg, #f7941d, #ffc107);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .top-grid {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .bottom-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }
        @media (max-width: 900px) {
            .top-grid { grid-template-columns: 1fr; }
            .bottom-grid { grid-template-columns: 1fr; }
        }
        .video-container {
            background: #1a1e2e;
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.5);
            border: 1px solid #2d3348;
        }
        .video-container h2 {
            margin-bottom: 10px;
            color: #f7941d;
            font-size: 1.1em;
        }
        .video-container img {
            width: 100%;
            height: 500px;
            object-fit: contain;
            border-radius: 8px;
            background: #000;
        }
        .panel {
            background: #1a1e2e;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.5);
            border: 1px solid #2d3348;
        }
        .panel h2 {
            color: #f7941d;
            font-size: 1.1em;
            margin-bottom: 15px;
            border-bottom: 1px solid #2d3348;
            padding-bottom: 10px;
        }
        .info-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-bottom: 15px;
        }
        .info-item {
            background: #0d0f1a;
            padding: 12px;
            border-radius: 8px;
            border: 1px solid #2d3348;
        }
        .info-label {
            font-size: 0.75em;
            color: #7a8fa0;
            text-transform: uppercase;
        }
        .info-value {
            font-size: 1.3em;
            font-weight: bold;
            font-family: 'SF Mono', Monaco, monospace;
            color: #4fc3f7;
        }
        .status-bar {
            background: #0d0f1a;
            padding: 12px;
            border-radius: 8px;
            margin-bottom: 15px;
            text-align: center;
            border: 1px solid #2d3348;
        }
        .status-text {
            font-size: 1em;
            color: #f7941d;
        }
        .control-section { margin-top: 20px; }
        .control-group {
            background: #0d0f1a;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 15px;
            border: 1px solid #2d3348;
        }
        .control-group h3 {
            font-size: 0.9em;
            color: #f7941d;
            margin-bottom: 12px;
        }
        .input-row {
            display: flex;
            gap: 10px;
            margin-bottom: 10px;
        }
        .input-group { flex: 1; }
        .input-group label {
            display: block;
            font-size: 0.75em;
            color: #888;
            margin-bottom: 4px;
        }
        .input-group input {
            width: 100%;
            padding: 10px;
            border: 1px solid #2d3348;
            border-radius: 6px;
            background: #0d0f1a;
            color: #fff;
            font-size: 1em;
            font-family: 'SF Mono', Monaco, monospace;
        }
        .btn {
            width: 100%;
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn-primary {
            background: linear-gradient(90deg, #4fc3f7, #2196f3);
            color: #fff;
        }
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 15px rgba(79,195,247,0.4);
        }
        .btn-danger {
            background: linear-gradient(90deg, #ed1c24, #c41017);
            color: #fff;
            font-size: 1.2em;
            padding: 16px;
            margin-top: 10px;
        }
        .btn-danger:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 15px rgba(237,28,36,0.4);
        }
        .btn:active { transform: translateY(0); }
        .speed-controls {
            display: flex;
            gap: 15px;
            margin-bottom: 15px;
        }
        .speed-controls .input-group { flex: 1; }
        .speed-controls input[type="range"] { width: 100%; margin: 5px 0; }
        .speed-controls span {
            font-family: 'SF Mono', Monaco, monospace;
            color: #4fc3f7;
        }
        .teleop-section {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }
        .teleop-pad {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 5px;
            padding: 15px;
            background: #0d0f1a;
            border-radius: 12px;
            border: 1px solid #2d3348;
        }
        .teleop-label {
            font-size: 0.75em;
            color: #7a8fa0;
            margin-bottom: 5px;
        }
        .teleop-row {
            display: flex;
            gap: 5px;
        }
        .teleop-btn {
            width: 60px;
            height: 60px;
            font-size: 24px;
            border: none;
            border-radius: 8px;
            background: linear-gradient(135deg, #2d3348 0%, #1a1e2e 100%);
            color: #4fc3f7;
            cursor: pointer;
            transition: all 0.15s;
            user-select: none;
            -webkit-user-select: none;
        }
        .teleop-btn:hover {
            background: linear-gradient(135deg, #3d4358 0%, #2d3348 100%);
            transform: scale(1.05);
        }
        .teleop-btn:active, .teleop-btn.active {
            background: linear-gradient(135deg, #f7941d 0%, #c77816 100%);
            color: #fff;
            transform: scale(0.95);
        }
        .teleop-btn.stop-btn {
            background: linear-gradient(135deg, #4a2020 0%, #3a1515 100%);
            color: #ed1c24;
        }
        .teleop-btn.stop-btn:hover {
            background: linear-gradient(135deg, #5a3030 0%, #4a2020 100%);
        }
        .teleop-btn.stop-btn:active {
            background: linear-gradient(135deg, #ed1c24 0%, #c41017 100%);
            color: #fff;
        }
        .rotate-row {
            display: flex;
            gap: 10px;
            margin-top: 5px;
        }
        .rotate-btn {
            width: 90px;
            height: 45px;
            font-size: 16px;
            border: none;
            border-radius: 8px;
            background: linear-gradient(135deg, #2d3348 0%, #1a1e2e 100%);
            color: #ffc107;
            cursor: pointer;
            transition: all 0.15s;
            user-select: none;
            -webkit-user-select: none;
        }
        .rotate-btn:hover {
            background: linear-gradient(135deg, #3d4358 0%, #2d3348 100%);
            transform: scale(1.05);
        }
        .rotate-btn:active, .rotate-btn.active {
            background: linear-gradient(135deg, #ffc107 0%, #c79600 100%);
            color: #000;
            transform: scale(0.95);
        }
        .log-box {
            background: #0d0f1a;
            border-radius: 6px;
            padding: 10px;
            height: 100px;
            overflow-y: auto;
            font-family: 'SF Mono', Monaco, monospace;
            font-size: 0.75em;
            line-height: 1.4;
            border: 1px solid #2d3348;
        }
        .lidar-container {
            margin-top: 15px;
            background: #0d0f1a;
            border-radius: 12px;
            padding: 15px;
            border: 1px solid #2d3348;
        }
        .lidar-container h2 {
            color: #f7941d;
            font-size: 1.1em;
            margin-bottom: 10px;
        }
        .lidar-container canvas {
            width: 100%;
            height: 500px;
            border-radius: 8px;
            background: #000;
        }
        .lidar-info {
            display: flex;
            justify-content: space-between;
            margin-top: 8px;
            font-size: 0.75em;
            color: #7a8fa0;
            font-family: 'SF Mono', Monaco, monospace;
        }
        .log-entry { color: #7a8fa0; margin-bottom: 2px; }
        .log-entry.cmd { color: #f7941d; }
        .log-entry.stop { color: #ed1c24; }
        .log-entry.start { color: #4fc3f7; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Ridgeback R100 Controller</h1>
        <p style="text-align: center; color: #888; margin-top: -10px; margin-bottom: 20px;">Omnidirectional Video & Teleop Dashboard</p>

        <div class="top-grid">
            <div class="video-container">
                <h2>RealSense Camera Feed (Live)</h2>
                <img id="video" src="/video_feed" alt="Camera Feed">
            </div>

            <div class="panel">
                <h2>Teleop Control</h2>
                <div class="speed-controls">
                    <div class="input-group">
                        <label>Linear Speed (m/s)</label>
                        <input type="range" id="lin-speed" min="0.05" max="0.5" step="0.05" value="0.2">
                        <span id="lin-speed-val">0.20</span>
                    </div>
                    <div class="input-group">
                        <label>Angular Speed (rad/s)</label>
                        <input type="range" id="ang-speed" min="0.1" max="1.5" step="0.1" value="0.5">
                        <span id="ang-speed-val">0.50</span>
                    </div>
                </div>
                <div class="teleop-section">
                    <div class="teleop-pad">
                        <div class="teleop-label">Omnidirectional Movement</div>
                        <div class="teleop-row">
                            <button class="teleop-btn" id="btn-fl" title="Forward + Strafe Left">&#8598;</button>
                            <button class="teleop-btn" id="btn-fwd" title="Forward">&#9650;</button>
                            <button class="teleop-btn" id="btn-fr" title="Forward + Strafe Right">&#8599;</button>
                        </div>
                        <div class="teleop-row">
                            <button class="teleop-btn" id="btn-sl" title="Strafe Left">&#9664;</button>
                            <button class="teleop-btn stop-btn" id="btn-stop" title="Stop">&#9632;</button>
                            <button class="teleop-btn" id="btn-sr" title="Strafe Right">&#9654;</button>
                        </div>
                        <div class="teleop-row">
                            <button class="teleop-btn" id="btn-bl" title="Backward + Strafe Left">&#8601;</button>
                            <button class="teleop-btn" id="btn-bwd" title="Backward">&#9660;</button>
                            <button class="teleop-btn" id="btn-br" title="Backward + Strafe Right">&#8600;</button>
                        </div>
                    </div>
                    <div class="rotate-row">
                        <button class="rotate-btn" id="btn-ccw" title="Rotate CCW">&#8634; CCW</button>
                        <button class="rotate-btn" id="btn-cw" title="Rotate CW">CW &#8635;</button>
                    </div>
                </div>
                <p style="text-align: center; color: #7a8fa0; font-size: 0.8em; margin-top: 10px;">
                    Click to move continuously | Click again or Stop to halt | Keyboard: WASD + QE (rotate)
                </p>

                <h2 style="margin-top: 15px;">Robot Status</h2>
                <div class="status-bar">
                    <span class="status-text" id="status">Connecting...</span>
                </div>

                <h2>Battery</h2>
                <div class="info-grid">
                    <div class="info-item">
                        <div class="info-label">Voltage</div>
                        <div class="info-value" id="battery-voltage">--</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Level</div>
                        <div class="info-value" id="battery-percent">--</div>
                    </div>
                </div>

                <h2>Motion Log</h2>
                <div class="log-box" id="log-box"></div>
            </div>
        </div>

        <div class="bottom-grid">
            <div class="panel">
                <h2>Pose</h2>
                <div class="info-grid">
                    <div class="info-item">
                        <div class="info-label">X (m)</div>
                        <div class="info-value" id="pos-x">0.000</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Y (m)</div>
                        <div class="info-value" id="pos-y">0.000</div>
                    </div>
                </div>
                <div class="info-item" style="margin-bottom: 15px;">
                    <div class="info-label">Yaw (deg)</div>
                    <div class="info-value" id="yaw">0.0</div>
                </div>

                <h2>Latency</h2>
                <div class="info-grid" style="grid-template-columns: 1fr 1fr 1fr;">
                    <div class="info-item">
                        <div class="info-label">Image</div>
                        <div class="info-value" id="lat-image">--</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Motion</div>
                        <div class="info-value" id="lat-motion">--</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Odom</div>
                        <div class="info-value" id="lat-odom">--</div>
                    </div>
                </div>
            </div>

            <div class="lidar-container">
                <h2>LiDAR Map (Top-Down)</h2>
                <canvas id="lidar-canvas" width="600" height="600"></canvas>
                <div class="lidar-info">
                    <span id="lidar-closest">Closest: --</span>
                    <span id="lidar-points">Points: --</span>
                    <span>Hokuyo UST-10LX (270&deg;)</span>
                </div>
            </div>
        </div>
    </div>

    <script>
        async function updateStatus() {
            try {
                const res = await fetch('/status');
                const data = await res.json();

                document.getElementById('status').textContent = data.status;
                document.getElementById('pos-x').textContent = data.pose.x.toFixed(3);
                document.getElementById('pos-y').textContent = data.pose.y.toFixed(3);
                document.getElementById('yaw').textContent = data.pose.yaw_deg.toFixed(1);

                if (data.latency) {
                    document.getElementById('lat-image').textContent = data.latency.image_ms.toFixed(1);
                    document.getElementById('lat-motion').textContent = data.latency.motion_ms.toFixed(1);
                    document.getElementById('lat-odom').textContent = data.latency.odom_ms.toFixed(1);
                }

                if (data.battery) {
                    document.getElementById('battery-voltage').textContent = data.battery.voltage.toFixed(2) + ' V';
                    document.getElementById('battery-percent').textContent = data.battery.percentage.toFixed(1) + ' %';
                }

                if (data.logs && data.logs.length > 0) {
                    const logBox = document.getElementById('log-box');
                    const wasAtBottom = logBox.scrollHeight - logBox.scrollTop <= logBox.clientHeight + 10;
                    logBox.innerHTML = data.logs.map(log => {
                        let cls = 'log-entry';
                        if (log.includes('MOTION')) cls += ' cmd';
                        else if (log.includes('STOP')) cls += ' stop';
                        else if (log.includes('Starting')) cls += ' start';
                        return '<div class="' + cls + '">' + log + '</div>';
                    }).join('');
                    if (wasAtBottom) logBox.scrollTop = logBox.scrollHeight;
                }
            } catch (e) {
                document.getElementById('status').textContent = 'Connection error';
            }
        }

        // Teleop state
        let teleopInterval = null;
        let activeBtn = null;
        let activeLin = 0, activeLat = 0, activeAng = 0;

        function getLinSpeed() { return parseFloat(document.getElementById('lin-speed').value); }
        function getAngSpeed() { return parseFloat(document.getElementById('ang-speed').value); }

        async function sendTeleop(linear, lateral, angular) {
            try {
                await fetch('/teleop', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({linear, lateral, angular})
                });
            } catch (e) { console.error('Teleop error:', e); }
        }

        function stopRobot() {
            if (teleopInterval) { clearInterval(teleopInterval); teleopInterval = null; }
            if (activeBtn) { activeBtn.classList.remove('active'); activeBtn = null; }
            activeLin = 0; activeLat = 0; activeAng = 0;
            sendTeleop(0, 0, 0);
        }

        function toggleTeleop(btn, getLinFn, getLatFn, getAngFn) {
            if (activeBtn === btn) { stopRobot(); return; }

            if (teleopInterval) clearInterval(teleopInterval);
            if (activeBtn) activeBtn.classList.remove('active');

            activeBtn = btn;
            btn.classList.add('active');

            function sendCurrent() {
                sendTeleop(getLinFn(), getLatFn(), getAngFn());
            }
            sendCurrent();
            teleopInterval = setInterval(sendCurrent, 50);
        }

        // Movement buttons — true omnidirectional (holonomic)
        const S = getLinSpeed;  // shorthand
        const A = getAngSpeed;
        const Z = () => 0;

        // Forward/backward
        document.getElementById('btn-fwd').addEventListener('click', function() {
            toggleTeleop(this, S, Z, Z);
        });
        document.getElementById('btn-bwd').addEventListener('click', function() {
            toggleTeleop(this, () => -getLinSpeed(), Z, Z);
        });

        // Strafe left/right
        document.getElementById('btn-sl').addEventListener('click', function() {
            toggleTeleop(this, Z, S, Z);
        });
        document.getElementById('btn-sr').addEventListener('click', function() {
            toggleTeleop(this, Z, () => -getLinSpeed(), Z);
        });

        // Diagonal: forward+strafe
        document.getElementById('btn-fl').addEventListener('click', function() {
            toggleTeleop(this, S, S, Z);
        });
        document.getElementById('btn-fr').addEventListener('click', function() {
            toggleTeleop(this, S, () => -getLinSpeed(), Z);
        });

        // Diagonal: backward+strafe
        document.getElementById('btn-bl').addEventListener('click', function() {
            toggleTeleop(this, () => -getLinSpeed(), S, Z);
        });
        document.getElementById('btn-br').addEventListener('click', function() {
            toggleTeleop(this, () => -getLinSpeed(), () => -getLinSpeed(), Z);
        });

        // Rotation buttons
        document.getElementById('btn-ccw').addEventListener('click', function() {
            toggleTeleop(this, Z, Z, A);
        });
        document.getElementById('btn-cw').addEventListener('click', function() {
            toggleTeleop(this, Z, Z, () => -getAngSpeed());
        });

        // Stop button
        document.getElementById('btn-stop').addEventListener('click', stopRobot);

        // Keyboard controls: WASD + QE for rotation, Space for stop
        const keyState = {};
        function updateFromKeyboard() {
            let lin = 0, lat = 0, ang = 0;
            const ls = getLinSpeed();
            const as_ = getAngSpeed();

            if (keyState['w'] || keyState['ArrowUp']) lin += ls;
            if (keyState['s'] || keyState['ArrowDown']) lin -= ls;
            if (keyState['a'] || keyState['ArrowLeft']) lat += ls;
            if (keyState['d'] || keyState['ArrowRight']) lat -= ls;
            if (keyState['q']) ang += as_;
            if (keyState['e']) ang -= as_;

            sendTeleop(lin, lat, ang);
        }

        let keyboardInterval = null;
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT') return;
            if (e.key === ' ') { e.preventDefault(); stopRobot(); return; }

            const key = e.key.toLowerCase();
            if (['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright'].includes(key)) {
                e.preventDefault();
                if (!keyState[key]) {
                    keyState[key] = true;
                    // Stop button-based teleop when using keyboard
                    if (teleopInterval) { clearInterval(teleopInterval); teleopInterval = null; }
                    if (activeBtn) { activeBtn.classList.remove('active'); activeBtn = null; }

                    if (!keyboardInterval) {
                        updateFromKeyboard();
                        keyboardInterval = setInterval(updateFromKeyboard, 50);
                    }
                }
            }
        });

        document.addEventListener('keyup', (e) => {
            const key = e.key.toLowerCase();
            delete keyState[key];
            if (Object.keys(keyState).length === 0 && keyboardInterval) {
                clearInterval(keyboardInterval);
                keyboardInterval = null;
                sendTeleop(0, 0, 0);
            }
        });

        // Speed slider display
        document.getElementById('lin-speed').addEventListener('input', (e) => {
            document.getElementById('lin-speed-val').textContent = parseFloat(e.target.value).toFixed(2);
        });
        document.getElementById('ang-speed').addEventListener('input', (e) => {
            document.getElementById('ang-speed-val').textContent = parseFloat(e.target.value).toFixed(2);
        });

        setInterval(updateStatus, 2000);
        updateStatus();

        // LiDAR visualization
        const lidarCanvas = document.getElementById('lidar-canvas');
        const ctx = lidarCanvas.getContext('2d');
        const LIDAR_SCALE = 60; // pixels per meter
        const LIDAR_MAX_RANGE = 4.0; // meters to display

        function drawLidar(data) {
            const w = lidarCanvas.width;
            const h = lidarCanvas.height;
            const cx = w / 2;
            const cy = h / 2;

            // Clear
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, w, h);

            // Draw range rings
            ctx.strokeStyle = '#1a2a1a';
            ctx.lineWidth = 1;
            for (let r = 1; r <= LIDAR_MAX_RANGE; r++) {
                ctx.beginPath();
                ctx.arc(cx, cy, r * LIDAR_SCALE, 0, Math.PI * 2);
                ctx.stroke();
                ctx.fillStyle = '#2a3a2a';
                ctx.font = '11px monospace';
                ctx.fillText(r + 'm', cx + r * LIDAR_SCALE + 3, cy - 3);
            }

            // Draw axes
            ctx.strokeStyle = '#1a2a3a';
            ctx.beginPath();
            ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
            ctx.moveTo(0, cy); ctx.lineTo(w, cy);
            ctx.stroke();

            // Draw forward direction indicator
            ctx.strokeStyle = '#2d4a2d';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx, cy - 30);
            ctx.stroke();
            ctx.fillStyle = '#4fc3f7';
            ctx.beginPath();
            ctx.moveTo(cx, cy - 35);
            ctx.lineTo(cx - 5, cy - 25);
            ctx.lineTo(cx + 5, cy - 25);
            ctx.fill();

            if (!data || !data.ranges) return;

            const ranges = data.ranges;
            let angleMin = data.angle_min;
            const angleInc = data.angle_increment;
            const rangeMax = Math.min(data.range_max, LIDAR_MAX_RANGE);
            let closest = Infinity;
            let validPoints = 0;

            // Draw scan points
            for (let i = 0; i < ranges.length; i++) {
                const range = ranges[i];
                if (range < 0.05 || range > rangeMax || !isFinite(range)) continue;

                const angle = angleMin + i * angleInc;
                // LiDAR frame: x=forward, y=left → canvas: up=forward, right=+x
                const px = cx - range * Math.sin(angle) * LIDAR_SCALE;
                const py = cy - range * Math.cos(angle) * LIDAR_SCALE;

                // Color by distance: close=red, mid=yellow, far=green
                const t = Math.min(range / rangeMax, 1.0);
                let r, g, b;
                if (t < 0.33) {
                    r = 255; g = Math.floor(t * 3 * 255); b = 0;
                } else if (t < 0.66) {
                    r = Math.floor((1 - (t - 0.33) * 3) * 255); g = 255; b = 0;
                } else {
                    r = 0; g = 255; b = Math.floor((t - 0.66) * 3 * 255);
                }

                ctx.fillStyle = `rgb(${r},${g},${b})`;
                ctx.beginPath();
                ctx.arc(px, py, 2, 0, Math.PI * 2);
                ctx.fill();

                if (range < closest) closest = range;
                validPoints++;
            }

            // Draw robot at center
            ctx.fillStyle = '#f7941d';
            ctx.beginPath();
            ctx.arc(cx, cy, 6, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#ffc107';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(cx, cy, 6, 0, Math.PI * 2);
            ctx.stroke();

            // Update info
            document.getElementById('lidar-closest').textContent =
                closest < Infinity ? 'Closest: ' + closest.toFixed(2) + 'm' : 'Closest: --';
            document.getElementById('lidar-points').textContent = 'Points: ' + validPoints;
        }

        async function updateLidar() {
            try {
                const res = await fetch('/lidar');
                const data = await res.json();
                if (!data.error) drawLidar(data);
            } catch (e) {
                // skip
            }
        }

        // Draw empty lidar on load
        drawLidar(null);
        setInterval(updateLidar, 200); // 5 Hz update
        updateLidar();
    </script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


def main(args=None):
    print("=" * 50)
    print("Ridgeback R100 - Web Controller")
    print("=" * 50)
    print("Open in browser: http://<ridgeback-ip>:8080")
    print("Video stream:    http://<ridgeback-ip>:8080/video_feed")
    print("=" * 50)

    uvicorn.run(app, host="0.0.0.0", port=8080)


if __name__ == '__main__':
    main()
