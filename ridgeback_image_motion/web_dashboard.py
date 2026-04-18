#!/usr/bin/env python3
"""Compact Jetson dashboard for Ridgeback autonomy and VLM chat."""

from __future__ import annotations

import base64
import math
import threading
import time
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from nav_msgs.msg import OccupancyGrid, Odometry
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
import uvicorn

try:
    from ridgeback_image_motion.spatial_memory import SpatialMemory
    from ridgeback_image_motion.vlm_client import build_vlm_client, chat_completion_messages
except ImportError:
    from spatial_memory import SpatialMemory
    from vlm_client import build_vlm_client, chat_completion_messages


class MissionRequest(BaseModel):
    command: str


class ChatRequest(BaseModel):
    message: str


PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Ridgeback Autonomy Dashboard</title>
  <style>
    :root {
      --bg: #0b1020;
      --panel: #121a31;
      --panel-2: #17213d;
      --border: rgba(167, 189, 255, 0.16);
      --text: #e8eeff;
      --muted: #a2b0d4;
      --accent: #7ee787;
      --accent-2: #7ab7ff;
      --warn: #ffcc66;
      --danger: #ff7f87;
    }
    * { box-sizing: border-box; }
    html, body { width: 100%; height: 100%; margin: 0; overflow: hidden; background:
      radial-gradient(circle at top left, rgba(122, 183, 255, 0.16), transparent 30%),
      radial-gradient(circle at bottom right, rgba(126, 231, 135, 0.08), transparent 24%),
      var(--bg); color: var(--text); font-family: Inter, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
    .shell { height: 100vh; display: grid; grid-template-rows: auto 1fr; gap: 12px; padding: 12px; }
    .topbar { display: grid; grid-template-columns: 1.3fr 1fr 1fr; gap: 12px; align-items: stretch; }
    .title, .mission, .stats, .camera, .map, .chat, .logs { background: linear-gradient(180deg, rgba(255,255,255,0.03), transparent 100%), var(--panel); border: 1px solid var(--border); border-radius: 16px; box-shadow: 0 12px 40px rgba(0,0,0,.25); }
    .title { padding: 14px 16px; display: flex; flex-direction: column; justify-content: space-between; }
    .title h1 { margin: 0; font-size: 22px; letter-spacing: 0.02em; }
    .title p { margin: 6px 0 0; color: var(--muted); font-size: 13px; }
    .mission { padding: 12px 14px; display: grid; gap: 10px; }
    .mission label, .section-label { font-size: 12px; text-transform: uppercase; letter-spacing: .12em; color: var(--muted); }
    .mission-row { display: grid; grid-template-columns: 1fr auto auto; gap: 8px; }
    .mission-row input, .chat-input input { width: 100%; background: var(--panel-2); color: var(--text); border: 1px solid var(--border); border-radius: 12px; padding: 11px 12px; outline: none; }
    .btn { background: linear-gradient(135deg, var(--accent-2), #5f8cff); border: 0; border-radius: 12px; color: #08101f; padding: 11px 14px; font-weight: 700; cursor: pointer; }
    .btn.secondary { background: var(--panel-2); color: var(--text); border: 1px solid var(--border); }
    .stats { padding: 12px 14px; display: grid; gap: 8px; grid-template-columns: repeat(3, minmax(0, 1fr)); }
    .metric { background: rgba(255,255,255,0.03); border-radius: 12px; padding: 10px; min-height: 72px; }
    .metric .k { color: var(--muted); font-size: 11px; margin-bottom: 8px; text-transform: uppercase; letter-spacing: .08em; }
    .metric .v { font-size: 18px; font-weight: 700; }
    .metric .s { margin-top: 4px; font-size: 12px; color: var(--muted); }
    .main { min-height: 0; display: grid; grid-template-columns: 1.25fr 0.95fr; gap: 12px; }
    .left, .right { min-height: 0; display: grid; gap: 12px; }
    .left { grid-template-rows: 1.08fr 0.92fr; }
    .right { grid-template-rows: 0.52fr 0.72fr 0.76fr; }
    .camera, .map, .chat, .logs { min-height: 0; padding: 12px; display: grid; grid-template-rows: auto 1fr; }
    .panel-head { display: flex; justify-content: space-between; align-items: baseline; gap: 12px; margin-bottom: 10px; }
    .panel-head h2 { margin: 0; font-size: 15px; }
    .panel-head span { color: var(--muted); font-size: 12px; }
    .camera img, .map img { width: 100%; height: 100%; object-fit: contain; background: #0a0f1d; border-radius: 12px; }
    .scroll { min-height: 0; overflow: auto; padding-right: 4px; }
    .chat-feed, .log-feed { display: grid; gap: 8px; align-content: start; }
    .bubble { background: rgba(255,255,255,0.04); border: 1px solid var(--border); border-radius: 12px; padding: 10px 12px; }
    .bubble .role { color: var(--accent-2); font-size: 11px; text-transform: uppercase; letter-spacing: .12em; margin-bottom: 4px; }
    .bubble.assistant .role { color: var(--accent); }
    .bubble .body { white-space: pre-wrap; line-height: 1.35; }
    .chat { grid-template-rows: auto 1fr auto; }
    .chat-input { display: grid; grid-template-columns: 1fr auto; gap: 8px; margin-top: 10px; }
    .tiny { font-size: 11px; color: var(--muted); }
    @media (max-width: 1200px) {
      body, html { overflow: auto; }
      .shell { height: auto; min-height: 100vh; }
      .main, .topbar { grid-template-columns: 1fr; }
      .left, .right { grid-template-rows: auto; }
      .camera, .map, .chat, .logs, .title, .mission, .stats { min-height: 240px; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="topbar">
      <div class="title">
        <div>
          <h1>Ridgeback Autonomy Dashboard</h1>
          <p>One-screen control plane for SLAM, Nav2, spatial memory, and external VLM chat.</p>
        </div>
        <div class="tiny" id="connectionText">Waiting for robot state...</div>
      </div>
      <div class="mission">
        <label for="missionInput">Mission command</label>
        <div class="mission-row">
          <input id="missionInput" type="text" placeholder="Go to room 305, find the landmark, and return home" />
          <button class="btn" onclick="sendMission()">Send</button>
          <button class="btn secondary" onclick="resetMission()">Clear</button>
        </div>
        <div class="tiny">Natural language is routed to the mission layer; the goal is tabula rasa SLAM and return-to-start.</div>
      </div>
      <div class="stats" id="statsGrid"></div>
    </div>

    <div class="main">
      <div class="left">
        <section class="camera">
          <div class="panel-head"><h2>Camera</h2><span id="cameraMeta">stream</span></div>
          <img src="/video_feed" alt="Camera feed" />
        </section>
        <section class="map">
          <div class="panel-head"><h2>SLAM Map</h2><span id="mapMeta">OccupancyGrid</span></div>
          <img id="mapImage" src="/api/map.png" alt="Map" />
        </section>
      </div>

      <div class="right">
        <section class="logs">
          <div class="panel-head"><h2>VLM Logs</h2><span>External vLLM chat + perception notes</span></div>
          <div class="scroll">
            <div id="logFeed" class="log-feed"></div>
          </div>
        </section>
        <section class="chat">
          <div class="panel-head"><h2>VLM Chat</h2><span>Qwen served from the external endpoint</span></div>
          <div class="scroll"><div id="chatFeed" class="chat-feed"></div></div>
          <div class="chat-input">
            <input id="chatInput" type="text" placeholder="Ask about the room number, route, or what the robot sees" />
            <button class="btn" onclick="sendChat()">Ask</button>
          </div>
        </section>
        <section class="logs">
          <div class="panel-head"><h2>Robot Status</h2><span id="statusMeta">live</span></div>
          <div class="scroll"><div id="statusFeed" class="log-feed"></div></div>
        </section>
      </div>
    </div>
  </div>

  <script>
    let lastMapStamp = "";
    let lastLogCount = 0;
    let lastChatCount = 0;

    function kvLine(label, value) {
      return `<div class="bubble"><div class="role">${label}</div><div class="body">${value}</div></div>`;
    }

    function escapeHtml(value) {
      return String(value)
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#039;');
    }

    async function refreshStatus() {
      const response = await fetch('/api/status');
      const data = await response.json();
      const stats = [
        ['Battery', `${data.battery_pct.toFixed(0)}%`, data.battery_voltage.toFixed(1) + ' V'],
        ['Pose', `${data.pose.x.toFixed(2)}, ${data.pose.y.toFixed(2)}`, `yaw ${data.pose.yaw_deg.toFixed(1)}°`],
        ['Map', `${data.map.width} x ${data.map.height}`, `${data.map.resolution.toFixed(3)} m/cell`],
        ['Camera', `${data.latency.image_ms.toFixed(0)} ms`, `odom ${data.latency.odom_ms.toFixed(0)} ms`],
        ['Mission', data.mission.state, data.mission.command || 'idle'],
        ['VLM', `${data.vlm.chat_count} chat`, `${data.vlm.log_count} log entries`],
        ['Memory', `${data.memory.location_count} locations`, `${data.memory.mission_count} missions`],
      ];
      document.getElementById('statsGrid').innerHTML = stats.map(([k, v, s]) => `
        <div class="metric"><div class="k">${k}</div><div class="v">${v}</div><div class="s">${s}</div></div>`).join('');
      document.getElementById('connectionText').textContent = data.connected ? 'Connected to ROS topics' : 'Waiting for ROS topics';
      document.getElementById('statusMeta').textContent = `${data.mission.state} • ${data.vlm.chat_count} chats`;

      const statusItems = [
        ['Connection', data.connected ? 'ROS topics active' : 'No active topics'],
        ['Closest obstacle', `${data.lidar.closest_m.toFixed(2)} m`],
        ['Mission queue', String(data.mission.queue_length)],
        ['Last map update', data.map.updated_at || 'none'],
      ];
      document.getElementById('statusFeed').innerHTML = statusItems.map(([k, v]) => kvLine(k, escapeHtml(v))).join('');

      if (data.map.image_stamp !== lastMapStamp && data.map.image_url) {
        lastMapStamp = data.map.image_stamp;
        const mapImage = document.getElementById('mapImage');
        mapImage.src = data.map.image_url + `?t=${Date.now()}`;
        document.getElementById('mapMeta').textContent = data.map.meta;
      }

      const logs = data.logs.slice(-12);
      if (logs.length !== lastLogCount) {
        lastLogCount = logs.length;
        document.getElementById('logFeed').innerHTML = logs.map((item) => `
          <div class="bubble">
            <div class="role">${escapeHtml(item.timestamp)} · ${escapeHtml(item.kind)}</div>
            <div class="body">${escapeHtml(item.text)}</div>
          </div>`).join('');
      }

      const chat = data.chat_history.slice(-12);
      if (chat.length !== lastChatCount) {
        lastChatCount = chat.length;
        document.getElementById('chatFeed').innerHTML = chat.map((item) => `
          <div class="bubble ${escapeHtml(item.role)}">
            <div class="role">${escapeHtml(item.role)}</div>
            <div class="body">${escapeHtml(item.message)}</div>
          </div>`).join('');
      }
    }

    async function sendMission() {
      const input = document.getElementById('missionInput');
      const command = input.value.trim();
      if (!command) return;
      await fetch('/api/mission', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({command})
      });
      input.value = '';
      await refreshStatus();
    }

    async function resetMission() {
      document.getElementById('missionInput').value = '';
    }

    async function sendChat() {
      const input = document.getElementById('chatInput');
      const message = input.value.trim();
      if (!message) return;
      input.value = '';
      await fetch('/api/chat', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({message})
      });
      await refreshStatus();
    }

    document.getElementById('chatInput').addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        sendChat();
      }
    });
    document.getElementById('missionInput').addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        sendMission();
      }
    });

    refreshStatus();
    setInterval(refreshStatus, 1200);
  </script>
</body>
</html>
"""


class DashboardNode(Node):
    def __init__(self):
        super().__init__("ridgeback_dashboard")

        self.declare_parameter("port", 8081)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("image_topic", "/r100_0140/image/compressed")
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("lidar_topic", "/r100_0140/sensors/lidar2d_0/scan")
        self.declare_parameter("battery_topic", "/r100_0140/platform/bms/state")
        self.declare_parameter("map_topic", "/map")

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        latch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(CompressedImage, self.get_parameter("image_topic").value, self._image_cb, sensor_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, sensor_qos)
        self.create_subscription(LaserScan, self.get_parameter("lidar_topic").value, self._lidar_cb, sensor_qos)
        self.create_subscription(BatteryState, self.get_parameter("battery_topic").value, self._battery_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self._map_cb, latch_qos)

        self.latest_frame: bytes | None = None
        self.latest_frame_stamp = ""
        self.frame_lock = threading.Lock()

        self.odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.lidar = {"closest_m": 99.0, "range_count": 0}
        self.battery = {"voltage": 0.0, "pct": 0.0}
        self.map_payload = {"image_bytes": None, "width": 0, "height": 0, "resolution": 0.0, "meta": "Waiting for map", "stamp": ""}

        self.logs: list[dict[str, str]] = []
        self.chat_history: list[dict[str, str]] = []
        self.mission_history: list[dict[str, str]] = []
        self.max_log_entries = 80
        self.max_chat_entries = 40
        self.memory = SpatialMemory()

        self.image_latency_ms = 0.0
        self.odom_latency_ms = 0.0
        self.vlm_client, self.vlm_config = build_vlm_client()
        self.connected = True

        self.get_logger().info(f"Dashboard ready on {self.vlm_config.base_url} / model {self.vlm_config.model_name}")

    def add_log(self, kind: str, text: str) -> None:
        self.logs.append({"timestamp": time.strftime("%H:%M:%S"), "kind": kind, "text": text})
        if len(self.logs) > self.max_log_entries:
            self.logs.pop(0)

    def add_chat(self, role: str, message: str) -> None:
        self.chat_history.append({"role": role, "message": message})
        if len(self.chat_history) > self.max_chat_entries:
            self.chat_history.pop(0)

    def _image_cb(self, msg: CompressedImage) -> None:
        try:
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.image_latency_ms = (now - stamp).nanoseconds / 1e6
        except Exception:
            pass
        with self.frame_lock:
            self.latest_frame = bytes(msg.data)
            self.latest_frame_stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

    def _odom_cb(self, msg: Odometry) -> None:
        try:
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.odom_latency_ms = (now - stamp).nanoseconds / 1e6
        except Exception:
            pass
        self.odom["x"] = msg.pose.pose.position.x
        self.odom["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom["yaw"] = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _lidar_cb(self, msg: LaserScan) -> None:
        finite_ranges = [value for value in msg.ranges if 0.05 < value < float("inf")]
        self.lidar["closest_m"] = min(finite_ranges) if finite_ranges else 99.0
        self.lidar["range_count"] = len(finite_ranges)

    def _battery_cb(self, msg: BatteryState) -> None:
        self.battery["voltage"] = msg.voltage
        self.battery["pct"] = msg.percentage * 100.0 if 0.0 <= msg.percentage <= 1.0 else msg.percentage

    def _map_cb(self, msg: OccupancyGrid) -> None:
        try:
            width = int(msg.info.width)
            height = int(msg.info.height)
            resolution = float(msg.info.resolution)
            data = np.array(msg.data, dtype=np.int16).reshape((height, width))

            image = np.full((height, width), 205, dtype=np.uint8)
            image[data == 0] = 255
            image[data == 100] = 0
            image = np.flipud(image)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            scale = max(2, min(5, 960 // max(width, 1)))
            image = cv2.resize(image, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)

            origin_x = float(msg.info.origin.position.x)
            origin_y = float(msg.info.origin.position.y)
            robot_px = int((self.odom["x"] - origin_x) / resolution)
            robot_py = int((self.odom["y"] - origin_y) / resolution)
            robot_py = height - robot_py - 1
            robot_x = max(0, min(width - 1, robot_px)) * scale + scale // 2
            robot_y = max(0, min(height - 1, robot_py)) * scale + scale // 2
            cv2.circle(image, (robot_x, robot_y), max(3, scale), (0, 0, 255), -1)
            cv2.line(
                image,
                (robot_x, robot_y),
                (int(robot_x + math.cos(math.radians(self.odom["yaw"])) * scale * 2), int(robot_y - math.sin(math.radians(self.odom["yaw"])) * scale * 2)),
                (0, 165, 255),
                2,
            )

            success, encoded = cv2.imencode(".png", image)
            if success:
                self.map_payload = {
                    "image_bytes": encoded.tobytes(),
                    "width": width,
                    "height": height,
                    "resolution": resolution,
                    "meta": f"{width} x {height} cells",
                    "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
                    "updated_at": time.strftime("%H:%M:%S"),
                }
        except Exception as exc:
            self.add_log("map", f"Failed to render map: {exc}")

    def latest_frame_copy(self) -> bytes | None:
        with self.frame_lock:
            return self.latest_frame

    def video_stream(self):
        while True:
            frame = self.latest_frame_copy()
            if frame is None:
                time.sleep(0.08)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.08)

    def status_payload(self) -> dict[str, Any]:
        map_payload = self.map_payload
        return {
            "connected": self.connected,
            "battery_pct": float(self.battery["pct"]),
            "battery_voltage": float(self.battery["voltage"]),
            "pose": {"x": float(self.odom["x"]), "y": float(self.odom["y"]), "yaw_deg": float(self.odom["yaw"])},
            "lidar": {"closest_m": float(self.lidar["closest_m"]), "range_count": int(self.lidar["range_count"])},
            "map": {
                "width": int(map_payload["width"]),
                "height": int(map_payload["height"]),
                "resolution": float(map_payload["resolution"]),
                "meta": map_payload["meta"],
                "image_stamp": map_payload["stamp"],
                "updated_at": map_payload.get("updated_at", ""),
                "image_url": "/api/map.png",
            },
            "latency": {"image_ms": float(self.image_latency_ms), "odom_ms": float(self.odom_latency_ms)},
            "mission": {"state": "IDLE", "command": self.mission_history[-1]["command"] if self.mission_history else "", "queue_length": len(self.mission_history)},
            "vlm": {"chat_count": len(self.chat_history), "log_count": len(self.logs)},
            "memory": {"location_count": len(self.memory.get_locations()), "mission_count": len(self.memory.get_recent_missions())},
            "logs": self.logs,
            "chat_history": self.chat_history,
        }


def create_app(node: DashboardNode) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        try:
            yield
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            spin_thread.join(timeout=2.0)

    app = FastAPI(lifespan=lifespan)

    @app.get("/", response_class=HTMLResponse)
    def index() -> HTMLResponse:
        return HTMLResponse(PAGE_HTML)

    @app.get("/api/status")
    def api_status() -> JSONResponse:
        return JSONResponse(node.status_payload())

    @app.get("/api/map.png")
    def api_map_png() -> Response:
        payload = node.map_payload.get("image_bytes")
        if payload is None:
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for SLAM map...", (18, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            success, encoded = cv2.imencode(".png", placeholder)
            payload = encoded.tobytes() if success else b""
        return Response(content=payload, media_type="image/png")

    @app.get("/video_feed")
    def video_feed() -> StreamingResponse:
        return StreamingResponse(node.video_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/api/mission")
    def api_mission(request: MissionRequest) -> JSONResponse:
        node.mission_history.append({"timestamp": time.strftime("%H:%M:%S"), "command": request.command})
        if len(node.mission_history) > 40:
            node.mission_history.pop(0)
        node.memory.record_mission(request.command)
        node.add_log("mission", request.command)
        return JSONResponse({"ok": True, "accepted": request.command})

    @app.post("/api/chat")
    def api_chat(request: ChatRequest) -> JSONResponse:
        node.add_chat("user", request.message)
        node.add_log("vlm", f"Chat query: {request.message}")
        try:
            response = node.vlm_client.chat.completions.create(
                model=node.vlm_config.model_name,
                messages=chat_completion_messages(
                    request.message,
                    system_prompt="You are a concise assistant for a Ridgeback R100 autonomous navigation dashboard. Answer in 1-5 sentences and be direct.",
                ),
                temperature=0.4,
                max_tokens=200,
                extra_body={"chat_template_kwargs": {"enable_thinking": node.vlm_config.enable_thinking}},
            )
            reply = response.choices[0].message.content or ""
            node.add_chat("assistant", reply)
            node.add_log("vlm", f"Reply: {reply[:180]}")
            return JSONResponse({"ok": True, "reply": reply})
        except Exception as exc:
            error = str(exc)
            node.add_chat("assistant", f"Error: {error}")
            node.add_log("vlm", f"Chat error: {error}")
            return JSONResponse({"ok": False, "error": error})

    return app


def main() -> None:
    rclpy.init()
    node = DashboardNode()
    app = create_app(node)
    port = int(node.get_parameter("port").value)
    host = str(node.get_parameter("host").value)
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()