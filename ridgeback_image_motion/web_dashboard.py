#!/usr/bin/env python3
"""Compact Jetson dashboard for Ridgeback autonomy and VLM chat."""

from __future__ import annotations

import base64
import math
import threading
import time
from contextlib import asynccontextmanager
from typing import Any

import cv2
import numpy as np
import rclpy
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import uvicorn

from ridgeback_image_motion.srv import Motion

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, json_loads, parse_intent_and_room
    from ridgeback_image_motion.spatial_memory import SpatialMemory
    from ridgeback_image_motion.vlm_client import build_vlm_client, chat_completion_messages
except ImportError:
    from autonomy_common import json_dumps, json_loads, parse_intent_and_room
    from spatial_memory import SpatialMemory
    from vlm_client import build_vlm_client, chat_completion_messages


class MissionRequest(BaseModel):
    command: str


class ChatRequest(BaseModel):
    message: str


class TeleopRequest(BaseModel):
    linear: float = 0.0
    lateral: float = 0.0
    angular: float = 0.0
    source: str = "keyboard"
    issued_at: float | None = None
    seq: int = 0


def extract_reasoning_trace(response: Any) -> str:
    """Best-effort extraction of model reasoning from OpenAI-compatible responses."""
    try:
        choice = response.choices[0]
        message = choice.message
    except Exception:
        return ""

    candidates: list[Any] = []
    for attr in ("reasoning_content", "thinking", "reasoning"):
        candidates.append(getattr(message, attr, None))

    # Some OpenAI-compatible servers place additional content in model_extra.
    model_extra = getattr(message, "model_extra", None)
    if isinstance(model_extra, dict):
        for key in ("reasoning_content", "thinking", "reasoning"):
            candidates.append(model_extra.get(key))

    for value in candidates:
        if isinstance(value, str) and value.strip():
            return value.strip()

    return ""


PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Ridgeback Autonomy Dashboard</title>
  <style>
    :root {
      --bg: #0b1020; --panel: #121a31; --panel2: #17213d;
      --border: rgba(167,189,255,0.16); --text: #e8eeff; --muted: #a2b0d4;
      --accent: #7ee787; --accent2: #7ab7ff; --warn: #ffcc66; --danger: #ff7f87;
      --orange: #f7941d;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body { width: 100%; height: 100%; overflow: hidden;
      background: radial-gradient(circle at top left, rgba(122,183,255,0.14), transparent 30%),
                  radial-gradient(circle at bottom right, rgba(126,231,135,0.07), transparent 24%),
                  var(--bg);
      color: var(--text); font-family: 'Courier New', monospace; }
    .shell { height: 100vh; display: flex; flex-direction: column; gap: 0; }
    .header { background: rgba(0,0,0,0.5); border-bottom: 2px solid var(--orange);
      padding: 7px 16px; display: flex; align-items: center; justify-content: space-between; flex-shrink: 0; }
    .header h1 { color: var(--orange); font-size: 1.05rem; letter-spacing: 2px; }
    .header .sub { color: var(--muted); font-size: 0.70rem; }
    .hdr-right { display: flex; align-items: center; gap: 12px; }
    .conn-badge { font-size: 0.70rem; }

    .grid { flex: 1; min-height: 0; display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      grid-template-rows: minmax(0, 1fr) minmax(0, 1fr);
      gap: 8px; padding: 8px; overflow: hidden; }
    .panel { background: linear-gradient(180deg, rgba(255,255,255,0.03), transparent),
                          var(--panel);
      border: 1px solid var(--border); border-radius: 12px;
      padding: 10px; min-height: 0; display: flex; flex-direction: column; overflow: hidden; }
    .panel h2 { color: var(--orange); font-size: 0.80rem; letter-spacing: 1px;
      margin-bottom: 8px; border-bottom: 1px solid rgba(247,148,29,0.3);
      padding-bottom: 5px; flex-shrink: 0; }

    #p-rgb   { grid-column:1; grid-row:1; position:relative; }
    #p-depth { grid-column:2; grid-row:1; position:relative; }
    #p-lidar { grid-column:3; grid-row:1; }
    #p-status  { grid-column:1; grid-row:2; }
    #p-vlmlog  { grid-column:2; grid-row:2; }
    #p-control { grid-column:3; grid-row:2; }

    .feed-img { width:100%; flex:1; min-height:0; object-fit:contain;
      border-radius:8px; display:block; max-height:calc(100% - 60px); }
    .no-signal { display:none; position:absolute; top:36px; left:10px; right:10px;
      background:rgba(237,28,36,0.85); color:#fff; font-size:0.72rem;
      padding:5px 8px; border-radius:6px; text-align:center; z-index:2; }
    .depth-legend { display:flex; align-items:center; gap:6px; font-size:0.64rem; margin-top:4px; flex-shrink:0; }
    .depth-grad { flex:1; height:5px; border-radius:3px;
      background:linear-gradient(to right,#4fc3f7,#00ff7f,#ffff00,#ff5252); }
    #depth-age { color:var(--muted); }

    #lidar-canvas { display:block; width:100%; aspect-ratio:1/1; border-radius:8px;
      background:#080d18; max-height:calc(100% - 74px); }
    .map-img { display:block; width:100%; flex:1; min-height:0; object-fit:contain;
      border-radius:8px; background:#080d18; max-height:calc(100% - 74px); }
    .lidar-info { display:flex; gap:10px; margin-top:5px; font-size:0.70rem; color:var(--muted); flex-shrink:0; }
    .lidar-info span { color:var(--accent2); }

    .btn { padding:6px 12px; border:none; border-radius:8px; cursor:pointer;
      font-family:'Courier New',monospace; font-size:0.75rem; transition:all 0.18s; }
    .btn-go  { background:linear-gradient(135deg,var(--accent2),#5f8cff); color:#08101f; font-weight:700; }
    .btn-go:hover { background:linear-gradient(135deg,#8fc8ff,#7ab7ff); }
    .btn-stop { background:var(--danger); color:#08101f; font-weight:700; }
    .btn-stop:hover { background:#ff9ea4; }
    .btn-sec { background:var(--panel2); color:var(--text); border:1px solid var(--border); }
    .btn-sec:hover { background:rgba(255,255,255,0.12); }
    .btn-xs { padding:2px 6px; font-size:0.62rem; }

    #p-status, #p-control { display:flex; flex-direction:column; }
    .tab-bar { display:flex; align-items:center; gap:6px; margin-bottom:8px;
      border-bottom:1px solid rgba(247,148,29,0.3); padding-bottom:5px; flex-shrink:0; }
    .tab-bar h2 { flex:1; border:none; margin:0; padding:0; }
    .tab-btn { padding:3px 9px; font-family:'Courier New',monospace; font-size:0.66rem;
      background:rgba(255,255,255,0.07); border:1px solid rgba(255,255,255,0.18);
      border-radius:6px; color:var(--muted); cursor:pointer; transition:all 0.15s; }
    .tab-btn:hover { background:rgba(247,148,29,0.2); color:var(--orange); }
    .tab-btn.on { background:rgba(247,148,29,0.22); border-color:var(--orange); color:var(--orange); }
    .pane { flex:1; min-height:0; display:flex; flex-direction:column; overflow:hidden; }

    .ss { border-bottom:1px solid rgba(255,255,255,0.07); padding:5px 0; }
    .sr { display:flex; align-items:center; gap:5px; font-size:0.68rem; flex-wrap:wrap; margin-bottom:2px; }
    .sl { color:var(--muted); font-size:0.61rem; text-transform:uppercase; }
    .sv { color:var(--accent2); font-size:0.73rem; }
    .lv { color:var(--orange); font-size:0.70rem; }
    .batt-wrap { flex:1; height:7px; background:rgba(255,255,255,0.1); border-radius:4px; overflow:hidden; }
    .batt-bar  { height:100%; border-radius:4px; transition:width 1s,background 1s; }
    .mlog { overflow-x:auto; white-space:nowrap; font-size:0.63rem; color:#9ad2ff;
      border:1px solid rgba(79,195,247,0.25); border-radius:4px; padding:3px 5px; margin-top:3px; }

    .list-wrap { flex:1; min-height:0; overflow-y:auto; border:1px solid var(--border); border-radius:8px; padding:6px; background:rgba(0,0,0,0.18); }
    .mono-line { font-size:0.65rem; color:#d8e7ff; border-bottom:1px dashed rgba(255,255,255,0.08); padding:4px 0; }
    .mono-line:last-child { border-bottom:none; }
    .small-note { color:var(--muted); font-size:0.62rem; margin-top:6px; }

    #vlm-log { flex:1; min-height:0; overflow-y:auto; font-size:0.68rem; }
    .vlm-filter { display:flex; gap:5px; }
    .bubble { background:rgba(255,255,255,0.04); border:1px solid var(--border);
      border-radius:10px; padding:8px 10px; margin-bottom:6px; }
    .bubble .role { color:var(--accent2); font-size:0.60rem; text-transform:uppercase;
      letter-spacing:.10em; margin-bottom:3px; }
    .bubble .body { white-space:pre-wrap; line-height:1.3; font-size:0.68rem; }

    .kbd-hint { font-size:0.63rem; color:var(--muted); margin-bottom:5px; flex-shrink:0; }
    .dpad { display:grid; grid-template-columns:repeat(3,clamp(38px,5vh,54px));
      grid-template-rows:repeat(3,clamp(38px,5vh,54px)); gap:4px; margin:4px auto;
      width:fit-content; flex-shrink:0; }
    .dk { width:clamp(38px,5vh,54px); height:clamp(38px,5vh,54px);
      background:linear-gradient(160deg,rgba(122,183,255,0.15),rgba(255,255,255,0.06));
      border:1px solid rgba(255,255,255,0.22); border-radius:9px; color:var(--text);
      cursor:pointer; font-size:1.05rem; display:flex; align-items:center;
      justify-content:center; transition:all 0.1s; user-select:none; }
    .dk:hover { background:rgba(122,183,255,0.28); border-color:var(--accent2); }
    .dk.active { background:var(--accent2); color:#08101f; border-color:#8fc8ff; }
    .dk-stop { background:rgba(237,28,36,0.25); border-color:var(--danger); }
    .spd { display:flex; gap:10px; margin-top:5px; font-size:0.70rem; flex-shrink:0; }
    .spd label { display:flex; flex-direction:column; gap:2px; }
    .spd input[type=range] { width:85px; }
    .toggles { margin-top:8px; display:flex; flex-direction:column; gap:4px; }
    .toggles label { font-size:0.64rem; color:var(--muted); display:flex; align-items:center; gap:6px; }

    .chat-hist { flex:1; min-height:0; overflow-y:auto; font-size:0.68rem;
      border:1px solid var(--border); border-radius:8px; padding:6px; margin-bottom:7px;
      background:rgba(0,0,0,0.18); }
    .chat-row { display:flex; gap:6px; flex-shrink:0; }
    .chat-row input { flex:1; background:var(--panel2); border:1px solid rgba(247,148,29,0.45);
      color:var(--text); padding:6px 9px; border-radius:8px; font-family:'Courier New',monospace;
      font-size:0.74rem; outline:none; }
    .chat-row input::placeholder { color:rgba(162,176,212,0.5); }
    .cu { color:var(--orange); margin-bottom:2px; }
    .ca { color:var(--accent2); margin-bottom:7px; }
    .ct { color:rgba(162,176,212,0.5); font-size:0.61rem; }

    .mission-pane { display:flex; flex-direction:column; gap:7px; }
    .mission-row { display:flex; gap:6px; }
    .mission-row input { flex:1; background:var(--panel2); border:1px solid rgba(247,148,29,0.45);
      color:var(--text); padding:6px 9px; border-radius:8px; font-family:'Courier New',monospace;
      font-size:0.72rem; outline:none; }
    .chip-row { display:flex; gap:6px; flex-wrap:wrap; }
    .chip { padding:5px 8px; border-radius:999px; border:1px solid rgba(255,255,255,0.2); background:rgba(255,255,255,0.06); color:var(--text); font-size:0.63rem; cursor:pointer; }
    .chip:hover { border-color:var(--orange); color:var(--orange); }

    @media (max-width:900px) {
      html,body { overflow:auto; }
      .grid { grid-template-columns:1fr; grid-template-rows:auto; overflow:visible; }
      #p-rgb,#p-depth,#p-lidar,#p-status,#p-vlmlog,#p-control { grid-column:1; grid-row:auto; min-height:280px; }
      .feed-img { max-height:200px; }
      #lidar-canvas, .map-img { max-height:200px; }
    }
  </style>
</head>
<body>
<div class="shell">
  <div class="header">
    <div>
      <h1>RIDGEBACK R100 — AUTONOMY DASHBOARD</h1>
      <div class="sub">r100-0140 · ROS2 Humble · SLAM + VLM</div>
    </div>
    <div class="hdr-right">
      <span id="conn-badge" class="conn-badge" style="color:var(--accent)">● CONNECTED</span>
    </div>
  </div>

  <div class="grid">

    <!-- Box 1: RGB Camera -->
    <div class="panel" id="p-rgb">
      <h2>RGB CAMERA</h2>
      <img class="feed-img" id="rgb-img" src="/video_feed" alt="Camera">
      <div class="no-signal" id="rgb-nosig">NO CAMERA SIGNAL</div>
    </div>

    <!-- Box 2: Depth Camera -->
    <div class="panel" id="p-depth">
      <h2>DEPTH CAMERA</h2>
      <img class="feed-img" id="depth-img" src="/depth_feed" alt="Depth">
      <div class="no-signal" id="depth-nosig">NO DEPTH SIGNAL</div>
      <div class="depth-legend">
        <span style="color:var(--accent2)">NEAR</span>
        <div class="depth-grad"></div>
        <span style="color:#ff5252">FAR 5m</span>
        <span id="depth-age"></span>
      </div>
    </div>

    <!-- Box 3: Switchable LiDAR / SLAM Map -->
    <div class="panel" id="p-lidar">
      <div class="tab-bar">
        <h2>WORLD VIEW</h2>
        <button id="view-tab-lidar" class="tab-btn on" onclick="switchWorldView('lidar')">LIDAR</button>
        <button id="view-tab-map" class="tab-btn" onclick="switchWorldView('map')">SLAM MAP</button>
      </div>
      <canvas id="lidar-canvas" width="600" height="600"></canvas>
      <img id="slam-map-img" class="map-img" src="/api/map.png" alt="SLAM Map" style="display:none">
      <div class="lidar-info">
        Closest: <span id="lid-closest">--</span>m &nbsp; Pts: <span id="lid-pts">--</span>
        &nbsp; Map: <span id="map-meta">WAITING</span>
      </div>
    </div>

    <!-- Box 4: System Status -->
    <div class="panel" id="p-status">
      <div class="tab-bar">
        <h2>SYSTEM STATUS</h2>
        <button id="st-tab-ops" class="tab-btn on" onclick="switchStatusTab('ops')">OPS+SAFETY</button>
        <button id="st-tab-mm" class="tab-btn" onclick="switchStatusTab('mm')">MISSION+MEM</button>
      </div>

      <div class="pane" id="status-pane-ops">
        <div class="ss">
          <div class="sr">
            <span class="sl">BATTERY</span>
            <div class="batt-wrap"><div id="batt-bar" class="batt-bar" style="width:0%;background:var(--accent)"></div></div>
            <span id="batt-pct" class="sv">--%</span>
            <span id="batt-v" class="sv">--V</span>
          </div>
        </div>
        <div class="ss">
          <div class="sr">
            X:<span id="p-x" class="sv">--</span>m
            Y:<span id="p-y" class="sv">--</span>m
            YAW:<span id="p-yaw" class="sv">--</span>°
          </div>
          <div class="sr">
            <span class="sl">AGE</span>
            IMG<span id="lat-img" class="lv">--</span>
            ODO<span id="lat-odo" class="lv">--</span>
          </div>
        </div>
        <div class="ss">
          <div class="sr">
            <span class="sl">LIDAR</span>
            Closest <span id="st-lid" class="sv">--</span>m
            &nbsp; Pts <span id="st-pts" class="sv">--</span>
          </div>
          <div class="sr">
            <span class="sl">CAM</span><span id="st-cam" class="sv">--</span>
            &nbsp;<span class="sl">MAP</span><span id="st-map" class="sv">--</span>
          </div>
        </div>
        <div class="ss">
          <div class="sr">
            <span class="sl">SAFETY</span>
            Risk <span id="st-risk" class="sv">UNKNOWN</span>
            &nbsp; Stop Rec <span id="st-stop-rec" class="sv">NO</span>
          </div>
          <div class="sr">
            <span class="sl">MODE</span><span id="st-teleop" class="sv">idle</span>
            &nbsp;<span class="sl">SOURCE</span><span id="st-source" class="sv">none</span>
          </div>
          <div class="sr">
            <span class="sl">BLOCK</span><span id="st-safety-reasons" class="sv">none</span>
          </div>
        </div>
        <div id="st-log" class="mlog">--</div>
      </div>

      <div class="pane" id="status-pane-mm" style="display:none">
        <div class="ss">
          <div class="sr">
            <span class="sl">MISSION</span>
            <span id="mm-intent" class="sv">--</span>
            <span id="mm-room" class="sv">--</span>
          </div>
          <div class="sr">
            <span class="sl">LAST CMD</span>
            <span id="mm-command" class="sv">--</span>
          </div>
        </div>

        <div class="ss" style="padding-bottom:8px">
          <div class="sr">
            <span class="sl">RECENT MISSIONS</span>
            <span id="mm-mission-count" class="sv">0</span>
          </div>
          <div id="mm-missions" class="list-wrap"></div>
        </div>

        <div class="ss" style="padding-bottom:8px">
          <div class="sr">
            <span class="sl">MEMORY LOCATIONS</span>
            <span id="mm-loc-count" class="sv">0</span>
          </div>
          <div id="mm-locs" class="list-wrap"></div>
        </div>

        <div class="small-note">Mission status here reflects dashboard-level mission queue and memory, not Nav2 mission ground truth.</div>
      </div>
    </div>

    <!-- Box 5: VLM Logs -->
    <div class="panel" id="p-vlmlog">
      <div class="tab-bar">
        <h2>VLM LOGS</h2>
        <div class="vlm-filter">
          <button id="vlm-f-all" class="tab-btn on" onclick="setVlmFilter('all')">ALL</button>
          <button id="vlm-f-prompt" class="tab-btn" onclick="setVlmFilter('prompt')">PROMPT</button>
          <button id="vlm-f-mission" class="tab-btn" onclick="setVlmFilter('mission')">MISSION</button>
          <button id="vlm-f-system" class="tab-btn" onclick="setVlmFilter('system')">SYSTEM</button>
        </div>
      </div>
      <div id="vlm-log"></div>
    </div>

    <!-- Box 6: Switchable Control -->
    <div class="panel" id="p-control">
      <div class="tab-bar">
        <h2>CONTROL</h2>
        <button id="tab-teleop"  class="tab-btn on"  onclick="switchControlTab('teleop')">TELEOP</button>
        <button id="tab-prompt"  class="tab-btn"     onclick="switchControlTab('prompt')">VLM PROMPT</button>
        <button id="tab-mission" class="tab-btn"     onclick="switchControlTab('mission')">MISSION</button>
      </div>

      <!-- Teleop pane -->
      <div class="pane" id="pane-teleop">
        <div class="kbd-hint">W/A/S/D = move &nbsp; Q/E = rotate &nbsp; Space = stop</div>
        <div class="dpad">
          <button class="dk" id="dk-fl" data-lin="1"  data-lat="1"  data-ang="0">↖</button>
          <button class="dk" id="dk-f"  data-lin="1"  data-lat="0"  data-ang="0">↑</button>
          <button class="dk" id="dk-fr" data-lin="1"  data-lat="-1" data-ang="0">↗</button>
          <button class="dk" id="dk-sl" data-lin="0"  data-lat="1"  data-ang="0">←</button>
          <button class="dk dk-stop"    onclick="stopRobot()">■</button>
          <button class="dk" id="dk-sr" data-lin="0"  data-lat="-1" data-ang="0">→</button>
          <button class="dk" id="dk-bl" data-lin="-1" data-lat="1"  data-ang="0">↙</button>
          <button class="dk" id="dk-b"  data-lin="-1" data-lat="0"  data-ang="0">↓</button>
          <button class="dk" id="dk-br" data-lin="-1" data-lat="-1" data-ang="0">↘</button>
        </div>
        <div style="display:flex;gap:8px;justify-content:center;margin-top:4px">
          <button class="dk" id="dk-ccw" data-lin="0" data-lat="0" data-ang="1">↺</button>
          <button class="dk" id="dk-cw"  data-lin="0" data-lat="0" data-ang="-1">↻</button>
        </div>
        <div class="spd">
          <label>Lin <span id="lbl-lin">0.28</span>m/s
            <input type="range" id="spd-lin" min="0.05" max="0.5" step="0.05" value="0.28"
                   oninput="document.getElementById('lbl-lin').textContent=parseFloat(this.value).toFixed(2)">
          </label>
          <label>Ang <span id="lbl-ang">0.85</span>r/s
            <input type="range" id="spd-ang" min="0.1" max="1.5" step="0.1" value="0.85"
                   oninput="document.getElementById('lbl-ang').textContent=parseFloat(this.value).toFixed(2)">
          </label>
        </div>
        <div class="toggles">
          <label><input id="tg-kbd" type="checkbox" checked> Enable keyboard teleop</label>
          <label><input id="tg-stop" type="checkbox" checked> Auto-stop on blur/tab hidden</label>
        </div>
        <button class="btn btn-stop" style="width:100%;margin-top:6px" onclick="stopRobot()">■ STOP</button>
      </div>

      <!-- VLM Prompt pane -->
      <div class="pane" id="pane-prompt" style="display:none">
        <div id="chat-hist" class="chat-hist"></div>
        <div class="chat-row">
          <input id="chat-input" type="text" placeholder='"what do you see" or "go to room 202 and return"'>
          <button class="btn btn-go" onclick="sendChat()">SEND</button>
        </div>
      </div>

      <!-- Mission pane -->
      <div class="pane" id="pane-mission" style="display:none">
        <div class="mission-pane">
          <div class="mission-row">
            <input id="mission-room" type="text" placeholder="Room number, e.g. 202">
            <button class="btn btn-go" onclick="sendGoRoom()">GO</button>
          </div>
          <div class="chip-row">
            <button class="chip" onclick="sendMissionCommand('go to room '+roomField()+' and return to start')">Go + Return</button>
            <button class="chip" onclick="sendMissionCommand('return to start')">Return to Start</button>
            <button class="chip" onclick="sendMissionCommand('stop mission')">Stop Mission</button>
            <button class="chip" onclick="sendMissionCommand('what do you see')">What do you see</button>
          </div>
          <div class="small-note">Natural language mission commands are queued and tracked in Box 4 Mission+Mem.</div>
        </div>
      </div>
    </div>

  </div>
</div>

<script>
const pressedKeys = new Set();
let teleopTimer = null;
let btnTeleopTimer = null;
let activeDk = null;
let teleopSpeeds = {linear: 0.28, lateral: 0.28, angular: 0.85};
let allowKeyboardTeleop = true;
let stopOnBlur = true;
let statusTab = 'ops';
let controlTab = 'teleop';
let vlmFilter = 'all';
let lastLogCount = 0;
let lastChatCount = 0;
let latestVlmEvents = [];
let worldView = 'lidar';
let latestMapUrl = '/api/map.png';
let serverClockOffsetMs = 0;
let teleopSeq = 0;
let teleopInFlight = false;
let pendingTeleop = null;

function updateServerClock(serverTimeSec) {
  const value = Number(serverTimeSec);
  if (!Number.isFinite(value) || value <= 0) return;
  serverClockOffsetMs = value * 1000 - Date.now();
}

function switchStatusTab(tab) {
  statusTab = tab;
  ['ops', 'mm'].forEach(t => {
    document.getElementById('status-pane-'+t).style.display = t===tab ? 'flex' : 'none';
    document.getElementById('st-tab-'+t).classList.toggle('on', t===tab);
  });
}

function switchControlTab(tab) {
  controlTab = tab;
  ['teleop','prompt', 'mission'].forEach(t => {
    document.getElementById('pane-'+t).style.display = t===tab ? 'flex' : 'none';
    document.getElementById('tab-'+t).classList.toggle('on', t===tab);
  });
}

function switchWorldView(view) {
  worldView = view;
  const lidar = document.getElementById('lidar-canvas');
  const map = document.getElementById('slam-map-img');
  if (lidar) lidar.style.display = view === 'lidar' ? 'block' : 'none';
  if (map) map.style.display = view === 'map' ? 'block' : 'none';
  ['lidar', 'map'].forEach(t => {
    const btn = document.getElementById('view-tab-' + t);
    if (btn) btn.classList.toggle('on', t === view);
  });
  if (view === 'map') updateMapView(true);
  if (view === 'lidar') updateLidar();
}

document.querySelectorAll('.dk[data-lin]').forEach(btn => {
  btn.addEventListener('click', () => toggleDk(btn));
});

function setVlmFilter(kind) {
  vlmFilter = kind;
  ['all', 'prompt', 'mission', 'system'].forEach(t => {
    document.getElementById('vlm-f-'+t).classList.toggle('on', t === kind);
  });
  renderVlmTimeline();
}

function toggleDk(btn) {
  if (teleopTimer) return;
  if (activeDk === btn) { stopRobot(); return; }
  if (activeDk) activeDk.classList.remove('active');
  activeDk = btn; btn.classList.add('active');
  clearInterval(btnTeleopTimer);
  sendDk(btn);
  btnTeleopTimer = setInterval(() => sendDk(btn), 100);
}

function sendDk(btn) {
  const lin = parseFloat(btn.dataset.lin);
  const lat = parseFloat(btn.dataset.lat);
  const ang = parseFloat(btn.dataset.ang);
  const ls = parseFloat(document.getElementById('spd-lin').value);
  const as = parseFloat(document.getElementById('spd-ang').value);
  sendTeleop(lin*ls, lat*ls, ang*as, 'button');
}

function stopRobot() {
  clearInterval(btnTeleopTimer); btnTeleopTimer = null;
  pressedKeys.clear();
  if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
  if (activeDk) { activeDk.classList.remove('active'); activeDk = null; }
  sendTeleop(0, 0, 0, 'stop');
}

const TKEYS = ['w','a','s','d','q','e'];

function kbdTick() {
  const ls = parseFloat(document.getElementById('spd-lin').value);
  const as = parseFloat(document.getElementById('spd-ang').value);
  let lin=0, lat=0, ang=0;
  if (pressedKeys.has('w')) lin =  ls;
  if (pressedKeys.has('s')) lin = -ls;
  if (pressedKeys.has('a')) lat =  ls;
  if (pressedKeys.has('d')) lat = -ls;
  if (pressedKeys.has('q')) ang =  as;
  if (pressedKeys.has('e')) ang = -as;
  sendTeleop(lin, lat, ang, 'keyboard');
}

function shouldIgnore(e) { const t=(e.target?.tagName||'').toLowerCase(); return t==='input'||t==='textarea'; }

window.addEventListener('keydown', e => {
  if (!allowKeyboardTeleop) return;
  if (e.repeat || shouldIgnore(e)) return;
  const k = e.key.toLowerCase();
  if (k === ' ') { e.preventDefault(); stopRobot(); return; }
  if (!TKEYS.includes(k)) return;
  e.preventDefault();
  pressedKeys.add(k);
  if (btnTeleopTimer) { clearInterval(btnTeleopTimer); btnTeleopTimer = null; }
  if (activeDk) { activeDk.classList.remove('active'); activeDk = null; }
  if (!teleopTimer) { kbdTick(); teleopTimer = setInterval(kbdTick, 100); }
});

window.addEventListener('keyup', e => {
  if (!allowKeyboardTeleop) return;
  if (shouldIgnore(e)) return;
  pressedKeys.delete(e.key.toLowerCase());
  if (pressedKeys.size === 0 && teleopTimer) {
    clearInterval(teleopTimer); teleopTimer = null;
    sendTeleop(0, 0, 0, 'keyboard_release');
  }
});

window.addEventListener('blur', () => {
  if (!stopOnBlur) return;
  pressedKeys.clear();
  if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; sendTeleop(0,0,0,'blur'); }
});

document.addEventListener('visibilitychange', () => {
  if (!stopOnBlur) return;
  if (document.visibilityState === 'hidden') {
    pressedKeys.clear();
    if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; sendTeleop(0,0,0,'blur'); }
  }
});

async function sendTeleop(linear, lateral, angular, source='keyboard') {
  pendingTeleop = {
    linear, lateral, angular, source,
    issued_at: (Date.now() + serverClockOffsetMs) / 1000.0,
    seq: ++teleopSeq
  };
  pumpTeleop();
}

async function pumpTeleop() {
  if (teleopInFlight || !pendingTeleop) return;
  const payload = pendingTeleop;
  pendingTeleop = null;
  teleopInFlight = true;
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 800);
  try {
    await fetch('/api/teleop', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(payload),
      signal: controller.signal
    });
  } catch(_) {
  } finally {
    clearTimeout(timeoutId);
    teleopInFlight = false;
    if (pendingTeleop) pumpTeleop();
  }
}

function escHtml(v) {
  return String(v).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;');
}

function roomField() {
  const value = document.getElementById('mission-room').value.trim();
  return value || '202';
}

async function sendMissionCommand(command) {
  await fetch('/api/mission', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({command})}).catch(()=>{});
  await refreshStatus();
}

async function sendGoRoom() {
  await sendMissionCommand('go to room ' + roomField());
}

function renderMissionList(items) {
  if (!items || !items.length) return '<div class="mono-line">No mission records yet.</div>';
  return items.slice(0, 12).map(item => {
    const intent = item.intent || 'UNKNOWN';
    const room = item.room ? ' room ' + item.room : '';
    const cmd = escHtml(item.command || '');
    return `<div class="mono-line">[${escHtml(item.timestamp || '--:--:--')}] ${escHtml(intent)}${escHtml(room)} :: ${cmd}</div>`;
  }).join('');
}

function renderLocationList(items) {
  if (!items || !items.length) return '<div class="mono-line">No stored locations yet.</div>';
  return items.slice(0, 12).map(item => {
    const label = item.label || item.room_number || 'location';
    const conf = Number(item.confidence ?? 0).toFixed(2);
    return `<div class="mono-line">${escHtml(label)} @ (${Number(item.x||0).toFixed(2)}, ${Number(item.y||0).toFixed(2)}) conf ${conf}</div>`;
  }).join('');
}

function renderVlmTimeline() {
  const list = document.getElementById('vlm-log');
  if (!list) return;
  const source = latestVlmEvents || [];
  const filtered = source.filter(item => {
    if (vlmFilter === 'all') return true;
    return (item.kind || '').toLowerCase() === vlmFilter;
  });
  if (!filtered.length) {
    list.innerHTML = '<div class="bubble"><div class="role">NO EVENTS</div><div class="body">No VLM timeline entries for this filter yet.</div></div>';
    return;
  }

  list.innerHTML = [...filtered].reverse().slice(0, 30).map(item => {
    const role = `${escHtml(item.timestamp || '--:--:--')} · ${escHtml(item.kind || 'event')} · ${escHtml(item.status || 'ok')}`;
    const prompt = item.prompt ? `PROMPT: ${item.prompt}\n` : '';
    const answer = item.answer ? `ANSWER: ${item.answer}\n` : '';
    const parse = item.intent ? `PARSED: ${item.intent}${item.room ? ' room ' + item.room : ''}\n` : '';
    const latency = Number(item.latency_ms || 0) > 0 ? `LATENCY: ${Number(item.latency_ms).toFixed(0)} ms\n` : '';
    const thought = item.thinking ? `THINKING:\n${item.thinking}` : '';
    const body = (prompt + answer + parse + latency + thought).trim() || item.text || '(empty)';
    return `<div class="bubble"><div class="role">${role}</div><div class="body">${escHtml(body)}</div></div>`;
  }).join('');
}

async function refreshStatus() {
  try {
    const [data, depthSt] = await Promise.all([
      fetch('/api/status').then(r => r.json()),
      fetch('/depth_status').then(r => r.json())
    ]);
    updateServerClock(data.server_time);
    teleopSpeeds = data.teleop?.speeds || teleopSpeeds;

    // Header connection
    const badge = document.getElementById('conn-badge');
    if (badge) { badge.textContent = data.connected ? '● CONNECTED' : '○ WAITING';
                 badge.style.color = data.connected ? 'var(--accent)' : 'var(--warn)'; }

    // Battery
    const pct = parseFloat(data.battery_pct ?? 0);
    const bar = document.getElementById('batt-bar');
    if (bar) { bar.style.width = pct.toFixed(0)+'%';
               bar.style.background = pct<20?'var(--danger)':pct<40?'var(--warn)':'var(--accent)'; }
    const e = id => document.getElementById(id);
    if (e('batt-pct')) e('batt-pct').textContent = pct.toFixed(1)+'%';
    if (e('batt-v'))   e('batt-v').textContent   = (data.battery_voltage??0).toFixed(1)+'V';

    // Pose / latency
    const pose = data.pose || {};
    if (e('p-x'))   e('p-x').textContent   = (pose.x??0).toFixed(2);
    if (e('p-y'))   e('p-y').textContent   = (pose.y??0).toFixed(2);
    if (e('p-yaw')) e('p-yaw').textContent = (pose.yaw_deg??0).toFixed(1);
    const imgAgeMs = data.feeds?.camera_age_ms ?? data.latency?.image_ms ?? 0;
    const odomAgeMs = data.feeds?.odom_age_ms ?? data.latency?.odom_ms ?? 0;
    if (e('lat-img')) e('lat-img').textContent = imgAgeMs.toFixed(0)+'ms';
    if (e('lat-odo')) e('lat-odo').textContent = odomAgeMs.toFixed(0)+'ms';

    // LiDAR
    if (e('st-lid')) e('st-lid').textContent = (data.lidar?.closest_m??0).toFixed(2);
    if (e('st-pts')) e('st-pts').textContent = (data.lidar?.range_count??0);

    // Camera / map connection
    const camOk = Boolean(data.feeds?.camera_alive);
    if (e('st-cam')) { e('st-cam').textContent = camOk ? 'OK' : 'LOST';
                       e('st-cam').style.color = camOk ? 'var(--accent)' : 'var(--danger)'; }
    const mapOk = data.map?.width > 0;
    if (e('st-map')) { e('st-map').textContent = mapOk ? data.map.meta : 'WAITING';
                       e('st-map').style.color = mapOk ? 'var(--accent2)' : 'var(--muted)'; }
    latestMapUrl = data.map?.image_url || '/api/map.png';
    if (e('map-meta')) e('map-meta').textContent = mapOk ? data.map.meta : 'WAITING';

    if (e('st-risk')) {
      e('st-risk').textContent = data.safety?.risk_level || 'UNKNOWN';
      const risk = (data.safety?.risk_level || '').toUpperCase();
      e('st-risk').style.color = risk === 'DANGER' ? 'var(--danger)' : (risk === 'WARNING' ? 'var(--warn)' : 'var(--accent)');
    }
    if (e('st-stop-rec')) e('st-stop-rec').textContent = data.safety?.stop_recommended ? 'YES' : 'NO';
    if (e('st-teleop')) e('st-teleop').textContent = data.teleop?.status || 'idle';
    if (e('st-source')) e('st-source').textContent = data.teleop?.mux?.source || data.teleop?.last?.source || 'none';
    if (e('st-safety-reasons')) {
      const reasons = Array.isArray(data.safety?.reasons) ? data.safety.reasons : [];
      e('st-safety-reasons').textContent = reasons.length ? reasons.slice(0, 3).join(', ') : 'none';
      e('st-safety-reasons').style.color = reasons.length ? 'var(--danger)' : 'var(--accent)';
    }

    if (e('st-log') && data.logs?.length) {
      const last = data.logs[data.logs.length-1];
      e('st-log').textContent = `[${last.timestamp}] ${last.kind}: ${last.text}`;
    }

    if (e('mm-intent')) e('mm-intent').textContent = data.mission?.last_intent || 'NONE';
    if (e('mm-room')) e('mm-room').textContent = data.mission?.last_room ? `ROOM ${data.mission.last_room}` : 'NO ROOM';
    if (e('mm-command')) e('mm-command').textContent = data.mission?.command || '--';
    if (e('mm-mission-count')) e('mm-mission-count').textContent = String(data.memory?.mission_count || 0);
    if (e('mm-loc-count')) e('mm-loc-count').textContent = String(data.memory?.location_count || 0);
    if (e('mm-missions')) e('mm-missions').innerHTML = renderMissionList(data.mission?.recent || []);
    if (e('mm-locs')) e('mm-locs').innerHTML = renderLocationList(data.memory?.recent_locations || []);

    const dno = document.getElementById('depth-nosig');
    if (dno) {
      dno.textContent = depthSt.enabled === false ? 'DEPTH DISABLED' : 'NO DEPTH SIGNAL';
      dno.style.display = (depthSt.enabled === false || !depthSt.has_frame || depthSt.age_s > 3) ? 'block' : 'none';
    }
    if (e('depth-age') && depthSt.age_s >= 0) e('depth-age').textContent = depthSt.age_s.toFixed(1)+'s';

    const cno = document.getElementById('rgb-nosig');
    if (cno) cno.style.display = !camOk ? 'block' : 'none';

    const logs = data.logs || [];
    if (logs.length !== lastLogCount) {
      lastLogCount = logs.length;
    }

    latestVlmEvents = data.vlm?.events || [];
    if (!latestVlmEvents.length && logs.length) {
      latestVlmEvents = logs.map(item => ({
        timestamp: item.timestamp,
        kind: item.kind,
        status: 'ok',
        text: item.text,
      }));
    }
    renderVlmTimeline();

    const chat = data.chat_history || [];
    if (chat.length !== lastChatCount) {
      lastChatCount = chat.length;
      const hist = document.getElementById('chat-hist');
      if (hist) {
        hist.innerHTML = chat.slice(-20).map(m =>
          `<div class="${m.role==='user'?'cu':'ca'}"><span class="ct">${m.role==='user'?'YOU':'ROBOT'}:</span> ${escHtml(m.message)}</div>`
        ).join('');
        hist.scrollTop = hist.scrollHeight;
      }
    }

  } catch(_) {}
}

const LIDAR_MAX = 4.0;

function syncCanvas(c) {
  const r = c.getBoundingClientRect();
  const s = Math.max(180, Math.floor(Math.min(r.width, r.height||r.width)));
  if (c.width!==s||c.height!==s) { c.width=s; c.height=s; }
}

async function updateLidar() {
  if (worldView !== 'lidar') return;
  try {
    const d = await fetch('/lidar_scan').then(r => r.json());
    if (!d.ranges || !d.ranges.length) return;
    const c = document.getElementById('lidar-canvas');
    if (!c) return;
    syncCanvas(c);
    const ctx = c.getContext('2d');
    const w=c.width, h=c.height, cx=w/2, cy=h/2;
    const rMax = Math.min(d.range_max||10, LIDAR_MAX);
    const sc = Math.min(w,h)/(2*(rMax+0.25));

    ctx.fillStyle='#080d18'; ctx.fillRect(0,0,w,h);

    ctx.strokeStyle='#132213'; ctx.lineWidth=1; ctx.font='10px monospace'; ctx.fillStyle='#1a3a1a';
    for (let r=1;r<=rMax;r++) {
      const pr=r*sc; ctx.beginPath(); ctx.arc(cx,cy,pr,0,Math.PI*2); ctx.stroke();
      ctx.fillText(r+'m',cx+pr+2,cy-2);
    }
    ctx.strokeStyle='#0d2a0d'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(cx,cy-rMax*sc-8); ctx.lineTo(cx,cy+rMax*sc+8); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx-rMax*sc-8,cy); ctx.lineTo(cx+rMax*sc+8,cy); ctx.stroke();
    ctx.fillStyle='#1a5a1a'; ctx.font='9px monospace';
    ctx.fillText('FWD',cx+3,cy-rMax*sc-2);

    let valid=0, closest=Infinity;
    const aMin=d.angle_min, aInc=d.angle_increment;
    for (let i=0;i<d.ranges.length;i++) {
      const range=d.ranges[i];
      if (!isFinite(range)||range<0.05||range>rMax) continue;
      valid++; if (range<closest) closest=range;
      const a=aMin+i*aInc;
      const px=cx-range*Math.sin(a)*sc;
      const py=cy-range*Math.cos(a)*sc;
      const t=Math.min(range/rMax,1);
      let r,g,b;
      if (t<0.33){r=255;g=Math.floor(t*3*255);b=0;}
      else if(t<0.66){r=Math.floor((1-(t-0.33)*3)*255);g=255;b=0;}
      else{r=0;g=255;b=Math.floor((t-0.66)*3*255);}
      ctx.fillStyle=`rgb(${r},${g},${b})`;
      ctx.beginPath(); ctx.arc(px,py,2,0,Math.PI*2); ctx.fill();
    }
    ctx.beginPath(); ctx.arc(cx,cy,6,0,Math.PI*2);
    ctx.fillStyle='var(--orange)'; ctx.fill();
    ctx.strokeStyle='#ffc107'; ctx.lineWidth=2; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx,cy-24); ctx.lineTo(cx-6,cy-13); ctx.lineTo(cx+6,cy-13);
    ctx.closePath(); ctx.fillStyle='#ffc107'; ctx.fill();

    const el=document.getElementById('lid-closest');
    const ep=document.getElementById('lid-pts');
    if (el) el.textContent=isFinite(closest)?closest.toFixed(2):'--';
    if (ep) ep.textContent=valid;
  } catch(_) {}
}

function updateMapView(force=false) {
  if (worldView !== 'map' && !force) return;
  const img = document.getElementById('slam-map-img');
  if (!img) return;
  const base = latestMapUrl || '/api/map.png';
  img.src = `${base}${base.includes('?') ? '&' : '?'}ts=${Date.now()}`;
}

async function sendChat() {
  const input = document.getElementById('chat-input');
  const message = input.value.trim();
  if (!message) return;
  input.value = '';
  await fetch('/api/chat', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({message})}).catch(()=>{});
  await refreshStatus();
}

async function sendHeartbeat() {
  if (document.hidden) return;
  const data = await fetch('/api/heartbeat', {method:'POST'}).then(r => r.json()).catch(()=>null);
  if (data?.stamp) updateServerClock(data.stamp);
}

document.addEventListener('DOMContentLoaded', () => {
  const ci = document.getElementById('chat-input');
  if (ci) ci.addEventListener('keydown', e => { if(e.key==='Enter') sendChat(); });

  const mk = document.getElementById('mission-room');
  if (mk) mk.addEventListener('keydown', e => {
    if (e.key === 'Enter') sendGoRoom();
  });

  const tgKbd = document.getElementById('tg-kbd');
  const tgStop = document.getElementById('tg-stop');
  if (tgKbd) tgKbd.addEventListener('change', () => {
    allowKeyboardTeleop = !!tgKbd.checked;
    if (!allowKeyboardTeleop) {
      pressedKeys.clear();
      if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
      sendTeleop(0, 0, 0, 'keyboard_disabled');
    }
  });
  if (tgStop) tgStop.addEventListener('change', () => {
    stopOnBlur = !!tgStop.checked;
  });
});

document.addEventListener('visibilitychange', sendHeartbeat);
setInterval(refreshStatus, 1500);
setInterval(updateLidar, 500);
setInterval(updateMapView, 2000);
setInterval(sendHeartbeat, 500);

sendHeartbeat();
refreshStatus();
updateLidar();
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
        self.declare_parameter("raw_image_topic", "")
        self.declare_parameter("depth_compressed_topic", "/r100_0140/image/depth_compressed")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("enable_depth_feed", True)
        self.declare_parameter("rgb_stream_hz", 8.0)
        self.declare_parameter("depth_render_hz", 4.0)
        self.declare_parameter("map_render_hz", 1.0)
        self.declare_parameter("map_target_long_side", 1000)
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("lidar_topic", "/r100_0140/sensors/lidar2d_0/scan")
        self.declare_parameter("battery_topic", "/r100_0140/platform/bms/state")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("motion_service", "motion_service")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_teleop")
        self.declare_parameter("mission_command_topic", "/ridgeback/mission/command")
        self.declare_parameter("operator_heartbeat_topic", "/pc_heartbeat")
        self.declare_parameter("mission_status_topic", "/ridgeback/mission/status")
        self.declare_parameter("safety_status_topic", "/ridgeback/safety/status")
        self.declare_parameter("vlm_status_topic", "/ridgeback/vlm/status")
        self.declare_parameter("mux_status_topic", "/ridgeback/cmd_vel_mux/status")
        self.declare_parameter("teleop_linear_speed", 0.28)
        self.declare_parameter("teleop_lateral_speed", 0.28)
        self.declare_parameter("teleop_angular_speed", 0.85)
        self.declare_parameter("teleop_command_max_age_s", 0.75)
        self.declare_parameter("auto_raw_camera_fallback", True)
        self.declare_parameter("raw_fallback_after_s", 8.0)
        self.declare_parameter("fallback_raw_image_topic", "/r100_0140/sensors/camera_0/color/image")
        self.declare_parameter("fallback_depth_topic", "/r100_0140/sensors/camera_0/depth/image")

        self._sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        sensor_qos = self._sensor_qos
        # BEST_EFFORT matches the image_publisher republisher and avoids
        # reliable-retransmit stalls over WiFi for the compressed RGB/depth feeds.
        self._compressed_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        compressed_qos = self._compressed_qos
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        image_topic = str(self.get_parameter("image_topic").value).strip()
        raw_image_topic = str(self.get_parameter("raw_image_topic").value).strip()
        depth_compressed_topic = str(self.get_parameter("depth_compressed_topic").value).strip()
        depth_topic = str(self.get_parameter("depth_topic").value).strip()
        self.enable_depth_feed = bool(self.get_parameter("enable_depth_feed").value)
        self.rgb_stream_hz = max(1.0, float(self.get_parameter("rgb_stream_hz").value))
        self.depth_render_hz = max(0.5, float(self.get_parameter("depth_render_hz").value))
        self.map_render_hz = max(0.1, float(self.get_parameter("map_render_hz").value))
        self.map_target_long_side = max(320, int(self.get_parameter("map_target_long_side").value))
        self.auto_raw_camera_fallback = bool(self.get_parameter("auto_raw_camera_fallback").value)
        self.raw_fallback_after_s = max(2.0, float(self.get_parameter("raw_fallback_after_s").value))
        self.fallback_raw_image_topic = str(self.get_parameter("fallback_raw_image_topic").value).strip()
        self.fallback_depth_topic = str(self.get_parameter("fallback_depth_topic").value).strip()
        self.started_at = time.time()
        self._fallback_attempted = {
            "raw_image": False,
            "depth_raw": False,
        }

        self._dashboard_subscriptions = {}
        self.subscription_topics = {
            "image": image_topic,
            "raw_image": raw_image_topic,
            "depth_compressed": depth_compressed_topic if self.enable_depth_feed else "",
            "depth_raw": depth_topic if self.enable_depth_feed else "",
            "fallback_raw_image": self.fallback_raw_image_topic if self.auto_raw_camera_fallback else "",
            "fallback_depth_raw": self.fallback_depth_topic if (self.auto_raw_camera_fallback and self.enable_depth_feed) else "",
            "odom": str(self.get_parameter("odom_topic").value),
            "lidar": str(self.get_parameter("lidar_topic").value),
            "battery": str(self.get_parameter("battery_topic").value),
            "map": str(self.get_parameter("map_topic").value),
        }

        if image_topic:
            self._dashboard_subscriptions["image"] = self.create_subscription(CompressedImage, image_topic, self._image_cb, compressed_qos)
        if raw_image_topic:
            self._dashboard_subscriptions["raw_image"] = self.create_subscription(Image, raw_image_topic, self._raw_image_cb, sensor_qos)
        if self.enable_depth_feed and depth_compressed_topic:
            self._dashboard_subscriptions["depth_compressed"] = self.create_subscription(CompressedImage, depth_compressed_topic, self._depth_compressed_cb, compressed_qos)
        if self.enable_depth_feed and depth_topic:
            self._dashboard_subscriptions["depth_raw"] = self.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos)
        self._dashboard_subscriptions["odom"] = self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, sensor_qos)
        self._dashboard_subscriptions["lidar"] = self.create_subscription(LaserScan, self.get_parameter("lidar_topic").value, self._lidar_cb, sensor_qos)
        self._dashboard_subscriptions["battery"] = self.create_subscription(BatteryState, self.get_parameter("battery_topic").value, self._battery_cb, sensor_qos)
        self._dashboard_subscriptions["map"] = self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self._map_cb, map_qos)
        self._dashboard_subscriptions["mission_status"] = self.create_subscription(String, self.get_parameter("mission_status_topic").value, self._mission_status_cb, reliable_qos)
        self._dashboard_subscriptions["safety_status"] = self.create_subscription(String, self.get_parameter("safety_status_topic").value, self._safety_status_cb, reliable_qos)
        self._dashboard_subscriptions["vlm_status"] = self.create_subscription(String, self.get_parameter("vlm_status_topic").value, self._vlm_status_cb, reliable_qos)
        self._dashboard_subscriptions["mux_status"] = self.create_subscription(String, self.get_parameter("mux_status_topic").value, self._mux_status_cb, reliable_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, sensor_qos)
        self.mission_command_pub = self.create_publisher(String, self.get_parameter("mission_command_topic").value, reliable_qos)
        self.operator_heartbeat_pub = self.create_publisher(
            Bool, self.get_parameter("operator_heartbeat_topic").value, reliable_qos
        )

        self.motion_client = self.create_client(Motion, self.get_parameter("motion_service").value)
        self.teleop_linear_speed = float(self.get_parameter("teleop_linear_speed").value)
        self.teleop_lateral_speed = float(self.get_parameter("teleop_lateral_speed").value)
        self.teleop_angular_speed = float(self.get_parameter("teleop_angular_speed").value)

        self.latest_frame: bytes | None = None
        self.latest_frame_stamp = ""
        self.frame_lock = threading.Lock()
        self.last_frame_time = 0.0

        # Depth camera
        self._bridge = CvBridge()
        self.depth_frame: bytes | None = None
        self.depth_lock = threading.Lock()
        self.depth_event = threading.Event()
        self.last_depth_time = 0.0
        self.last_depth_render_time = 0.0
        self.depth_compressed_frame: bytes | None = None
        self.depth_compressed_format = ""
        self.callback_counts = {
            "image": 0,
            "raw_image": 0,
            "depth_compressed": 0,
            "depth_raw": 0,
            "depth_rendered": 0,
            "depth_errors": 0,
            "odom": 0,
            "lidar": 0,
            "battery": 0,
            "map": 0,
        }
        self.last_depth_error = ""

        self.odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.last_odom_time = 0.0
        self.lidar = {"closest_m": 99.0, "range_count": 0,
                      "ranges": [], "angle_min": 0.0, "angle_increment": 0.0, "range_max": 10.0}
        self.last_lidar_time = 0.0
        self.battery = {"voltage": 0.0, "pct": 0.0}
        self.last_battery_time = 0.0
        self.map_payload = {"image_bytes": None, "width": 0, "height": 0, "resolution": 0.0, "meta": "Waiting for map", "stamp": ""}
        self.last_map_render_time = 0.0
        self.last_map_time = 0.0

        self.logs: list[dict[str, str]] = []
        self.chat_history: list[dict[str, str]] = []
        self.mission_history: list[dict[str, Any]] = []
        self.vlm_events: list[dict[str, Any]] = []
        self.max_log_entries = 80
        self.max_chat_entries = 40
        self.max_vlm_events = 120
        self.memory = SpatialMemory()
        self.mission_status: dict[str, Any] = {"state": "IDLE", "phase": "dashboard_only"}
        self.safety_status: dict[str, Any] = {}
        self.vlm_status: dict[str, Any] = {}
        self.mux_status: dict[str, Any] = {}

        self.image_latency_ms = 0.0
        self.odom_latency_ms = 0.0
        self.vlm_client, self.vlm_config = build_vlm_client()
        self.teleop_status = "idle"
        self.last_teleop = {"linear": 0.0, "lateral": 0.0, "angular": 0.0, "source": "none"}
        self.last_browser_heartbeat_time = 0.0
        self.safety_warning_m = 0.80
        self.safety_danger_m = 0.45

        self.create_timer(0.2, self._publish_operator_heartbeat)
        if self.auto_raw_camera_fallback:
            self.create_timer(2.0, self._enable_raw_camera_fallbacks)

        self.get_logger().info(f"Dashboard ready on {self.vlm_config.base_url} / model {self.vlm_config.model_name}")
        self.get_logger().info(
          "Dashboard topics: compressed=%s raw=%s depth_compressed=%s depth_raw=%s fallback_raw=%s fallback_depth=%s cmd_vel=%s"
            % (
                image_topic or "disabled",
                raw_image_topic or "disabled",
                depth_compressed_topic if self.enable_depth_feed else "disabled",
                depth_topic if self.enable_depth_feed else "disabled",
              self.fallback_raw_image_topic if self.auto_raw_camera_fallback else "disabled",
              self.fallback_depth_topic if (self.auto_raw_camera_fallback and self.enable_depth_feed) else "disabled",
                self.get_parameter("cmd_vel_topic").value,
            )
        )

    def add_log(self, kind: str, text: str) -> None:
        self.logs.append({"timestamp": time.strftime("%H:%M:%S"), "kind": kind, "text": text})
        if len(self.logs) > self.max_log_entries:
            self.logs.pop(0)

    def add_chat(self, role: str, message: str) -> None:
        self.chat_history.append({"role": role, "message": message})
        if len(self.chat_history) > self.max_chat_entries:
            self.chat_history.pop(0)

    def add_vlm_event(
        self,
        kind: str,
        status: str = "ok",
        prompt: str = "",
        answer: str = "",
        thinking: str = "",
        intent: str = "",
        room: str = "",
        latency_ms: float = 0.0,
        text: str = "",
    ) -> None:
        self.vlm_events.append(
            {
                "timestamp": time.strftime("%H:%M:%S"),
                "kind": kind,
                "status": status,
                "prompt": prompt,
                "answer": answer,
                "thinking": thinking,
                "intent": intent,
                "room": room,
                "latency_ms": float(latency_ms),
                "text": text,
            }
        )
        if len(self.vlm_events) > self.max_vlm_events:
            self.vlm_events.pop(0)

    def mark_browser_heartbeat(self) -> None:
        self.last_browser_heartbeat_time = time.time()

    def _publish_operator_heartbeat(self) -> None:
        # Network-survival heartbeat: as long as the dashboard process is
        # alive on the Jetson, the autonomy stack is by definition reachable.
        # Browser activity is tracked separately via last_browser_heartbeat_time
        # for the status panel + auto-stop-on-blur UX, but the watchdog must
        # not fire whenever the operator hides the tab.
        self.operator_heartbeat_pub.publish(Bool(data=True))

    def _enable_raw_camera_fallbacks(self) -> None:
        now = time.time()
        if now - self.started_at < self.raw_fallback_after_s:
            return

        camera_stale = self.last_frame_time <= 0.0 or (now - self.last_frame_time) > self.raw_fallback_after_s
        if (
            camera_stale
            and "raw_image" not in self._dashboard_subscriptions
            and not self._fallback_attempted["raw_image"]
        ):
            topic = self.fallback_raw_image_topic
            if topic and self.count_publishers(topic) > 0:
                self._fallback_attempted["raw_image"] = True
                try:
                    self._dashboard_subscriptions["raw_image"] = self.create_subscription(
                        Image,
                        topic,
                        self._raw_image_cb,
                        self._sensor_qos,
                    )
                    self.subscription_topics["raw_image"] = topic
                    self.add_log("camera", f"RGB raw fallback enabled: {topic}")
                    self.get_logger().warn(f"Enabled RGB raw fallback subscription on {topic}")
                except Exception as exc:
                    self.add_log("camera", f"RGB fallback subscribe failed: {exc}")

        if not self.enable_depth_feed:
            return

        depth_stale = self.last_depth_time <= 0.0 or (now - self.last_depth_time) > self.raw_fallback_after_s
        if (
            depth_stale
            and "depth_raw" not in self._dashboard_subscriptions
            and not self._fallback_attempted["depth_raw"]
        ):
            topic = self.fallback_depth_topic
            if topic and self.count_publishers(topic) > 0:
                self._fallback_attempted["depth_raw"] = True
                try:
                    self._dashboard_subscriptions["depth_raw"] = self.create_subscription(
                        Image,
                        topic,
                        self._depth_cb,
                        self._sensor_qos,
                    )
                    self.subscription_topics["depth_raw"] = topic
                    self.add_log("camera", f"Depth raw fallback enabled: {topic}")
                    self.get_logger().warn(f"Enabled depth raw fallback subscription on {topic}")
                except Exception as exc:
                    self.add_log("camera", f"Depth fallback subscribe failed: {exc}")

    def _image_cb(self, msg: CompressedImage) -> None:
        self.callback_counts["image"] += 1
        try:
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.image_latency_ms = (now - stamp).nanoseconds / 1e6
        except Exception:
            pass
        with self.frame_lock:
            self.latest_frame = bytes(msg.data)
            self.latest_frame_stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        self.last_frame_time = time.time()

    def _raw_image_cb(self, msg: Image) -> None:
        self.callback_counts["raw_image"] += 1
        try:
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.image_latency_ms = (now - stamp).nanoseconds / 1e6
        except Exception:
            pass

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                return
            with self.frame_lock:
                self.latest_frame = buf.tobytes()
                self.latest_frame_stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            self.last_frame_time = time.time()
        except Exception as exc:
            self.get_logger().error(f"Raw image cb error: {exc}")

    def _odom_cb(self, msg: Odometry) -> None:
        self.callback_counts["odom"] += 1
        self.last_odom_time = time.time()
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

    def _store_depth_visualization(self, depth: np.ndarray, encoding: str) -> None:
        try:
            depth_mm = np.asarray(depth, dtype=np.float32)
            if np.issubdtype(np.asarray(depth).dtype, np.floating) or "32F" in encoding:
                depth_mm *= 1000.0
            depth_clipped = np.clip(depth_mm, 0, 5000).astype(np.float32)
            depth_norm = (depth_clipped / 5000.0 * 255).astype(np.uint8)
            colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
            colored[(depth_mm <= 0) | ~np.isfinite(depth_mm)] = [30, 30, 30]
            ok, buf = cv2.imencode('.jpg', colored, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self.depth_lock:
                    self.depth_frame = buf.tobytes()
                self.last_depth_time = time.time()
                self.callback_counts["depth_rendered"] += 1
                self.depth_event.set()
        except Exception as exc:
            self.callback_counts["depth_errors"] += 1
            self.last_depth_error = str(exc)
            self.get_logger().error(f"Depth visualization error: {exc}")

    def _depth_compressed_cb(self, msg: CompressedImage) -> None:
        self.callback_counts["depth_compressed"] += 1
        self.last_depth_time = time.time()
        if self.last_depth_time - self.last_depth_render_time < 1.0 / self.depth_render_hz:
            return
        self.last_depth_render_time = self.last_depth_time
        try:
            encoded = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            depth = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
            if depth is not None:
                self._store_depth_visualization(depth, msg.format)
            else:
                self.callback_counts["depth_errors"] += 1
                self.last_depth_error = "cv2.imdecode returned None"
        except Exception as exc:
            self.callback_counts["depth_errors"] += 1
            self.last_depth_error = str(exc)
            self.get_logger().error(f"Compressed depth render error: {exc}")

    def _depth_cb(self, msg: Image) -> None:
        self.callback_counts["depth_raw"] += 1
        now = time.time()
        self.last_depth_time = now
        if now - self.last_depth_render_time < 1.0 / self.depth_render_hz:
            return
        self.last_depth_render_time = now
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self._store_depth_visualization(depth, msg.encoding)
        except Exception as exc:
            self.callback_counts["depth_errors"] += 1
            self.last_depth_error = str(exc)
            self.get_logger().error(f"Raw depth cb error: {exc}")

    def _lidar_cb(self, msg: LaserScan) -> None:
        self.callback_counts["lidar"] += 1
        self.last_lidar_time = time.time()
        finite_ranges = [value for value in msg.ranges if 0.05 < value < float("inf")]
        self.lidar["closest_m"] = min(finite_ranges) if finite_ranges else 99.0
        self.lidar["range_count"] = len(finite_ranges)
        self.lidar["ranges"] = list(msg.ranges[::3])  # downsample 3x
        self.lidar["angle_min"] = msg.angle_min
        self.lidar["angle_increment"] = msg.angle_increment * 3
        self.lidar["range_max"] = msg.range_max

    def _battery_cb(self, msg: BatteryState) -> None:
        self.callback_counts["battery"] += 1
        self.last_battery_time = time.time()
        self.battery["voltage"] = msg.voltage
        self.battery["pct"] = msg.percentage * 100.0 if 0.0 <= msg.percentage <= 1.0 else msg.percentage

    def _mission_status_cb(self, msg: String) -> None:
        self.mission_status = json_loads(msg.data, self.mission_status)

    def _safety_status_cb(self, msg: String) -> None:
        self.safety_status = json_loads(msg.data, self.safety_status)

    def _vlm_status_cb(self, msg: String) -> None:
        self.vlm_status = json_loads(msg.data, self.vlm_status)

    def _mux_status_cb(self, msg: String) -> None:
        self.mux_status = json_loads(msg.data, self.mux_status)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        try:
            self.callback_counts["map"] += 1
            self.last_map_time = time.time()
            now = time.time()
            if now - self.last_map_render_time < 1.0 / self.map_render_hz:
                return
            self.last_map_render_time = now

            width = int(msg.info.width)
            height = int(msg.info.height)
            resolution = float(msg.info.resolution)
            data = np.array(msg.data, dtype=np.int16).reshape((height, width))

            image = np.full((height, width), 205, dtype=np.uint8)
            image[data == 0] = 255
            image[data == 100] = 0
            image = np.flipud(image)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            target_long_side = self.map_target_long_side
            scale = max(1, min(10, target_long_side // max(width, height, 1)))
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

    def send_motion_command(
        self,
        linear: float,
        lateral: float,
        angular: float,
        source: str = "manual",
        issued_at: float | None = None,
        seq: int = 0,
    ) -> tuple[bool, str]:
        now = time.time()
        command_is_zero = (abs(linear) + abs(lateral) + abs(angular)) <= 1e-6
        age_s = now - float(issued_at) if issued_at is not None else 0.0
        max_age_s = float(self.get_parameter("teleop_command_max_age_s").value)
        if issued_at is not None and age_s > max_age_s and not command_is_zero:
            self.last_teleop = {
                "linear": float(linear),
                "lateral": float(lateral),
                "angular": float(angular),
                "source": source,
                "seq": int(seq),
                "age_s": float(age_s),
                "ignored": True,
            }
            self.teleop_status = "stale/ignored"
            return False, f"stale teleop command ignored age={age_s:.2f}s"

        self.last_teleop = {
            "linear": float(linear),
            "lateral": float(lateral),
            "angular": float(angular),
            "source": source,
            "seq": int(seq),
            "age_s": float(age_s),
            "ignored": False,
        }
        self._publish_direct_motion(linear, lateral, angular)
        self.teleop_status = "active/mux" if (abs(linear) + abs(lateral) + abs(angular)) > 1e-6 else "idle/mux"
        return True, "OK teleop command queued through cmd_vel_mux"

    def _publish_direct_motion(self, linear: float, lateral: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear)
        twist.linear.y = float(lateral)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)

    def video_stream(self):
        interval = 1.0 / self.rgb_stream_hz
        while True:
            frame = self.latest_frame_copy()
            if frame is None:
                time.sleep(0.04)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(interval)

    def depth_stream(self):
        last_render = 0.0
        while True:
            if not self.enable_depth_feed:
                time.sleep(1.0)
                continue
            min_interval = 1.0 / self.depth_render_hz
            elapsed = time.time() - last_render
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)
            self.depth_event.wait(timeout=2.0)
            self.depth_event.clear()
            with self.depth_lock:
                frame = self.depth_frame
            last_render = time.time()
            if frame:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    def _safety_payload(self) -> dict[str, Any]:
        closest = float(self.lidar.get("closest_m", 99.0))
        risk_level = "OK"
        stop_recommended = False
        if closest < self.safety_danger_m:
            risk_level = "DANGER"
            stop_recommended = True
        elif closest < self.safety_warning_m:
            risk_level = "WARNING"

        return {
            "closest_m": closest,
            "warning_threshold_m": self.safety_warning_m,
            "danger_threshold_m": self.safety_danger_m,
            "reasons": [],
            "risk_level": risk_level,
            "stop_recommended": stop_recommended,
        }

    def status_payload(self) -> dict[str, Any]:
        now = time.time()
        map_payload = self.map_payload
        mission_last = self.mission_history[-1] if self.mission_history else {}
        active_session = str(self.mission_status.get("session_id", "")) or self.memory.get_active_session()
        memory_locations = self.memory.get_locations(active_session) if active_session else self.memory.get_locations()
        memory_missions = self.memory.get_recent_missions(limit=20, session_id=active_session) if active_session else self.memory.get_recent_missions(limit=20)

        camera_alive = (now - self.last_frame_time) < 5.0 if self.last_frame_time > 0 else False
        depth_alive = (now - self.last_depth_time) < 5.0 if self.last_depth_time > 0 else False
        odom_alive = (now - self.last_odom_time) < 5.0 if self.last_odom_time > 0 else False
        lidar_alive = (now - self.last_lidar_time) < 5.0 if self.last_lidar_time > 0 else False
        battery_alive = (now - self.last_battery_time) < 10.0 if self.last_battery_time > 0 else False
        camera_age_ms = (now - self.last_frame_time) * 1000.0 if self.last_frame_time > 0 else 0.0
        odom_age_ms = (now - self.last_odom_time) * 1000.0 if self.last_odom_time > 0 else 0.0
        lidar_age_ms = (now - self.last_lidar_time) * 1000.0 if self.last_lidar_time > 0 else 0.0
        battery_age_ms = (now - self.last_battery_time) * 1000.0 if self.last_battery_time > 0 else 0.0
        map_ready = int(map_payload["width"]) > 0
        connected = camera_alive or depth_alive or odom_alive or lidar_alive or battery_alive or map_ready
        heartbeat_age = now - self.last_browser_heartbeat_time if self.last_browser_heartbeat_time else 999.0

        return {
            "server_time": now,
            "connected": connected,
            "battery_pct": float(self.battery["pct"]),
            "battery_voltage": float(self.battery["voltage"]),
            "pose": {"x": float(self.odom["x"]), "y": float(self.odom["y"]), "yaw_deg": float(self.odom["yaw"])},
            "lidar": {"closest_m": float(self.lidar["closest_m"]), "range_count": int(self.lidar["range_count"])},
            "feeds": {
                "camera_alive": camera_alive,
                "depth_alive": depth_alive,
                "odom_alive": odom_alive,
                "lidar_alive": lidar_alive,
                "battery_alive": battery_alive,
                "map_ready": map_ready,
                "camera_age_ms": camera_age_ms,
                "odom_age_ms": odom_age_ms,
                "lidar_age_ms": lidar_age_ms,
                "battery_age_ms": battery_age_ms,
            },
            "safety": self.safety_status or self._safety_payload(),
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
            "mission": {
                "state": str(self.mission_status.get("state") or ("QUEUED" if mission_last else "IDLE")),
                "phase": str(self.mission_status.get("phase", "")),
                "command": str(self.mission_status.get("command") or mission_last.get("command", "")),
                "last_intent": str(self.mission_status.get("intent") or mission_last.get("intent", "")),
                "last_room": str(self.mission_status.get("target_room") or mission_last.get("room", "")),
                "queue_length": len(self.mission_history),
                "recent": self.mission_history[-12:],
                "live": self.mission_status,
            },
            "vlm": {
                "chat_count": len(self.chat_history),
                "log_count": len(self.logs),
                "event_count": len(self.vlm_events),
                "events": self.vlm_events[-60:],
                "status": self.vlm_status,
            },
            "memory": {
                "location_count": len(memory_locations),
                "mission_count": len(memory_missions),
                "recent_locations": memory_locations[:12],
                "recent_missions": memory_missions[:12],
            },
            "teleop": {
                "status": self.teleop_status,
                "last": self.last_teleop,
                "mux": self.mux_status,
                "operator_heartbeat": {
                    "age_s": heartbeat_age,
                    "active": heartbeat_age <= 5.0,
                },
                "speeds": {
                    "linear": self.teleop_linear_speed,
                    "lateral": self.teleop_lateral_speed,
                    "angular": self.teleop_angular_speed,
                },
            },
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

    @app.get("/health")
    def health() -> JSONResponse:
        return JSONResponse({"ok": True, "node": "ridgeback_dashboard", "mission": node.mission_status})

    @app.post("/api/heartbeat")
    def api_heartbeat() -> JSONResponse:
        node.mark_browser_heartbeat()
        return JSONResponse({"ok": True, "stamp": time.time()})

    @app.get("/api/mission/status")
    def api_mission_status() -> JSONResponse:
        return JSONResponse(node.mission_status)

    @app.get("/api/metrics")
    def api_metrics() -> JSONResponse:
        status = node.mission_status
        return JSONResponse(
            {
                "result": status.get("state", "UNKNOWN"),
                "state": status.get("state", "UNKNOWN"),
                "phase": status.get("phase", ""),
                "elapsed_s": float(status.get("elapsed_s", 0.0) or 0.0),
                "target_room": status.get("target_room", ""),
                "last_error": status.get("last_error", ""),
            }
        )

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

    @app.get("/depth_feed")
    def depth_feed() -> StreamingResponse:
        return StreamingResponse(node.depth_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/depth_status")
    def depth_status() -> JSONResponse:
        age = time.time() - node.last_depth_time if node.last_depth_time > 0 else -1.0
        return JSONResponse({
            "enabled": bool(node.enable_depth_feed),
            "has_frame": node.last_depth_time > 0,
            "age_s": round(age, 1) if age >= 0 else -1.0,
            "callbacks": dict(node.callback_counts),
            "last_error": node.last_depth_error,
        })

    @app.get("/lidar_scan")
    def lidar_scan() -> JSONResponse:
        return JSONResponse(node.lidar)

    @app.get("/api/debug/feeds")
    def api_debug_feeds() -> JSONResponse:
        now = time.time()
        return JSONResponse({
            "topics": node.subscription_topics,
            "subscriptions": sorted(node._dashboard_subscriptions.keys()),
            "callbacks": dict(node.callback_counts),
            "fallback": {
                "enabled": bool(node.auto_raw_camera_fallback),
                "after_s": float(node.raw_fallback_after_s),
                "attempted": dict(node._fallback_attempted),
            },
            "last_error": node.last_depth_error,
            "camera_age_s": round(now - node.last_frame_time, 2) if node.last_frame_time > 0 else -1.0,
            "depth_age_s": round(now - node.last_depth_time, 2) if node.last_depth_time > 0 else -1.0,
            "odom_age_s": round(now - node.last_odom_time, 2) if node.last_odom_time > 0 else -1.0,
            "lidar_age_s": round(now - node.last_lidar_time, 2) if node.last_lidar_time > 0 else -1.0,
            "battery_age_s": round(now - node.last_battery_time, 2) if node.last_battery_time > 0 else -1.0,
            "map_age_s": round(now - node.last_map_time, 2) if node.last_map_time > 0 else -1.0,
            "has_rgb_frame": node.latest_frame_copy() is not None,
            "has_depth_frame": node.depth_frame is not None,
            "publishers": {
                topic: node.count_publishers(topic)
                for topic in (
                    node.subscription_topics.get("image", ""),
                    node.subscription_topics.get("depth_compressed", ""),
                    node.subscription_topics.get("raw_image", ""),
                    node.subscription_topics.get("depth_raw", ""),
                    node.subscription_topics.get("fallback_raw_image", ""),
                    node.subscription_topics.get("fallback_depth_raw", ""),
                    node.subscription_topics.get("odom", ""),
                    node.subscription_topics.get("lidar", ""),
                    node.subscription_topics.get("battery", ""),
                    node.subscription_topics.get("map", ""),
                )
                if topic
            },
        })

    @app.post("/api/mission")
    def api_mission(request: MissionRequest) -> JSONResponse:
        parsed = parse_intent_and_room(request.command)
        payload = {
            "command": request.command,
            "intent": parsed["intent"],
            "room": parsed["room"],
            "source": "dashboard",
            "timestamp": time.time(),
        }
        node.mission_command_pub.publish(String(data=json_dumps(payload)))
        node.mission_history.append(
            {
                "timestamp": time.strftime("%H:%M:%S"),
                "command": request.command,
                "intent": parsed["intent"],
                "room": parsed["room"],
                "source": "mission",
            }
        )
        if len(node.mission_history) > 40:
            node.mission_history.pop(0)
        node.memory.record_mission(
            request.command,
            status="queued",
            metadata=parsed,
            session_id=str(node.mission_status.get("session_id", "")),
        )
        node.add_log("mission", f"{parsed['intent']} {parsed['room']} :: {request.command}".strip())
        node.add_vlm_event(
            kind="mission",
            status="queued",
            prompt=request.command,
            intent=parsed["intent"],
            room=parsed["room"],
            text="Mission command queued",
        )
        return JSONResponse({"ok": True, "accepted": request.command, "parsed": parsed, "mission_status": node.mission_status})

    @app.post("/api/teleop")
    def api_teleop(request: TeleopRequest) -> JSONResponse:
        linear = float(request.linear)
        lateral = float(request.lateral)
        angular = float(request.angular)
        ok, message = node.send_motion_command(
            linear,
            lateral,
            angular,
            source=request.source,
            issued_at=request.issued_at,
            seq=request.seq,
        )
        if not ok:
            if (abs(linear) + abs(lateral) + abs(angular)) > 1e-6:
                node.add_log("teleop", f"Rejected command ({request.source}): {message}")
            return JSONResponse({"ok": False, "error": message})
        if (abs(linear) + abs(lateral) + abs(angular)) > 1e-6:
            node.add_log("teleop", f"{request.source}: x={linear:.2f} y={lateral:.2f} z={angular:.2f}")
        return JSONResponse({"ok": True, "message": message})

    @app.post("/api/chat")
    def api_chat(request: ChatRequest) -> JSONResponse:
        parsed = parse_intent_and_room(request.message)
        node.add_chat("user", request.message)
        node.add_log("vlm", f"Chat query: {request.message}")
        started_at = time.time()
        try:
            latest_frame = node.latest_frame_copy()
            if latest_frame:
                image_b64 = base64.b64encode(latest_frame).decode("utf-8")
                user_content: list[dict[str, Any]] = [
                    {
                        "type": "text",
                        "text": f"User question about current camera view: {request.message}",
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                    },
                ]
                messages: list[dict[str, Any]] = [
                    {
                        "role": "system",
                        "content": "You are a concise assistant for a Ridgeback R100 autonomous navigation dashboard. Use the provided camera image when relevant. Answer in 1-5 sentences and be direct.",
                    },
                    {"role": "user", "content": user_content},
                ]
            else:
                messages = chat_completion_messages(
                    request.message,
                    system_prompt="You are a concise assistant for a Ridgeback R100 autonomous navigation dashboard. Answer in 1-5 sentences and be direct.",
                )

            response = node.vlm_client.chat.completions.create(
                model=node.vlm_config.model_name,
                messages=messages,
                temperature=0.4,
                max_tokens=200,
                extra_body={"chat_template_kwargs": {"enable_thinking": node.vlm_config.enable_thinking}},
            )
            raw_reply = response.choices[0].message.content
            if isinstance(raw_reply, list):
                reply = " ".join(str(part.get("text", "")) if isinstance(part, dict) else str(part) for part in raw_reply).strip()
            else:
                reply = str(raw_reply or "")
            thinking = extract_reasoning_trace(response)
            latency_ms = (time.time() - started_at) * 1000.0
            event_kind = "mission" if parsed["intent"] in {"GO_TO_ROOM", "RETURN_TO_START", "STOP"} else "prompt"
            if event_kind == "mission":
                node.mission_command_pub.publish(
                    String(
                        data=json_dumps(
                            {
                                "command": request.message,
                                "intent": parsed["intent"],
                                "room": parsed["room"],
                                "source": "chat",
                                "timestamp": time.time(),
                            }
                        )
                    )
                )

            node.add_chat("assistant", reply)
            node.add_log("vlm", f"Reply: {reply[:180]}")
            node.add_vlm_event(
                kind=event_kind,
                status="ok",
                prompt=request.message,
                answer=reply,
                thinking=thinking,
                intent=parsed["intent"],
                room=parsed["room"],
                latency_ms=latency_ms,
                text=reply[:220],
            )
            return JSONResponse({"ok": True, "reply": reply, "parsed": parsed, "latency_ms": latency_ms})
        except Exception as exc:
            error = str(exc)
            node.add_chat("assistant", f"Error: {error}")
            node.add_log("vlm", f"Chat error: {error}")
            node.add_vlm_event(
                kind="prompt",
                status="error",
                prompt=request.message,
                answer="",
                thinking="",
                intent=parsed["intent"],
                room=parsed["room"],
                latency_ms=(time.time() - started_at) * 1000.0,
                text=error,
            )
            return JSONResponse({"ok": False, "error": error})

    return app


def main() -> None:
  rclpy.init()
  node = DashboardNode()
  app = create_app(node)
  port = int(node.get_parameter("port").value)
  host = str(node.get_parameter("host").value)
  # Force ASGI lifespan so the ROS spin thread in create_app(...lifespan=...)
  # is always started. Without this, subscriptions can be created but never
  # serviced, which leaves image/depth callbacks stuck at zero.
  uvicorn.run(app, host=host, port=port, log_level="info", lifespan="on")


if __name__ == "__main__":
    main()
