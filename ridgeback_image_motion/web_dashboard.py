#!/usr/bin/env python3
"""Ridgeback autonomy dashboard — FastAPI + ROS2 bridge.

UI HTML is preserved verbatim from the previous dashboard. The Python
backend is trimmed to only the endpoints the UI calls, and pushes all
mission / memory / VLM / safety state into ROS topics so other nodes own
the logic.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from contextlib import asynccontextmanager
from typing import Any

import cv2
import numpy as np
import rclpy
import uvicorn
from cv_bridge import CvBridge
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from pydantic import BaseModel
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

try:
    from ridgeback_image_motion.autonomy_common import (
        json_dumps,
        json_loads,
        parse_intent_and_room,
        quaternion_to_yaw_rad,
    )
except ImportError:
    from autonomy_common import (  # type: ignore[no-redef]
        json_dumps,
        json_loads,
        parse_intent_and_room,
        quaternion_to_yaw_rad,
    )


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
          <div class="sr">
            <span class="sl">OVERRIDE</span>
            <button id="btn-safety-override" type="button" class="btn"
                    onclick="toggleSafetyOverride()">OFF</button>
            <span id="st-safety-override-hint" class="sv">safety checks active</span>
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
        <div id="teleop-feedback" class="small-note">Teleop bridge idle.</div>
        <button class="btn btn-stop" style="width:100%;margin-top:6px" onclick="stopRobot()">■ STOP</button>
      </div>

      <!-- VLM Prompt pane -->
      <div class="pane" id="pane-prompt" style="display:none">
        <div id="chat-hist" class="chat-hist"></div>
        <div class="chat-row">
          <input id="chat-input" type="text" placeholder='"explore", "go to room 202 and return", or "what do you see"'>
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

async function toggleSafetyOverride() {
  const btn = document.getElementById('btn-safety-override');
  const turningOn = (btn.textContent.trim() !== 'ON');
  if (turningOn && !confirm('Disable safety latch?\\nThe robot will accept full-speed cmd_vel with no heartbeat/liveness check.')) {
    return;
  }
  try {
    await fetch('/api/safety/override', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({enabled: turningOn})
    });
  } catch (_) {}
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
  const feedback = document.getElementById('teleop-feedback');
  pendingTeleop = {
    linear, lateral, angular, source,
    issued_at: (Date.now() + serverClockOffsetMs) / 1000.0,
    seq: ++teleopSeq
  };
  if (feedback) feedback.textContent = `Teleop queued (${source}).`;
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
    const response = await fetch('/api/teleop', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(payload),
      signal: controller.signal
    });
    const feedback = document.getElementById('teleop-feedback');
    if (feedback) {
      feedback.textContent = response.ok
        ? `Teleop sent (${payload.source}).`
        : `Teleop backend returned ${response.status}.`;
      feedback.style.color = response.ok ? 'var(--muted)' : 'var(--danger)';
    }
  } catch(_) {
    const feedback = document.getElementById('teleop-feedback');
    if (feedback) {
      feedback.textContent = 'Teleop request failed: dashboard backend unreachable.';
      feedback.style.color = 'var(--danger)';
    }
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
    const isNew = Boolean(item.is_new);
    const dot = isNew ? '<span style="color:var(--accent);font-weight:bold">● </span>' : '';
    const badge = isNew ? ' <span style="color:var(--accent);font-weight:bold">[NEW]</span>' : '';
    return `<div class="mono-line">${dot}${escHtml(label)} @ (${Number(item.x||0).toFixed(2)}, ${Number(item.y||0).toFixed(2)}) conf ${conf}${badge}</div>`;
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
    if (e('btn-safety-override')) {
      const ov = Boolean(data.safety?.override);
      e('btn-safety-override').textContent = ov ? 'ON' : 'OFF';
      e('btn-safety-override').style.background = ov ? 'var(--danger)' : '';
      e('btn-safety-override').style.color = ov ? '#fff' : '';
      if (e('st-safety-override-hint')) {
        e('st-safety-override-hint').textContent = ov
          ? 'CLAMP BYPASSED — full-speed cmd_vel'
          : 'safety checks active';
        e('st-safety-override-hint').style.color = ov ? 'var(--danger)' : 'var(--muted)';
      }
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


# ---------------------------------------------------------------------------
# Pydantic request models
# ---------------------------------------------------------------------------


class TeleopRequest(BaseModel):
    linear: float = 0.0
    lateral: float = 0.0
    angular: float = 0.0
    source: str = "keyboard"
    issued_at: float | None = None
    seq: int = 0


class MissionRequest(BaseModel):
    command: str


class ChatRequest(BaseModel):
    message: str


class SafetyOverrideRequest(BaseModel):
    enabled: bool


class VlmTriggerRequest(BaseModel):
    prompt: str = "What do you see?"


# ---------------------------------------------------------------------------
# ROS bridge
# ---------------------------------------------------------------------------


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class DashboardNode(Node):
    """ROS bridge for the FastAPI dashboard.

    Owns subscriptions for sensor/state topics and publishers for operator
    actions. All heavy logic (mission FSM, safety latch, frontier exploration,
    VLM) lives in dedicated nodes.
    """

    def __init__(self) -> None:
        super().__init__("ridgeback_dashboard")

        # Parameters
        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("scan_topic", "")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("battery_topic", "")
        self.declare_parameter("color_image_topic", "")
        self.declare_parameter("depth_image_topic", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("teleop_topic", "/cmd_vel/teleop")
        self.declare_parameter("heartbeat_topic", "/operator/heartbeat")
        self.declare_parameter("safety_latched_topic", "/safety/latched")
        self.declare_parameter("safety_reset_service", "/safety/reset")
        self.declare_parameter("safety_override_topic", "/safety/override")
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("mission_goal_topic", "/mission/goal")
        self.declare_parameter("frontier_status_topic", "/frontier/status")
        self.declare_parameter("vlm_observation_topic", "/vlm/observation")
        self.declare_parameter("vlm_trigger_topic", "/vlm/trigger")
        self.declare_parameter("teleop_max_linear", 0.5)
        self.declare_parameter("teleop_max_lateral", 0.5)
        self.declare_parameter("teleop_max_angular", 1.5)

        ns = self.get_parameter("namespace").value
        self._params: dict[str, Any] = {
            "scan": self.get_parameter("scan_topic").value or f"/{ns}/sensors/lidar2d_0/scan",
            "odom": self.get_parameter("odom_topic").value or f"/{ns}/platform/odom/filtered",
            "battery": self.get_parameter("battery_topic").value or f"/{ns}/platform/bms/state",
            "color": self.get_parameter("color_image_topic").value
            or f"/{ns}/sensors/camera_0/color/image_raw",
            "depth": self.get_parameter("depth_image_topic").value
            or f"/{ns}/sensors/camera_0/aligned_depth_to_color/image_raw",
            "map": self.get_parameter("map_topic").value,
            "teleop": self.get_parameter("teleop_topic").value,
            "heartbeat": self.get_parameter("heartbeat_topic").value,
            "safety_latched": self.get_parameter("safety_latched_topic").value,
            "safety_reset": self.get_parameter("safety_reset_service").value,
            "safety_override": self.get_parameter("safety_override_topic").value,
            "mission_state": self.get_parameter("mission_state_topic").value,
            "mission_goal": self.get_parameter("mission_goal_topic").value,
            "frontier_status": self.get_parameter("frontier_status_topic").value,
            "vlm_observation": self.get_parameter("vlm_observation_topic").value,
            "vlm_trigger": self.get_parameter("vlm_trigger_topic").value,
        }
        self._teleop_max = (
            float(self.get_parameter("teleop_max_linear").value),
            float(self.get_parameter("teleop_max_lateral").value),
            float(self.get_parameter("teleop_max_angular").value),
        )

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # Frame buffers
        self._color_jpeg: bytes | None = None
        self._color_stamp: float = 0.0
        self._depth_jpeg: bytes | None = None
        self._depth_stamp: float = 0.0
        self._depth_enabled: bool = False
        self._scan: dict[str, Any] = {}
        self._scan_stamp: float = 0.0
        self._battery_pct: float = 0.0
        self._battery_v: float = 0.0
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_yaw: float = 0.0
        self._odom_stamp: float = 0.0
        self._map_meta: str = ""
        self._map_width: int = 0
        self._map_height: int = 0
        self._map_png: bytes | None = None
        self._safety_latched: bool = True
        self._safety_override: bool = False
        self._mission_state: dict[str, Any] = {}
        self._frontier_status: dict[str, Any] = {}
        self._vlm_events: list[dict[str, Any]] = []
        self._chat_history: list[dict[str, Any]] = []
        self._last_teleop_source: str = "none"
        self._last_teleop_stamp: float = 0.0

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscriptions
        self.create_subscription(LaserScan, self._params["scan"], self._on_scan, sensor_qos)
        self.create_subscription(Odometry, self._params["odom"], self._on_odom, 10)
        self.create_subscription(BatteryState, self._params["battery"], self._on_battery, sensor_qos)
        self.create_subscription(Image, self._params["color"], self._on_color, sensor_qos)
        self.create_subscription(Image, self._params["depth"], self._on_depth, sensor_qos)
        self.create_subscription(OccupancyGrid, self._params["map"], self._on_map, _LATCHED_QOS)
        self.create_subscription(Bool, self._params["safety_latched"], self._on_safety, _LATCHED_QOS)
        self.create_subscription(String, self._params["mission_state"], self._on_mission_state, _LATCHED_QOS)
        self.create_subscription(String, self._params["frontier_status"], self._on_frontier_status, 10)
        self.create_subscription(String, self._params["vlm_observation"], self._on_vlm, 10)

        # Publishers
        self._pub_teleop = self.create_publisher(Twist, self._params["teleop"], 10)
        self._pub_heartbeat = self.create_publisher(Header, self._params["heartbeat"], 10)
        self._pub_mission = self.create_publisher(String, self._params["mission_goal"], 10)
        self._pub_safety_override = self.create_publisher(
            Bool, self._params["safety_override"], _LATCHED_QOS
        )
        self._pub_vlm_trigger = self.create_publisher(String, self._params["vlm_trigger"], 10)

        # Service client (safety reset; created lazily on first call)
        self._safety_reset_client = self.create_client(Trigger, self._params["safety_reset"])

        self.get_logger().info("ridgeback_dashboard ready")

    # --- subscription callbacks ---------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        finite = [r for r in ranges if math.isfinite(r) and r > 0.05]
        closest = min(finite) if finite else 0.0
        with self._lock:
            self._scan = {
                "angle_min": float(msg.angle_min),
                "angle_max": float(msg.angle_max),
                "angle_increment": float(msg.angle_increment),
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "ranges": ranges,
                "closest_m": closest,
                "range_count": len(finite),
            }
            self._scan_stamp = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_x = float(msg.pose.pose.position.x)
            self._odom_y = float(msg.pose.pose.position.y)
            self._odom_yaw = quaternion_to_yaw_rad(msg.pose.pose.orientation)
            self._odom_stamp = time.time()

    def _on_battery(self, msg: BatteryState) -> None:
        pct = float(msg.percentage) if msg.percentage > 0 else 0.0
        # Normalize 0..1 → 0..100 if needed
        if 0.0 < pct <= 1.0:
            pct *= 100.0
        with self._lock:
            self._battery_pct = pct
            self._battery_v = float(msg.voltage)

    def _on_color(self, msg: Image) -> None:
        try:
            cv = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(
                f"_on_color cv_bridge convert failed (encoding={msg.encoding}): {exc}",
                throttle_duration_sec=5.0,
            )
            return
        ok, buf = cv2.imencode(".jpg", cv, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            self.get_logger().warn("_on_color JPEG encode failed", throttle_duration_sec=5.0)
            return
        with self._lock:
            self._color_jpeg = bytes(buf)
            self._color_stamp = time.time()

    def _on_depth(self, msg: Image) -> None:
        try:
            cv = self._bridge.imgmsg_to_cv2(msg)
        except Exception:
            return
        # Normalize to 0..5m heatmap.
        depth = np.asarray(cv, dtype=np.float32)
        if depth.dtype == np.uint16 or depth.max() > 100:
            depth = depth / 1000.0
        depth = np.clip(depth, 0.0, 5.0)
        norm = (255.0 * (depth / 5.0)).astype(np.uint8)
        heat = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        ok, buf = cv2.imencode(".jpg", heat, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if not ok:
            return
        with self._lock:
            self._depth_jpeg = bytes(buf)
            self._depth_stamp = time.time()
            self._depth_enabled = True

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        w, h = int(info.width), int(info.height)
        if w == 0 or h == 0:
            return
        data = np.asarray(msg.data, dtype=np.int8).reshape((h, w))
        # Map -1 (unknown) → 127, 0..100 → 255..0 grayscale.
        img = np.full((h, w), 127, dtype=np.uint8)
        known = data >= 0
        img[known] = (255 - (data[known].astype(np.int32) * 255 // 100)).astype(np.uint8)
        # Flip vertically so +y is up.
        img = cv2.flip(img, 0)
        ok, buf = cv2.imencode(".png", img)
        if not ok:
            return
        meta = f"{w}x{h} @{info.resolution:.2f}m"
        with self._lock:
            self._map_png = bytes(buf)
            self._map_meta = meta
            self._map_width = w
            self._map_height = h

    def _on_safety(self, msg: Bool) -> None:
        with self._lock:
            self._safety_latched = bool(msg.data)

    def _on_mission_state(self, msg: String) -> None:
        with self._lock:
            self._mission_state = json_loads(msg.data, default={})

    def _on_frontier_status(self, msg: String) -> None:
        with self._lock:
            self._frontier_status = json_loads(msg.data, default={})

    def _on_vlm(self, msg: String) -> None:
        evt = json_loads(msg.data, default={})
        if not evt:
            return
        ts = evt.get("timestamp") or time.strftime("%H:%M:%S")
        item = {
            "timestamp": ts,
            "kind": evt.get("kind", "vlm"),
            "status": evt.get("status", "ok"),
            "answer": evt.get("text") or evt.get("room") or "",
            "intent": evt.get("intent", ""),
            "room": evt.get("room", ""),
            "latency_ms": evt.get("latency_ms", 0),
        }
        with self._lock:
            self._vlm_events.append(item)
            if len(self._vlm_events) > 60:
                self._vlm_events = self._vlm_events[-60:]
            # Q&A replies surface in the chat panel too, not just the VLM log.
            if item["kind"] == "answer" and item["answer"]:
                self._chat_history.append({"role": "assistant", "message": item["answer"]})
                if len(self._chat_history) > 40:
                    self._chat_history = self._chat_history[-40:]

    # --- publish helpers ----------------------------------------------------
    def publish_heartbeat(self) -> float:
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "operator"
        self._pub_heartbeat.publish(msg)
        return msg.stamp.sec + msg.stamp.nanosec * 1e-9

    def publish_teleop(self, req: TeleopRequest) -> None:
        max_lin, max_lat, max_ang = self._teleop_max
        msg = Twist()
        msg.linear.x = max(-max_lin, min(max_lin, float(req.linear)))
        msg.linear.y = max(-max_lat, min(max_lat, float(req.lateral)))
        msg.angular.z = max(-max_ang, min(max_ang, float(req.angular)))
        self._pub_teleop.publish(msg)
        with self._lock:
            self._last_teleop_source = req.source
            self._last_teleop_stamp = time.time()

    def publish_mission(self, intent: str, room: str, command: str) -> None:
        payload = {
            "intent": intent,
            "room": room,
            "command": command,
            "timestamp": time.strftime("%H:%M:%S"),
        }
        msg = String()
        msg.data = json_dumps(payload)
        self._pub_mission.publish(msg)
        with self._lock:
            self._chat_history.append({"role": "user", "message": command})
            ack = f"intent={intent}" + (f" room={room}" if room else "")
            ack += f' (parsed from "{command}")'
            if intent == "QUERY":
                ack += " — asking VLM…"
            self._chat_history.append({"role": "assistant", "message": ack})
            if len(self._chat_history) > 40:
                self._chat_history = self._chat_history[-40:]

    def publish_vlm_trigger(self, prompt: str) -> None:
        """Force a one-shot VLM Q&A call with the given prompt.

        Independent of mission_orchestrator's QUERY branch: lets the operator
        ask the VLM something even when no chat command has been issued.
        """
        msg = String()
        msg.data = json_dumps({"prompt": prompt, "intent": "QUERY"})
        self._pub_vlm_trigger.publish(msg)

    def publish_safety_override(self, enabled: bool) -> None:
        with self._lock:
            self._safety_override = bool(enabled)
        msg = Bool()
        msg.data = bool(enabled)
        self._pub_safety_override.publish(msg)

    def request_safety_reset(self) -> tuple[bool, str]:
        if not self._safety_reset_client.service_is_ready():
            self._safety_reset_client.wait_for_service(timeout_sec=0.2)
        if not self._safety_reset_client.service_is_ready():
            return False, "safety_reset service not available"
        future = self._safety_reset_client.call_async(Trigger.Request())
        # We're already running an executor in the bg thread, so just wait briefly.
        deadline = time.time() + 1.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, "safety_reset timed out"
        result = future.result()
        return bool(result.success), result.message or ("ok" if result.success else "failed")

    # --- snapshot for /api/status -------------------------------------------
    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            color_age = (now - self._color_stamp) if self._color_stamp else 9999.0
            depth_age = (now - self._depth_stamp) if self._depth_stamp else 9999.0
            odom_age = (now - self._odom_stamp) if self._odom_stamp else 9999.0
            scan_age = (now - self._scan_stamp) if self._scan_stamp else 9999.0
            payload = {
                "server_time": now,
                "connected": color_age < 5.0 or odom_age < 5.0,
                "battery_pct": self._battery_pct,
                "battery_voltage": self._battery_v,
                "pose": {
                    "x": self._odom_x,
                    "y": self._odom_y,
                    "yaw_deg": math.degrees(self._odom_yaw),
                },
                "feeds": {
                    "camera_age_ms": color_age * 1000.0,
                    "odom_age_ms": odom_age * 1000.0,
                    "scan_age_ms": scan_age * 1000.0,
                    "camera_alive": color_age < 3.0,
                },
                "lidar": {
                    "closest_m": self._scan.get("closest_m", 0.0),
                    "range_count": self._scan.get("range_count", 0),
                },
                "map": {
                    "width": self._map_width,
                    "height": self._map_height,
                    "meta": self._map_meta or "WAITING",
                    "image_url": "/api/map.png",
                },
                "safety": {
                    "risk_level": "OVERRIDE" if self._safety_override else ("DANGER" if self._safety_latched else "OK"),
                    "stop_recommended": self._safety_latched and not self._safety_override,
                    "reasons": (["override"] if self._safety_override else (["safety_latched"] if self._safety_latched else [])),
                    "override": self._safety_override,
                },
                "teleop": {
                    "status": "active" if (now - self._last_teleop_stamp) < 1.0 else "idle",
                    "speeds": {"linear": 0.28, "lateral": 0.28, "angular": 0.85},
                    "last": {"source": self._last_teleop_source},
                    "mux": {"source": self._last_teleop_source},
                },
                "mission": {
                    "last_intent": self._mission_state.get("intent", ""),
                    "last_room": self._mission_state.get("room", ""),
                    "command": self._mission_state.get("state", ""),
                    "recent": self._mission_state.get("recent", []),
                },
                "memory": {
                    "mission_count": self._mission_state.get("mission_count", 0),
                    "location_count": self._mission_state.get("location_count", 0),
                    "recent_locations": self._mission_state.get("recent_locations", []),
                },
                "vlm": {"events": list(self._vlm_events)},
                "chat_history": list(self._chat_history),
                "logs": [],
            }
        return payload

    def lidar_snapshot(self) -> dict[str, Any]:
        with self._lock:
            scan = dict(self._scan)
        return scan

    def map_png(self) -> bytes | None:
        with self._lock:
            return self._map_png

    def color_jpeg(self) -> bytes | None:
        with self._lock:
            return self._color_jpeg

    def depth_jpeg(self) -> bytes | None:
        with self._lock:
            return self._depth_jpeg

    def depth_status(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            age = (now - self._depth_stamp) if self._depth_stamp else 9999.0
            return {
                "enabled": self._depth_enabled,
                "has_frame": self._depth_jpeg is not None,
                "age_s": age,
            }


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------


def build_app(node: DashboardNode) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_app: FastAPI):  # noqa: ARG001
        yield

    app = FastAPI(lifespan=lifespan)

    @app.get("/", response_class=HTMLResponse)
    async def root() -> HTMLResponse:
        return HTMLResponse(PAGE_HTML)

    @app.get("/health")
    async def health() -> dict[str, Any]:
        return {"ok": True, "ts": time.time()}

    @app.post("/api/heartbeat")
    async def heartbeat() -> dict[str, Any]:
        stamp = node.publish_heartbeat()
        return {"stamp": stamp}

    @app.get("/api/status")
    async def status() -> JSONResponse:
        return JSONResponse(node.snapshot())

    @app.get("/api/mission/status")
    async def mission_status() -> JSONResponse:
        with node._lock:  # noqa: SLF001
            return JSONResponse(dict(node._mission_state))  # noqa: SLF001

    @app.get("/api/map.png")
    async def map_png() -> Response:
        png = node.map_png()
        if not png:
            return Response(status_code=204)
        return Response(content=png, media_type="image/png")

    @app.get("/depth_status")
    async def depth_status() -> JSONResponse:
        return JSONResponse(node.depth_status())

    @app.get("/lidar_scan")
    async def lidar_scan() -> JSONResponse:
        return JSONResponse(node.lidar_snapshot())

    def _mjpeg_stream(getter, hz: float, placeholder_text: str):
        boundary = b"--frame"

        async def gen():
            import asyncio

            interval = 1.0 / max(hz, 1.0)
            while True:
                jpeg = getter()
                if jpeg is None:
                    img = np.zeros((180, 320, 3), dtype=np.uint8)
                    cv2.putText(
                        img,
                        placeholder_text,
                        (12, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
                    ok, buf = cv2.imencode(".jpg", img)
                    if ok:
                        jpeg = bytes(buf)
                if jpeg:
                    yield boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    yield str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n"
                await asyncio.sleep(interval)

        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/video_feed")
    async def video_feed():
        return _mjpeg_stream(node.color_jpeg, 8.0, "WAITING FOR CAMERA")

    @app.get("/depth_feed")
    async def depth_feed():
        return _mjpeg_stream(node.depth_jpeg, 1.0, "NO DEPTH (USB2?)")

    @app.post("/api/teleop")
    async def api_teleop(req: TeleopRequest) -> dict[str, Any]:
        node.publish_teleop(req)
        return {"ok": True}

    @app.post("/api/mission")
    async def api_mission(req: MissionRequest) -> dict[str, Any]:
        intent_room = parse_intent_and_room(req.command)
        node.publish_mission(intent_room["intent"], intent_room["room"], req.command)
        return {"ok": True, **intent_room}

    @app.post("/api/chat")
    async def api_chat(req: ChatRequest) -> dict[str, Any]:
        intent_room = parse_intent_and_room(req.message)
        node.publish_mission(intent_room["intent"], intent_room["room"], req.message)
        return {"ok": True, **intent_room}

    @app.post("/api/safety/reset")
    async def api_safety_reset() -> dict[str, Any]:
        ok, message = node.request_safety_reset()
        return {"ok": ok, "message": message}

    @app.post("/api/safety/override")
    async def api_safety_override(req: SafetyOverrideRequest) -> dict[str, Any]:
        node.publish_safety_override(req.enabled)
        return {"ok": True, "enabled": req.enabled}

    @app.post("/api/vlm/trigger")
    async def api_vlm_trigger(req: VlmTriggerRequest) -> dict[str, Any]:
        node.publish_vlm_trigger(req.prompt)
        return {"ok": True, "prompt": req.prompt}

    return app


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DashboardNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = build_app(node)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
