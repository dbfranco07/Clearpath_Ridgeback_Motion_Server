#!/usr/bin/env python3
"""
Ridgeback MCP Server — FastMCP / streamable-http
=================================================
Replaces the original stdio-based ridgeback_mcp_server.py.

Local endpoint:  http://127.0.0.1:8010/mcp
OnIt config:     url: http://127.0.0.1:8010/mcp   (NOT port 8000 — that is Qwen3.5-27B)

Run:
    cd ~/ridgeback
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=0
    export RMW_FASTRTPS_USE_SHM=0
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/ridgeback/config/fastrtps_jetson.xml
    source ridgeback_mcp_env/bin/activate
    python3 ridgeback_mcp_server.py

Install deps (once):
    source ridgeback_mcp_env/bin/activate
    pip install fastmcp uvicorn httpx
"""

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool          # ← correct type for /pc_heartbeat

try:
    from fastmcp import FastMCP
except ImportError:
    raise ImportError("Run: pip install fastmcp uvicorn")

# ── Safety limits ────────────────────────────────────────────────────────────
MAX_LINEAR  = 0.5   # m/s
MAX_ANGULAR = 0.5   # rad/s
MAX_DURATION = 10.0  # seconds — cap single move commands

# ── Persistent landmark store ─────────────────────────────────────────────────
LANDMARK_FILE = os.path.expanduser("~/.ridgeback/landmarks.json")


def _load_landmarks() -> dict:
    os.makedirs(os.path.dirname(LANDMARK_FILE), exist_ok=True)
    if os.path.exists(LANDMARK_FILE):
        try:
            with open(LANDMARK_FILE) as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def _save_landmarks(data: dict) -> None:
    os.makedirs(os.path.dirname(LANDMARK_FILE), exist_ok=True)
    with open(LANDMARK_FILE, "w") as f:
        json.dump(data, f, indent=2)


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class RidgebackNode(Node):
    """
    Publishes to /cmd_vel_raw (Jetson velocity gate forwards to /r100_0140/cmd_vel).
    Publishes /pc_heartbeat (std_msgs/Bool) at 2 Hz for Jetson watchdog.
    Subscribes to /r100_0140/platform/odom/filtered for pose tracking.
    """

    def __init__(self):
        super().__init__("ridgeback_mcp_node")

        self.landmark_memory: dict    = _load_landmarks()
        self.current_pose: dict       = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.start_pose: dict | None  = None

        # Publish to /cmd_vel_raw — Jetson velocity gate sits between here
        # and /r100_0140/cmd_vel. If Jetson safety nodes are NOT running yet,
        # temporarily change this to /r100_0140/cmd_vel for testing only.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_raw", 10)

        # Heartbeat — std_msgs/Bool so Jetson watchdog_node receives it
        self.hb_pub = self.create_publisher(Bool, "/pc_heartbeat", 10)
        self.create_timer(0.5, lambda: self.hb_pub.publish(Bool(data=True)))

        # Odometry
        self.create_subscription(
            Odometry,
            "/r100_0140/platform/odom/filtered",
            self._odom_cb,
            10,
        )

        self.get_logger().info("✅ Ridgeback MCP node ready")
        self.get_logger().info("   cmd  → /cmd_vel_raw  (→ Jetson gate → /r100_0140/cmd_vel)")
        self.get_logger().info("   beat → /pc_heartbeat (std_msgs/Bool, 2 Hz)")

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_pose["x"] = msg.pose.pose.position.x
        self.current_pose["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_pose["theta"] = math.atan2(siny, cosy)
        if self.start_pose is None:
            self.start_pose = self.current_pose.copy()
            self.get_logger().info(f"📍 Start pose: {self.start_pose}")

    @staticmethod
    def _clamp(v: float, limit: float) -> float:
        return max(-limit, min(limit, float(v)))

    def publish_vel(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
    ) -> None:
        t = Twist()
        t.linear.x  = self._clamp(linear_x,  MAX_LINEAR)
        t.linear.y  = self._clamp(linear_y,   MAX_LINEAR)
        t.angular.z = self._clamp(angular_z,  MAX_ANGULAR)
        self.cmd_pub.publish(t)

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())


# ── Spin ROS2 in background ───────────────────────────────────────────────────
rclpy.init()
ros_node = RidgebackNode()
_executor = MultiThreadedExecutor()
_executor.add_node(ros_node)
threading.Thread(target=_executor.spin, daemon=True).start()

# ── FastMCP server ────────────────────────────────────────────────────────────
mcp = FastMCP(
    "ridgeback-mcp-server",
    instructions=(
        "You control a Clearpath Ridgeback R100 holonomic robot. "
        "It can move forward/backward, strafe sideways, rotate, and move diagonally. "
        "Always call stop() before changing direction. "
        "Default speed: 0.2 m/s. "
        "Use start_exploration() for autonomous mapping — "
        "do NOT send motion commands while exploration is running."
    ),
)


# ── Motion tools ──────────────────────────────────────────────────────────────

@mcp.tool()
def move(
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
    duration: float = 1.0,
) -> str:
    """
    Move the robot holonomically for a fixed duration then stop.

    Args:
        linear_x:  Forward (+) / backward (−) m/s. Clamped ±0.5.
        linear_y:  Strafe left (+) / right (−) m/s. Clamped ±0.5.
        angular_z: Rotate CCW (+) / CW (−) rad/s. Clamped ±0.5.
        duration:  Seconds to move (0.1 – 10.0).
    """
    duration = max(0.1, min(MAX_DURATION, float(duration)))
    ros_node.publish_vel(linear_x, linear_y, angular_z)
    time.sleep(duration)
    ros_node.stop()
    return (
        f"✅ Moved {duration:.1f}s: "
        f"x={ros_node._clamp(linear_x,MAX_LINEAR):.2f} "
        f"y={ros_node._clamp(linear_y,MAX_LINEAR):.2f} "
        f"ω={ros_node._clamp(angular_z,MAX_ANGULAR):.2f}"
    )


@mcp.tool()
def move_forward(distance: float = 1.0, speed: float = 0.2) -> str:
    """Move forward a given distance (m) at speed (m/s)."""
    return move(abs(speed), 0.0, 0.0, abs(distance) / abs(speed))


@mcp.tool()
def move_backward(distance: float = 1.0, speed: float = 0.2) -> str:
    """Move backward a given distance (m) at speed (m/s)."""
    return move(-abs(speed), 0.0, 0.0, abs(distance) / abs(speed))


@mcp.tool()
def strafe_left(distance: float = 1.0, speed: float = 0.2) -> str:
    """Strafe left a given distance (m) at speed (m/s)."""
    return move(0.0, abs(speed), 0.0, abs(distance) / abs(speed))


@mcp.tool()
def strafe_right(distance: float = 1.0, speed: float = 0.2) -> str:
    """Strafe right a given distance (m) at speed (m/s)."""
    return move(0.0, -abs(speed), 0.0, abs(distance) / abs(speed))


@mcp.tool()
def rotate(angle_deg: float = 90.0, speed: float = 0.3) -> str:
    """
    Rotate in place. Positive = CCW, negative = CW.
    angle_deg: degrees to rotate. speed: rad/s.
    """
    rad = math.radians(abs(angle_deg))
    duration = rad / abs(speed)
    angular_z = abs(speed) if angle_deg >= 0 else -abs(speed)
    return move(0.0, 0.0, angular_z, duration)


@mcp.tool()
def move_diagonal(
    forward_dist: float = 1.0,
    strafe_dist: float = 1.0,
    speed: float = 0.2,
) -> str:
    """
    Move diagonally — forward/backward and strafe simultaneously.
    forward_dist: metres, positive = forward. strafe_dist: metres, positive = left.
    """
    duration = max(abs(forward_dist), abs(strafe_dist)) / abs(speed)
    vx = abs(speed) if forward_dist >= 0 else -abs(speed)
    vy = abs(speed) if strafe_dist >= 0 else -abs(speed)
    return move(vx, vy, 0.0, duration)


@mcp.tool()
def stop() -> str:
    """Emergency stop — halt all motion immediately."""
    ros_node.stop()
    return "🛑 STOP"


# ── Pose ──────────────────────────────────────────────────────────────────────

@mcp.tool()
def get_pose() -> str:
    """Return current odometry pose: x (m), y (m), heading (degrees)."""
    p = ros_node.current_pose
    return (
        f"x={p['x']:.3f}m  y={p['y']:.3f}m  "
        f"heading={math.degrees(p['theta']):.1f}°"
    )


# ── Landmarks ─────────────────────────────────────────────────────────────────

@mcp.tool()
def record_landmark(room_number: str) -> str:
    """
    Save the robot's current position as a named landmark.
    room_number: the identifier seen on the door/wall sign, e.g. '101'.
    Persists across server restarts.
    """
    p = ros_node.current_pose.copy()
    ros_node.landmark_memory[room_number] = p
    _save_landmarks(ros_node.landmark_memory)
    return (
        f"✅ Saved '{room_number}': "
        f"x={p['x']:.3f} y={p['y']:.3f} θ={math.degrees(p['theta']):.1f}°"
    )


@mcp.tool()
def list_landmarks() -> str:
    """List all saved landmarks and their map positions."""
    if not ros_node.landmark_memory:
        return "No landmarks saved yet."
    lines = ["Saved landmarks:"]
    for name, p in ros_node.landmark_memory.items():
        lines.append(
            f"  {name}: x={p['x']:.2f} y={p['y']:.2f} "
            f"θ={math.degrees(p['theta']):.1f}°"
        )
    return "\n".join(lines)


@mcp.tool()
def navigate_to_landmark(room_number: str) -> str:
    """
    Navigate to a previously recorded landmark.
    Requires Nav2 to be running (Terminal 4 / complete_autonomy.launch.py).
    """
    if room_number not in ros_node.landmark_memory:
        known = list(ros_node.landmark_memory.keys())
        return f"❌ Unknown landmark '{room_number}'. Known: {known}"
    target = ros_node.landmark_memory[room_number]
    # TODO: wire Nav2 NavigateToPose action client here
    return (
        f"🎯 Target '{room_number}': x={target['x']:.2f} y={target['y']:.2f}. "
        "Nav2 action client not yet wired — see TODO in navigate_to_landmark()."
    )


@mcp.tool()
def return_home() -> str:
    """Navigate back to the position recorded at server startup."""
    if ros_node.start_pose is None:
        return "❌ Start pose not yet recorded — odometry not received."
    p = ros_node.start_pose
    # TODO: wire Nav2 NavigateToPose action client here
    return (
        f"🏠 Home: x={p['x']:.2f} y={p['y']:.2f}. "
        "Nav2 action client not yet wired — see TODO in return_home()."
    )


# ── Exploration ───────────────────────────────────────────────────────────────

@mcp.tool()
def start_exploration() -> str:
    """
    Start autonomous SLAM exploration.
    This is handled by complete_autonomy.launch.py (Terminal 4).
    The simple_wanderer node drives the robot while SLAM Toolbox builds the map.
    Call stop_wanderer() when you want to stop.
    """
    return (
        "ℹ️  Autonomous exploration is launched via Terminal 4:\n"
        "  ros2 launch ~/ridgeback/launch/complete_autonomy.launch.py\n"
        "The wanderer node will drive the robot automatically.\n"
        "Watch the map grow in RViz (Terminal 6)."
    )


@mcp.tool()
def get_status() -> str:
    """Get a full status summary: pose, start pose, and landmark count."""
    p = ros_node.current_pose
    sp = ros_node.start_pose
    return (
        f"🤖 Ridgeback status\n"
        f"   Pose:       x={p['x']:.2f} y={p['y']:.2f} "
        f"θ={math.degrees(p['theta']):.1f}°\n"
        f"   Start pose: {'recorded' if sp else 'not yet'}\n"
        f"   Landmarks:  {len(ros_node.landmark_memory)} saved\n"
        f"   Landmark file: {LANDMARK_FILE}"
    )


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    host = os.environ.get("MCP_HOST", "127.0.0.1")
    port = int(os.environ.get("MCP_PORT", "8010"))

    print("=" * 60)
    print("  Ridgeback MCP Server (FastMCP / streamable-http)")
    print("=" * 60)
    print(f"  Endpoint : http://{host}:{port}/mcp")
    print(f"  Landmarks: {LANDMARK_FILE}")
    print()
    print("  Port assignments:")
    print("    :8000  Qwen3.5-27B  (remote model server — ONIT_HOST)")
    print("    :8001  Qwen3.5-9B   (remote — VLM A2A)")
    print("    :8002  Qwen3.5-4B   (remote)")
    print(f"    :{port}  THIS MCP server (local)")
    print()
    print("  ridgeback_config.yaml → mcp.servers:")
    print(f"    url: http://{host}:{port}/mcp")
    print("=" * 60)

    mcp.run(transport="streamable-http", host=host, port=port)
