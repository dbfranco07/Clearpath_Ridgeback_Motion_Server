#!/usr/bin/env python3
"""Standalone Jetson safety controller.

Two independent safety layers:

1. Network liveness latch (original): boots tripped, clears once the
   operator's browser heartbeat AND the Ridgeback platform liveness are
   both fresh. When latched, cmd_vel_mux clamps to slow-mode caps.

2. Holonomic LiDAR FOV veto: subscribes to the 2D scan, decomposes it
   into sectors, publishes a per-axis Boolean mask on /safety/fov_block
   describing which directions are unsafe to move (forward / lateral /
   rotation). cmd_vel_mux zeroes the matching velocity components. The
   Ridgeback's 270° LiDAR doesn't cover the rear ~90°, so minus_x is
   always reported clear — backing up is never blocked by FOV alone.

Reset via std_srvs/Trigger on /safety/reset (only honored once both
liveness conditions are healthy again — the FOV check is purely advisory
and is not part of the latch).
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

try:
    from ridgeback_image_motion.autonomy_common import json_dumps
except ImportError:
    from autonomy_common import json_dumps  # type: ignore[no-redef]


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


# Sector half-widths in radians, measured from base_link +x.
# "back" is intentionally absent — the Hokuyo 270° LiDAR doesn't see behind
# the robot, so backing up is never blocked by this check.
_SECTORS: tuple[tuple[str, float, float], ...] = (
    ("front",       -math.pi / 8.0,  math.pi / 8.0),
    ("front_left",   math.pi / 8.0,  3.0 * math.pi / 8.0),
    ("left",         3.0 * math.pi / 8.0,  5.0 * math.pi / 8.0),
    ("back_left",    5.0 * math.pi / 8.0,  7.0 * math.pi / 8.0),
    ("back_right",  -7.0 * math.pi / 8.0, -5.0 * math.pi / 8.0),
    ("right",       -5.0 * math.pi / 8.0, -3.0 * math.pi / 8.0),
    ("front_right", -3.0 * math.pi / 8.0, -math.pi / 8.0),
)


def _sector_of(angle: float) -> str | None:
    a = math.atan2(math.sin(angle), math.cos(angle))  # wrap to [-π, π]
    for name, lo, hi in _SECTORS:
        if lo <= a < hi:
            return name
    return None


class SafetyController(Node):
    def __init__(self) -> None:
        super().__init__("safety_controller")

        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("heartbeat_topic", "/operator/heartbeat")
        self.declare_parameter("liveness_topic", "/platform/liveness")
        self.declare_parameter("safety_latched_topic", "/safety/latched")
        self.declare_parameter("safety_reset_service", "/safety/reset")
        self.declare_parameter("safety_override_topic", "/safety/override")
        self.declare_parameter("heartbeat_timeout_s", 1.0)
        self.declare_parameter("liveness_timeout_s", 2.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("require_reset", True)
        # FOV obstacle veto
        self.declare_parameter("scan_topic", "")
        self.declare_parameter("fov_block_topic", "/safety/fov_block")
        self.declare_parameter("fov_min_clearance_forward_m", 0.45)
        self.declare_parameter("fov_min_clearance_lateral_m", 0.35)
        self.declare_parameter("fov_min_clearance_rot_m", 0.30)
        self.declare_parameter("fov_block_hysteresis_s", 0.30)
        self.declare_parameter("fov_scan_max_age_s", 0.5)

        self._heartbeat_timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self._liveness_timeout = float(self.get_parameter("liveness_timeout_s").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._require_reset = bool(self.get_parameter("require_reset").value)

        ns = self.get_parameter("namespace").value
        self._scan_topic = (
            self.get_parameter("scan_topic").value or f"/{ns}/sensors/lidar2d_0/scan"
        )
        self._clr_fwd = float(self.get_parameter("fov_min_clearance_forward_m").value)
        self._clr_lat = float(self.get_parameter("fov_min_clearance_lateral_m").value)
        self._clr_rot = float(self.get_parameter("fov_min_clearance_rot_m").value)
        self._fov_hysteresis = max(float(self.get_parameter("fov_block_hysteresis_s").value), 0.0)
        self._fov_max_age = max(float(self.get_parameter("fov_scan_max_age_s").value), 0.05)

        self._lock = threading.Lock()
        self._latched = True
        self._latch_published = False
        self._reasons: list[str] = ["startup"]
        self._heartbeat_stamp = 0.0
        self._liveness_stamp = 0.0
        self._liveness_value = False
        self._waiting_reset = False
        self._override = False
        # FOV check state.
        self._scan_stamp = 0.0
        self._fov_block: dict[str, bool] = {
            "plus_x": True, "minus_x": False,
            "plus_y": True, "minus_y": True,
            "plus_yaw": True, "minus_yaw": True,
        }
        self._fov_last_clear_at: dict[str, float] = {k: 0.0 for k in self._fov_block}
        self._fov_min_d: dict[str, float] = {name: float("inf") for name, _, _ in _SECTORS}

        self.create_subscription(
            Header,
            self.get_parameter("heartbeat_topic").value,
            self._on_heartbeat,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("liveness_topic").value,
            self._on_liveness,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("safety_override_topic").value,
            self._on_override,
            _LATCHED_QOS,
        )
        self._pub_latched = self.create_publisher(
            Bool, self.get_parameter("safety_latched_topic").value, _LATCHED_QOS
        )
        self._pub_fov = self.create_publisher(
            String, self.get_parameter("fov_block_topic").value, _LATCHED_QOS
        )
        self.create_service(
            Trigger,
            self.get_parameter("safety_reset_service").value,
            self._on_reset,
        )

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, sensor_qos)

        self._publish_latched()
        self._publish_fov_block()
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"safety_controller ready (heartbeat≤{self._heartbeat_timeout}s, "
            f"liveness≤{self._liveness_timeout}s, require_reset={self._require_reset}, "
            f"FOV scan={self._scan_topic}, clr_fwd={self._clr_fwd}m, "
            f"clr_lat={self._clr_lat}m, clr_rot={self._clr_rot}m)"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_heartbeat(self, msg: Header) -> None:
        with self._lock:
            self._heartbeat_stamp = self._now()

    def _on_liveness(self, msg: Bool) -> None:
        with self._lock:
            self._liveness_value = bool(msg.data)
            self._liveness_stamp = self._now()

    def _on_scan(self, msg: LaserScan) -> None:
        now = self._now()
        min_d: dict[str, float] = {name: float("inf") for name, _, _ in _SECTORS}
        amin = float(msg.angle_min)
        ainc = float(msg.angle_increment)
        rmin = float(msg.range_min) if msg.range_min else 0.0
        for i, r in enumerate(msg.ranges):
            r = float(r)
            if not math.isfinite(r) or r <= rmin:
                continue
            sector = _sector_of(amin + i * ainc)
            if sector is None:
                continue
            if r < min_d[sector]:
                min_d[sector] = r

        with self._lock:
            self._scan_stamp = now
            self._fov_min_d = min_d
            self._update_fov_block_locked(now)
        self._publish_fov_block()

    def _on_override(self, msg: Bool) -> None:
        new_value = bool(msg.data)
        with self._lock:
            changed = new_value != self._override
            self._override = new_value
        if changed:
            if new_value:
                self.get_logger().warn(
                    "SAFETY OVERRIDE ENABLED — latch forced clear; heartbeat/liveness ignored"
                )
            else:
                self.get_logger().info("safety override disabled — normal checks resumed")

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._try_clear()
        response.success = ok
        response.message = message
        return response

    # --- core loop ----------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
            if self._override:
                if self._latched:
                    self.get_logger().info("safety cleared via override")
                self._latched = False
                self._waiting_reset = False
                self._reasons = ["override"]
                self._publish_latched_locked()
                return

            now = self._now()
            reasons: list[str] = []
            heartbeat_age = now - self._heartbeat_stamp if self._heartbeat_stamp else 1e9
            liveness_age = now - self._liveness_stamp if self._liveness_stamp else 1e9
            if heartbeat_age > self._heartbeat_timeout:
                reasons.append("heartbeat_stale")
            if liveness_age > self._liveness_timeout:
                reasons.append("liveness_stale")
            elif not self._liveness_value:
                reasons.append("platform_down")

            if reasons:
                if not self._latched:
                    self.get_logger().warn(f"safety latched: {','.join(reasons)}")
                    self._waiting_reset = self._require_reset
                self._latched = True
                self._reasons = reasons
            else:
                if self._latched and not self._waiting_reset:
                    self.get_logger().info("safety cleared (auto)")
                    self._latched = False
                    self._reasons = []

            self._publish_latched_locked()
            # Re-evaluate FOV staleness on the publish tick so a missing scan
            # gets reported as fully-blocked even if no new scan arrived.
            self._update_fov_block_locked(now)
        self._publish_fov_block()
        # The clamping happens in cmd_vel_mux when /safety/latched=true.

    def _try_clear(self) -> tuple[bool, str]:
        with self._lock:
            now = self._now()
            heartbeat_age = now - self._heartbeat_stamp if self._heartbeat_stamp else 1e9
            liveness_age = now - self._liveness_stamp if self._liveness_stamp else 1e9
            if heartbeat_age > self._heartbeat_timeout:
                return False, f"heartbeat stale ({heartbeat_age:.2f}s)"
            if liveness_age > self._liveness_timeout:
                return False, f"liveness stale ({liveness_age:.2f}s)"
            if not self._liveness_value:
                return False, "platform_liveness=false"
            self._latched = False
            self._waiting_reset = False
            self._reasons = []
            self._publish_latched_locked()
            self.get_logger().info("safety cleared via /safety/reset")
            return True, "cleared"

    def _publish_latched(self) -> None:
        with self._lock:
            self._publish_latched_locked()

    def _publish_latched_locked(self) -> None:
        msg = Bool()
        msg.data = self._latched
        # Always republish to keep transient_local subscribers fresh.
        self._pub_latched.publish(msg)
        self._latch_published = True

    def _update_fov_block_locked(self, now: float) -> None:
        """Recompute the per-axis veto mask. Must be called with _lock held."""
        # Stale scan -> conservative: block everything except minus_x (we
        # can't see behind, so blocking reverse wouldn't be helpful anyway).
        if self._scan_stamp == 0.0 or (now - self._scan_stamp) > self._fov_max_age:
            desired = {
                "plus_x": True, "minus_x": False,
                "plus_y": True, "minus_y": True,
                "plus_yaw": True, "minus_yaw": True,
            }
        else:
            d = self._fov_min_d
            left_min = min(d["left"], d["front_left"], d["back_left"])
            right_min = min(d["right"], d["front_right"], d["back_right"])
            desired = {
                "plus_x": d["front"] < self._clr_fwd,
                "minus_x": False,
                "plus_y": left_min < self._clr_lat,
                "minus_y": right_min < self._clr_lat,
                # +yaw is CCW (toward +y), so the swept arc is the left half.
                "plus_yaw": min(d["front"], d["front_left"], d["left"]) < self._clr_rot,
                "minus_yaw": min(d["front"], d["front_right"], d["right"]) < self._clr_rot,
            }
        # Hysteresis: a sector must stay clear for fov_block_hysteresis_s
        # continuously before we drop the block. Going *into* block is
        # immediate; coming *out* of block is delayed.
        for axis, want_block in desired.items():
            currently_block = self._fov_block[axis]
            if want_block:
                self._fov_block[axis] = True
                self._fov_last_clear_at[axis] = 0.0
            else:
                if not currently_block:
                    continue
                if self._fov_last_clear_at[axis] == 0.0:
                    self._fov_last_clear_at[axis] = now
                if (now - self._fov_last_clear_at[axis]) >= self._fov_hysteresis:
                    self._fov_block[axis] = False
                    self._fov_last_clear_at[axis] = 0.0

    def _publish_fov_block(self) -> None:
        with self._lock:
            payload: dict[str, Any] = dict(self._fov_block)
            payload["ts"] = self._now()
            payload["scan_age_s"] = (
                (self._now() - self._scan_stamp) if self._scan_stamp else -1.0
            )
            payload["min_d"] = {k: (None if math.isinf(v) else round(v, 3))
                                for k, v in self._fov_min_d.items()}
        msg = String()
        msg.data = json_dumps(payload)
        self._pub_fov.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main() -> None:
    rclpy.init()
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
