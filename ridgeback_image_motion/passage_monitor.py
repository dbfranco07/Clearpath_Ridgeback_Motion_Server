#!/usr/bin/env python3
"""Passage / doorway fit gate from the 2D LiDAR.

Continuously estimates the free lateral width of the opening directly ahead in
the robot's current travel direction. When that width drops below what the
robot needs to fit (body width + 2x safety margin), it reports `fits: false` on
/passage/status and — while the robot is actually being driven into the gap —
blacklists the spot via /frontier/blacklist_point so the explorer/Nav2 reroute
instead of clipping a frame the robot cannot clear.

This is a backstop, NOT the primary obstacle avoider (Nav2's costmap +
ObstacleFootprint critic own that). It exists to give the robot an explicit
"do I fit through this door?" judgement and to refuse partially-open doors or
clutter pinch-points that look passable to a point-goal planner. Doors here are
1.5-2x robot width, so it should rarely trip — it earns its keep on the edge
cases.

Status JSON on /passage/status:
    {width_m, required_m, fits, heading_rad, moving, ts}
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, quaternion_to_yaw_rad
except ImportError:
    from autonomy_common import json_dumps, quaternion_to_yaw_rad  # type: ignore[no-redef]


def passage_width(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    heading: float,
    forward_min: float,
    forward_max: float,
    lateral_window: float,
) -> float:
    """Free lateral width of the opening ahead, along `heading`.

    Pure function (numpy) so it can be unit-tested without ROS. Projects each
    valid scan return into the travel frame (forward along heading, left
    positive), keeps only points inside the forward window and lateral band,
    and returns nearest-left-clear + nearest-right-clear. Returns +inf when no
    obstacle bounds the corridor on a given side within the window.
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    n = ranges.size
    if n == 0:
        return float("inf")
    angles = angle_min + np.arange(n, dtype=np.float32) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(valid):
        return float("inf")
    r = ranges[valid]
    a = angles[valid]
    px = r * np.cos(a)
    py = r * np.sin(a)
    # Rotate into the travel frame (forward along heading, left positive).
    c, s = math.cos(heading), math.sin(heading)
    forward = px * c + py * s
    lateral = -px * s + py * c
    band = (
        (forward >= forward_min)
        & (forward <= forward_max)
        & (np.abs(lateral) <= lateral_window)
    )
    if not np.any(band):
        return float("inf")
    lat = lateral[band]
    left = lat[lat >= 0.0]
    right = lat[lat < 0.0]
    left_clear = float(np.min(left)) if left.size else lateral_window
    right_clear = float(-np.max(right)) if right.size else lateral_window
    return left_clear + right_clear


class PassageMonitor(Node):
    def __init__(self) -> None:
        super().__init__("passage_monitor")

        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("scan_topic", "")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("status_topic", "/passage/status")
        self.declare_parameter("blacklist_point_topic", "/frontier/blacklist_point")
        self.declare_parameter("home_frame", "map")
        self.declare_parameter("base_frame", "")
        # Geometry of the fit test. required = robot_width + 2*safety_margin.
        self.declare_parameter("robot_width_m", 0.80)
        self.declare_parameter("safety_margin_m", 0.15)
        # Forward window: assess the imminent opening, not a distant one.
        self.declare_parameter("forward_min_m", 0.15)
        self.declare_parameter("forward_max_m", 1.8)
        # Ignore obstacles farther sideways than this (wide-hall walls aren't
        # the doorway we're judging).
        self.declare_parameter("lateral_window_m", 1.5)
        self.declare_parameter("min_speed_mps", 0.03)
        # Require N consecutive not-fit reads before flagging, so a stray
        # return can't blacklist a genuine doorway.
        self.declare_parameter("block_streak_required", 3)
        self.declare_parameter("reroute_on_block", True)
        self.declare_parameter("publish_rate_hz", 5.0)

        ns = self.get_parameter("namespace").value
        self._scan_topic = (
            self.get_parameter("scan_topic").value or f"/{ns}/sensors/lidar2d_0/scan"
        )
        self._odom_topic = (
            self.get_parameter("odom_topic").value or f"/{ns}/platform/odom/filtered"
        )
        self._home_frame = self.get_parameter("home_frame").value
        self._base_frame = self.get_parameter("base_frame").value or "base_link"
        self._robot_width = float(self.get_parameter("robot_width_m").value)
        self._margin = float(self.get_parameter("safety_margin_m").value)
        self._required = self._robot_width + 2.0 * self._margin
        self._fwd_min = float(self.get_parameter("forward_min_m").value)
        self._fwd_max = float(self.get_parameter("forward_max_m").value)
        self._lat_window = float(self.get_parameter("lateral_window_m").value)
        self._min_speed = float(self.get_parameter("min_speed_mps").value)
        self._block_required = int(self.get_parameter("block_streak_required").value)
        self._reroute = bool(self.get_parameter("reroute_on_block").value)
        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        self._lock = threading.Lock()
        self._scan: LaserScan | None = None
        self._vx = 0.0
        self._vy = 0.0
        self._block_streak = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._pub_status = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self._pub_blacklist = self.create_publisher(
            String, self.get_parameter("blacklist_point_topic").value, 10
        )
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"passage_monitor ready (required_width={self._required:.2f} m, "
            f"forward={self._fwd_min:.2f}-{self._fwd_max:.2f} m, scan={self._scan_topic})"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._vx = float(msg.twist.twist.linear.x)
            self._vy = float(msg.twist.twist.linear.y)

    def _tick(self) -> None:
        with self._lock:
            scan = self._scan
            vx, vy = self._vx, self._vy
        if scan is None:
            return
        speed = math.hypot(vx, vy)
        moving = speed >= self._min_speed
        # Travel direction in the base/laser frame (laser is mounted rpy 0 on
        # chassis, so its x aligns with base forward). When essentially
        # stopped, assess straight ahead.
        heading = math.atan2(vy, vx) if moving else 0.0
        width = passage_width(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            heading,
            self._fwd_min,
            self._fwd_max,
            self._lat_window,
        )
        fits = width >= self._required

        # Only act on a not-fit while the robot is actually driving into it,
        # and only after a few consecutive reads.
        flagged = False
        if not fits and moving:
            self._block_streak += 1
            if self._block_streak >= self._block_required:
                flagged = True
        else:
            self._block_streak = 0

        self._publish_status(width, fits, heading, moving)
        if flagged and self._reroute:
            self._flag_blockage(heading)
            self._block_streak = 0

    def _flag_blockage(self, heading: float) -> None:
        # Project the gap point ~mid-corridor ahead into the map frame and
        # publish it for the frontier explorer to blacklist + reroute.
        try:
            tf = self._tf_buffer.lookup_transform(
                self._home_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return
        rx = float(tf.transform.translation.x)
        ry = float(tf.transform.translation.y)
        ryaw = quaternion_to_yaw_rad(tf.transform.rotation)
        mid = 0.5 * (self._fwd_min + self._fwd_max)
        # Gap point in base frame (along travel heading), then base->map.
        bx = mid * math.cos(heading)
        by = mid * math.sin(heading)
        wx = rx + bx * math.cos(ryaw) - by * math.sin(ryaw)
        wy = ry + bx * math.sin(ryaw) + by * math.cos(ryaw)
        msg = String()
        msg.data = json_dumps({"x": round(wx, 2), "y": round(wy, 2), "reason": "too_narrow"})
        self._pub_blacklist.publish(msg)
        self.get_logger().warn(
            f"opening ahead too narrow ({self._required:.2f} m needed); "
            f"flagging ({wx:.2f}, {wy:.2f}) for reroute"
        )

    def _publish_status(
        self, width: float, fits: bool, heading: float, moving: bool
    ) -> None:
        payload = {
            "width_m": None if math.isinf(width) else round(width, 2),
            "required_m": round(self._required, 2),
            "fits": bool(fits),
            "heading_rad": round(heading, 3),
            "moving": bool(moving),
            "ts": time.time(),
        }
        msg = String()
        msg.data = json_dumps(payload)
        self._pub_status.publish(msg)


def main() -> None:
    rclpy.init()
    node = PassageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
