#!/usr/bin/env python3
"""Jetson-side watchdog for the Ridgeback platform.

Two health signals are combined into /platform/liveness (Bool):
  - Ridgeback ICMP ping reachable.
  - Recent platform status / odom topic activity.

If either degrades for liveness_grace_s, /platform/liveness=false. The
safety_controller subscribes to that and trips the e-stop latch.
"""

from __future__ import annotations

import shutil
import subprocess
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    from clearpath_platform_msgs.msg import Status as PlatformStatus  # type: ignore[import-not-found]
except ImportError:  # missing on sim machines without clearpath drivers
    PlatformStatus = None  # type: ignore[assignment]
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class JetsonWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("jetson_watchdog")

        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("ridgeback_ip", "192.168.131.1")
        self.declare_parameter("liveness_topic", "/platform/liveness")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("ping_period_s", 1.0)
        self.declare_parameter("liveness_grace_s", 2.5)
        self.declare_parameter("ping_timeout_s", 0.7)

        ns = self.get_parameter("namespace").value
        self._ridgeback_ip = self.get_parameter("ridgeback_ip").value
        self._status_topic = (
            self.get_parameter("status_topic").value or f"/{ns}/platform/mcu/status"
        )
        self._odom_topic = (
            self.get_parameter("odom_topic").value or f"/{ns}/platform/odom/filtered"
        )
        self._ping_period = float(self.get_parameter("ping_period_s").value)
        self._grace = float(self.get_parameter("liveness_grace_s").value)
        self._ping_timeout = float(self.get_parameter("ping_timeout_s").value)

        self._lock = threading.Lock()
        self._last_status_stamp = 0.0
        self._last_odom_stamp = 0.0
        self._last_ping_ok_stamp = 0.0
        self._published_state: bool | None = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        if PlatformStatus is not None:
            self.create_subscription(PlatformStatus, self._status_topic, self._on_status, sensor_qos)
        else:
            self.get_logger().warn(
                "clearpath_platform_msgs not installed; falling back to ping+odom liveness"
            )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self._pub_liveness = self.create_publisher(
            Bool, self.get_parameter("liveness_topic").value, 10
        )
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self._ping_path = shutil.which("ping") or "/bin/ping"
        threading.Thread(target=self._ping_loop, daemon=True).start()
        self._timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"jetson_watchdog ready (ridgeback={self._ridgeback_ip}, grace={self._grace}s)"
        )

    # --- ROS callbacks ------------------------------------------------------
    def _on_status(self, _msg) -> None:
        with self._lock:
            self._last_status_stamp = self._now()

    def _on_odom(self, _msg: Odometry) -> None:
        with self._lock:
            self._last_odom_stamp = self._now()

    # --- ping loop ----------------------------------------------------------
    def _ping_loop(self) -> None:
        while rclpy.ok():
            try:
                proc = subprocess.run(
                    [self._ping_path, "-c", "1", "-W", str(int(max(1, self._ping_timeout))), self._ridgeback_ip],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=self._ping_timeout + 0.5,
                )
                if proc.returncode == 0:
                    with self._lock:
                        self._last_ping_ok_stamp = self._now()
            except Exception:
                pass
            time.sleep(self._ping_period)

    # --- main loop ----------------------------------------------------------
    def _tick(self) -> None:
        now = self._now()
        with self._lock:
            ping_age = now - self._last_ping_ok_stamp if self._last_ping_ok_stamp else 1e9
            status_age = now - self._last_status_stamp if self._last_status_stamp else 1e9
            odom_age = now - self._last_odom_stamp if self._last_odom_stamp else 1e9

        topic_age = min(status_age, odom_age)
        alive = (ping_age <= self._grace) and (topic_age <= self._grace)

        msg = Bool()
        msg.data = bool(alive)
        self._pub_liveness.publish(msg)
        self._publish_diag(alive, ping_age, status_age, odom_age)

        if self._published_state is None or self._published_state != alive:
            self._published_state = alive
            msg_text = (
                f"platform liveness={alive} (ping={ping_age:.1f}s "
                f"status={status_age:.1f}s odom={odom_age:.1f}s)"
            )
            if alive:
                self.get_logger().info(msg_text)
            else:
                self.get_logger().warn(msg_text)

    def _publish_diag(self, alive: bool, ping_age: float, status_age: float, odom_age: float) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        d = DiagnosticStatus()
        d.name = "ridgeback_platform"
        d.hardware_id = self._ridgeback_ip
        d.level = DiagnosticStatus.OK if alive else DiagnosticStatus.ERROR
        d.message = "alive" if alive else "stale"
        d.values = [
            KeyValue(key="ping_age_s", value=f"{ping_age:.2f}"),
            KeyValue(key="status_age_s", value=f"{status_age:.2f}"),
            KeyValue(key="odom_age_s", value=f"{odom_age:.2f}"),
        ]
        arr.status.append(d)
        self._pub_diag.publish(arr)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main() -> None:
    rclpy.init()
    node = JetsonWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
