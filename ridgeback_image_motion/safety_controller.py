#!/usr/bin/env python3
"""Onboard Jetson safety controller for Ridgeback autonomy."""

from __future__ import annotations

import time
import threading
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

try:
    from ridgeback_image_motion.autonomy_common import ACTIVE_MISSION_STATES, json_dumps, json_loads
    from ridgeback_image_motion.vlm_client import load_vlm_config
except ImportError:
    from autonomy_common import ACTIVE_MISSION_STATES, json_dumps, json_loads
    from vlm_client import load_vlm_config


class SafetyController(Node):
    def __init__(self) -> None:
        super().__init__("safety_controller")

        self.declare_parameter("lidar_topic", "/r100_0140/sensors/lidar2d_0/scan")
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("emergency_stop_topic", "/r100_0140/platform/emergency_stop")
        self.declare_parameter("mission_status_topic", "/ridgeback/mission/status")
        self.declare_parameter("vlm_status_topic", "/ridgeback/vlm/status")
        self.declare_parameter("safety_cmd_topic", "/cmd_vel_safety")
        self.declare_parameter("status_topic", "/ridgeback/safety/status")
        self.declare_parameter("warning_distance_m", 0.80)
        self.declare_parameter("danger_distance_m", 0.45)
        self.declare_parameter("lidar_timeout_s", 1.0)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("vlm_timeout_s", 8.0)
        self.declare_parameter("check_vlm_network", True)
        self.declare_parameter("vlm_check_period_s", 5.0)
        self.declare_parameter("vlm_http_timeout_s", 1.5)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.stop_pub = self.create_publisher(Twist, self.get_parameter("safety_cmd_topic").value, qos)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, reliable_qos)
        self.create_subscription(LaserScan, self.get_parameter("lidar_topic").value, self._lidar_cb, qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, qos)
        self.create_subscription(Bool, self.get_parameter("emergency_stop_topic").value, self._estop_cb, reliable_qos)
        self.create_subscription(String, self.get_parameter("mission_status_topic").value, self._mission_status_cb, reliable_qos)
        self.create_subscription(String, self.get_parameter("vlm_status_topic").value, self._vlm_status_cb, reliable_qos)

        self.closest_obstacle_m = 99.0
        self.last_lidar_time = 0.0
        self.last_odom_time = 0.0
        self.estop_active = False
        self.mission_state = "IDLE"
        self.vlm_healthy = True
        self.last_vlm_status_time = 0.0
        self.network_healthy = True
        self.network_error = ""
        self.last_network_check = 0.0
        self.network_check_inflight = False

        self.create_timer(0.10, self._tick)
        self.get_logger().info("safety_controller ready")

    def _lidar_cb(self, msg: LaserScan) -> None:
        finite = [value for value in msg.ranges if msg.range_min <= value <= msg.range_max]
        self.closest_obstacle_m = min(finite) if finite else 99.0
        self.last_lidar_time = time.time()

    def _odom_cb(self, _: Odometry) -> None:
        self.last_odom_time = time.time()

    def _estop_cb(self, msg: Bool) -> None:
        self.estop_active = bool(msg.data)

    def _mission_status_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        self.mission_state = str(payload.get("state", "IDLE"))

    def _vlm_status_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        self.vlm_healthy = bool(payload.get("healthy", False))
        self.last_vlm_status_time = time.time()

    def _check_vlm_network(self) -> None:
        if not bool(self.get_parameter("check_vlm_network").value):
            self.network_healthy = True
            return
        now = time.time()
        period = float(self.get_parameter("vlm_check_period_s").value)
        if now - self.last_network_check < period:
            return
        if self.network_check_inflight:
            return
        self.last_network_check = now

        def worker() -> None:
            try:
                config = load_vlm_config()
                request = urllib.request.Request(config.openai_base_url.rstrip("/") + "/models", method="GET")
                with urllib.request.urlopen(request, timeout=float(self.get_parameter("vlm_http_timeout_s").value)) as response:
                    self.network_healthy = response.status < 500
                    self.network_error = "" if self.network_healthy else f"http {response.status}"
            except Exception as exc:
                self.network_healthy = False
                self.network_error = str(exc)[:160]
            finally:
                self.network_check_inflight = False

        self.network_check_inflight = True
        try:
            threading.Thread(target=worker, daemon=True).start()
        except Exception as exc:
            self.network_healthy = False
            self.network_error = str(exc)[:160]
            self.network_check_inflight = False

    def _tick(self) -> None:
        now = time.time()
        self._check_vlm_network()

        warning_distance = float(self.get_parameter("warning_distance_m").value)
        danger_distance = float(self.get_parameter("danger_distance_m").value)
        lidar_age = now - self.last_lidar_time if self.last_lidar_time else 999.0
        odom_age = now - self.last_odom_time if self.last_odom_time else 999.0
        vlm_age = now - self.last_vlm_status_time if self.last_vlm_status_time else 999.0
        mission_active = self.mission_state in ACTIVE_MISSION_STATES

        reasons: list[str] = []
        if self.closest_obstacle_m < danger_distance:
            reasons.append(f"obstacle {self.closest_obstacle_m:.2f}m")
        if lidar_age > float(self.get_parameter("lidar_timeout_s").value):
            reasons.append("stale_lidar")
        if odom_age > float(self.get_parameter("odom_timeout_s").value):
            reasons.append("stale_odom")
        if self.estop_active:
            reasons.append("robot_estop")
        if mission_active and (not self.network_healthy):
            reasons.append("vlm_network_lost")
        if mission_active and vlm_age < float(self.get_parameter("vlm_timeout_s").value) and not self.vlm_healthy:
            reasons.append("vlm_unhealthy")

        is_safe = not reasons
        if not is_safe:
            self.stop_pub.publish(Twist())

        risk = "OK"
        if reasons:
            risk = "DANGER"
        elif self.closest_obstacle_m < warning_distance:
            risk = "WARNING"

        status = {
            "stamp": now,
            "is_safe": is_safe,
            "risk_level": risk,
            "stop_recommended": not is_safe,
            "closest_obstacle_m": self.closest_obstacle_m,
            "warning_threshold_m": warning_distance,
            "danger_threshold_m": danger_distance,
            "warning_distance_m": warning_distance,
            "danger_distance_m": danger_distance,
            "lidar_age_s": lidar_age,
            "odom_age_s": odom_age,
            "mission_state": self.mission_state,
            "vlm_network_healthy": self.network_healthy,
            "vlm_status_healthy": self.vlm_healthy,
            "reasons": reasons,
            "network_error": self.network_error,
        }
        self.status_pub.publish(String(data=json_dumps(status)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
