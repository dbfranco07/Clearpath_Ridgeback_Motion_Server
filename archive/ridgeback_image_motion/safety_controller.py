#!/usr/bin/env python3
"""Onboard Jetson safety controller for Ridgeback autonomy."""

from __future__ import annotations

import math
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
    from ridgeback_image_motion.autonomy_common import json_dumps, json_loads
    from ridgeback_image_motion.safety_policy import SafetyDecisionFilter, SafetyInputs, SafetyPolicyConfig
    from ridgeback_image_motion.vlm_client import load_vlm_config
except ImportError:
    from autonomy_common import json_dumps, json_loads
    from safety_policy import SafetyDecisionFilter, SafetyInputs, SafetyPolicyConfig
    from vlm_client import load_vlm_config


class SafetyController(Node):
    def __init__(self) -> None:
        super().__init__("safety_controller")

        self.declare_parameter("lidar_topic", "/r100_0140/sensors/lidar2d_0/scan")
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("emergency_stop_topic", "/r100_0140/platform/emergency_stop")
        self.declare_parameter("mission_status_topic", "/ridgeback/mission/status")
        self.declare_parameter("vlm_status_topic", "/ridgeback/vlm/status")
        self.declare_parameter("operator_heartbeat_topic", "/pc_heartbeat")
        self.declare_parameter("safety_cmd_topic", "/cmd_vel_safety")
        self.declare_parameter("status_topic", "/ridgeback/safety/status")
        self.declare_parameter("warning_distance_m", 0.80)
        self.declare_parameter("danger_distance_m", 0.45)
        self.declare_parameter("danger_release_distance_m", 0.60)
        self.declare_parameter("lidar_timeout_s", 1.0)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("vlm_timeout_s", 8.0)
        self.declare_parameter("operator_heartbeat_timeout_s", 2.0)
        self.declare_parameter("require_operator_heartbeat", True)
        self.declare_parameter("stop_hold_s", 1.0)
        self.declare_parameter("check_vlm_network", True)
        self.declare_parameter("vlm_check_period_s", 5.0)
        self.declare_parameter("vlm_http_timeout_s", 1.5)
        # Forward sector half-angle (radians) used for obstacle stop. Beams
        # whose bearing falls outside [-half, +half] of robot forward (scan
        # angle 0 on a chassis-mounted Hokuyo) are ignored, so a wall to the
        # side or behind the robot cannot trigger a stop on its own.
        self.declare_parameter("forward_sector_half_angle_rad", math.pi / 3.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.stop_pub = self.create_publisher(Twist, self.get_parameter("safety_cmd_topic").value, qos)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, reliable_qos)
        self.create_subscription(LaserScan, self.get_parameter("lidar_topic").value, self._lidar_cb, qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, qos)
        self.create_subscription(Bool, self.get_parameter("emergency_stop_topic").value, self._estop_cb, qos)
        self.create_subscription(Bool, self.get_parameter("operator_heartbeat_topic").value, self._heartbeat_cb, reliable_qos)
        self.create_subscription(String, self.get_parameter("mission_status_topic").value, self._mission_status_cb, reliable_qos)
        self.create_subscription(String, self.get_parameter("vlm_status_topic").value, self._vlm_status_cb, reliable_qos)

        self.policy = self._build_policy()
        self.closest_obstacle_m = 99.0
        self.last_lidar_time = 0.0
        self.last_odom_time = 0.0
        self.last_operator_heartbeat_time = 0.0
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
        half = float(self.get_parameter("forward_sector_half_angle_rad").value)
        closest = 99.0
        angle = msg.angle_min
        for value in msg.ranges:
            if msg.range_min <= value <= msg.range_max and abs(angle) <= half and value < closest:
                closest = value
            angle += msg.angle_increment
        self.closest_obstacle_m = closest
        self.last_lidar_time = time.time()

    def _odom_cb(self, _: Odometry) -> None:
        self.last_odom_time = time.time()

    def _estop_cb(self, msg: Bool) -> None:
        self.estop_active = bool(msg.data)

    def _heartbeat_cb(self, msg: Bool) -> None:
        if bool(msg.data):
            self.last_operator_heartbeat_time = time.time()

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

    def _build_policy(self) -> SafetyDecisionFilter:
        return SafetyDecisionFilter(
            SafetyPolicyConfig(
                warning_distance_m=float(self.get_parameter("warning_distance_m").value),
                danger_distance_m=float(self.get_parameter("danger_distance_m").value),
                danger_release_distance_m=float(self.get_parameter("danger_release_distance_m").value),
                lidar_timeout_s=float(self.get_parameter("lidar_timeout_s").value),
                odom_timeout_s=float(self.get_parameter("odom_timeout_s").value),
                vlm_timeout_s=float(self.get_parameter("vlm_timeout_s").value),
                operator_heartbeat_timeout_s=float(self.get_parameter("operator_heartbeat_timeout_s").value),
                stop_hold_s=float(self.get_parameter("stop_hold_s").value),
                require_operator_heartbeat=bool(self.get_parameter("require_operator_heartbeat").value),
            )
        )

    def _tick(self) -> None:
        now = time.time()
        self._check_vlm_network()

        warning_distance = float(self.get_parameter("warning_distance_m").value)
        danger_distance = float(self.get_parameter("danger_distance_m").value)
        decision = self.policy.evaluate(
            SafetyInputs(
                now=now,
                closest_obstacle_m=self.closest_obstacle_m,
                last_lidar_time=self.last_lidar_time,
                last_odom_time=self.last_odom_time,
                estop_active=self.estop_active,
                mission_state=self.mission_state,
                network_healthy=self.network_healthy,
                vlm_healthy=self.vlm_healthy,
                last_vlm_status_time=self.last_vlm_status_time,
                last_operator_heartbeat_time=self.last_operator_heartbeat_time,
            )
        )

        if not decision.is_safe:
            self.stop_pub.publish(Twist())

        status = {
            "stamp": now,
            "is_safe": decision.is_safe,
            "risk_level": decision.risk_level,
            "stop_recommended": decision.stop_recommended,
            "stop_latched": decision.stop_latched,
            "hold_remaining_s": decision.hold_remaining_s,
            "closest_obstacle_m": self.closest_obstacle_m,
            "warning_threshold_m": warning_distance,
            "danger_threshold_m": danger_distance,
            "danger_release_distance_m": decision.release_distance_m,
            "warning_distance_m": warning_distance,
            "danger_distance_m": danger_distance,
            "lidar_age_s": decision.lidar_age_s,
            "odom_age_s": decision.odom_age_s,
            "mission_state": self.mission_state,
            "vlm_network_healthy": self.network_healthy,
            "vlm_status_healthy": self.vlm_healthy,
            "operator_heartbeat_age_s": decision.operator_heartbeat_age_s,
            "operator_heartbeat_required": bool(self.get_parameter("require_operator_heartbeat").value),
            "reasons": decision.reasons,
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
