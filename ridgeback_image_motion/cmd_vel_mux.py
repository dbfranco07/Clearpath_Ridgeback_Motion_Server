#!/usr/bin/env python3
"""Priority velocity mux for Jetson-owned Ridgeback autonomy."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from ridgeback_image_motion.autonomy_common import json_loads, twist_is_nonzero
except ImportError:
    from autonomy_common import json_loads, twist_is_nonzero


def clamp(value: float, limit: float) -> float:
    return max(-float(limit), min(float(limit), float(value)))


class CmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("safety_topic", "/cmd_vel_safety")
        self.declare_parameter("nav_topic", "/cmd_vel_nav")
        self.declare_parameter("teleop_topic", "/cmd_vel_teleop")
        self.declare_parameter("output_topic", "/r100_0140/cmd_vel")
        self.declare_parameter("safety_status_topic", "/ridgeback/safety/status")
        self.declare_parameter("nav_timeout_s", 0.75)
        self.declare_parameter("teleop_timeout_s", 0.35)
        self.declare_parameter("safety_timeout_s", 0.50)
        self.declare_parameter("max_linear_mps", 0.45)
        self.declare_parameter("max_lateral_mps", 0.35)
        self.declare_parameter("max_angular_rps", 0.80)
        self.declare_parameter("publish_hz", 20.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.output = self.create_publisher(Twist, self.get_parameter("output_topic").value, qos)
        self.create_subscription(Twist, self.get_parameter("safety_topic").value, self._safety_cb, qos)
        self.create_subscription(Twist, self.get_parameter("nav_topic").value, self._nav_cb, qos)
        self.create_subscription(Twist, self.get_parameter("teleop_topic").value, self._teleop_cb, qos)
        self.create_subscription(String, self.get_parameter("safety_status_topic").value, self._safety_status_cb, status_qos)

        self.last_safety = Twist()
        self.last_nav = Twist()
        self.last_teleop = Twist()
        self.last_safety_time = 0.0
        self.last_nav_time = 0.0
        self.last_teleop_time = 0.0
        self.safety_forced_stop = False
        self.last_source = "none"

        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info("cmd_vel_mux ready: safety > nav2 > teleop -> %s" % self.get_parameter("output_topic").value)

    def _safety_cb(self, msg: Twist) -> None:
        self.last_safety = msg
        self.last_safety_time = time.time()

    def _nav_cb(self, msg: Twist) -> None:
        self.last_nav = msg
        self.last_nav_time = time.time()

    def _teleop_cb(self, msg: Twist) -> None:
        self.last_teleop = msg
        self.last_teleop_time = time.time()

    def _safety_status_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        self.safety_forced_stop = not bool(payload.get("is_safe", True))

    def _tick(self) -> None:
        now = time.time()
        safety_age = now - self.last_safety_time if self.last_safety_time else 999.0
        nav_age = now - self.last_nav_time if self.last_nav_time else 999.0
        teleop_age = now - self.last_teleop_time if self.last_teleop_time else 999.0

        selected = Twist()
        source = "idle"
        if self.safety_forced_stop or safety_age <= float(self.get_parameter("safety_timeout_s").value):
            selected = Twist()
            source = "safety"
        elif nav_age <= float(self.get_parameter("nav_timeout_s").value) and twist_is_nonzero(self.last_nav):
            selected = self.last_nav
            source = "nav2"
        elif teleop_age <= float(self.get_parameter("teleop_timeout_s").value) and twist_is_nonzero(self.last_teleop):
            selected = self.last_teleop
            source = "teleop"

        safe = Twist()
        safe.linear.x = clamp(selected.linear.x, float(self.get_parameter("max_linear_mps").value))
        safe.linear.y = clamp(selected.linear.y, float(self.get_parameter("max_lateral_mps").value))
        safe.angular.z = clamp(selected.angular.z, float(self.get_parameter("max_angular_rps").value))
        self.output.publish(safe)

        if source != self.last_source:
            self.get_logger().info(f"cmd_vel source: {source}")
            self.last_source = source


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

