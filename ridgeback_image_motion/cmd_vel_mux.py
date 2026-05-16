#!/usr/bin/env python3
"""Strict-priority cmd_vel mux.

Sources (highest -> lowest): safety, teleop, nav. While the safety latch
is true, only the safety source is forwarded — even if it is publishing
zero, that overrides everything else. Otherwise, the highest-priority
source with a fresh sample wins.
"""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class CmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("safety_topic", "/cmd_vel/safety")
        self.declare_parameter("teleop_topic", "/cmd_vel/teleop")
        self.declare_parameter("nav_topic", "/cmd_vel/nav")
        self.declare_parameter("output_topic", "/cmd_vel/mux_out")
        self.declare_parameter("safety_latched_topic", "/safety/latched")
        self.declare_parameter("teleop_timeout_s", 0.4)
        self.declare_parameter("nav_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 50.0)

        self._teleop_timeout = float(self.get_parameter("teleop_timeout_s").value)
        self._nav_timeout = float(self.get_parameter("nav_timeout_s").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        self._lock = threading.Lock()
        self._safety_latched = True
        self._latest = {
            "safety": (Twist(), 0.0),
            "teleop": (Twist(), 0.0),
            "nav": (Twist(), 0.0),
        }
        self._last_source = "none"

        self.create_subscription(
            Twist, self.get_parameter("safety_topic").value, self._make_cb("safety"), 10
        )
        self.create_subscription(
            Twist, self.get_parameter("teleop_topic").value, self._make_cb("teleop"), 10
        )
        self.create_subscription(
            Twist, self.get_parameter("nav_topic").value, self._make_cb("nav"), 10
        )
        self.create_subscription(
            Bool,
            self.get_parameter("safety_latched_topic").value,
            self._on_safety_latched,
            _LATCHED_QOS,
        )

        self._pub = self.create_publisher(Twist, self.get_parameter("output_topic").value, 10)
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"cmd_vel_mux ready (priority safety>teleop>nav, "
            f"timeouts teleop={self._teleop_timeout}s nav={self._nav_timeout}s)"
        )

    def _make_cb(self, source: str):
        def _cb(msg: Twist) -> None:
            with self._lock:
                self._latest[source] = (msg, self._now())

        return _cb

    def _on_safety_latched(self, msg: Bool) -> None:
        with self._lock:
            self._safety_latched = bool(msg.data)

    def _tick(self) -> None:
        now = self._now()
        with self._lock:
            if self._safety_latched:
                twist, _ = self._latest["safety"]
                # Force zero if no safety sample yet.
                self._pub.publish(twist if twist is not None else Twist())
                self._last_source = "safety"
                return

            for source, timeout in (
                ("teleop", self._teleop_timeout),
                ("nav", self._nav_timeout),
            ):
                twist, stamp = self._latest[source]
                if stamp == 0.0:
                    continue
                if (now - stamp) <= timeout:
                    self._pub.publish(twist)
                    self._last_source = source
                    return

            # No fresh source — publish zero to keep the watchdog fed.
            self._pub.publish(Twist())
            self._last_source = "none"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main() -> None:
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
