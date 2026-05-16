#!/usr/bin/env python3
"""Standalone Jetson safety controller.

Fail-safe e-stop for the Ridgeback. Boots **tripped** so the robot cannot
move until the operator's browser is connected AND the watchdog reports
the Ridgeback platform is alive. Either input going stale re-trips the
latch. While tripped, publishes zero Twist on /cmd_vel/safety; the
cmd_vel mux gives this source absolute priority.

Reset via std_srvs/Trigger on /safety/reset (only honored once both
conditions are healthy again).
"""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class SafetyController(Node):
    def __init__(self) -> None:
        super().__init__("safety_controller")

        self.declare_parameter("heartbeat_topic", "/operator/heartbeat")
        self.declare_parameter("liveness_topic", "/platform/liveness")
        self.declare_parameter("safety_cmd_topic", "/cmd_vel/safety")
        self.declare_parameter("safety_latched_topic", "/safety/latched")
        self.declare_parameter("safety_reset_service", "/safety/reset")
        self.declare_parameter("heartbeat_timeout_s", 1.0)
        self.declare_parameter("liveness_timeout_s", 2.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("require_reset", True)

        self._heartbeat_timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self._liveness_timeout = float(self.get_parameter("liveness_timeout_s").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._require_reset = bool(self.get_parameter("require_reset").value)

        self._lock = threading.Lock()
        self._latched = True
        self._latch_published = False
        self._reasons: list[str] = ["startup"]
        self._heartbeat_stamp = 0.0
        self._liveness_stamp = 0.0
        self._liveness_value = False
        self._waiting_reset = False

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
        self._pub_safety = self.create_publisher(
            Twist, self.get_parameter("safety_cmd_topic").value, 10
        )
        self._pub_latched = self.create_publisher(
            Bool, self.get_parameter("safety_latched_topic").value, _LATCHED_QOS
        )
        self.create_service(
            Trigger,
            self.get_parameter("safety_reset_service").value,
            self._on_reset,
        )

        self._publish_latched()
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"safety_controller ready (heartbeat≤{self._heartbeat_timeout}s, "
            f"liveness≤{self._liveness_timeout}s, require_reset={self._require_reset})"
        )

    # --- callbacks ----------------------------------------------------------
    def _on_heartbeat(self, msg: Header) -> None:
        with self._lock:
            self._heartbeat_stamp = self._now()

    def _on_liveness(self, msg: Bool) -> None:
        with self._lock:
            self._liveness_value = bool(msg.data)
            self._liveness_stamp = self._now()

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._try_clear()
        response.success = ok
        response.message = message
        return response

    # --- core loop ----------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
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

        # Always publish zero Twist while latched.
        if self._latched:
            self._pub_safety.publish(Twist())

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
