#!/usr/bin/env python3
"""Ridgeback motion server.

Bridges the Jetson cmd_vel_mux output onto the Clearpath platform input,
clamps to safety limits, applies a watchdog (zeros if no fresh command),
and exposes a Motion service for one-shot velocity requests (used by the
dashboard teleop fallback and CLI testing).
"""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ridgeback_image_motion.srv import Motion


class MotionServer(Node):
    def __init__(self) -> None:
        super().__init__("motion_server")

        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("input_topic", "/cmd_vel/mux_out")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("max_linear", 0.6)
        self.declare_parameter("max_lateral", 0.6)
        self.declare_parameter("max_angular", 1.5)
        self.declare_parameter("watchdog_timeout_s", 0.3)
        self.declare_parameter("publish_rate_hz", 50.0)

        ns = self.get_parameter("namespace").value
        out_topic = (
            self.get_parameter("output_topic").value
            or f"/{ns}/platform/cmd_vel_unstamped"
        )
        self._max = (
            float(self.get_parameter("max_linear").value),
            float(self.get_parameter("max_lateral").value),
            float(self.get_parameter("max_angular").value),
        )
        self._watchdog = float(self.get_parameter("watchdog_timeout_s").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        self._lock = threading.Lock()
        self._latest = Twist()
        self._latest_stamp = 0.0
        self._zero_streak = 0

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Twist,
            self.get_parameter("input_topic").value,
            self._on_cmd,
            sensor_qos,
        )
        self._pub = self.create_publisher(Twist, out_topic, sensor_qos)
        self.create_service(Motion, "~/motion", self._on_motion_srv)
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"motion_server started: in={self.get_parameter('input_topic').value} -> out={out_topic} "
            f"max_lin={self._max[0]} max_lat={self._max[1]} max_ang={self._max[2]} watchdog={self._watchdog}s"
        )
        self.get_logger().info("Motion Service Server started")

    def _clamp(self, twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = max(-self._max[0], min(self._max[0], float(twist.linear.x)))
        out.linear.y = max(-self._max[1], min(self._max[1], float(twist.linear.y)))
        out.angular.z = max(-self._max[2], min(self._max[2], float(twist.angular.z)))
        return out

    def _on_cmd(self, msg: Twist) -> None:
        clamped = self._clamp(msg)
        with self._lock:
            self._latest = clamped
            self._latest_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _on_motion_srv(self, request: Motion.Request, response: Motion.Response) -> Motion.Response:
        twist = Twist()
        twist.linear.x = float(request.linear)
        twist.linear.y = float(request.lateral)
        twist.angular.z = float(request.angular)
        clamped = self._clamp(twist)
        with self._lock:
            self._latest = clamped
            self._latest_stamp = self.get_clock().now().nanoseconds * 1e-9
        response.success = True
        response.message = (
            f"motion x={clamped.linear.x:.2f} y={clamped.linear.y:.2f} w={clamped.angular.z:.2f}"
        )
        return response

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            stale = (now - self._latest_stamp) > self._watchdog
            twist = Twist() if stale else self._latest
        # Only publish a couple of zero frames after going stale to keep
        # platform queues clear without flooding the bus.
        if stale:
            if self._zero_streak >= 5:
                return
            self._zero_streak += 1
        else:
            self._zero_streak = 0
        self._pub.publish(twist)

    def shutdown(self) -> None:
        zero = Twist()
        for _ in range(10):
            self._pub.publish(zero)


def main() -> None:
    rclpy.init()
    node = MotionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
