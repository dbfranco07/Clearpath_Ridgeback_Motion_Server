#!/usr/bin/env python3
"""Jetson heartbeat watchdog for Ridgeback network-loss safety."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class JetsonWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("jetson_watchdog")

        self.declare_parameter("heartbeat_timeout", 2.0)
        self.declare_parameter("init_grace_period", 5.0)
        self.declare_parameter("cmd_vel_topic", "/r100_0140/cmd_vel")
        self.declare_parameter("heartbeat_topic", "/pc_heartbeat")

        min_timeout = 0.5
        max_timeout = 5.0
        raw_timeout = float(self.get_parameter("heartbeat_timeout").value)
        self.heartbeat_timeout = max(min_timeout, min(max_timeout, raw_timeout))
        if raw_timeout != self.heartbeat_timeout:
            self.get_logger().warn(
                f"heartbeat_timeout {raw_timeout}s clamped to {self.heartbeat_timeout}s"
            )

        self.init_grace_period = float(self.get_parameter("init_grace_period").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)

        heartbeat_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, cmd_qos)
        self.create_subscription(Bool, heartbeat_topic, self._heartbeat_cb, heartbeat_qos)

        self.last_heartbeat = self.get_clock().now()
        self.estop_active = False
        self.is_initialized = False
        self.start_time = self.get_clock().now()

        self.create_timer(0.1, self._check_cb)
        self.get_logger().info(
            f"Jetson watchdog ready timeout={self.heartbeat_timeout}s "
            f"grace={self.init_grace_period}s heartbeat={heartbeat_topic} stop={cmd_vel_topic}"
        )

    def _heartbeat_cb(self, msg: Bool) -> None:
        if msg.data:
            self.last_heartbeat = self.get_clock().now()
            if self.estop_active:
                self.get_logger().info("Heartbeat restored. Watchdog stop cleared.")
                self.estop_active = False

    def _check_cb(self) -> None:
        now = self.get_clock().now()
        if not self.is_initialized:
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed < self.init_grace_period:
                return
            self.is_initialized = True
            self.get_logger().info("Grace period complete. Watchdog active.")

        elapsed = (now - self.last_heartbeat).nanoseconds / 1e9
        if elapsed > self.heartbeat_timeout:
            if not self.estop_active:
                self.get_logger().error(
                    f"Heartbeat lost ({elapsed:.1f}s silent). Publishing emergency stop."
                )
                self.estop_active = True
            self.cmd_pub.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JetsonWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Watchdog shutting down. Sending final stop.")
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
