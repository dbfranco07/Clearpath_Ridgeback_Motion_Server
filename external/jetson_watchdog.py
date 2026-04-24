#!/usr/bin/env python3
"""
Jetson Safety Node 1 — Heartbeat Watchdog
==========================================
Listens for /pc_heartbeat (std_msgs/Bool) published by the PC's
MCP server every 0.5s. If silent for longer than heartbeat_timeout,
publishes zero-velocity to /r100_0140/cmd_vel to stop the robot.

This node publishes DIRECTLY to /r100_0140/cmd_vel (not /cmd_vel_raw)
because stop commands must always reach the robot regardless of whether
the velocity gate node is running.

Run:
    ros2 run ridgeback_image_motion jetson_watchdog

Or via launch file:
    ros2 launch ridgeback_image_motion jetson_safety.launch.py
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool


class JetsonWatchdog(Node):
    def __init__(self):
        super().__init__("jetson_watchdog")

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter("heartbeat_timeout", 2.0)
        self.declare_parameter("init_grace_period", 5.0)
        self.declare_parameter("cmd_vel_topic", "/r100_0140/cmd_vel")

        # Clamp heartbeat_timeout to a safe range
        MIN_TIMEOUT = 0.5
        MAX_TIMEOUT = 5.0
        raw_timeout = self.get_parameter("heartbeat_timeout").value
        self.heartbeat_timeout = max(MIN_TIMEOUT, min(MAX_TIMEOUT, raw_timeout))
        if raw_timeout != self.heartbeat_timeout:
            self.get_logger().warn(
                f"heartbeat_timeout {raw_timeout}s clamped to "
                f"{self.heartbeat_timeout}s (safe range "
                f"[{MIN_TIMEOUT}, {MAX_TIMEOUT}])"
            )

        self.init_grace_period = self.get_parameter("init_grace_period").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        # ── QoS ───────────────────────────────────────────────────────────
        # IMPORTANT: Use BEST_EFFORT here to match the MCP server publisher.
        # If this is RELIABLE and the publisher is BEST_EFFORT, FastRTPS
        # will refuse the match and no heartbeats will ever arrive — the
        # watchdog fires immediately and never recovers.
        hb_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # cmd_vel uses reliable delivery so stop commands are not dropped
        cmd_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── pub / sub ─────────────────────────────────────────────────────
        # Publishes stop directly to the robot — bypasses velocity gate
        # intentionally (safety stop must always reach the motors).
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, cmd_qos)

        self.create_subscription(
            Bool, "/pc_heartbeat", self._heartbeat_cb, hb_qos
        )

        # ── state ─────────────────────────────────────────────────────────
        self.last_heartbeat  = self.get_clock().now()
        self.estop_active    = False
        self.is_initialized  = False
        self.start_time      = self.get_clock().now()

        # ── watchdog timer (10 Hz) ────────────────────────────────────────
        self.create_timer(0.1, self._check_cb)

        self.get_logger().info(
            f"Jetson watchdog ready  "
            f"timeout={self.heartbeat_timeout}s  "
            f"grace={self.init_grace_period}s  "
            f"→ {cmd_vel_topic}"
        )

    # ── callbacks ──────────────────────────────────────────────────────────

    def _heartbeat_cb(self, msg: Bool) -> None:
        """Record time of most recent valid heartbeat."""
        if msg.data:
            self.last_heartbeat = self.get_clock().now()
            if self.estop_active:
                self.get_logger().info(
                    "Heartbeat restored — PC back online. E-stop cleared."
                )
                self.estop_active = False

    def _check_cb(self) -> None:
        """Publish stop if heartbeat has been silent too long."""
        # Startup grace period — allow PC to come online before activating
        if not self.is_initialized:
            elapsed = (
                self.get_clock().now() - self.start_time
            ).nanoseconds / 1e9
            if elapsed < self.init_grace_period:
                return
            self.is_initialized = True
            self.get_logger().info("Grace period complete — watchdog ACTIVE.")

        elapsed = (
            self.get_clock().now() - self.last_heartbeat
        ).nanoseconds / 1e9

        if elapsed > self.heartbeat_timeout:
            # Log only on transition, not every tick
            if not self.estop_active:
                self.get_logger().error(
                    f"HEARTBEAT LOST ({elapsed:.1f}s silent) — "
                    "publishing emergency stop."
                )
                self.estop_active = True
            self.cmd_pub.publish(Twist())   # zero-velocity stop


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetsonWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always send a final stop on exit
        node.get_logger().info("Watchdog shutting down — sending final stop.")
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
