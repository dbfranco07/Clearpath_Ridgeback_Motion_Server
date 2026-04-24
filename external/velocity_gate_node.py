#!/usr/bin/env python3
"""
Jetson Safety Node 2 — Velocity Gate
=====================================
Intercepts ALL motion commands from the PC on /cmd_vel_raw,
clamps them to safe limits, and republishes to /r100_0140/cmd_vel.

This means EVERY command from the PC (MCP server, Nav2, wanderer)
passes through this node before reaching the robot's motors.
Even if software on the PC sends speed=99.0, the robot receives
at most MAX_LINEAR m/s.

The heartbeat watchdog publishes directly to /r100_0140/cmd_vel
(bypassing this gate) — that is intentional. Stop commands should
never be intercepted.

Run:
    ros2 run ridgeback_image_motion velocity_gate

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

# Default limits — match ridgeback_mcp_server.py
DEFAULT_MAX_LINEAR  = 0.5   # m/s
DEFAULT_MAX_ANGULAR = 0.5   # rad/s


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


class VelocityGate(Node):
    def __init__(self):
        super().__init__("velocity_gate")

        # ── parameters (tunable at runtime) ───────────────────────────────
        self.declare_parameter("max_linear",   DEFAULT_MAX_LINEAR)
        self.declare_parameter("max_angular",  DEFAULT_MAX_ANGULAR)
        self.declare_parameter("input_topic",  "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/r100_0140/cmd_vel")
        self.declare_parameter("log_clamps",   True)

        input_topic  = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # ── QoS ───────────────────────────────────────────────────────────
        # BEST_EFFORT matches the MCP server and Nav2 publishers.
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── pub / sub ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, output_topic, qos)
        self.create_subscription(Twist, input_topic, self._gate_cb, qos)

        self.get_logger().info(
            f"Velocity gate ready  "
            f"{input_topic} → {output_topic}  "
            f"max_linear={self.get_parameter('max_linear').value} m/s  "
            f"max_angular={self.get_parameter('max_angular').value} rad/s"
        )

    def _gate_cb(self, msg: Twist) -> None:
        max_lin = self.get_parameter("max_linear").value
        max_ang = self.get_parameter("max_angular").value
        log     = self.get_parameter("log_clamps").value

        safe = Twist()
        safe.linear.x  = _clamp(msg.linear.x,  max_lin)
        safe.linear.y  = _clamp(msg.linear.y,  max_lin)   # strafe
        safe.linear.z  = 0.0
        safe.angular.x = 0.0
        safe.angular.y = 0.0
        safe.angular.z = _clamp(msg.angular.z, max_ang)

        if log and (
            safe.linear.x  != msg.linear.x  or
            safe.linear.y  != msg.linear.y  or
            safe.angular.z != msg.angular.z
        ):
            self.get_logger().warn(
                f"Clamped: "
                f"vx {msg.linear.x:.2f}→{safe.linear.x:.2f}  "
                f"vy {msg.linear.y:.2f}→{safe.linear.y:.2f}  "
                f"ω {msg.angular.z:.2f}→{safe.angular.z:.2f}"
            )

        self._pub.publish(safe)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
