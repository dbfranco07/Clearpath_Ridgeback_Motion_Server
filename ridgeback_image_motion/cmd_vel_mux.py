#!/usr/bin/env python3
"""Strict-priority cmd_vel mux with safety slow-mode and FOV veto.

Sources (highest -> lowest): teleop, nav. The mux always emits the
highest-priority fresh source. When /safety/latched=true the chosen
Twist is *clamped* to the configured slow-mode maxes instead of being
dropped — the robot stays steerable but rate-limited.

In addition, the safety_controller publishes a per-axis FOV veto mask on
/safety/fov_block. After clamping, the mux zeroes any velocity component
whose axis is currently blocked (e.g. plus_x blocked means linear.x > 0
is dropped to 0 while negative motion still passes). This catches the
holonomic strafe blind spot the slow-mode clamp never could.
"""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from ridgeback_image_motion.autonomy_common import json_loads
except ImportError:
    from autonomy_common import json_loads  # type: ignore[no-redef]


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class CmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("teleop_topic", "/cmd_vel/teleop")
        self.declare_parameter("nav_topic", "/cmd_vel/nav")
        self.declare_parameter("output_topic", "/cmd_vel/mux_out")
        self.declare_parameter("safety_latched_topic", "/safety/latched")
        self.declare_parameter("teleop_timeout_s", 0.4)
        self.declare_parameter("nav_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 50.0)
        # Slow-mode caps applied while /safety/latched=true. The robot is
        # still steerable, just rate-limited.
        self.declare_parameter("slow_max_linear", 0.10)
        self.declare_parameter("slow_max_lateral", 0.10)
        self.declare_parameter("slow_max_angular", 0.30)
        # Per-axis veto driven by the safety_controller LiDAR FOV check.
        self.declare_parameter("fov_block_topic", "/safety/fov_block")
        self.declare_parameter("fov_block_apply", True)

        self._teleop_timeout = float(self.get_parameter("teleop_timeout_s").value)
        self._nav_timeout = float(self.get_parameter("nav_timeout_s").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._slow_max = (
            float(self.get_parameter("slow_max_linear").value),
            float(self.get_parameter("slow_max_lateral").value),
            float(self.get_parameter("slow_max_angular").value),
        )
        self._fov_block_apply = bool(self.get_parameter("fov_block_apply").value)

        self._lock = threading.Lock()
        self._safety_latched = True
        self._latest = {
            "teleop": (Twist(), 0.0),
            "nav": (Twist(), 0.0),
        }
        self._last_source = "none"
        self._fov_block: dict[str, bool] = {
            "plus_x": False, "minus_x": False,
            "plus_y": False, "minus_y": False,
            "plus_yaw": False, "minus_yaw": False,
        }

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
        self.create_subscription(
            String,
            self.get_parameter("fov_block_topic").value,
            self._on_fov_block,
            _LATCHED_QOS,
        )

        self._pub = self.create_publisher(Twist, self.get_parameter("output_topic").value, 10)
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"cmd_vel_mux ready (priority teleop>nav, "
            f"timeouts teleop={self._teleop_timeout}s nav={self._nav_timeout}s, "
            f"slow_max={self._slow_max[0]:.2f}/{self._slow_max[1]:.2f} m/s, "
            f"{self._slow_max[2]:.2f} rad/s, fov_apply={self._fov_block_apply})"
        )

    def _make_cb(self, source: str):
        def _cb(msg: Twist) -> None:
            with self._lock:
                self._latest[source] = (msg, self._now())

        return _cb

    def _on_safety_latched(self, msg: Bool) -> None:
        with self._lock:
            self._safety_latched = bool(msg.data)

    def _on_fov_block(self, msg: String) -> None:
        payload = json_loads(msg.data, default={})
        with self._lock:
            for axis in self._fov_block:
                if axis in payload:
                    self._fov_block[axis] = bool(payload[axis])

    def _tick(self) -> None:
        now = self._now()
        with self._lock:
            chosen = Twist()
            chosen_source = "none"
            for source, timeout in (
                ("teleop", self._teleop_timeout),
                ("nav", self._nav_timeout),
            ):
                twist, stamp = self._latest[source]
                if stamp == 0.0:
                    continue
                if (now - stamp) <= timeout:
                    chosen = twist
                    chosen_source = source
                    break
            latched = self._safety_latched
            fov = dict(self._fov_block)
            fov_apply = self._fov_block_apply

        if latched:
            chosen = self._clamp(chosen, self._slow_max)
            chosen_source = f"{chosen_source}+slow"

        if fov_apply:
            chosen, vetoed = self._apply_fov(chosen, fov)
            if vetoed:
                chosen_source = f"{chosen_source}+fov[{','.join(vetoed)}]"

        self._pub.publish(chosen)
        with self._lock:
            self._last_source = chosen_source

    @staticmethod
    def _apply_fov(twist: Twist, block: dict[str, bool]) -> tuple[Twist, list[str]]:
        out = Twist()
        out.linear.x = float(twist.linear.x)
        out.linear.y = float(twist.linear.y)
        out.angular.z = float(twist.angular.z)
        vetoed: list[str] = []
        if out.linear.x > 0.0 and block.get("plus_x"):
            out.linear.x = 0.0
            vetoed.append("+x")
        if out.linear.x < 0.0 and block.get("minus_x"):
            out.linear.x = 0.0
            vetoed.append("-x")
        if out.linear.y > 0.0 and block.get("plus_y"):
            out.linear.y = 0.0
            vetoed.append("+y")
        if out.linear.y < 0.0 and block.get("minus_y"):
            out.linear.y = 0.0
            vetoed.append("-y")
        if out.angular.z > 0.0 and block.get("plus_yaw"):
            out.angular.z = 0.0
            vetoed.append("+yaw")
        if out.angular.z < 0.0 and block.get("minus_yaw"):
            out.angular.z = 0.0
            vetoed.append("-yaw")
        return out, vetoed

    @staticmethod
    def _clamp(twist: Twist, limits: tuple[float, float, float]) -> Twist:
        out = Twist()
        out.linear.x = max(-limits[0], min(limits[0], float(twist.linear.x)))
        out.linear.y = max(-limits[1], min(limits[1], float(twist.linear.y)))
        out.angular.z = max(-limits[2], min(limits[2], float(twist.angular.z)))
        return out

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
