#!/usr/bin/env python3
"""TF bridge — republish a namespaced TF tree onto root /tf and /tf_static.

slam_toolbox and nav2 (run at root) need transforms on /tf and /tf_static.
The platform publishes them under /r100_0140/tf and /r100_0140/tf_static.
This node forwards them with the right QoS so transient_local subscribers
(slam_toolbox, costmaps) actually get the cached static frames.

Why not topic_tools relay? In Humble, relay's auto-QoS-match races against
the platform's publisher discovery and frequently ends up publishing
/tf_static with VOLATILE durability, which silently breaks SLAM's tf2
listener.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


_TF_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


_TF_STATIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TfBridge(Node):
    def __init__(self) -> None:
        super().__init__("tf_bridge")
        self.declare_parameter("source_tf", "/r100_0140/tf")
        self.declare_parameter("source_tf_static", "/r100_0140/tf_static")
        self.declare_parameter("output_tf", "/tf")
        self.declare_parameter("output_tf_static", "/tf_static")

        src_tf = self.get_parameter("source_tf").value
        src_static = self.get_parameter("source_tf_static").value
        out_tf = self.get_parameter("output_tf").value
        out_static = self.get_parameter("output_tf_static").value

        # /tf: dynamic transforms, RELIABLE+VOLATILE.
        self._pub_tf = self.create_publisher(TFMessage, out_tf, _TF_QOS)
        self.create_subscription(TFMessage, src_tf, self._on_tf, _TF_QOS)

        # /tf_static: latched, RELIABLE+TRANSIENT_LOCAL.
        self._pub_tf_static = self.create_publisher(TFMessage, out_static, _TF_STATIC_QOS)
        self.create_subscription(TFMessage, src_static, self._on_tf_static, _TF_STATIC_QOS)

        self.get_logger().info(
            f"tf_bridge ready: {src_tf} -> {out_tf}, {src_static} -> {out_static}"
        )

    def _on_tf(self, msg: TFMessage) -> None:
        self._pub_tf.publish(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._pub_tf_static.publish(msg)


def main() -> None:
    rclpy.init()
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
