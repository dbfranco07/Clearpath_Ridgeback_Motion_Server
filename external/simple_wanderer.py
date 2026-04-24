#!/usr/bin/env python3
"""
simple_wanderer.py
==================
Basic autonomous exploration for Ridgeback R100.
Drives forward, avoids obstacles using LiDAR, and wanders to expose
new areas for SLAM Toolbox to map.

Publishes to /cmd_vel_raw so the Jetson velocity gate applies safety
clamping before commands reach /r100_0140/cmd_vel.

If Jetson safety nodes are not yet running, temporarily change
CMD_VEL_TOPIC to '/r100_0140/cmd_vel' for bench testing only.
"""

import math
import random
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Publish here → Jetson gate → /r100_0140/cmd_vel
CMD_VEL_TOPIC = "/cmd_vel_raw"

# Safety thresholds
OBSTACLE_STOP_DIST  = 0.40   # m — stop and turn
OBSTACLE_SLOW_DIST  = 0.70   # m — slow down
FORWARD_SPEED       = 0.20   # m/s
SLOW_SPEED          = 0.10   # m/s
TURN_SPEED          = 0.30   # rad/s
WANDER_NOISE        = 0.08   # rad/s random yaw perturbation


class SimpleWanderer(Node):
    def __init__(self) -> None:
        super().__init__("simple_wanderer")

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.create_subscription(
            LaserScan,
            "/r100_0140/sensors/lidar2d_0/scan",
            self._scan_cb,
            10,
        )

        # Distances in key directions (initialised to safe "far" value)
        self._front = 2.0
        self._left  = 2.0
        self._right = 2.0

        # Control loop at 10 Hz
        self.create_timer(0.1, self._wander_cb)

        self.get_logger().info(
            f"Simple Wanderer started — publishing to {CMD_VEL_TOPIC}"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        """Extract minimum distances in front, left, and right sectors."""
        ranges = msg.ranges
        n = len(ranges)

        def sector_min(lo: float, hi: float) -> float:
            sl = ranges[int(n * lo) : int(n * hi)]
            valid = [
                r for r in sl
                if not math.isinf(r) and not math.isnan(r) and r > 0.05
            ]
            return min(valid) if valid else 3.0

        # Sectors tuned for Hokuyo UST-10LX 270° scan
        self._front = sector_min(0.44, 0.56)   # centre ±6%
        self._left  = sector_min(0.22, 0.34)   # left quarter
        self._right = sector_min(0.66, 0.78)   # right quarter

    def _wander_cb(self) -> None:
        """Decide motion based on obstacle distances."""
        twist = Twist()

        if self._front < OBSTACLE_STOP_DIST:
            # Blocked — rotate toward clearer side
            if self._left >= self._right:
                twist.angular.z = TURN_SPEED
                self.get_logger().info(
                    f"Obstacle {self._front:.2f}m → turning left"
                )
            else:
                twist.angular.z = -TURN_SPEED
                self.get_logger().info(
                    f"Obstacle {self._front:.2f}m → turning right"
                )

        elif self._front < OBSTACLE_SLOW_DIST:
            # Getting close — slow down, bias toward clearer side
            twist.linear.x = SLOW_SPEED
            twist.angular.z = (
                WANDER_NOISE if self._left >= self._right else -WANDER_NOISE
            )

        else:
            # Clear path — move forward with gentle random wander
            twist.linear.x = FORWARD_SPEED
            twist.angular.z = random.uniform(-WANDER_NOISE, WANDER_NOISE)

        self.cmd_pub.publish(twist)

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.get_logger().info("Wanderer stopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
