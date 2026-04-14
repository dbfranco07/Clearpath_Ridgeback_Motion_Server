#!/usr/bin/env python3
"""
Simulation Topic Bridge
========================
Bridges TurtleBot3 Gazebo simulation topics to Ridgeback's namespaced topics.
This allows all autonomy nodes to run unchanged in simulation.

Bridged topics:
  /scan            → /r100_0140/sensors/lidar2d_0/scan
  /odom            → /r100_0140/platform/odom/filtered
  /camera/image_raw→ /r100_0140/sensors/camera_0/color/image (fake compressed)
  /r100_0140/cmd_vel → /cmd_vel  (robot command passthrough)

Only runs in simulation mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import cv2
import numpy as np


class SimTopicBridge(Node):
    def __init__(self):
        super().__init__('sim_topic_bridge')

        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # LiDAR bridge: /scan → /r100_0140/sensors/lidar2d_0/scan
        self.lidar_pub = self.create_publisher(
            LaserScan, '/r100_0140/sensors/lidar2d_0/scan', best_effort_qos
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', lambda m: self.lidar_pub.publish(m), best_effort_qos
        )

        # Odom bridge: /odom → /r100_0140/platform/odom/filtered
        self.odom_pub = self.create_publisher(
            Odometry, '/r100_0140/platform/odom/filtered', best_effort_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', lambda m: self.odom_pub.publish(m), best_effort_qos
        )

        # Cmd_vel bridge: /r100_0140/cmd_vel → /cmd_vel (in sim, drive the robot directly)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.cmd_sub = self.create_subscription(
            Twist, '/r100_0140/cmd_vel', lambda m: self.cmd_pub.publish(m), reliable_qos
        )

        # Image bridge: /camera/image_raw → compressed on /r100_0140/image/compressed
        self.img_pub = self.create_publisher(
            CompressedImage, '/r100_0140/image/compressed', best_effort_qos
        )
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, best_effort_qos
        )

        self.get_logger().info('Simulation topic bridge active')

    def _image_cb(self, msg: Image):
        try:
            # Convert raw image to compressed
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if msg.encoding == 'rgb8':
                img = data.reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = data.reshape((msg.height, msg.width, 3))

            _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.img_pub.publish(out)
        except Exception as e:
            self.get_logger().debug(f'Image bridge error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
