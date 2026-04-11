#!/usr/bin/env python3
"""
Image Publisher Node for Ridgeback R100
Subscribes to RealSense raw Image topic and re-publishes as CompressedImage (JPEG)
This provides efficient bandwidth for streaming to the web controller.
Runs on: Ridgeback R100
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Parameters
        self.declare_parameter('image_topic', '/r100_0140/sensors/camera_0/color/image')
        self.declare_parameter('compressed_topic', '/r100_0140/image/compressed')
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('max_fps', 15.0)

        image_topic = self.get_parameter('image_topic').value
        compressed_topic = self.get_parameter('compressed_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        max_fps = self.get_parameter('max_fps').value

        self.bridge = CvBridge()
        self.min_interval = 1.0 / max_fps
        self.last_publish_time = 0.0

        # Publisher for compressed images
        self.publisher_ = self.create_publisher(CompressedImage, compressed_topic, 1)

        # Subscriber to raw RealSense images
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos
        )

        self.depth_pub = self.create_publisher(
            CompressedImage, '/r100_0140/image/depth_compressed', 10
        )
        self.create_subscription(
            Image,
            '/r100_0140/sensors/camera_0/depth/image',
            self._depth_cb,
            sensor_qos
        )

        self.frame_count = 0

        self.get_logger().info('Image Publisher started')
        self.get_logger().info(f'  Subscribing to: {image_topic}')
        self.get_logger().info(f'  Publishing to: {compressed_topic}')
        self.get_logger().info(f'  JPEG quality: {self.jpeg_quality}, Max FPS: {max_fps}')

    def image_callback(self, msg):
        # Rate limit
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_publish_time < self.min_interval:
            return
        self.last_publish_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Compress to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, compressed = cv2.imencode('.jpg', cv_image, encode_param)

            # Create CompressedImage message
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = "jpeg"
            comp_msg.data = np.array(compressed).tobytes()

            self.publisher_.publish(comp_msg)

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} compressed frames')

        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def _depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')  # uint16
            success, buf = cv2.imencode('.png', depth)
            if success:
                out = CompressedImage()
                out.header = msg.header
                out.format = '16UC1; png'
                out.data = buf.tobytes()
                self.depth_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')


def main(args=None):
    print("=" * 50)
    print("Ridgeback R100 - Image Publisher")
    print("=" * 50)

    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
