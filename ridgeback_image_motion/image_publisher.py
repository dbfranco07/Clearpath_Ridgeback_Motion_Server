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
        self.declare_parameter('depth_image_topic', '/r100_0140/sensors/camera_0/depth/image')
        self.declare_parameter('depth_compressed_topic', '/r100_0140/image/depth_compressed')
        self.declare_parameter('jpeg_quality', 65)
        self.declare_parameter('max_fps', 10.0)
        self.declare_parameter('depth_max_fps', 1.0)
        self.declare_parameter('depth_png_compression', 3)
        self.declare_parameter('max_width', 640)
        self.declare_parameter('depth_max_width', 320)

        image_topic = self.get_parameter('image_topic').value
        compressed_topic = self.get_parameter('compressed_topic').value
        depth_image_topic = self.get_parameter('depth_image_topic').value
        depth_compressed_topic = self.get_parameter('depth_compressed_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        max_fps = self.get_parameter('max_fps').value
        depth_max_fps = self.get_parameter('depth_max_fps').value
        self.depth_png_compression = int(self.get_parameter('depth_png_compression').value)
        self.max_width = int(self.get_parameter('max_width').value)
        self.depth_max_width = int(self.get_parameter('depth_max_width').value)

        self.bridge = CvBridge()
        self.min_interval = 1.0 / max(0.1, float(max_fps))
        self.depth_min_interval = 1.0 / max(0.1, float(depth_max_fps))
        self.last_publish_time = 0.0
        self.last_depth_publish_time = 0.0
        self.frame_count = 0
        self.depth_frame_count = 0

        # Sensor QoS: BEST_EFFORT matches RealSense raw publishers (sub side)
        # and avoids reliable-retransmit failures over WiFi for the compressed
        # republished topics (pub side, consumed by the Jetson dashboard).
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(CompressedImage, compressed_topic, sensor_qos)

        self.image_subscription = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos
        )

        self.depth_pub = None
        self.depth_subscription = None
        if depth_image_topic and depth_compressed_topic:
            self.depth_pub = self.create_publisher(
                CompressedImage, depth_compressed_topic, sensor_qos
            )
            self.depth_subscription = self.create_subscription(
                Image,
                depth_image_topic,
                self._depth_cb,
                sensor_qos
            )

        self.get_logger().info('Image Publisher started')
        self.get_logger().info(f'  Subscribing to: {image_topic}')
        self.get_logger().info(f'  Publishing to: {compressed_topic}')
        self.get_logger().info(
            f'  JPEG quality: {self.jpeg_quality}, Max FPS: {max_fps}, Max width: {self.max_width}'
        )
        if self.depth_pub is not None:
            self.get_logger().info(f'  Depth subscribing to: {depth_image_topic}')
            self.get_logger().info(f'  Depth publishing to: {depth_compressed_topic}')
            self.get_logger().info(
                f'  Depth PNG compression: {self.depth_png_compression}, '
                f'Max FPS: {depth_max_fps}, Max width: {self.depth_max_width}'
            )
        else:
            self.get_logger().warn('Depth republisher disabled because depth topics are empty')

    @staticmethod
    def _resize_to_max_width(image: np.ndarray, max_width: int, interpolation: int) -> np.ndarray:
        if max_width <= 0 or image.ndim < 2:
            return image
        height, width = image.shape[:2]
        if width <= max_width:
            return image
        scale = float(max_width) / float(width)
        new_height = max(1, int(round(height * scale)))
        return cv2.resize(image, (int(max_width), new_height), interpolation=interpolation)

    def image_callback(self, msg):
        # Rate limit
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_publish_time < self.min_interval:
            return
        self.last_publish_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = self._resize_to_max_width(cv_image, self.max_width, cv2.INTER_AREA)

            # Compress to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, compressed = cv2.imencode('.jpg', cv_image, encode_param)
            if not success:
                self.get_logger().warn('RGB JPEG compression failed; dropping frame')
                return

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
        if self.depth_pub is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_depth_publish_time < self.depth_min_interval:
            return
        self.last_depth_publish_time = now

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            depth = np.asarray(depth)
            if np.issubdtype(depth.dtype, np.floating):
                depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
                depth = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
            elif depth.dtype != np.uint16:
                depth = np.clip(depth, 0, 65535).astype(np.uint16)
            depth = self._resize_to_max_width(depth, self.depth_max_width, cv2.INTER_NEAREST)

            success, buf = cv2.imencode(
                '.png',
                depth,
                [int(cv2.IMWRITE_PNG_COMPRESSION), max(0, min(9, self.depth_png_compression))],
            )
            if not success:
                self.get_logger().warn('Depth PNG compression failed; dropping frame')
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = '16UC1; png'
            out.data = buf.tobytes()
            self.depth_pub.publish(out)
            self.depth_frame_count += 1
            if self.depth_frame_count % 20 == 0:
                self.get_logger().info(
                    f'Published {self.depth_frame_count} compressed depth frames'
                )
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
