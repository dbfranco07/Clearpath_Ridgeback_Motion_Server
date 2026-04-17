#!/usr/bin/env python3
"""
VLM Perception Node — Runs on Jetson Orin
==========================================
Periodically captures camera frames and sends them to the VLLM-served
Qwen 3.5-27B model for scene understanding and room number detection.

Behavior:
  - At a configurable rate (e.g. 1 frame/3s), grabs the latest camera image
  - Encodes as base64 JPEG and sends to VLLM OpenAI-compatible API
  - Parses response for ROOM_DETECTED: <num> tags and environment classification
  - Publishes structured Perception messages on /vlm/perception
  - Keeps a rolling perception log accessible via /vlm/perception_log service

Designed to be rate-limited: if VLM is still processing, new frames are dropped.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage

import base64
import re
import threading
import time
from datetime import datetime

from openai import OpenAI

import numpy as np

from ridgeback_autonomy.msg import Perception


# System prompt for the VLM
SYSTEM_PROMPT = """You are a perception module for an autonomous mobile robot navigating a building.
Your task is to analyze camera images and provide structured observations.

Always respond in this exact format:
ENVIRONMENT: <one of: hallway, intersection, room_interior, open_area, elevator, stairwell, unknown>
ROOM_NUMBERS: <comma-separated room numbers if visible, or NONE>
OBSTACLES: <YES or NO — people, chairs, carts, boxes, or other movable obstacles>
DESCRIPTION: <1-2 sentence human-readable description of what the robot sees>

Be concise. Only report room numbers you can actually read in the image."""

USER_PROMPT = "Analyze this image from the robot's forward-facing camera."


class VlmPerceptionNode(Node):
    def __init__(self):
        super().__init__('vlm_perception')

        # Parameters
        self.declare_parameter('vllm_endpoint', 'http://localhost:8000')
        self.declare_parameter('model_name', 'Qwen/Qwen2.5-VL-7B-Instruct')
        self.declare_parameter('capture_interval_s', 3.0)     # Seconds between captures
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('max_image_width', 640)        # Resize before sending to VLM
        self.declare_parameter('request_timeout_s', 15.0)     # VLM HTTP timeout
        self.declare_parameter('max_log_entries', 100)
        self.declare_parameter('image_topic', '/r100_0140/image/compressed')
        self.declare_parameter('perception_topic', '/vlm/perception')
        self.declare_parameter('active', True)                # Can be toggled at runtime

        # .env override — load deployment-specific VLM config.
        # Looks for .env at $RIDGEBACK_ENV_FILE or $CWD/.env; falls back to yaml params.
        import os as _os
        _env_file = _os.environ.get('RIDGEBACK_ENV_FILE', _os.path.join(_os.getcwd(), '.env'))
        try:
            from dotenv import dotenv_values as _dotenv_values
            _dotenv = _dotenv_values(_env_file) if _os.path.exists(_env_file) else {}
            if _dotenv:
                self.get_logger().info(f'.env loaded from {_env_file}')
        except ImportError:
            _dotenv = {}
            self.get_logger().warn(
                'python-dotenv not installed — .env support disabled. '
                'Install with: pip install python-dotenv'
            )

        self.model_name = self.get_parameter('model_name').value

        if 'VLM_MODEL_NAME' in _dotenv:
            self.model_name = _dotenv['VLM_MODEL_NAME']
            self.get_logger().info(f'.env: model -> {self.model_name}')

        # Build OpenAI-compatible client pointing at the vLLM server
        _host = _dotenv.get('VLM_ENDPOINT', 'http://localhost').rstrip('/')
        _port = _dotenv.get('VLM_PORT', '')
        _base_url = f'{_host}:{_port}/v1' if _port else f'{_host}/v1'
        self.enable_thinking = _dotenv.get('VLM_THINK', 'false').lower() == 'true'
        self.vlm_client = OpenAI(base_url=_base_url, api_key='EMPTY')

        self.capture_interval = self.get_parameter('capture_interval_s').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.max_width = self.get_parameter('max_image_width').value
        self.request_timeout = self.get_parameter('request_timeout_s').value
        self.max_log = self.get_parameter('max_log_entries').value
        image_topic = self.get_parameter('image_topic').value
        perception_topic = self.get_parameter('perception_topic').value

        # QoS
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriber + Publisher
        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self._image_cb, sensor_qos
        )
        self.perception_pub = self.create_publisher(Perception, perception_topic, reliable_qos)

        # State
        self.latest_frame_bytes = None
        self.latest_frame_stamp = None
        self.frame_lock = threading.Lock()
        self.vlm_busy = False
        self.vlm_lock = threading.Lock()
        self.perception_log = []      # List of dicts for the web dashboard
        self.frame_counter = 0
        self.success_count = 0
        self.error_count = 0

        # Capture timer
        self.capture_timer = self.create_timer(self.capture_interval, self._capture_timer_cb)

        self.get_logger().info('VLM Perception Node started')
        self.get_logger().info(f'  VLLM base URL: {_base_url}')
        self.get_logger().info(f'  Model: {self.model_name}')
        self.get_logger().info(f'  Thinking: {self.enable_thinking}')
        self.get_logger().info(f'  Capture interval: {self.capture_interval}s')

    def _image_cb(self, msg: CompressedImage):
        with self.frame_lock:
            self.latest_frame_bytes = bytes(msg.data)
            self.latest_frame_stamp = msg.header.stamp

    def _capture_timer_cb(self):
        """Timer callback — grab latest frame and query VLM if not busy."""
        if not self.get_parameter('active').value:
            return

        with self.frame_lock:
            if self.latest_frame_bytes is None:
                return
            frame_bytes = self.latest_frame_bytes
            frame_stamp = self.latest_frame_stamp

        with self.vlm_lock:
            if self.vlm_busy:
                self.get_logger().debug('VLM still processing — skipping frame')
                return
            self.vlm_busy = True

        # Run VLM query in a background thread (don't block ROS spin)
        self.frame_counter += 1
        frame_id = f'frame_{self.frame_counter:06d}'
        thread = threading.Thread(
            target=self._query_vlm,
            args=(frame_bytes, frame_stamp, frame_id),
            daemon=True
        )
        thread.start()

    def _query_vlm(self, frame_bytes: bytes, frame_stamp, frame_id: str):
        """Send image to VLLM and publish result. Runs in background thread."""
        try:
            # Decode + optionally resize image
            img_b64 = self._prepare_image(frame_bytes)
            if img_b64 is None:
                return

            # Query vLLM via OpenAI-compatible SDK
            t0 = time.time()
            response = self.vlm_client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {
                        'role': 'user',
                        'content': [
                            {
                                'type': 'image_url',
                                'image_url': {'url': f'data:image/jpeg;base64,{img_b64}'}
                            },
                            {'type': 'text', 'text': USER_PROMPT}
                        ]
                    }
                ],
                max_tokens=256,
                temperature=0.1,
                timeout=self.request_timeout,
                extra_body={'chat_template_kwargs': {'enable_thinking': self.enable_thinking}}
            )
            latency_ms = (time.time() - t0) * 1000
            response_text = response.choices[0].message.content

            # Parse response
            parsed = self._parse_response(response_text)

            # Publish ROS2 message
            msg = Perception()
            msg.stamp = self.get_clock().now().to_msg()
            msg.description = parsed['description']
            msg.detected_rooms = parsed['rooms']
            msg.room_confidence = [0.85] * len(parsed['rooms'])
            msg.environment_type = parsed['environment']
            msg.has_obstacles = parsed['has_obstacles']
            msg.image_id = frame_id
            self.perception_pub.publish(msg)

            # Append to log for web dashboard
            log_entry = {
                'timestamp': datetime.now().strftime('%H:%M:%S'),
                'iso_time': datetime.now().isoformat(),
                'frame_id': frame_id,
                'environment': parsed['environment'],
                'rooms': parsed['rooms'],
                'has_obstacles': parsed['has_obstacles'],
                'description': parsed['description'],
                'raw_response': response_text,
                'latency_ms': round(latency_ms),
                'image_b64': img_b64   # Store thumbnail for web UI
            }
            self.perception_log.append(log_entry)
            if len(self.perception_log) > self.max_log:
                self.perception_log.pop(0)

            self.success_count += 1

            if parsed['rooms']:
                self.get_logger().info(
                    f'VLM [{frame_id}] ROOM DETECTED: {parsed["rooms"]} | '
                    f'{parsed["environment"]} | {latency_ms:.0f}ms'
                )
            else:
                self.get_logger().debug(
                    f'VLM [{frame_id}] {parsed["environment"]} | {latency_ms:.0f}ms'
                )

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'VLM query failed [{frame_id}]: {e}')
        finally:
            with self.vlm_lock:
                self.vlm_busy = False

    def _prepare_image(self, frame_bytes: bytes) -> str | None:
        """Decode JPEG, optionally resize, return base64 string."""
        try:
            import cv2
            arr = np.frombuffer(frame_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                return None

            # Resize if too large (to save bandwidth + VLM inference time)
            h, w = img.shape[:2]
            if w > self.max_width:
                scale = self.max_width / w
                new_w = self.max_width
                new_h = int(h * scale)
                img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, buf = cv2.imencode('.jpg', img, encode_param)
            return base64.b64encode(buf.tobytes()).decode('utf-8')

        except Exception as e:
            self.get_logger().error(f'Image prepare failed: {e}')
            return None

    def _parse_response(self, text: str) -> dict:
        """Parse the structured VLM response into a dict."""
        result = {
            'environment': 'unknown',
            'rooms': [],
            'has_obstacles': False,
            'description': text.strip()
        }

        lines = text.strip().split('\n')
        for line in lines:
            line = line.strip()
            if line.startswith('ENVIRONMENT:'):
                env = line.split(':', 1)[1].strip().lower()
                result['environment'] = env if env else 'unknown'

            elif line.startswith('ROOM_NUMBERS:'):
                rooms_str = line.split(':', 1)[1].strip()
                if rooms_str.upper() != 'NONE' and rooms_str:
                    # Extract numbers (handles formats like "305, 306" or "305/306")
                    room_candidates = re.findall(r'\b\d{2,4}\b', rooms_str)
                    result['rooms'] = list(set(room_candidates))

            elif line.startswith('OBSTACLES:'):
                obs = line.split(':', 1)[1].strip().upper()
                result['has_obstacles'] = obs == 'YES'

            elif line.startswith('DESCRIPTION:'):
                result['description'] = line.split(':', 1)[1].strip()

        return result

    def get_recent_log(self, n: int = 20) -> list:
        """Return the N most recent perception log entries."""
        return self.perception_log[-n:]


def main(args=None):
    print('=' * 50)
    print('Ridgeback Autonomy — VLM Perception Node')
    print('=' * 50)
    rclpy.init(args=args)
    node = VlmPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
