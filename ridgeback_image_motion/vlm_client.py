#!/usr/bin/env python3
"""VLM client — periodic and on-trigger snapshot OCR for room numbers.

Subscribes to color image + odom + (TF map -> base_link). Sends a JPEG
snapshot to an OpenAI-compatible chat-completions endpoint with an
OCR-style prompt that asks for a room number and confidence. Publishes
parsed JSON observations.

Snapshots are taken when:
  (a) period_s elapsed AND robot speed > motion_threshold_mps, OR
  (b) /vlm/trigger received (room hint as String payload).
"""

from __future__ import annotations

import base64
import json
import math
import os
import re
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, quaternion_to_yaw_rad
except ImportError:
    from autonomy_common import json_dumps, quaternion_to_yaw_rad  # type: ignore[no-redef]


_DEFAULT_PROMPT = (
    "You are a building navigator. Look at this image and extract any visible "
    "room number sign or door plate (e.g. '202', 'B312', '101A'). "
    "Reply with strict JSON only: "
    '{"room": "<string or empty>", "confidence": <0..1>, "reason": "<short note>"}.'
)


class VlmClient(Node):
    def __init__(self) -> None:
        super().__init__("vlm_client")

        self.declare_parameter("namespace", "r100_0140")
        self.declare_parameter("color_topic", "")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("trigger_topic", "/vlm/trigger")
        self.declare_parameter("observation_topic", "/vlm/observation")
        self.declare_parameter("home_frame", "map")
        self.declare_parameter("base_frame", "")
        self.declare_parameter(
            "vlm_url", os.environ.get("VLM_URL", "http://202.92.159.240:8000/v1")
        )
        self.declare_parameter("vlm_model", os.environ.get("VLM_MODEL", "qwen2-vl"))
        self.declare_parameter("api_key_env", "VLM_API_KEY")
        self.declare_parameter("period_s", 3.0)
        self.declare_parameter("motion_threshold_mps", 0.05)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("request_timeout_s", 8.0)
        self.declare_parameter("prompt", _DEFAULT_PROMPT)

        ns = self.get_parameter("namespace").value
        self._color_topic = (
            self.get_parameter("color_topic").value
            or f"/{ns}/sensors/camera_0/color/image_raw"
        )
        self._odom_topic = (
            self.get_parameter("odom_topic").value or f"/{ns}/platform/odom/filtered"
        )
        self._home_frame = self.get_parameter("home_frame").value
        self._base_frame = self.get_parameter("base_frame").value or f"{ns}/base_link"
        self._period = float(self.get_parameter("period_s").value)
        self._motion_threshold = float(self.get_parameter("motion_threshold_mps").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._timeout = float(self.get_parameter("request_timeout_s").value)
        self._vlm_url = str(self.get_parameter("vlm_url").value).rstrip("/")
        self._vlm_model = str(self.get_parameter("vlm_model").value)
        self._api_key = os.environ.get(str(self.get_parameter("api_key_env").value), "")
        self._prompt = str(self.get_parameter("prompt").value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_image: np.ndarray | None = None
        self._image_stamp = 0.0
        self._speed = 0.0
        self._last_periodic = 0.0
        self._busy = False

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(Image, self._color_topic, self._on_color, sensor_qos)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(
            String,
            self.get_parameter("trigger_topic").value,
            self._on_trigger,
            10,
        )
        self._pub_obs = self.create_publisher(
            String, self.get_parameter("observation_topic").value, 10
        )
        self._timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"vlm_client ready url={self._vlm_url} model={self._vlm_model} period={self._period}s"
        )

    # --- ROS callbacks ------------------------------------------------------
    def _on_color(self, msg: Image) -> None:
        try:
            cv = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        with self._lock:
            self._latest_image = cv
            self._image_stamp = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        speed = math.hypot(float(v.x), float(v.y))
        with self._lock:
            self._speed = speed

    def _on_trigger(self, msg: String) -> None:
        room_hint = (msg.data or "").strip().upper()
        threading.Thread(
            target=self._snapshot_and_query,
            args=(room_hint, "trigger"),
            daemon=True,
        ).start()

    # --- main loop ----------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
            now = time.time()
            ready = (now - self._last_periodic) >= self._period
            moving = self._speed >= self._motion_threshold
            busy = self._busy
        if not ready or busy or not moving:
            return
        self._last_periodic = now
        threading.Thread(
            target=self._snapshot_and_query,
            args=("", "periodic"),
            daemon=True,
        ).start()

    # --- VLM call -----------------------------------------------------------
    def _snapshot_and_query(self, room_hint: str, kind: str) -> None:
        with self._lock:
            if self._busy:
                return
            self._busy = True
            img = None if self._latest_image is None else self._latest_image.copy()
            stamp = self._image_stamp
        if img is None:
            with self._lock:
                self._busy = False
            return
        try:
            ok, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not ok:
                return
            b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
            t0 = time.time()
            answer, error = self._call_vlm(b64, room_hint)
            latency_ms = (time.time() - t0) * 1000.0
            parsed = self._parse_answer(answer)
            pose = self._lookup_pose()
            obs = {
                "kind": kind,
                "room_hint": room_hint,
                "room": parsed.get("room", ""),
                "confidence": float(parsed.get("confidence") or 0.0),
                "reason": parsed.get("reason", ""),
                "text": answer or error,
                "status": "ok" if not error else "error",
                "pose": pose,
                "image_ts": stamp,
                "latency_ms": latency_ms,
                "ts": time.time(),
                "timestamp": time.strftime("%H:%M:%S"),
            }
            msg = String()
            msg.data = json_dumps(obs)
            self._pub_obs.publish(msg)
            if obs["room"]:
                self.get_logger().info(
                    f"vlm[{kind}] -> room={obs['room']} conf={obs['confidence']:.2f} ({latency_ms:.0f} ms)"
                )
        finally:
            with self._lock:
                self._busy = False

    def _call_vlm(self, jpeg_b64: str, room_hint: str) -> tuple[str, str]:
        url = f"{self._vlm_url}/chat/completions"
        prompt = self._prompt
        if room_hint:
            prompt += f"\nThe operator is currently looking for room {room_hint}. Prefer that room if visible."
        body = {
            "model": self._vlm_model,
            "max_tokens": 200,
            "temperature": 0.0,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{jpeg_b64}"},
                        },
                    ],
                }
            ],
        }
        headers = {"Content-Type": "application/json"}
        if self._api_key:
            headers["Authorization"] = f"Bearer {self._api_key}"
        try:
            resp = requests.post(url, headers=headers, json=body, timeout=self._timeout)
            resp.raise_for_status()
        except Exception as exc:  # noqa: BLE001
            return "", f"vlm_request_failed:{exc}"
        try:
            data = resp.json()
            return data["choices"][0]["message"]["content"], ""
        except Exception as exc:  # noqa: BLE001
            return "", f"vlm_parse_failed:{exc}"

    @staticmethod
    def _parse_answer(answer: str) -> dict[str, Any]:
        if not answer:
            return {}
        # Try JSON first.
        try:
            return json.loads(answer)
        except Exception:
            pass
        # Fall back: find a JSON object inside the response.
        match = re.search(r"\{.*\}", answer, re.DOTALL)
        if match:
            try:
                return json.loads(match.group(0))
            except Exception:
                pass
        # Last resort: bare room number.
        m = re.search(r"\b([A-Z]?\d{2,4}[A-Z]?)\b", answer.upper())
        if m:
            return {"room": m.group(1), "confidence": 0.5}
        return {}

    def _lookup_pose(self) -> dict[str, float]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._home_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return {}
        return {
            "x": float(tf.transform.translation.x),
            "y": float(tf.transform.translation.y),
            "yaw": quaternion_to_yaw_rad(tf.transform.rotation),
        }


def main() -> None:
    rclpy.init()
    node = VlmClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
