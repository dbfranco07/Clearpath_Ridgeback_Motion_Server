#!/usr/bin/env python3
"""VLM-backed room-number detector for Ridgeback camera frames."""

from __future__ import annotations

import base64
import json
import re
import time
from typing import Any

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from ridgeback_image_motion.autonomy_common import (
        ACTIVE_MISSION_STATES,
        json_dumps,
        json_loads,
        quaternion_to_yaw_rad,
    )
    from ridgeback_image_motion.vlm_client import build_vlm_client
except ImportError:
    from autonomy_common import ACTIVE_MISSION_STATES, json_dumps, json_loads, quaternion_to_yaw_rad
    from vlm_client import build_vlm_client


ROOM_JSON_PROMPT = (
    "Inspect the image for room number signs or labels. Return strict JSON only: "
    '{"room_detections":[{"room_number":"302","confidence":0.0,"evidence":"short text"}]}. '
    "Use an empty room_detections array if no readable room number is visible."
)


class RoomDetector(Node):
    def __init__(self) -> None:
        super().__init__("room_detector")

        self.declare_parameter("image_topic", "/r100_0140/image/compressed")
        self.declare_parameter("odom_topic", "/r100_0140/platform/odom/filtered")
        self.declare_parameter("mission_status_topic", "/ridgeback/mission/status")
        self.declare_parameter("detections_topic", "/ridgeback/semantic/room_detections")
        self.declare_parameter("vlm_status_topic", "/ridgeback/vlm/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "r100_0140/base_link")
        self.declare_parameter("detect_period_s", 3.0)
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("max_tokens", 240)

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(CompressedImage, self.get_parameter("image_topic").value, self._image_cb, sensor_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_cb, sensor_qos)
        self.create_subscription(String, self.get_parameter("mission_status_topic").value, self._mission_status_cb, reliable_qos)
        self.detection_pub = self.create_publisher(String, self.get_parameter("detections_topic").value, reliable_qos)
        self.status_pub = self.create_publisher(String, self.get_parameter("vlm_status_topic").value, reliable_qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.vlm_client, self.vlm_config = build_vlm_client()
        self.latest_frame: bytes | None = None
        self.latest_frame_stamp = ""
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.mission_state = "IDLE"
        self.target_room = ""
        self.busy = False
        self.last_detection_time = 0.0

        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"room_detector ready: {self.vlm_config.base_url} model={self.vlm_config.model_name}")

    def _image_cb(self, msg: CompressedImage) -> None:
        self.latest_frame = bytes(msg.data)
        self.latest_frame_stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

    def _odom_cb(self, msg: Odometry) -> None:
        self.pose["x"] = float(msg.pose.pose.position.x)
        self.pose["y"] = float(msg.pose.pose.position.y)
        self.pose["yaw"] = float(quaternion_to_yaw_rad(msg.pose.pose.orientation))

    def _refresh_map_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(),
            )
            self.pose["x"] = float(transform.transform.translation.x)
            self.pose["y"] = float(transform.transform.translation.y)
            self.pose["yaw"] = float(quaternion_to_yaw_rad(transform.transform.rotation))
        except Exception:
            pass

    def _mission_status_cb(self, msg: String) -> None:
        payload = json_loads(msg.data)
        self.mission_state = str(payload.get("state", "IDLE"))
        self.target_room = str(payload.get("target_room", ""))

    def _tick(self) -> None:
        now = time.time()
        if self.busy:
            return
        if self.mission_state not in ACTIVE_MISSION_STATES:
            self._publish_status(True, "idle")
            return
        if self.latest_frame is None:
            self._publish_status(False, "no_frame")
            return
        if now - self.last_detection_time < float(self.get_parameter("detect_period_s").value):
            return

        self.last_detection_time = now
        self._refresh_map_pose()
        self.busy = True
        try:
            detections = self._call_vlm(self.latest_frame)
            accepted = []
            min_conf = float(self.get_parameter("min_confidence").value)
            for detection in detections:
                room = str(detection.get("room_number") or detection.get("room_id") or "").strip().upper()
                confidence = float(detection.get("confidence", 0.0) or 0.0)
                if room and confidence >= min_conf:
                    accepted.append(
                        {
                            "room_number": room,
                            "confidence": confidence,
                            "evidence": str(detection.get("evidence", ""))[:160],
                        }
                    )

            payload = {
                "stamp": now,
                "frame_stamp": self.latest_frame_stamp,
                "target_room": self.target_room,
                "pose": dict(self.pose),
                "detections": accepted,
            }
            if accepted:
                self.detection_pub.publish(String(data=json_dumps(payload)))
                self.get_logger().info("room detections: %s" % ", ".join(item["room_number"] for item in accepted))
            self._publish_status(True, "ok", detection_count=len(accepted))
        except Exception as exc:
            self.get_logger().warn(f"VLM room detection failed: {exc}")
            self._publish_status(False, str(exc)[:180])
        finally:
            self.busy = False

    def _call_vlm(self, jpeg: bytes) -> list[dict[str, Any]]:
        image_b64 = base64.b64encode(jpeg).decode("ascii")
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": "You are a precise OCR assistant for robot navigation. Reply with JSON only."},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": ROOM_JSON_PROMPT},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
                ],
            },
        ]
        response = self.vlm_client.chat.completions.create(
            model=self.vlm_config.model_name,
            messages=messages,
            temperature=0.0,
            max_tokens=int(self.get_parameter("max_tokens").value),
            extra_body={"chat_template_kwargs": {"enable_thinking": self.vlm_config.enable_thinking}},
        )
        content = response.choices[0].message.content
        if isinstance(content, list):
            text = " ".join(str(part.get("text", "")) if isinstance(part, dict) else str(part) for part in content)
        else:
            text = str(content or "")
        parsed = self._extract_json(text)
        detections = parsed.get("room_detections", [])
        return detections if isinstance(detections, list) else []

    def _extract_json(self, text: str) -> dict[str, Any]:
        try:
            value = json.loads(text)
            return value if isinstance(value, dict) else {}
        except Exception:
            pass
        match = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if not match:
            return {}
        try:
            value = json.loads(match.group(0))
        except Exception:
            return {}
        return value if isinstance(value, dict) else {}

    def _publish_status(self, healthy: bool, message: str, detection_count: int = 0) -> None:
        payload = {
            "stamp": time.time(),
            "healthy": bool(healthy),
            "message": message,
            "endpoint": self.vlm_config.base_url,
            "model": self.vlm_config.model_name,
            "detection_count": int(detection_count),
        }
        self.status_pub.publish(String(data=json_dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoomDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
