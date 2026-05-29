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
from urllib.parse import urlsplit, urlunsplit

import cv2
import numpy as np
import rclpy
import requests
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ridgeback_image_motion.autonomy_common import json_dumps, json_loads, quaternion_to_yaw_rad
except ImportError:
    from autonomy_common import json_dumps, json_loads, quaternion_to_yaw_rad  # type: ignore[no-redef]


_DEFAULT_PROMPT = (
    "You are a building navigator. Look at this image and extract any visible "
    "room number sign or door plate (e.g. '202', 'B312', '101A'). "
    "Reply with strict JSON only: "
    '{"room": "<string or empty>", "confidence": <0..1>, "reason": "<short note>"}.'
)


def _resolve_vlm_url() -> str:
    """Pick the VLM base URL from env.

    Priority: VLM_URL (full URL incl /v1) > VLM_ENDPOINT + VLM_PORT > default.
    """
    explicit = os.environ.get("VLM_URL")
    if explicit:
        return explicit.rstrip("/")
    endpoint = os.environ.get("VLM_ENDPOINT", "http://202.92.159.240").rstrip("/")
    if "://" not in endpoint:
        endpoint = f"http://{endpoint}"
    port = os.environ.get("VLM_PORT", "8000")
    return f"{endpoint}:{port}/v1"


def _vlm_url_env_present() -> bool:
    return any(name in os.environ for name in ("VLM_URL", "VLM_ENDPOINT", "VLM_PORT"))


def _env_bool(name: str) -> bool | None:
    raw = os.environ.get(name)
    if raw is None:
        return None
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _join_url(base: str, path: str) -> str:
    if path.startswith(("http://", "https://")):
        return path.rstrip("/")
    return f"{base.rstrip('/')}/{path.lstrip('/')}"


def _without_trailing_v1(url: str) -> str:
    parts = urlsplit(url)
    path = parts.path.rstrip("/")
    if path != "/v1":
        return url.rstrip("/")
    return urlunsplit((parts.scheme, parts.netloc, "", "", "")).rstrip("/")


def _dedupe(values: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for value in values:
        if value and value not in seen:
            seen.add(value)
            out.append(value)
    return out


def _chat_endpoint_candidates(
    base_url: str,
    chat_url: str = "",
    chat_path: str = "",
) -> list[str]:
    """Build OpenAI-compatible chat routes to try in order.

    Some local/proxy VLM servers expose /v1/models but mount chat at
    /chat/completions. Keep the normal /v1/chat/completions path first, then
    try the root variant when the base URL ends in /v1.
    """
    if chat_url:
        return [chat_url.rstrip("/")]

    base = base_url.rstrip("/")
    base_path = urlsplit(base).path.rstrip("/")
    if base_path.endswith("/chat/completions"):
        return [base]

    if chat_path:
        return [_join_url(base, chat_path)]

    candidates = [_join_url(base, "/chat/completions")]
    root_base = _without_trailing_v1(base)
    if root_base != base:
        candidates.append(_join_url(root_base, "/chat/completions"))
    elif not base_path.endswith("/v1"):
        candidates.append(_join_url(base, "/v1/chat/completions"))
    return _dedupe(candidates)


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
        self.declare_parameter("vlm_url", _resolve_vlm_url())
        self.declare_parameter("vlm_chat_url", os.environ.get("VLM_CHAT_URL", ""))
        self.declare_parameter("vlm_chat_path", os.environ.get("VLM_CHAT_PATH", ""))
        self.declare_parameter(
            "vlm_model",
            os.environ.get("VLM_MODEL")
            or os.environ.get("VLM_MODEL_NAME")
            or "Qwen/Qwen3-VL-7B-Instruct",
        )
        self.declare_parameter("api_key_env", "VLM_API_KEY")
        # VLM_THINK=false disables Qwen's extended-reasoning tokens via the
        # vLLM chat_template_kwargs hook.
        self.declare_parameter(
            "enable_thinking",
            (os.environ.get("VLM_THINK", "false").lower() == "true"),
        )
        self.declare_parameter("period_s", 3.0)
        self.declare_parameter("motion_threshold_mps", 0.05)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("request_timeout_s", 8.0)
        self.declare_parameter("prompt", _DEFAULT_PROMPT)
        # Majority-vote dedup: a non-empty room detection that hasn't been
        # corroborated by another frame in the last vote_window_s at a similar
        # pose (bucketed to vote_bucket_m) is published with confidence
        # multiplied by vote_singleton_factor. Once two detections agree at
        # the same pose bucket, full confidence is passed through.
        self.declare_parameter("vote_window_s", 2.0)
        self.declare_parameter("vote_buffer_size", 3)
        # Bucket 2.0 m (was 0.5). At 0.15 m/s the robot can move ~30 cm
        # between two periodic VLM reads spaced ~2 s apart — well inside
        # 2 m, so consecutive reads of the same sign actually corroborate
        # instead of landing in different buckets and counting as
        # independent singletons.
        self.declare_parameter("vote_bucket_m", 2.0)
        self.declare_parameter("vote_singleton_factor", 0.5)

        ns = self.get_parameter("namespace").value
        self._color_topic = (
            self.get_parameter("color_topic").value
            or f"/{ns}/sensors/camera_0/color/image_raw"
        )
        self._odom_topic = (
            self.get_parameter("odom_topic").value or f"/{ns}/platform/odom/filtered"
        )
        self._home_frame = self.get_parameter("home_frame").value
        # Platform's robot_state_publisher emits URDF frame names verbatim
        # (no ROS namespace prefix), so default to bare "base_link".
        self._base_frame = self.get_parameter("base_frame").value or "base_link"
        self._period = float(self.get_parameter("period_s").value)
        self._motion_threshold = float(self.get_parameter("motion_threshold_mps").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._timeout = float(self.get_parameter("request_timeout_s").value)
        param_vlm_url = str(self.get_parameter("vlm_url").value).rstrip("/")
        self._vlm_url = _resolve_vlm_url() if _vlm_url_env_present() else param_vlm_url
        chat_url = os.environ.get("VLM_CHAT_URL") or str(
            self.get_parameter("vlm_chat_url").value
        ).strip()
        chat_path = os.environ.get("VLM_CHAT_PATH") or str(
            self.get_parameter("vlm_chat_path").value
        ).strip()
        self._vlm_chat_urls = _chat_endpoint_candidates(
            self._vlm_url,
            chat_url,
            chat_path,
        )
        self._vlm_model = (
            os.environ.get("VLM_MODEL")
            or os.environ.get("VLM_MODEL_NAME")
            or str(self.get_parameter("vlm_model").value)
        )
        self._api_key = os.environ.get(str(self.get_parameter("api_key_env").value), "")
        self._prompt = str(self.get_parameter("prompt").value)
        env_think = _env_bool("VLM_THINK")
        self._enable_thinking = (
            env_think if env_think is not None else bool(self.get_parameter("enable_thinking").value)
        )

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_image: np.ndarray | None = None
        self._image_stamp = 0.0
        self._image_ros_stamp: TimeMsg | None = None
        self._speed = 0.0
        self._last_periodic = 0.0
        self._busy = False
        # Recent detections for majority-vote dedup: each entry is
        # (room, bucket_x, bucket_y, ts).
        self._vote_window = max(float(self.get_parameter("vote_window_s").value), 0.1)
        self._vote_buffer_size = max(int(self.get_parameter("vote_buffer_size").value), 1)
        self._vote_bucket = max(float(self.get_parameter("vote_bucket_m").value), 0.05)
        self._vote_singleton_factor = max(
            min(float(self.get_parameter("vote_singleton_factor").value), 1.0), 0.0
        )
        self._recent_detections: list[tuple[str, int, int, float]] = []

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
            f"vlm_client ready url={self._vlm_url} chat={self._vlm_chat_urls[0]} "
            f"model={self._vlm_model} period={self._period}s"
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
            # Hold the ROS header stamp so _lookup_pose() can query TF at the
            # exact moment of capture, not whenever the VLM happens to reply.
            self._image_ros_stamp = msg.header.stamp

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        speed = math.hypot(float(v.x), float(v.y))
        with self._lock:
            self._speed = speed

    def _on_trigger(self, msg: String) -> None:
        # Two payload shapes:
        #   1. Bare room hint, e.g. "222"  → existing behavior (room OCR with hint).
        #   2. JSON {"prompt": "...", "intent": "QUERY"} → free-form Q&A; the
        #      operator's question replaces the default room-OCR prompt and the
        #      observation is tagged with kind="answer" so the dashboard renders
        #      the VLM's reply in the chat instead of treating it as a detection.
        raw = (msg.data or "").strip()
        payload = json_loads(raw, default={}) if raw.startswith("{") else {}
        prompt_override = str(payload.get("prompt") or "").strip()
        intent = str(payload.get("intent") or "").strip().upper()
        if prompt_override:
            room_hint = ""
            kind = "answer"
        else:
            room_hint = raw.upper()
            kind = "trigger"
        threading.Thread(
            target=self._snapshot_and_query,
            args=(room_hint, kind, prompt_override, intent),
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
            args=("", "periodic", "", ""),
            daemon=True,
        ).start()

    # --- VLM call -----------------------------------------------------------
    def _snapshot_and_query(
        self,
        room_hint: str,
        kind: str,
        prompt_override: str = "",
        intent: str = "",
    ) -> None:
        with self._lock:
            if self._busy:
                return
            self._busy = True
            img = None if self._latest_image is None else self._latest_image.copy()
            stamp = self._image_stamp
            ros_stamp = self._image_ros_stamp
        if img is None:
            with self._lock:
                self._busy = False
            self.get_logger().warn(
                f"vlm[{kind}] skipped: no camera frame yet (subscribed {self._color_topic})",
                throttle_duration_sec=5.0,
            )
            return
        try:
            ok, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not ok:
                return
            b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
            t0 = time.time()
            answer, error = self._call_vlm(b64, room_hint, prompt_override)
            latency_ms = (time.time() - t0) * 1000.0
            parsed = self._parse_answer(answer) if not prompt_override else {}
            pose = self._lookup_pose(ros_stamp)
            raw_room = str(parsed.get("room", "") or "").strip().upper()
            raw_conf = float(parsed.get("confidence") or 0.0)
            adjusted_conf, vote_count = self._apply_vote(raw_room, pose, raw_conf)
            obs = {
                "kind": kind,
                "intent": intent,
                "room_hint": room_hint,
                "room": raw_room,
                "confidence": adjusted_conf,
                "raw_confidence": raw_conf,
                "vote_count": vote_count,
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
                vote_note = f" vote={vote_count}/{self._vote_buffer_size}"
                self.get_logger().info(
                    f"vlm[{kind}] -> room={obs['room']} conf={obs['confidence']:.2f}"
                    f" (raw={raw_conf:.2f}{vote_note}, {latency_ms:.0f} ms)"
                )
            elif kind == "answer":
                snippet = (answer or error or "").strip().replace("\n", " ")[:80]
                self.get_logger().info(f"vlm[answer] -> {snippet} ({latency_ms:.0f} ms)")
        finally:
            with self._lock:
                self._busy = False

    def _call_vlm(self, jpeg_b64: str, room_hint: str, prompt_override: str = "") -> tuple[str, str]:
        prompt = prompt_override or self._prompt
        if room_hint and not prompt_override:
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
        # vLLM honours chat_template_kwargs for Qwen-family models — disable
        # the extended-reasoning ("thinking") block when VLM_THINK=false so the
        # response is pure JSON instead of reasoning + JSON.
        if not self._enable_thinking:
            body["chat_template_kwargs"] = {"enable_thinking": False}
        headers = {"Content-Type": "application/json"}
        if self._api_key:
            headers["Authorization"] = f"Bearer {self._api_key}"
        errors: list[str] = []
        last_status = 0
        for url in self._vlm_chat_urls:
            try:
                resp = requests.post(url, headers=headers, json=body, timeout=self._timeout)
                last_status = resp.status_code
                resp.raise_for_status()
            except requests.HTTPError as exc:
                status = exc.response.status_code if exc.response is not None else last_status
                errors.append(f"{url} -> HTTP {status}")
                if status in (404, 405):
                    continue
                return "", f"vlm_request_failed:{exc}"
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{url} -> {exc}")
                continue
            try:
                data = resp.json()
                return data["choices"][0]["message"]["content"], ""
            except Exception as exc:  # noqa: BLE001
                return "", f"vlm_parse_failed:{exc}"
        return "", "vlm_request_failed:" + "; ".join(errors)

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

    def _lookup_pose(self, image_stamp: TimeMsg | None = None) -> dict[str, float]:
        # Query TF at the *image capture time* so the pose attached to a
        # detection matches where the robot actually was when it took the
        # picture — not where it ended up by the time the VLM replied (which
        # can be hundreds of ms later for remote VLLM endpoints).
        if image_stamp is not None:
            stamp = RclpyTime.from_msg(image_stamp)
        else:
            stamp = rclpy.time.Time()
        try:
            tf = self._tf_buffer.lookup_transform(
                self._home_frame,
                self._base_frame,
                stamp,
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            # Fall back to "latest available" — better an approximate pose
            # than none, but log throttled so we know when we lose precision.
            if image_stamp is None:
                return {}
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._home_frame,
                    self._base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException:
                return {}
            self.get_logger().warn(
                "TF lookup at image stamp failed; using latest TF (pose may be stale)",
                throttle_duration_sec=5.0,
            )
        return {
            "x": float(tf.transform.translation.x),
            "y": float(tf.transform.translation.y),
            "yaw": quaternion_to_yaw_rad(tf.transform.rotation),
        }

    def _apply_vote(
        self, room: str, pose: dict[str, float], raw_conf: float,
    ) -> tuple[float, int]:
        """Majority-vote dedup over a small ring of recent detections.

        Returns (adjusted_confidence, vote_count). Empty rooms bypass the
        vote entirely and pass through with their original confidence.
        """
        if not room:
            return raw_conf, 0
        now = time.time()
        bx = int(round(float(pose.get("x", 0.0) or 0.0) / self._vote_bucket))
        by = int(round(float(pose.get("y", 0.0) or 0.0) / self._vote_bucket))
        with self._lock:
            # Drop expired entries first.
            cutoff = now - self._vote_window
            self._recent_detections = [
                d for d in self._recent_detections if d[3] >= cutoff
            ]
            count = 1 + sum(
                1 for r, ex, ey, _ts in self._recent_detections
                if r == room and ex == bx and ey == by
            )
            self._recent_detections.append((room, bx, by, now))
            if len(self._recent_detections) > self._vote_buffer_size:
                self._recent_detections = self._recent_detections[-self._vote_buffer_size:]
        if count >= 2:
            return raw_conf, count
        return raw_conf * self._vote_singleton_factor, count


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
