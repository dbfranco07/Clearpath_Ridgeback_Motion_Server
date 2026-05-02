#!/usr/bin/env python3
"""Helpers for loading VLM configuration and creating an OpenAI client."""

from __future__ import annotations

import base64
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit, urlunsplit

from openai import OpenAI


@dataclass(frozen=True)
class VlmConfig:
    endpoint: str
    port: str
    model_name: str
    enable_thinking: bool

    @property
    def base_url(self) -> str:
        return normalize_base_url(self.endpoint, self.port)

    @property
    def openai_base_url(self) -> str:
        return f"{self.base_url.rstrip('/')}/v1"


def _env_file_path() -> Path:
    explicit = os.environ.get("RIDGEBACK_ENV_FILE")
    if explicit:
        return Path(explicit).expanduser()
    return Path(__file__).resolve().parent / ".env"


def _parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            values[key] = value
    return values


_DEFAULT_ENV = _parse_env_file(Path(__file__).resolve().parent / ".env")
DEFAULT_ENDPOINT = _DEFAULT_ENV.get("VLM_ENDPOINT", "http://202.92.159.240")
DEFAULT_PORT = _DEFAULT_ENV.get("VLM_PORT", "8000")
DEFAULT_MODEL = _DEFAULT_ENV.get("VLM_MODEL_NAME", "Qwen/Qwen3.5-27B")


def load_vlm_config() -> VlmConfig:
    env = _parse_env_file(_env_file_path())
    return VlmConfig(
        endpoint=env.get("VLM_ENDPOINT", DEFAULT_ENDPOINT).strip(),
        port=env.get("VLM_PORT", DEFAULT_PORT).strip(),
        model_name=env.get("VLM_MODEL_NAME", DEFAULT_MODEL).strip(),
        enable_thinking=env.get("VLM_THINK", "false").strip().lower() in {"1", "true", "yes", "on"},
    )


def normalize_base_url(endpoint: str, port: str = "") -> str:
    raw = endpoint.strip()
    if not raw:
        raw = DEFAULT_ENDPOINT
    parsed = urlsplit(raw if "://" in raw else f"http://{raw}")
    host = parsed.hostname or parsed.path or DEFAULT_ENDPOINT.replace("http://", "")
    selected_port = port.strip() or (str(parsed.port) if parsed.port else "")
    netloc = f"{host}:{selected_port}" if selected_port else host
    scheme = parsed.scheme or "http"
    return urlunsplit((scheme, netloc, "", "", ""))


def build_vlm_client(config: VlmConfig | None = None) -> tuple[OpenAI, VlmConfig]:
    resolved = config or load_vlm_config()
    client = OpenAI(base_url=resolved.openai_base_url, api_key="EMPTY")
    return client, resolved


def chat_completion_messages(user_message: str, system_prompt: str | None = None) -> list[dict[str, object]]:
    messages: list[dict[str, object]] = []
    if system_prompt:
        messages.append({"role": "system", "content": system_prompt})
    messages.append({"role": "user", "content": user_message})
    return messages


FRONTIER_RANK_SYSTEM = (
    "You are a navigation assistant choosing where a wheeled mobile robot should explore next. "
    "Reply with strict JSON only."
)

FRONTIER_RANK_USER_TEMPLATE = (
    "Choose the most informative frontier for the robot to drive to next, given the camera view "
    "and the candidate frontiers. Each candidate has bearing_deg (relative to robot heading: 0=forward, "
    "+left, -right), distance_m, and cluster_size (larger = bigger unknown area). Prefer frontiers "
    "that look reachable in the image (open floor, doorways, hallways) and avoid frontiers that face "
    "obstacles, people, or stairs. Reply with JSON only: "
    '{{"chosen_index": <int 0..{n_minus_1}>, "reasoning": "<short>"}}\n\n'
    "Candidates:\n{candidates_block}"
)


def _candidates_to_text(candidates: list[dict[str, Any]]) -> str:
    lines = []
    for i, c in enumerate(candidates):
        lines.append(
            f"  [{i}] bearing_deg={float(c.get('bearing_deg', 0.0)):+.1f} "
            f"distance_m={float(c.get('distance_m', 0.0)):.2f} "
            f"cluster_size={int(c.get('cluster_size', 0))}"
        )
    return "\n".join(lines)


def _extract_json_object(text: str) -> dict[str, Any]:
    try:
        value = json.loads(text)
        if isinstance(value, dict):
            return value
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


def rank_frontiers(
    client: OpenAI,
    config: VlmConfig,
    image_jpeg: bytes,
    candidates: list[dict[str, Any]],
    timeout_s: float = 2.0,
    max_tokens: int = 120,
) -> dict[str, Any]:
    """Ask the VLM to pick the best frontier index from a JPEG view + candidate list.

    Raises on parse error or invalid index — caller should fall back to deterministic ranking.
    Returns dict with keys ``chosen_index`` (int) and ``reasoning`` (str).
    """
    if not candidates:
        raise ValueError("no candidates")

    image_b64 = base64.b64encode(image_jpeg).decode("ascii")
    user_text = FRONTIER_RANK_USER_TEMPLATE.format(
        n_minus_1=len(candidates) - 1,
        candidates_block=_candidates_to_text(candidates),
    )
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": FRONTIER_RANK_SYSTEM},
        {
            "role": "user",
            "content": [
                {"type": "text", "text": user_text},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
            ],
        },
    ]
    response = client.with_options(timeout=float(timeout_s)).chat.completions.create(
        model=config.model_name,
        messages=messages,
        temperature=0.0,
        max_tokens=int(max_tokens),
        extra_body={"chat_template_kwargs": {"enable_thinking": config.enable_thinking}},
    )
    content = response.choices[0].message.content
    if isinstance(content, list):
        text = " ".join(str(p.get("text", "")) if isinstance(p, dict) else str(p) for p in content)
    else:
        text = str(content or "")
    parsed = _extract_json_object(text)
    if "chosen_index" not in parsed:
        raise ValueError(f"missing chosen_index in VLM response: {text[:200]}")
    chosen = int(parsed["chosen_index"])
    if not 0 <= chosen < len(candidates):
        raise ValueError(f"chosen_index {chosen} out of range 0..{len(candidates) - 1}")
    return {
        "chosen_index": chosen,
        "reasoning": str(parsed.get("reasoning", ""))[:240],
    }