#!/usr/bin/env python3
"""Helpers for loading VLM configuration and creating an OpenAI client."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
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