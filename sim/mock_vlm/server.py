"""
mock_vlm/server.py — local mock of the DGX vLLM endpoint.

Mimics OpenAI-compatible /v1/chat/completions. Used ONLY when
VLM_BACKEND=mock (CI runs, offline dev, chaos testing). Normal sim
sessions call the real DGX endpoint.

The mock returns canned responses keyed by robot pose and prompt pattern,
loaded from mock_vlm_fixtures.yaml. Robot pose is passed as a custom
header: X-Robot-Pose: x,y,yaw (set by vlm_client in sim mode).
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import random
import time

import yaml
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

# ── Config ────────────────────────────────────────────────────────────
LATENCY_MS = float(os.getenv("SIM_VLM_LATENCY_MS", "1500"))
FAIL_RATE = float(os.getenv("SIM_VLM_FAIL_RATE", "0.05"))
TIMEOUT_MS = float(os.getenv("SIM_VLM_TIMEOUT_MS", "10000"))
FIXTURES_FILE = os.getenv("SIM_VLM_FIXTURES_FILE", "/app/config/mock_vlm_fixtures.yaml")

app = FastAPI(title="Mock vLLM (Ridgeback Sim)")

# Load fixtures once at startup
_fixtures: dict = {}

@app.on_event("startup")
def load_fixtures():
    global _fixtures
    try:
        with open(FIXTURES_FILE) as f:
            _fixtures = yaml.safe_load(f) or {}
        print(f"Loaded fixtures from {FIXTURES_FILE}: "
              f"{len(_fixtures.get('fixtures', []))} detection fixtures, "
              f"{len(_fixtures.get('direction_hints', []))} direction hints")
    except FileNotFoundError:
        print(f"WARNING: Fixtures file not found: {FIXTURES_FILE}. All responses will be empty.")
        _fixtures = {}


def _pose_from_header(request: Request) -> tuple[float, float, float]:
    pose_header = request.headers.get("X-Robot-Pose", "0,0,0")
    parts = pose_header.split(",")
    try:
        return float(parts[0]), float(parts[1]), float(parts[2]) if len(parts) > 2 else 0.0
    except (ValueError, IndexError):
        return 0.0, 0.0, 0.0


def _find_fixture(x: float, y: float, prompt_lower: str) -> dict | None:
    for f in _fixtures.get("fixtures", []):
        dx = x - f.get("cx", 0)
        dy = y - f.get("cy", 0)
        dist = math.sqrt(dx*dx + dy*dy)
        pattern = f.get("prompt_pattern", "")
        if dist <= f.get("radius", 2.0) and pattern.lower() in prompt_lower:
            return f.get("response", {})
    return None


def _find_direction_hint(x: float, y: float, prompt_lower: str) -> dict | None:
    for h in _fixtures.get("direction_hints", []):
        dx = x - h.get("cx", 0)
        dy = y - h.get("cy", 0)
        dist = math.sqrt(dx*dx + dy*dy)
        pattern = h.get("prompt_pattern", "")
        if dist <= h.get("radius", 3.0) and pattern.lower() in prompt_lower:
            return h.get("response", {})
    return None


def _build_openai_response(content: str, model: str = "mock-qwen2.5-vl") -> dict:
    return {
        "id": f"mock-{int(time.time()*1000)}",
        "object": "chat.completion",
        "created": int(time.time()),
        "model": model,
        "choices": [
            {
                "index": 0,
                "message": {"role": "assistant", "content": content},
                "finish_reason": "stop",
            }
        ],
        "usage": {"prompt_tokens": 50, "completion_tokens": 30, "total_tokens": 80},
    }


@app.get("/health")
def health():
    return {"status": "ok", "mode": "mock", "latency_ms": LATENCY_MS}


@app.post("/v1/chat/completions")
async def chat_completions(request: Request):
    # Inject configured failure
    if random.random() < FAIL_RATE:
        return JSONResponse(
            status_code=503,
            content={"error": {"message": "Mock VLM simulated failure", "type": "service_unavailable"}},
        )

    # Inject latency
    await asyncio.sleep(LATENCY_MS / 1000.0)

    body = await request.json()
    messages = body.get("messages", [])
    prompt = " ".join(
        m.get("content", "") if isinstance(m.get("content"), str)
        else " ".join(
            p.get("text", "") for p in m.get("content", []) if isinstance(p, dict) and p.get("type") == "text"
        )
        for m in messages
    ).lower()

    rx, ry, ryaw = _pose_from_header(request)

    # Try detection fixture first
    fixture = _find_fixture(rx, ry, prompt)
    if fixture:
        detections = fixture.get("detections", [])
        content = json.dumps({
            "room_detections": detections,
            "description": f"Mock VLM: found {len(detections)} room sign(s) near pose ({rx:.1f},{ry:.1f})",
        })
        return JSONResponse(_build_openai_response(content))

    # Try direction hint
    hint = _find_direction_hint(rx, ry, prompt)
    if hint:
        content = json.dumps({
            "room_detections": [],
            "direction": hint.get("direction", "unknown"),
            "reasoning": hint.get("reasoning", "Mock VLM direction suggestion."),
            "suggested_bearing_deg": hint.get("suggested_bearing_deg", 0.0),
        })
        return JSONResponse(_build_openai_response(content))

    # No fixture match — return empty detection
    content = json.dumps({
        "room_detections": [],
        "description": f"Mock VLM: no recognizable room signs near pose ({rx:.1f},{ry:.1f})",
    })
    return JSONResponse(_build_openai_response(content))
