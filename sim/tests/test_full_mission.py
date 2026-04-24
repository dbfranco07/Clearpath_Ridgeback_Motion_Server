"""
test_full_mission.py — CI smoke test for the full sim mission pipeline.

Runs against the mock VLM (VLM_BACKEND=mock) so no DGX dependency.
The sim stack must already be running when this is executed, OR this
test starts it (controlled by SIM_MANAGED env var).

Run:
    cd sim && pytest tests/test_full_mission.py -v
"""

import json
import os
import time

import pytest
import httpx

DASHBOARD_URL = os.getenv("DASHBOARD_URL", "http://localhost:8081")
MOCK_VLM_URL = os.getenv("MOCK_VLM_URL", "http://localhost:9000")
MISSION_TIMEOUT_S = int(os.getenv("MISSION_TIMEOUT_S", "120"))


@pytest.fixture(scope="session")
def http():
    return httpx.Client(timeout=10.0)


# ── Mock VLM ─────────────────────────────────────────────────────────

class TestMockVlm:
    def test_health(self, http):
        r = http.get(f"{MOCK_VLM_URL}/health")
        assert r.status_code == 200
        assert r.json()["mode"] == "mock"

    def test_chat_completions_no_fixture(self, http):
        r = http.post(
            f"{MOCK_VLM_URL}/v1/chat/completions",
            json={
                "model": "mock",
                "messages": [{"role": "user", "content": "What room numbers do you see?"}],
            },
            headers={"X-Robot-Pose": "0,0,0"},
        )
        assert r.status_code == 200
        body = r.json()
        assert "choices" in body
        content = json.loads(body["choices"][0]["message"]["content"])
        assert "room_detections" in content

    def test_chat_completions_fixture_hit(self, http):
        """Robot near sign_301 in small_room.world → should return detection."""
        r = http.post(
            f"{MOCK_VLM_URL}/v1/chat/completions",
            json={
                "model": "mock",
                "messages": [{"role": "user", "content": "What room number do you see?"}],
            },
            headers={"X-Robot-Pose": "3.0,0.0,0"},  # near small_room sign_301
        )
        assert r.status_code == 200
        body = r.json()
        content = json.loads(body["choices"][0]["message"]["content"])
        detections = content.get("room_detections", [])
        assert any(d["room_id"] == "301" for d in detections), (
            f"Expected detection of room 301 near (3.0, 0.0). Got: {detections}"
        )


# ── Dashboard ─────────────────────────────────────────────────────────

class TestDashboard:
    def test_health(self, http):
        r = http.get(f"{DASHBOARD_URL}/health")
        assert r.status_code == 200

    def test_mission_submit(self, http):
        r = http.post(
            f"{DASHBOARD_URL}/api/mission",
            json={"command": "go to room 301 and come back", "strategy": "frontier_nearest"},
        )
        assert r.status_code in (200, 201, 202), f"Unexpected status: {r.status_code} — {r.text}"

    def test_mission_status(self, http):
        r = http.get(f"{DASHBOARD_URL}/api/mission/status")
        assert r.status_code == 200
        body = r.json()
        assert "state" in body


# ── Integration: full mission ─────────────────────────────────────────

@pytest.mark.integration
class TestFullMission:
    """Submits a mission and waits for completion. Marks slow — run separately."""

    def test_find_room_301_small_room(self, http):
        # Submit mission
        r = http.post(
            f"{DASHBOARD_URL}/api/mission",
            json={"command": "go to room 301 and come back", "strategy": "frontier_nearest"},
            timeout=10.0,
        )
        assert r.status_code in (200, 201, 202)

        # Poll until DONE/FAILED
        start = time.time()
        final_state = None
        while time.time() - start < MISSION_TIMEOUT_S:
            time.sleep(5)
            sr = http.get(f"{DASHBOARD_URL}/api/mission/status", timeout=5.0)
            state = sr.json().get("state", "UNKNOWN")
            if state in ("DONE", "FAILED", "ERROR", "TIMEOUT"):
                final_state = state
                break

        assert final_state == "DONE", (
            f"Mission did not complete successfully. Final state: {final_state}"
        )

        # Verify metrics logged
        mr = http.get(f"{DASHBOARD_URL}/api/metrics", timeout=5.0)
        metrics = mr.json()
        assert metrics.get("result") in ("success", "DONE")
        assert metrics.get("elapsed_s", 0) > 0
