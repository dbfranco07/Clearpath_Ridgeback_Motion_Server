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


# ── Safety FOV veto + map-complete termination ────────────────────────


class TestFovBlockExposure:
    """The dashboard surfaces safety_controller's per-axis veto mask."""

    def test_fov_block_in_status(self, http):
        r = http.get(f"{DASHBOARD_URL}/api/status", timeout=5.0)
        assert r.status_code == 200
        body = r.json()
        fov = body.get("safety", {}).get("fov_block", {})
        # Mask may be {} during the first few hundred ms while the safety
        # controller waits for its first scan — accept that and only validate
        # the schema once it has been populated.
        if not fov:
            time.sleep(1.5)
            r = http.get(f"{DASHBOARD_URL}/api/status", timeout=5.0)
            fov = r.json().get("safety", {}).get("fov_block", {})
        assert fov, "safety.fov_block never appeared in /api/status — is safety_controller publishing /safety/fov_block?"
        for axis in ("plus_x", "minus_x", "plus_y", "minus_y", "plus_yaw", "minus_yaw"):
            assert axis in fov, f"fov_block missing axis: {axis}"
            assert isinstance(fov[axis], bool), f"fov_block.{axis} is not bool: {fov[axis]!r}"
        # The 270° LiDAR doesn't cover the rear sector; minus_x must report
        # clear (False) at all times — backing up is never blocked by FOV.
        assert fov["minus_x"] is False, "minus_x must always be clear (rear unmeasured)"


@pytest.mark.integration
class TestMapCompleteTermination:
    """When the explorer runs out of viable frontiers, the orchestrator
    transitions to idle_map_complete instead of aborting hard."""

    def test_unknown_room_eventually_idle_map_complete(self, http):
        # Pick a room id that no fixture publishes so the explorer cannot
        # converge on it. We expect the explorer to exhaust frontiers and
        # the orchestrator to publish state=idle_map_complete.
        r = http.post(
            f"{DASHBOARD_URL}/api/mission",
            json={"command": "go to room 9999", "strategy": "frontier_nearest"},
            timeout=10.0,
        )
        assert r.status_code in (200, 201, 202)

        start = time.time()
        last_state = None
        while time.time() - start < MISSION_TIMEOUT_S:
            time.sleep(5)
            sr = http.get(f"{DASHBOARD_URL}/api/status", timeout=5.0)
            state = (sr.json().get("mission", {}).get("command") or "").lower()
            last_state = state
            if state == "idle_map_complete":
                note = sr.json().get("mission", {}).get("note") or ""
                # Either generic map_complete or target_not_found is fine —
                # both correctly indicate the explorer terminated cleanly.
                assert note in ("map_complete", "target_not_found"), (
                    f"unexpected idle_map_complete note: {note!r}"
                )
                return
        pytest.fail(
            f"Mission never reached idle_map_complete (last state: {last_state!r})"
        )
