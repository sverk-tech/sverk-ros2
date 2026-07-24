"""
Web-layer security tests.

Run from the package root (needs fastapi + rclpy — i.e. the container/CI env):
    python -m pytest test/test_web_security.py -q

Covered:
* token auth — off by default, enforced when a token is configured (REST + WS);
* run_name validation — HTML/JS payloads rejected with 400, sane names accepted;
* NaN-safe JSON — /api/analysis and the _r/_json_safe helpers never emit the
  non-standard NaN/Infinity tokens that break the browser's JSON.parse.

The tests import the real FastAPI app (no ROS spinning — the TelemetryNode is
never started), so they exercise the actual middleware and endpoints.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

# These pull in fastapi (+ rclpy via telemetry_node). Skip cleanly where absent.
pytest.importorskip("fastapi")
rclpy = pytest.importorskip("rclpy")

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from pid_tuning_assistant.web_server import (
    create_app, _r, _json_safe, _sanitize_run_name, _RUN_NAME_RE,
)
from pid_tuning_assistant.data_buffer import RecordingBuffer, TelemetryFrame


def _client(auth_token=None):
    buffer = RecordingBuffer(maxlen=10_000)
    app = create_app(buffer, auth_token=auth_token)
    return TestClient(app), buffer


# ── token auth ─────────────────────────────────────────────────────────────────

def test_status_open_without_token():
    """No token configured → compatibility mode, /api/status is reachable."""
    client, _ = _client(auth_token=None)
    r = client.get("/api/status")
    assert r.status_code == 200
    assert "recording" in r.json()


def test_status_requires_token_when_configured():
    client, _ = _client(auth_token="s3cret")
    # No header → 401
    assert client.get("/api/status").status_code == 401
    # Wrong token → 401
    assert client.get(
        "/api/status", headers={"Authorization": "Bearer nope"}).status_code == 401
    # Correct bearer token → 200
    r = client.get("/api/status", headers={"Authorization": "Bearer s3cret"})
    assert r.status_code == 200
    # Query-param fallback (download links) → 200
    assert client.get("/api/status?token=s3cret").status_code == 200


def test_index_page_stays_open_with_token():
    """The HTML shell must load unauthenticated so the operator can enter a token."""
    client, _ = _client(auth_token="s3cret")
    assert client.get("/").status_code in (200, 500)  # 500 only if STATIC_DIR missing


def test_ws_rejected_without_token():
    client, _ = _client(auth_token="s3cret")
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/ws/stream"):
            pass


def test_ws_accepted_with_token():
    client, _ = _client(auth_token="s3cret")
    with client.websocket_connect("/ws/stream?token=s3cret") as ws:
        # Connection accepted; close immediately (no frames buffered).
        ws.close()


# ── run_name validation / XSS ────────────────────────────────────────────────────

def test_start_rejects_xss_run_name():
    client, _ = _client()
    r = client.post("/api/start", json={"run_name": "<img src=x onerror=alert(1)>"})
    assert r.status_code == 400


def test_start_accepts_safe_run_name():
    client, buffer = _client()
    r = client.post("/api/start", json={"run_name": "ok_run-1.2"})
    assert r.status_code == 200
    assert r.json()["run_name"] == "ok_run-1.2"
    assert buffer.recording


def test_empty_run_name_gets_safe_default():
    client, _ = _client()
    r = client.post("/api/start", json={})
    assert r.status_code == 200
    assert _RUN_NAME_RE.match(r.json()["run_name"])


def test_sanitize_run_name_maps_illegal_chars():
    assert _sanitize_run_name('log 2026-07/<b>.ulg') == "log_2026-07__b_.ulg"
    assert _RUN_NAME_RE.match(_sanitize_run_name("../../etc/passwd"))
    assert _sanitize_run_name("") == "run"
    assert len(_sanitize_run_name("x" * 200)) == 64


def test_timeseries_rejects_unknown_signal():
    client, buffer = _client()
    buffer.start_recording("r1")
    buffer.push(TelemetryFrame(t=0.0, roll_rate=0.1))
    client.post("/api/stop", json={})
    # Real field is fine; arbitrary attribute is rejected before getattr.
    assert client.get("/api/runs/r1/timeseries?signal=roll_rate").status_code == 200
    assert client.get("/api/runs/r1/timeseries?signal=__class__").status_code == 400


# ── NaN-safe JSON ────────────────────────────────────────────────────────────────

def test_r_maps_nonfinite_to_none():
    assert _r(float("nan")) is None
    assert _r(float("inf")) is None
    assert _r(float("-inf")) is None
    assert _r(1.23456, 2) == 1.23
    assert _r(None) is None


def test_json_safe_strips_nan_inf():
    out = _json_safe({"a": float("nan"), "b": [1.0, float("inf")], "c": "ok", "d": True})
    assert out["a"] is None
    assert out["b"] == [1.0, None]
    assert out["c"] == "ok"
    assert out["d"] is True


def test_analysis_json_is_valid_with_nan_frames():
    """PX4 sends NaN in setpoints; the serialized analysis must still parse."""
    client, buffer = _client()
    buffer.start_recording("nan_run")
    for i in range(120):
        t = i * 0.02
        # Alternate real values with NaN setpoints to seed non-finite floats.
        sp = float("nan") if i % 2 else 0.5
        buffer.push(TelemetryFrame(
            t=t, roll_rate=0.4, roll_rate_sp=sp,
            pitch_rate=0.0, pitch_rate_sp=float("nan"),
            nav_state=10,
        ))
    assert client.post("/api/stop", json={}).status_code == 200

    r = client.get("/api/analysis")
    assert r.status_code == 200
    # Must be strict-JSON parseable and contain no NaN/Infinity tokens.
    import json
    body = r.text
    assert "NaN" not in body and "Infinity" not in body
    json.loads(body)  # raises if invalid
