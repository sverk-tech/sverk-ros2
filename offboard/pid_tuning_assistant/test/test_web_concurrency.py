"""
Web-layer concurrency / memory tests.

Run from the package root (needs fastapi; rclpy is pulled in transitively by
web_server → telemetry_node). Skip cleanly where either is absent:
    python -m pytest test/test_web_concurrency.py -q

Covered:
* live fan-out — two WebSocket clients each receive every frame (the old
  destructive pop_live() let a second tab steal frames from the first);
* non-blocking analysis — /api/status keeps answering while a stop-recording
  analysis of 100k frames runs (analyse() moved off the event loop);
* run memory cap — max_runs_in_memory evicts the oldest run; /api/runs stays
  consistent and the evicted run's timeseries 404s (History degrades cleanly);
* honest 503 in --no-ros mode for /api/start and /api/params/live.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

# These pull in fastapi (+ rclpy via telemetry_node). Skip cleanly where absent.
pytest.importorskip("fastapi")
pytest.importorskip("rclpy")

from fastapi.testclient import TestClient

from pid_tuning_assistant.web_server import create_app
from pid_tuning_assistant.data_buffer import RecordingBuffer, TelemetryFrame


def _frame(i, sp=0.1, nav=10):
    return TelemetryFrame(
        t=round(i * 0.01, 3), roll_rate=0.1, roll_rate_sp=sp,
        pitch_rate=0.0, pitch_rate_sp=0.0, nav_state=nav)


# ── live fan-out (buffer level, exact counts) ────────────────────────────────────

def test_fanout_each_consumer_gets_all_frames():
    """attach_live_consumer registers an independent ring; push() fans out to all."""
    buffer = RecordingBuffer(maxlen=1000)
    a = buffer.attach_live_consumer()
    b = buffer.attach_live_consumer()

    N = 50
    for i in range(N):
        buffer.push(TelemetryFrame(t=round(i * 0.01, 3), roll_rate=float(i)))

    fa = buffer.drain_live_consumer(a)
    fb = buffer.drain_live_consumer(b)
    assert len(fa) == len(fb) == N            # neither steals from the other
    assert [f.t for f in fa] == [f.t for f in fb]

    # detach stops delivery to a while b keeps receiving
    buffer.detach_live_consumer(a)
    buffer.push(TelemetryFrame(t=99.0))
    assert len(buffer.drain_live_consumer(b)) == 1
    assert buffer.drain_live_consumer(a) == []


# ── live fan-out (two real WebSocket clients) ────────────────────────────────────

def test_ws_two_clients_receive_identical_frames():
    """
    Two open tabs used to fight over one destructive ring — the first pop_live()
    emptied it and the second client saw nothing. With per-client rings both get
    the same frames.
    """
    buffer = RecordingBuffer(maxlen=10_000)
    client = TestClient(create_app(buffer))

    with client.websocket_connect("/ws/stream") as ws_a, \
         client.websocket_connect("/ws/stream") as ws_b:
        time.sleep(0.15)   # let both handlers attach their consumer

        got_a, got_b = [], []
        for i in range(3):
            buffer.push(_frame(i, sp=float(i)))
            # > one 0.1 s server tick → this frame is drained as its own message,
            # so exactly one message is pending per client this round (no hang).
            time.sleep(0.15)
            got_a.append(ws_a.receive_json())
            got_b.append(ws_b.receive_json())

    assert len(got_a) == len(got_b) == 3
    assert [f["t"] for f in got_a] == [f["t"] for f in got_b]


# ── non-blocking analysis ────────────────────────────────────────────────────────

def test_status_responsive_during_100k_frame_analysis():
    """
    /api/stop analyses the full frame list; on 100k frames that used to freeze
    the event loop (and every other request) for seconds. Analysis now runs in a
    thread pool, so /api/status must keep answering while stop is in flight.
    """
    buffer = RecordingBuffer(maxlen=200_000)
    client = TestClient(create_app(buffer))

    buffer.start_recording("big")
    # Steps every 100 frames → many step events → non-trivial analysis time,
    # which is exactly the load that would block a synchronous handler.
    for i in range(100_000):
        sp = 0.5 if (i // 100) % 2 else -0.5
        buffer.push(TelemetryFrame(
            t=round(i * 0.01, 3), roll_rate=sp * 0.9, roll_rate_sp=sp,
            pitch_rate=0.0, pitch_rate_sp=0.0, nav_state=15))

    done = threading.Event()

    def _do_stop():
        try:
            client.post("/api/stop", json={})
        finally:
            done.set()

    th = threading.Thread(target=_do_stop, daemon=True)
    th.start()

    served = 0
    for _ in range(50):
        t0 = time.perf_counter()
        r = client.get("/api/status")
        dt = time.perf_counter() - t0
        assert r.status_code == 200
        assert dt < 1.0, f"/api/status blocked {dt:.2f}s during analysis"
        served += 1
        if done.is_set():
            break
        time.sleep(0.02)

    th.join(timeout=60)
    assert not th.is_alive(), "stop-recording never completed"
    assert served >= 1


# ── run memory cap ───────────────────────────────────────────────────────────────

def test_max_runs_in_memory_evicts_oldest():
    buffer = RecordingBuffer(maxlen=10_000)
    client = TestClient(create_app(buffer, max_runs_in_memory=3))

    for i in range(5):
        buffer.start_recording(f"run{i}")
        buffer.push(_frame(0))
        buffer.push(_frame(1))
        assert client.post("/api/stop", json={}).status_code == 200

    runs = client.get("/api/runs").json()
    assert set(runs.keys()) == {"run2", "run3", "run4"}   # newest 3 kept

    # Evicted run → timeseries 404 (History tab handles this gracefully);
    # a retained run still resolves.
    assert client.get("/api/runs/run0/timeseries?signal=roll_rate").status_code == 404
    assert client.get("/api/runs/run4/timeseries?signal=roll_rate").status_code == 200


# ── --no-ros honest 503 ──────────────────────────────────────────────────────────

def test_start_returns_503_when_ros_disabled():
    buffer = RecordingBuffer(maxlen=1000)
    client = TestClient(create_app(buffer, ros_enabled=False))
    r = client.post("/api/start", json={"run_name": "x"})
    assert r.status_code == 503
    assert not buffer.recording   # no empty recording started


def test_params_live_returns_503_when_ros_disabled():
    buffer = RecordingBuffer(maxlen=1000)
    client = TestClient(create_app(buffer, ros_enabled=False))
    assert client.post("/api/params/live").status_code == 503
