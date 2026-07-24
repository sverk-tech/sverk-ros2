"""
Unit tests for topic delivery statistics (no ROS needed).

Run from the package root:
    python -m pytest test/test_topic_stats.py -v

These exercise the pure ``compute_stats_from_arrivals`` helper and the
recording-reset wiring, isolated from rclpy.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

from pid_tuning_assistant.data_buffer import (
    compute_stats_from_arrivals,
    TopicStats,
    RecordingBuffer,
)


# ── Defect 1: windowed rate is not under-reported on a truncated deque ────────

def test_windowed_rate_survives_deque_truncation():
    """250 Hz for 120 s = 30 000 arrivals; the deque keeps only the last
    20 000 (_TS_MAXLEN). The windowed rate must read ~250 Hz, whereas the old
    n / full_duration formula would read ~167 Hz."""
    dt = 1.0 / 250.0
    full = [i * dt for i in range(30_000)]      # 120 s of 250 Hz arrivals
    truncated = full[-20_000:]                  # deque cap emulation

    st = compute_stats_from_arrivals(
        truncated, "VehicleAngularVelocity", total_duration_s=120.0)

    assert 249.0 <= st.achieved_rate_hz <= 251.0
    # Old code: 20000 / 120 ≈ 166.7 Hz — assert we are nowhere near it.
    assert st.achieved_rate_hz > 200.0
    # Window span ≈ 19999 / 250 ≈ 80.0 s; full recording kept separately.
    assert 79.0 <= st.duration_s <= 81.0
    assert st.total_duration_s == 120.0
    assert st.f_nyquist_hz == pytest.approx(st.achieved_rate_hz / 2.0, abs=0.1)
    assert st.jitter_ms == 0.0                  # perfectly uniform arrivals


# ── Defect 1: jitter and drop are measured over the same window ───────────────

def test_dropouts_raise_drop_pct_but_rate_stays_reasonable():
    """A 100 Hz stream with periodic burst dropouts (4 missing samples every
    40) yields drop_pct > 0 while the rate stays in the O(90–100) range."""
    dt = 0.01                                   # 100 Hz nominal
    ts = []
    t = 0.0
    for i in range(2000):
        ts.append(t)
        if i % 40 == 39:
            t += 5 * dt                         # burst gap (4 dropped samples)
        else:
            t += dt

    st = compute_stats_from_arrivals(ts, "VehicleAngularVelocity")

    assert st.drop_pct > 0.0
    assert 90.0 <= st.achieved_rate_hz <= 100.0
    assert st.jitter_ms >= 0.0
    assert st.msg_count == 2000


def test_uniform_stream_has_no_drops():
    ts = [i * 0.004 for i in range(5000)]       # clean 250 Hz
    st = compute_stats_from_arrivals(ts, "VehicleAngularVelocity")
    assert st.drop_pct == 0.0
    assert st.jitter_ms == 0.0
    assert 249.0 <= st.achieved_rate_hz <= 251.0


# ── Degenerate inputs ─────────────────────────────────────────────────────────

def test_empty_arrivals_give_empty_stats():
    st = compute_stats_from_arrivals([], "VehicleAngularVelocity",
                                     total_duration_s=5.0)
    assert st.msg_count == 0
    assert st.achieved_rate_hz == 0.0
    assert st.f_nyquist_hz == 0.0
    assert st.duration_s == 0.0
    assert st.total_duration_s == 5.0


def test_single_arrival_gives_zero_rate():
    st = compute_stats_from_arrivals([1.23], "VehicleAngularVelocity")
    assert st.msg_count == 1
    assert st.achieved_rate_hz == 0.0


# ── Defect 2: recording start resets node stats ───────────────────────────────

def test_start_recording_resets_node_stats():
    """RecordingBuffer.start_recording() must ask the node to reset its
    per-topic arrival deques so stats don't bleed between recordings."""
    class FakeNode:
        def __init__(self):
            self.reset_calls = 0

        def reset_stats(self):
            self.reset_calls += 1

    buf = RecordingBuffer()
    fake = FakeNode()
    buf._node_ref = fake

    buf.start_recording("run_a")
    buf.stop_recording()
    buf.start_recording("run_b")

    assert fake.reset_calls == 2


def test_start_recording_without_node_ref_is_safe():
    buf = RecordingBuffer()          # _node_ref stays None
    name = buf.start_recording("solo")
    assert name == "solo"
