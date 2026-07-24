"""
Unit tests for vio_live — live VIO/EV DDS watchdog verdict logic.

Pure logic only, no ROS: exercises compute_verdict() directly and drives
VioLiveMonitor through its update_*/snapshot API.  Run from the package root:

    python -m pytest test/test_vio_live.py -v

Covers: OK / WARN(low-rate) / CRIT(coasting) / CRIT(EV-input silent) verdicts
and the edge-triggered reject counters.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pid_tuning_assistant.vio_live import (
    VioLiveMonitor, compute_verdict, DEFAULT_DATA_TIMEOUT_S,
)

# Realistic wall-clock epoch (µs range ≈ 1.7e15 > WALL_CLOCK_US_MIN) so the
# latency discriminator treats timestamp_sample as a wall-clock stamp, not a
# PX4 boot-relative one.
BASE_T = 1_700_000_000.0


# ── helpers ───────────────────────────────────────────────────────────────────

def _healthy_flags(mon: VioLiveMonitor, *, reject_hor=False, reject_ver=False,
                   reject_yaw=False, dead_reckoning=False):
    mon.update_flags(
        cs_ev_pos=True, cs_ev_yaw=True, cs_ev_hgt=True, cs_ev_vel=False,
        cs_inertial_dead_reckoning=dead_reckoning,
        reject_hor_pos=reject_hor, reject_ver_pos=reject_ver, reject_yaw=reject_yaw,
    )


def _feed_ev(mon: VioLiveMonitor, *, start: float, rate_hz: float, n: int,
             latency_ms: float = 40.0, source: str = "visual"):
    """Push n EV-input messages spaced at 1/rate, with a fixed latency."""
    dt = 1.0 / rate_hz
    for i in range(n):
        now = start + i * dt
        ts_sample_us = int((now - latency_ms / 1000.0) * 1e6)  # wall-clock domain
        mon.update_ev_input(
            now=now, source=source, timestamp_sample_us=ts_sample_us,
            reset_counter=0, quality=0, position=[0.0, 0.0, 0.0],
            q=[1.0, 0.0, 0.0, 0.0])
    return start + (n - 1) * dt


# ── (1) OK ────────────────────────────────────────────────────────────────────

def test_verdict_ok():
    """Fusion flags true, rate 30 Hz, latency 40 ms → OK."""
    mon = VioLiveMonitor()
    _healthy_flags(mon)
    last = _feed_ev(mon, start=BASE_T, rate_hz=30.0, n=60, latency_ms=40.0)
    st = mon.snapshot(now=last, expected_rate_hz=30.0, in_air=True)
    assert st.verdict == "OK", st.verdict_reasons
    assert st.verdict_reasons == []
    assert abs(st.ev_rate_hz - 30.0) < 3.0
    assert st.ev_latency_ms is not None and st.ev_latency_ms < 100.0


# ── (2) CRIT — coasting ───────────────────────────────────────────────────────

def test_verdict_crit_dead_reckoning():
    """cs_inertial_dead_reckoning=True → CRIT regardless of an otherwise-fine EV feed."""
    mon = VioLiveMonitor()
    _healthy_flags(mon, dead_reckoning=True)
    last = _feed_ev(mon, start=BASE_T, rate_hz=30.0, n=60)
    st = mon.snapshot(now=last, expected_rate_hz=30.0, in_air=True)
    assert st.verdict == "CRIT"
    assert "coasting" in st.verdict_reasons


# ── (3) CRIT — EV input silent ────────────────────────────────────────────────

def test_verdict_crit_ev_input_silent():
    """Last EV input older than data_timeout → CRIT."""
    mon = VioLiveMonitor()
    _healthy_flags(mon)
    last = _feed_ev(mon, start=BASE_T, rate_hz=30.0, n=60)
    # Poll well after the last arrival: age > DEFAULT_DATA_TIMEOUT_S.
    st = mon.snapshot(now=last + DEFAULT_DATA_TIMEOUT_S + 0.5,
                      expected_rate_hz=30.0, in_air=True)
    assert st.verdict == "CRIT"
    assert "ev_input_silent" in st.verdict_reasons
    assert st.ev_input_age_s is not None and st.ev_input_age_s > DEFAULT_DATA_TIMEOUT_S


# ── (4) WARN — low rate ───────────────────────────────────────────────────────

def test_verdict_warn_low_rate():
    """Rate 8 Hz with an expected 30 Hz and healthy flags → WARN (not CRIT)."""
    mon = VioLiveMonitor()
    _healthy_flags(mon)
    last = _feed_ev(mon, start=BASE_T, rate_hz=8.0, n=40, latency_ms=40.0)
    st = mon.snapshot(now=last, expected_rate_hz=30.0, in_air=True)
    assert st.verdict == "WARN"
    assert "low_rate" in st.verdict_reasons
    assert st.ev_rate_hz < 0.5 * 30.0


# ── (5) reject counters ───────────────────────────────────────────────────────

def test_reject_counters():
    """A False→True→False→True sequence on reject_hor_pos counts two edges."""
    mon = VioLiveMonitor()
    seq = [False, True, True, False, True, False]  # two rising edges
    for r in seq:
        mon.update_flags(
            cs_ev_pos=True, cs_ev_yaw=True, cs_ev_hgt=True, cs_ev_vel=False,
            cs_inertial_dead_reckoning=False,
            reject_hor_pos=r, reject_ver_pos=False, reject_yaw=False)
    _feed_ev(mon, start=BASE_T, rate_hz=30.0, n=5)
    st = mon.snapshot(now=BASE_T + 1.0, expected_rate_hz=30.0, in_air=True)
    assert st.reject_hor_pos_count == 2
    assert st.reject_ver_pos_count == 0
    assert st.reject_yaw_count == 0


# ── extra coverage: verdict function edges ────────────────────────────────────

def test_verdict_unavailable_when_no_topics():
    mon = VioLiveMonitor()
    st = mon.snapshot(now=BASE_T, expected_rate_hz=30.0)
    assert st.verdict == "UNAVAILABLE"
    assert st.available is False


def test_verdict_crit_ev_pos_not_fused_airborne():
    v, reasons = compute_verdict(
        available=True, ev_input_available=True,
        cs_ev_pos=False, cs_ev_yaw=True, cs_inertial_dead_reckoning=False,
        reject_hor_pos=False, reject_ver_pos=False, reject_yaw=False,
        ev_rate_hz=30.0, ev_latency_ms=40.0, ev_input_age_s=0.02,
        expected_rate_hz=30.0, data_timeout_s=0.5, in_air=True)
    assert v == "CRIT"
    assert "ev_pos_not_fused_airborne" in reasons


def test_verdict_warn_high_latency():
    v, reasons = compute_verdict(
        available=True, ev_input_available=True,
        cs_ev_pos=True, cs_ev_yaw=True, cs_inertial_dead_reckoning=False,
        reject_hor_pos=False, reject_ver_pos=False, reject_yaw=False,
        ev_rate_hz=30.0, ev_latency_ms=150.0, ev_input_age_s=0.02,
        expected_rate_hz=30.0, data_timeout_s=0.5, in_air=True)
    assert v == "WARN"
    assert "high_latency" in reasons


def test_latency_none_for_boot_time_stamp():
    """A PX4 boot-relative timestamp_sample (small µs) yields no latency figure."""
    mon = VioLiveMonitor()
    mon.update_ev_input(
        now=BASE_T, source="visual", timestamp_sample_us=5_000_000,  # 5 s uptime
        reset_counter=0, quality=0, position=[0.0, 0.0, 0.0],
        q=[1.0, 0.0, 0.0, 0.0])
    st = mon.snapshot(now=BASE_T, expected_rate_hz=30.0)
    assert st.ev_latency_ms is None


def test_divergence_computed():
    mon = VioLiveMonitor()
    mon.update_ekf_output(position=[0.0, 0.0, 0.0])
    mon.update_ev_input(
        now=BASE_T, source="visual", timestamp_sample_us=int((BASE_T - 0.04) * 1e6),
        reset_counter=0, quality=0, position=[3.0, 4.0, 0.0],
        q=[1.0, 0.0, 0.0, 0.0])
    st = mon.snapshot(now=BASE_T, expected_rate_hz=30.0)
    assert st.ev_ekf_divergence_m is not None
    assert abs(st.ev_ekf_divergence_m - 5.0) < 1e-6
