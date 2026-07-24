"""
Unit tests for vio_analyzer — VIO/EV external-vision analysis.

Synthetic data with known ground truth, no real .ulg / pyulog needed.  Run from
the package root:

    python -m pytest test/test_vio.py -v

Covers: EV_DELAY recovery, delay-vs-extrinsic bias discrimination, fusion
dropout intervals, test-ratio statistics, and aid_src gating of the delay
estimate.
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest

from pid_tuning_assistant.ulog_adapter import EstimatorSeries
from pid_tuning_assistant.vio_analyzer import (
    VioAnalyzer, estimate_ev_delay, innovation_bias, fusion_dropouts,
    axis_test_ratio_stats,
)


# ── fake UlogParseResult for the orchestrator ─────────────────────────────────

def _fake_parse(series: EstimatorSeries, *, delay_estimable: bool,
                params: dict = None) -> types.SimpleNamespace:
    caps = types.SimpleNamespace(
        vio_basic_available=bool(series.has_flags and series.has_innov),
        vio_aidsrc_available=bool(series.has_aidsrc_pos),
        vio_delay_estimable=delay_estimable,
    )
    meta = types.SimpleNamespace(
        source_name="synthetic.ulg", duration_s=20.0,
        capabilities=caps, params=params or {}, params_all=params or {},
    )
    return types.SimpleNamespace(metadata=meta, estimator_series=series)


# ── (1) EV_DELAY recovery ─────────────────────────────────────────────────────

def test_delay_recovered():
    """EV position lags EKF velocity by a known 30 ms → recovered within ±10 ms."""
    fs = 100.0
    dt = 1.0 / fs
    t = np.arange(0.0, 25.0, dt)
    # Rich horizontal velocity (needs dynamics for the cross-correlation).
    vx = 0.6 * np.sin(2 * np.pi * 0.5 * t) + 0.4 * np.sin(2 * np.pi * 1.3 * t + 0.5)
    vy = 0.5 * np.sin(2 * np.pi * 0.7 * t + 1.0)
    # True position = integral of velocity.
    px = np.cumsum(vx) * dt
    py = np.cumsum(vy) * dt
    tau = 0.030
    # EV position at time t equals the true position at t − tau (EV lags).
    px_ev = np.interp(t - tau, t, px)
    py_ev = np.interp(t - tau, t, py)

    s = EstimatorSeries(
        t_lpos=t, lpos_vel=np.column_stack([vx, vy, np.zeros_like(t)]), has_lpos=True,
        t_vio=t, vio_pos=np.column_stack([px_ev, py_ev, np.zeros_like(t)]),
        vio_timestamp_sample=(t * 1e6), has_vio_input=True,
    )

    est = estimate_ev_delay(s, expected_rate_hz=100.0)
    assert est.estimable
    assert est.delay_ms is not None
    assert abs(est.delay_ms - 30.0) <= 10.0, est.delay_ms
    assert est.r_squared > 0.5


# ── (2) constant bias → extrinsic ─────────────────────────────────────────────

def test_constant_bias_flagged_as_extrinsic():
    """A constant innovation offset that does not grow with speed → extrinsic."""
    t_lpos = np.arange(0.0, 20.0, 0.1)            # 10 Hz
    speed = 0.5 + 0.5 * np.sin(2 * np.pi * 0.3 * t_lpos)   # varies 0..1
    t_innov = np.arange(0.0, 20.0, 0.5)           # 2 Hz
    rng = np.random.default_rng(0)
    north = 0.10 + rng.normal(0, 0.01, len(t_innov))   # constant 10 cm offset
    east = rng.normal(0, 0.01, len(t_innov))

    s = EstimatorSeries(
        t_innov=t_innov,
        ev_hpos=np.column_stack([north, east]),
        ev_vpos=np.zeros(len(t_innov)),
        has_innov=True,
        t_lpos=t_lpos,
        lpos_vel=np.column_stack([speed, np.zeros_like(speed), np.zeros_like(speed)]),
        has_lpos=True,
    )
    biases = innovation_bias(s)
    north_bias = next(b for b in biases if b.axis == "north")
    assert north_bias.classification == "extrinsic", north_bias
    assert abs(north_bias.speed_corr) < 0.5

    # And the recommendation points at EKF2_EV_POS (extrinsic), not EKF2_EV_DELAY.
    res = VioAnalyzer().analyse(_fake_parse(s, delay_estimable=False))
    params_rec = {r.param for r in res.recommendations}
    assert "EKF2_EV_POS" in params_rec
    assert "EKF2_EV_DELAY" not in params_rec


# ── (3) growing bias → delay ──────────────────────────────────────────────────

def test_growing_bias_flagged_as_delay():
    """An innovation that scales with speed → delay-like classification."""
    t_lpos = np.arange(0.0, 20.0, 0.1)
    speed = np.linspace(0.1, 2.0, len(t_lpos))         # monotone-rising speed
    t_innov = np.arange(0.0, 20.0, 0.5)
    speed_on_innov = np.interp(t_innov, t_lpos, speed)
    rng = np.random.default_rng(1)
    north = 0.1 * speed_on_innov + rng.normal(0, 0.005, len(t_innov))

    s = EstimatorSeries(
        t_innov=t_innov,
        ev_hpos=np.column_stack([north, np.zeros(len(t_innov))]),
        ev_vpos=np.zeros(len(t_innov)),
        has_innov=True,
        t_lpos=t_lpos,
        lpos_vel=np.column_stack([speed, np.zeros_like(speed), np.zeros_like(speed)]),
        has_lpos=True,
    )
    biases = innovation_bias(s)
    north_bias = next(b for b in biases if b.axis == "north")
    assert north_bias.classification == "delay", north_bias
    assert abs(north_bias.speed_corr) >= 0.5


# ── (4) dropout intervals ─────────────────────────────────────────────────────

def test_dropout_intervals():
    """cs_ev_pos with two false runs → correct count / total / max duration."""
    t_flags = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    #                     T    T    F    F    T    F    T
    cs_ev_pos = np.array([1, 1, 0, 0, 1, 0, 1], dtype=bool)
    s = EstimatorSeries(t_flags=t_flags, cs_ev_pos=cs_ev_pos, has_flags=True)

    rep = fusion_dropouts(s)
    ev_pos_gaps = [iv for iv in rep.intervals if iv.cause == "cs_ev_pos_false"]
    assert len(ev_pos_gaps) == 2
    assert sum(iv.duration_s for iv in ev_pos_gaps) == pytest.approx(3.0)
    assert max(iv.duration_s for iv in ev_pos_gaps) == pytest.approx(2.0)


# ── (5) test-ratio statistics ─────────────────────────────────────────────────

def test_test_ratio_stats():
    """A known fraction of test-ratio samples > 1 → correct %>1."""
    n = 100
    col0 = np.full(n, 0.5)
    col0[:20] = 1.5              # 20 % rejected (test_ratio > 1)
    tr = np.column_stack([col0, np.zeros(n)])
    s = EstimatorSeries(
        t_innov=np.arange(n, dtype=float) * 0.5,
        ev_hpos=np.zeros((n, 2)), has_innov=True,
        ev_hpos_tr=tr,
    )
    stats = axis_test_ratio_stats(s)
    hpos = next(st for st in stats if st.axis == "ev_hpos")
    assert hpos.pct_over_1 == pytest.approx(20.0)
    assert hpos.n == n


# ── (6) missing aid_src gates the delay estimate ─────────────────────────────

def test_missing_aidsrc_gated():
    """Default-log series (no aid_src, no VIO input) → delay not estimable and a
    logger_topics.txt hint is surfaced; no EKF2_EV_DELAY recommendation."""
    t_flags = np.linspace(0, 20, 40)
    s = EstimatorSeries(
        t_flags=t_flags,
        cs_ev_pos=np.ones(len(t_flags), dtype=bool),
        cs_ev_yaw=np.ones(len(t_flags), dtype=bool),
        has_flags=True,
        t_innov=np.arange(0, 20, 0.5),
        ev_hpos=np.zeros((40, 2)),
        ev_vpos=np.zeros(40),
        has_innov=True,
    )
    res = VioAnalyzer().analyse(_fake_parse(s, delay_estimable=False))

    assert res.delay is not None
    assert res.delay.estimable is False
    assert res.delay.delay_ms is None
    assert "logger_topics.txt" in res.delay.note
    assert all(r.param != "EKF2_EV_DELAY" for r in res.recommendations)
    # Basic capability is still advertised.
    assert res.capabilities.vio_basic_available
