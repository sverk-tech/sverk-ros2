"""
Tests for the offline PX4-autotune pipeline.

Covered:
  1. Synthetic ARX(2,2,1) identification — coefficient recovery within ±10 %.
  2. Synthetic physics end-to-end — discretised G(s)=80/(s(0.025s+1)) closed with
     the PX4 rate controller, step excitation → pipeline accepts with VAF ≥ 80,
     gate passes, kc in a sane range.
  3. GMVC formula unit test against hand-frozen reference values.
  4. Rejection on excitation-free (noise-only) data — rejected, not an exception.

No ROS / no real .ulg file — everything is built in-memory.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from pid_tuning_assistant.autotune_offline import (
    OfflineAutotune,
    identify_arx221,
    synthesize_gmvc,
    are_gains_good,
    map_to_px4_params,
    MIN_VAF_PCT,
)


# ── helpers ────────────────────────────────────────────────────────────────────

def _make_parse_result(t_us, u_xyz, y_xyz, params, rate_hz, name="sim"):
    """Minimal duck-typed stand-in for UlogParseResult that run() consumes."""
    meta = SimpleNamespace(params=dict(params), source_name=name)
    return SimpleNamespace(
        raw_torque_t=np.asarray(t_us, dtype=np.int64),
        raw_torque_xyz=np.asarray(u_xyz, dtype=float),
        raw_angvel_t=np.asarray(t_us, dtype=np.int64),
        raw_angvel_xyz=np.asarray(y_xyz, dtype=float),
        raw_angvel_rate_hz=float(rate_hz),
        metadata=meta,
    )


def _default_params():
    p = {"IMU_GYRO_CUTOFF": 40.0}
    for tag in ("ROLL", "PITCH", "YAW"):
        p[f"MC_{tag}RATE_P"] = 0.15
        p[f"MC_{tag}RATE_K"] = 1.0
    return p


# ── (1) synthetic ARX identification ───────────────────────────────────────────

def test_arx_identification_recovers_coefficients():
    rng = np.random.default_rng(42)
    a1, a2, b0, b1, b2 = -1.4, 0.5, 0.8, -0.3, 0.1
    n = 4000
    u = rng.standard_normal(n)
    y = np.zeros(n)
    for k in range(3, n):
        y[k] = (-a1 * y[k - 1] - a2 * y[k - 2]
                + b0 * u[k - 1] + b1 * u[k - 2] + b2 * u[k - 3]
                + 0.01 * rng.standard_normal())

    theta, theta_std, vaf, vaf_free, n_fit, n_val = identify_arx221(u, y)
    truth = np.array([a1, a2, b0, b1, b2])
    rel_err = np.abs(theta - truth) / np.abs(truth)

    assert np.all(rel_err < 0.10), f"coeffs off > 10%: {theta} vs {truth}"
    assert vaf > 95.0
    assert n_fit > 0 and n_val > 0
    # covariance-based std should be finite and small for a clean fit
    assert np.all(np.isfinite(theta_std))


# ── (2) synthetic physics end-to-end ──────────────────────────────────────────

def _simulate_closed_loop(dt=0.005, seconds=20.0,
                          K=1.0, P=0.15, I=0.2, D=0.003):
    """
    Closed loop: PX4 rate controller driving the ZOH-discretised plant
    G(s) = 80 / (s (0.025 s + 1)).  Square-wave setpoint → rich excitation.
    Returns (t_us, u, y).
    """
    from scipy.signal import cont2discrete
    numd, dend, _ = cont2discrete(([80.0], [0.025, 1.0, 0.0]), dt=dt, method="zoh")
    numd = numd.ravel()
    dend = dend.ravel()

    n = int(seconds / dt)
    half = int(0.5 / dt)
    r = np.array([1.0 if (k // half) % 2 == 0 else -1.0 for k in range(n)])
    y = np.zeros(n)
    u = np.zeros(n)
    integ = 0.0
    for k in range(2, n):
        e = r[k] - y[k - 1]
        integ += e * dt
        dy = (y[k - 1] - y[k - 2]) / dt
        u[k] = K * (P * e + I * integ - D * dy)
        y[k] = (-dend[1] * y[k - 1] - dend[2] * y[k - 2]
                + numd[1] * u[k - 1] + numd[2] * u[k - 2])
    t_us = (np.arange(n) * dt * 1e6).astype(np.int64)
    return t_us, u, y


def test_physics_end_to_end_accepts():
    dt = 0.005
    t_us, u, y = _simulate_closed_loop(dt=dt)
    xyz_u = np.column_stack([u, u, u])
    xyz_y = np.column_stack([y, y, y])
    pr = _make_parse_result(t_us, xyz_u, xyz_y, _default_params(), 1.0 / dt)

    res = OfflineAutotune().run(pr)
    assert len(res.axes) == 3
    roll = next(a for a in res.axes if a.axis == "roll")

    assert roll.accepted, f"expected accept, got: {roll.reason}"
    assert roll.vaf_pct >= 80.0
    assert roll.gate_pass
    assert 0.05 <= roll.kc <= 0.5
    # parameter mapping present and coherent
    p = roll.params["MC_ROLLRATE_P"]
    assert p["suggested"] == round(roll.kc, 5)
    assert roll.params["MC_ROLLRATE_K"]["suggested"] == 1.0
    assert 2.0 <= roll.params["MC_ROLL_P"]["suggested"] <= 6.5


# ── (3) GMVC formula reference ─────────────────────────────────────────────────

def test_gmvc_matches_frozen_reference():
    # a1,a2,b0,b1,b2 chosen to land in the accepted region; dt=4 ms, sigma=0.14 s.
    # Reference kc/ki/kd computed from the ported PX4 formula (frozen literals).
    a1, a2, b0, b1, b2 = -1.8, 0.85, 0.005, 0.008, 0.002
    dt, sigma = 0.004, 0.14
    kc, ki, kd = synthesize_gmvc(a1, a2, b0, b1, b2, dt, sigma)

    assert kc == pytest.approx(0.021309673974, rel=1e-6)
    assert ki == pytest.approx(9.933214323032, rel=1e-6)
    assert kd == pytest.approx(0.199494445153, rel=1e-6)

    # these particular values satisfy the sanity gate
    good, reason = are_gains_good(kc, ki, kd)
    assert good, reason

    params = map_to_px4_params("roll", kc, ki, kd, current={"MC_ROLLRATE_P": 0.1})
    assert params["MC_ROLLRATE_I"]["suggested"] == round(kc * ki, 5)
    assert params["MC_ROLLRATE_D"]["suggested"] == round(kc * kd, 5)
    assert params["MC_ROLLRATE_P"]["current"] == 0.1


def test_gate_rejects_out_of_range_gains():
    # negative gain → rejected
    ok, _ = are_gains_good(-0.1, 1.0, 0.01)
    assert not ok
    # kc too large → rejected
    ok, _ = are_gains_good(0.9, 1.0, 0.01)
    assert not ok
    # kc*kd too large → rejected
    ok, _ = are_gains_good(0.2, 1.0, 1.0)
    assert not ok


# ── (4) rejection on no excitation ─────────────────────────────────────────────

def test_no_excitation_is_rejected_not_raised():
    rng = np.random.default_rng(7)
    dt = 0.005
    n = 4000
    # Independent noise on input and output — no causal relationship.
    u = 0.02 * rng.standard_normal((n, 3))
    y = 0.5 * rng.standard_normal((n, 3))
    t_us = (np.arange(n) * dt * 1e6).astype(np.int64)
    pr = _make_parse_result(t_us, u, y, _default_params(), 1.0 / dt)

    res = OfflineAutotune().run(pr)          # must not raise
    assert len(res.axes) == 3
    for ax in res.axes:
        assert ax.rejected and not ax.accepted
        assert ax.reason  # a human-readable cause is set


def test_low_rate_log_rejected_with_hint():
    dt = 0.02   # 50 Hz — below the high-rate requirement
    n = 2000
    t_us = (np.arange(n) * dt * 1e6).astype(np.int64)
    z = np.zeros((n, 3))
    pr = _make_parse_result(t_us, z, z, _default_params(), 1.0 / dt)

    res = OfflineAutotune().run(pr)
    assert res.axes == []
    assert any("SDLOG_PROFILE" in w for w in res.warnings)


def test_missing_params_rejected():
    dt = 0.005
    n = 2000
    t_us = (np.arange(n) * dt * 1e6).astype(np.int64)
    z = np.zeros((n, 3))
    pr = _make_parse_result(t_us, z, z, {}, 1.0 / dt)   # no params

    res = OfflineAutotune().run(pr)
    assert res.axes == []
    assert any("param" in w.lower() for w in res.warnings)
