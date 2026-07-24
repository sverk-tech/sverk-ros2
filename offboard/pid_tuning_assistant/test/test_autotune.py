"""
Tests for the PX4 native-autotune integration:

  1. parse_autotune_series extracts per-axis converged ARX coeffs from a synthetic
     autotune_attitude_control_status stream (state machine roll→pitch→yaw).
  2. reconstruct_arx_plant rebuilds num/den and recovers the continuous-equivalent
     (wn, zeta) of a known 2nd-order plant sampled at a known dt (round-trip).
  3. build_tuning_comparison renders three columns when current + autotune +
     analyzer are all present, and degrades to two (no autotune) without crashing.

No ROS / no pyulog / no real .ulg — everything is built in-memory with the same
fake-dataset pattern as test_ulog_adapter.py.
"""
import math
import sys
import os
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest

from pid_tuning_assistant.ulog_adapter import (
    parse_autotune_series,
    reconstruct_arx_plant,
    AUTOTUNE_STATE,
)
from pid_tuning_assistant.tuning_compare import build_tuning_comparison


# ── fake-dataset helpers ───────────────────────────────────────────────────────

def _ds(name, data, multi_id=0):
    return types.SimpleNamespace(name=name, multi_id=multi_id, data=data)


# Converged coeffs per axis. Roll uses the frozen GMVC-reference vector (known to
# pass the sanity gate) so the px4_autotune comparison column is populated.
_ROLL_COEFF = (-1.8, 0.85, 0.005, 0.008, 0.002)
_PITCH_COEFF = (-1.7, 0.80, 0.006, 0.007, 0.002)
_YAW_COEFF = (-1.6, 0.75, 0.004, 0.006, 0.001)
_MODEL_DT = 0.004


def _autotune_status_ds(dt_us=4000):
    """Synthetic autotune_attitude_control_status: idle→init→roll→pitch→yaw→apply.

    Each axis's identification state carries a short ramp of coeffs that *ends* on
    the per-axis converged vector, so the "last sample in state" extraction has a
    non-trivial value to pick.
    """
    seq = []   # list of (state, coeff5)
    seq.append((AUTOTUNE_STATE["idle"], (0, 0, 0, 0, 0)))
    seq.append((AUTOTUNE_STATE["init"], (0, 0, 0, 0, 0)))
    # roll: two noisy samples then the converged vector
    seq.append((AUTOTUNE_STATE["roll"], (-1.0, 0.5, 0.01, 0.01, 0.0)))
    seq.append((AUTOTUNE_STATE["roll"], _ROLL_COEFF))
    seq.append((AUTOTUNE_STATE["roll_pause"], _ROLL_COEFF))
    seq.append((AUTOTUNE_STATE["pitch"], (-1.0, 0.5, 0.01, 0.01, 0.0)))
    seq.append((AUTOTUNE_STATE["pitch"], _PITCH_COEFF))
    seq.append((AUTOTUNE_STATE["pitch_pause"], _PITCH_COEFF))
    seq.append((AUTOTUNE_STATE["yaw"], (-1.0, 0.5, 0.01, 0.01, 0.0)))
    seq.append((AUTOTUNE_STATE["yaw"], _YAW_COEFF))
    seq.append((AUTOTUNE_STATE["apply"], _YAW_COEFF))

    n = len(seq)
    t = np.arange(n, dtype=np.int64) * dt_us
    data = {"timestamp": t}
    for i in range(5):
        data[f"coeff[{i}]"] = np.array([s[1][i] for s in seq], dtype=float)
        data[f"coeff_var[{i}]"] = np.full(n, 0.01)
    data["state"] = np.array([s[0] for s in seq], dtype=float)
    data["fitness"] = np.linspace(0.2, 0.95, n)
    data["dt_model"] = np.full(n, _MODEL_DT)
    for i in range(3):
        data[f"rate_sp[{i}]"] = np.zeros(n)
    return _ds("autotune_attitude_control_status", data)


# ── (1) parse_autotune_series ──────────────────────────────────────────────────

def test_parse_autotune_series_absent_returns_none():
    assert parse_autotune_series([_ds("vehicle_angular_velocity", {"timestamp": np.arange(3)})]) is None


def test_parse_autotune_series_extracts_per_axis_coeffs():
    series = parse_autotune_series([_autotune_status_ds()])
    assert series is not None
    assert series.has_coeff and series.has_fitness
    assert set(series.axes.keys()) == {"roll", "pitch", "yaw"}

    roll = series.axes["roll"]
    assert roll.coeff == [round(v, 6) for v in _ROLL_COEFF]
    assert roll.n_samples == 2               # two samples in STATE_ROLL
    assert roll.fitness is not None
    assert roll.model_dt_s == pytest.approx(_MODEL_DT)

    # Plant reconstruction: num=[b0,b1,b2], den=[1,a1,a2]
    m = roll.model
    assert m is not None
    assert m.num == [round(v, 6) for v in _ROLL_COEFF[2:]]
    assert m.den == [1.0, round(_ROLL_COEFF[0], 6), round(_ROLL_COEFF[1], 6)]
    # DC gain = Σb / (1+a1+a2)
    exp_dc = sum(_ROLL_COEFF[2:]) / (1.0 + _ROLL_COEFF[0] + _ROLL_COEFF[1])
    assert m.dc_gain == pytest.approx(exp_dc, rel=1e-3)


def test_parse_autotune_series_missing_axis_states():
    """A run that aborted before pitch/yaw → only roll converged."""
    ds = _autotune_status_ds()
    # Truncate the stream right after the roll identification.
    for k, v in ds.data.items():
        ds.data[k] = np.asarray(v)[:5]
    series = parse_autotune_series([ds])
    assert "roll" in series.axes
    assert "pitch" not in series.axes and "yaw" not in series.axes


# ── (2) reconstruct_arx_plant round-trip ───────────────────────────────────────

def test_reconstruct_arx_plant_recovers_continuous_equivalent():
    # Known 2nd-order continuous plant, sampled at dt: the AR denominator poles are
    # exp(s·dt). reconstruct_arx_plant must map them back to (wn, zeta).
    wn = 2.0 * math.pi * 8.0     # 8 Hz
    zeta = 0.5
    dt = 0.004
    s = np.array([
        -zeta * wn + 1j * wn * math.sqrt(1 - zeta**2),
        -zeta * wn - 1j * wn * math.sqrt(1 - zeta**2),
    ])
    z = np.exp(s * dt)
    den = np.real(np.poly(z))           # [1, a1, a2]
    a1, a2 = float(den[1]), float(den[2])
    coeff = [a1, a2, 0.01, 0.005, 0.001]

    model = reconstruct_arx_plant(coeff, dt)
    assert model is not None
    assert model.wn_hz == pytest.approx(8.0, rel=1e-2)
    assert model.zeta == pytest.approx(0.5, rel=2e-2)


def test_reconstruct_arx_plant_without_dt_has_no_continuous_equiv():
    model = reconstruct_arx_plant([-1.8, 0.85, 0.005, 0.008, 0.002], None)
    assert model is not None
    assert model.num == [0.005, 0.008, 0.002]
    assert model.wn_hz is None and model.zeta is None


def test_reconstruct_arx_plant_rejects_nonfinite():
    assert reconstruct_arx_plant([float("nan"), 0, 0, 0, 0], 0.004) is None
    assert reconstruct_arx_plant([1.0, 2.0], 0.004) is None


# ── (3) build_tuning_comparison ────────────────────────────────────────────────

def _current_params():
    p = {}
    for tag in ("ROLL", "PITCH", "YAW"):
        p[f"MC_{tag}RATE_K"] = 1.0
        p[f"MC_{tag}RATE_P"] = 0.15
        p[f"MC_{tag}RATE_I"] = 0.2
        p[f"MC_{tag}RATE_D"] = 0.003
    p["MC_ROLL_P"] = 6.5
    p["MC_PITCH_P"] = 6.5
    p["MC_YAW_P"] = 2.8
    return p


def _fake_plant_estimations():
    """Duck-typed PlantEstimation stand-ins with synthesis + a plant model."""
    out = []
    for axis in ("roll", "pitch", "yaw"):
        synth = types.SimpleNamespace(axis=axis, K=1.0, Kp=0.18, Ki=0.22, Kd=0.004)
        model = types.SimpleNamespace(order="integrator_pole", omega_n=2 * math.pi * 9,
                                      zeta=0.6, delay_s=0.008, fit_vaf=82.0)
        out.append(types.SimpleNamespace(axis=axis, synthesized=synth, plant_model=model))
    return out


def test_comparison_three_columns_when_all_present():
    series = parse_autotune_series([_autotune_status_ds()])
    report = build_tuning_comparison(
        params=_current_params(),
        autotune_series=series,
        plant_estimations=_fake_plant_estimations(),
    )
    assert len(report["axes"]) == 3
    roll = next(a for a in report["axes"] if a["axis"] == "roll")
    # All three gain columns populated for roll (roll coeffs pass the sanity gate).
    assert set(roll["available"]) == {"current", "px4_autotune", "analyzer"}
    assert roll["gains"]["current"]["MC_ROLLRATE_P"] == 0.15
    assert roll["gains"]["px4_autotune"]["MC_ROLLRATE_P"] is not None
    assert roll["gains"]["analyzer"]["MC_ROLLRATE_P"] == 0.18
    # Both models present with their own quality metric.
    assert roll["models"]["px4_autotune"]["type"] == "ARX(2,2,1)"
    assert roll["models"]["analyzer"]["fit_vaf"] == 82.0
    assert roll["quality"]["px4_fitness"] is not None
    assert report["sources"]["px4_autotune"] is True


def test_comparison_degrades_to_two_columns_without_autotune():
    report = build_tuning_comparison(
        params=_current_params(),
        autotune_series=None,               # no autotune run in the log
        plant_estimations=_fake_plant_estimations(),
    )
    assert len(report["axes"]) == 3
    for ax in report["axes"]:
        assert "px4_autotune" not in ax["available"]
        assert ax["available"] == ["current", "analyzer"]
        assert ax["gains"]["px4_autotune"] is None
        assert ax["models"]["px4_autotune"] is None
    assert report["sources"]["px4_autotune"] is False


def test_comparison_current_only_does_not_crash():
    report = build_tuning_comparison(params=_current_params())
    assert len(report["axes"]) == 3
    for ax in report["axes"]:
        assert ax["available"] == ["current"]
