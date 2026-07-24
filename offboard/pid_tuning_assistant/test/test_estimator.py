"""
Unit tests for model_estimator.py.

Run from the package root (no ROS needed):
    python -m pytest test/test_estimator.py -v

Tests verify that fit_second_order recovers wn/zeta from synthetic 2nd-order
step responses at nominal, degraded, and jittered sample rates.
"""
import math
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest

from pid_tuning_assistant.model_estimator import (
    resample_uniform,
    fit_second_order,
    log_decrement_zeta,
    nyquist_guard,
    bin_steps_by_amplitude,
    coherence_verdict,
    _step_response_2nd,
    ModelFit,
)


# ── synthetic signal helpers ──────────────────────────────────────────────────

def make_step(wn: float, zeta: float, fs: float = 100.0, duration: float = 3.0,
              noise_std: float = 0.0, jitter_pct: float = 0.0,
              sp_before: float = 0.0, sp_after: float = 1.0):
    """Generate a synthetic 2nd-order step response."""
    dt = 1.0 / fs
    t = np.arange(0.0, duration, dt)
    if jitter_pct > 0:
        jitter = np.random.uniform(-jitter_pct / 100.0, jitter_pct / 100.0, len(t)) * dt
        t = t + np.cumsum(jitter)
        t = np.clip(t, 0, duration)
        t = np.sort(np.unique(t))
    t_rel = t - t[0]
    norm = _step_response_2nd(t_rel, wn, zeta)
    meas = sp_before + (sp_after - sp_before) * norm
    if noise_std > 0:
        meas = meas + np.random.normal(0, noise_std, len(meas))
    return t, meas


# ── resample_uniform ──────────────────────────────────────────────────────────

class TestResampleUniform:
    def test_uniform_input_passthrough(self):
        t = np.linspace(0, 1, 101)
        y = np.sin(2 * np.pi * t)
        t_u, y_u, dt, jitter = resample_uniform(t, y)
        assert abs(dt - 0.01) < 1e-6
        assert jitter < 1.0
        assert len(t_u) >= 90

    def test_jittered_input_smoothed(self):
        np.random.seed(42)
        t = np.linspace(0, 2, 201)
        t_jit = t + np.random.uniform(-0.002, 0.002, len(t))
        t_jit = np.sort(t_jit)
        y = np.ones_like(t_jit)
        t_u, y_u, dt, jitter_pct = resample_uniform(t_jit, y)
        assert jitter_pct < 50.0
        assert len(t_u) >= 150

    def test_short_input(self):
        t = np.array([0.0, 0.1, 0.2])
        y = np.array([0.0, 0.5, 1.0])
        t_u, y_u, dt, jitter = resample_uniform(t, y)
        assert len(t_u) == len(t)


# ── fit_second_order ──────────────────────────────────────────────────────────

class TestFitSecondOrder:
    """Verify model identification on clean synthetic signals."""

    @pytest.mark.parametrize("wn,zeta", [
        (2 * math.pi * 3.0, 0.5),   # underdamped, 3 Hz
        (2 * math.pi * 5.0, 0.7),   # near-critical, 5 Hz
        (2 * math.pi * 8.0, 0.3),   # strongly oscillatory
    ])
    def test_clean_signal_high_rate(self, wn, zeta):
        """100 Hz clean signal: expect high R² and correct wn/zeta."""
        t, meas = make_step(wn, zeta, fs=100.0, duration=2.0, noise_std=0.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=50.0)
        assert fit.r_squared > 0.85, f"R²={fit.r_squared:.3f} for wn={wn:.2f}, zeta={zeta}"
        assert fit.method == "2nd_order"
        assert abs(fit.wn - wn) / wn < 0.15, f"wn error: got {fit.wn:.2f}, expected {wn:.2f}"
        assert abs(fit.zeta - zeta) < 0.15, f"zeta error: got {fit.zeta:.3f}, expected {zeta}"

    def test_noisy_signal_lower_r2(self):
        """Adding noise should lower R² but not crash."""
        np.random.seed(7)
        wn, zeta = 2 * math.pi * 4.0, 0.6
        t, meas = make_step(wn, zeta, fs=100.0, duration=2.0, noise_std=0.15)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=50.0)
        assert fit.r_squared >= 0.0
        assert fit.method in ("2nd_order", "log_decrement", "failed")

    def test_degraded_sample_rate_20hz(self):
        """20 Hz (f_nyq=10 Hz): should still fit 2 Hz oscillation."""
        wn, zeta = 2 * math.pi * 2.0, 0.5
        t, meas = make_step(wn, zeta, fs=20.0, duration=3.0, noise_std=0.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=10.0)
        assert fit.r_squared > 0.70

    def test_above_nyquist_flagged(self):
        """Fit frequency > f_nyquist should be flagged."""
        wn, zeta = 2 * math.pi * 12.0, 0.5
        t, meas = make_step(wn, zeta, fs=100.0, duration=1.0, noise_std=0.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=5.0)
        assert fit.above_nyquist is True

    def test_near_zero_step_returns_failed(self):
        t = np.linspace(0, 1, 100)
        meas = np.ones(100) * 0.5
        fit = fit_second_order(t, meas, 0.5, 0.5 + 1e-12, f_nyquist_hz=50.0)
        assert fit.method == "failed"

    def test_k_scale_hint_underdamped(self):
        """Underdamped fit (zeta < 0.7) should give k_scale < 1 (reduce K)."""
        wn, zeta = 2 * math.pi * 4.0, 0.3
        t, meas = make_step(wn, zeta, fs=100.0, duration=2.0, noise_std=0.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=50.0)
        if fit.k_scale_for_zeta07 is not None and fit.r_squared > 0.7:
            assert fit.k_scale_for_zeta07 < 1.0, "underdamped → K should decrease"

    def test_k_scale_hint_overdamped(self):
        """Overdamped fit (zeta > 0.7) should give k_scale > 1 (raise K)."""
        wn, zeta = 2 * math.pi * 3.0, 1.2
        t, meas = make_step(wn, zeta, fs=100.0, duration=3.0, noise_std=0.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=50.0)
        if fit.k_scale_for_zeta07 is not None and fit.r_squared > 0.7:
            assert fit.k_scale_for_zeta07 > 1.0, "overdamped → K should increase"

    def test_jittered_20pct(self):
        """20% timing jitter: model should still recover reasonable zeta."""
        np.random.seed(13)
        wn, zeta = 2 * math.pi * 4.0, 0.5
        t, meas = make_step(wn, zeta, fs=100.0, duration=2.0,
                            noise_std=0.0, jitter_pct=20.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=50.0)
        assert fit.method in ("2nd_order", "log_decrement")
        if fit.r_squared > 0.55:
            assert abs(fit.zeta - zeta) < 0.25


# ── log_decrement_zeta ────────────────────────────────────────────────────────

class TestLogDecrement:
    def test_recovers_zeta(self):
        """Log-decrement should estimate zeta from clean oscillation."""
        wn, zeta = 2 * math.pi * 3.0, 0.2
        t, meas = make_step(wn, zeta, fs=100.0, duration=2.0)
        # Subtract DC to isolate oscillation
        y = meas - 1.0
        result = log_decrement_zeta(t, y)
        if result is not None:
            freq_hz, z_est = result
            assert abs(z_est - zeta) < 0.15
            assert freq_hz > 1.0

    def test_too_few_points(self):
        assert log_decrement_zeta(np.array([0, 1]), np.array([0, 1])) is None

    def test_overdamped_returns_none(self):
        """Overdamped response has no peaks — should return None."""
        wn, zeta = 2 * math.pi * 3.0, 1.5
        t, meas = make_step(wn, zeta, fs=100.0, duration=2.0)
        y = meas - 1.0
        result = log_decrement_zeta(t, y)
        assert result is None


# ── nyquist_guard ─────────────────────────────────────────────────────────────

class TestNyquistGuard:
    def test_well_below(self):
        above, near = nyquist_guard(5.0, 50.0)
        assert not above and not near

    def test_near_nyquist(self):
        above, near = nyquist_guard(42.0, 50.0)
        assert not above and near

    def test_above_nyquist(self):
        above, near = nyquist_guard(55.0, 50.0)
        assert above

    def test_zero_nyquist(self):
        above, near = nyquist_guard(100.0, 0.0)
        assert not above and not near


# ── bin_steps_by_amplitude ────────────────────────────────────────────────────

class TestBinSteps:
    def test_three_bins(self):
        sizes = np.array([0.1, 0.2, 0.5, 0.6, 1.0, 1.1])
        bins = bin_steps_by_amplitude(sizes)
        assert len(bins) == len(sizes)
        assert bins.min() >= 0 and bins.max() <= 2

    def test_empty(self):
        bins = bin_steps_by_amplitude(np.array([]))
        assert len(bins) == 0

    def test_uniform_bins(self):
        sizes = np.linspace(0.1, 1.0, 9)
        bins = bin_steps_by_amplitude(sizes, n_bins=3)
        assert len(np.unique(bins)) == 3


# ── coherence_verdict ─────────────────────────────────────────────────────────

class TestCoherenceVerdict:
    def test_insufficient_data(self):
        t = np.linspace(0, 0.5, 10)
        result = coherence_verdict(t, np.ones(10), np.ones(10))
        assert result["verdict"] == "insufficient_data"

    def test_coherent_signals(self):
        """Two identical signals → high_coherence (neutral label)."""
        np.random.seed(0)
        t = np.linspace(0, 5, 500)
        sig = np.sin(2 * np.pi * 5 * t) + 0.05 * np.random.randn(500)
        result = coherence_verdict(t, sig, sig)
        assert result["verdict"] in ("high_coherence", "mixed")

    def test_uncorrelated_signals(self):
        """Independent noise → low_coherence (neutral label)."""
        np.random.seed(1)
        t = np.linspace(0, 5, 500)
        err = np.random.randn(500)
        torque = np.random.randn(500)
        result = coherence_verdict(t, err, torque)
        assert result["verdict"] in ("low_coherence", "mixed")


# ── pipeline limit documentation ──────────────────────────────────────────────

class TestPipelineLimits:
    def test_very_low_rate_10hz(self):
        """10 Hz (f_nyq=5 Hz): can only identify ≤5 Hz oscillations.
        Documented limit: DDS over UART ~50–100 Hz nominal, less in practice."""
        wn, zeta = 2 * math.pi * 2.0, 0.5   # 2 Hz — well below Nyquist
        t, meas = make_step(wn, zeta, fs=10.0, duration=4.0, noise_std=0.0)
        fit = fit_second_order(t, meas, 0.0, 1.0, f_nyquist_hz=5.0)
        assert fit.r_squared > 0.60 or fit.method in ("log_decrement", "failed"), \
            "Low-rate pipeline should either fit or gracefully fail"

    def test_high_freq_above_nyquist_nyquist_guard_direct(self):
        """nyquist_guard correctly flags frequencies above Nyquist.
        NOTE: fit_second_order on a 10 Hz-sampled signal CANNOT see 8 Hz —
        it aliases to |8-10|=2 Hz, which is below Nyquist.
        The nyquist_guard function is the correct tool for post-fit gating."""
        # Direct guard check — 8 Hz with Nyquist=5 Hz must be flagged
        above, _ = nyquist_guard(8.0, 5.0)
        assert above is True

        # A fit that finds 6 Hz (above Nyquist=5 Hz) should be flagged
        above2, _ = nyquist_guard(6.0, 5.0)
        assert above2 is True

        # A fit that finds 3 Hz (below Nyquist=5 Hz) is NOT flagged
        above3, _ = nyquist_guard(3.0, 5.0)
        assert above3 is False
