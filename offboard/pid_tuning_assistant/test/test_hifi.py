"""
Unit tests for hifi_analyzer.py and plant_estimator.py.

Run from the package root (no ROS needed):
    python -m pytest test/test_hifi.py -v

All tests use synthetic signals so scipy/numpy are the only hard deps.  The
results of both modules are dataclasses (BodePoint, FRFResult, StabilityMargins,
LoopDelay, ARXModel, ExcitationInfo, ActuatorSaturation, PlantFRF, PlantModel);
tests access their fields as attributes, never as dict keys.

Two groups of tests live here:
  * Task 1 — exercise the *current* API with synthetic signals that guarantee a
    result, so assertions are unconditional.
  * Task 2 — strict-xfail tests that encode the *correct* behaviour targeted by
-24.  They MUST fail against the current code and are un-xfailed by
    the respective fix prompt.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import math
import numpy as np
import pytest

from pid_tuning_assistant.hifi_analyzer import (
    compute_frf,
    stability_margins_from_frf,
    estimate_loop_delay,
    arx_fit,
    classify_excitation,
    detect_actuator_saturation,
    vibration_analysis,
    VibrationResult,
    BodePoint,
    FRFResult,
    StabilityMargins,
    LoopDelay,
    ARXModel,
    ExcitationInfo,
    ActuatorSaturation,
    MIN_ARX_GOOD_VAF,
)
from pid_tuning_assistant.plant_estimator import (
    px4_pid_tf,
    _second_order_tf,
    _step_response_sim,
    _step_metrics,
    fit_plant_model,
    recover_plant_frf,
    synthesize_pid_from_plant,
    synthesize_simc,
    PlantFRF,
    PlantModel,
    SimcGains,
    GATE_KP_MAX,
    GATE_KI_MAX,
    GATE_KD_MAX,
)

from hifi_helpers import (
    white_noise_loop_data,
    analytic_T_2nd_order,
    analytic_pm_2nd_order_deg,
    px4_controller_frf,
    frf_from_complex,
)


# ── signal helpers ────────────────────────────────────────────────────────────
FS_HIGH = 500.0   # Hz — typical high-rate ULog gyro rate
FS_MID = 250.0    # Hz — comfortably above MIN_RATE_FOR_FRF_HZ (100)


def _make_time(duration_s: float, fs: float) -> np.ndarray:
    return np.arange(0, duration_s, 1.0 / fs)


def _chirp(t: np.ndarray, f0: float, f1: float, amp: float = 1.0) -> np.ndarray:
    """Linear-sweep chirp from f0 to f1 over t[-1] seconds."""
    rate = (f1 - f0) / t[-1]
    phi = 2 * math.pi * (f0 * t + 0.5 * rate * t ** 2)
    return amp * np.sin(phi)


def _slow_steps(t: np.ndarray) -> np.ndarray:
    """Low-bandwidth step-like setpoint (a slow 0.3 Hz square wave)."""
    return np.sign(np.sin(2 * math.pi * 0.3 * t)) * 0.5 + 0.5


# ═══════════════════════════════════════════════════════════════════════════════
# compute_frf
# ═══════════════════════════════════════════════════════════════════════════════

class TestComputeFRF:
    """FRF via H = Pyx/Pxx — pass-through and gain tests."""

    def test_identity_plant_frf_near_one(self):
        """If measured == setpoint, FRF magnitude should be ~1 (0 dB) everywhere."""
        t, sp, meas = white_noise_loop_data(FS_MID, 25.0, delay_ms=0.0, seed=1)
        result = compute_frf(t, sp, meas, FS_MID, axis="roll", loop="rate",
                             f_nyquist_hz=FS_MID / 2)
        assert isinstance(result, FRFResult)
        valid_mags = [b.mag_db for b in result.bode if b.valid]
        assert len(valid_mags) > 5
        for m in valid_mags:
            assert abs(m) < 3.0, f"Expected |H|~1 (0 dB), got {m:.2f} dB"

    def test_known_gain_plant(self):
        """Plant with known static gain G=2: FRF magnitude should be ~6 dB."""
        t, sp, _ = white_noise_loop_data(FS_MID, 25.0, delay_ms=0.0, seed=2)
        meas = 2.0 * sp  # pure gain
        result = compute_frf(t, sp, meas, FS_MID, axis="roll", loop="rate",
                             f_nyquist_hz=FS_MID / 2)
        valid_mags = [b.mag_db for b in result.bode if b.valid]
        assert len(valid_mags) >= 5
        for m in valid_mags:
            assert abs(m - 6.02) < 2.0, f"Expected ~6 dB gain, got {m:.2f} dB"

    def test_frf_returns_frf_result(self):
        t, sp, _ = white_noise_loop_data(FS_MID, 22.0, delay_ms=0.0, seed=3)
        rng = np.random.default_rng(0)
        meas = sp + 0.01 * rng.standard_normal(len(sp))
        result = compute_frf(t, sp, meas, FS_MID, axis="pitch", loop="rate",
                             f_nyquist_hz=FS_MID / 2)
        assert isinstance(result, FRFResult)
        assert result.axis == "pitch"
        assert result.loop == "rate"
        assert len(result.bode) > 0
        assert all(isinstance(b, BodePoint) for b in result.bode)

    def test_below_nyquist_not_reported_above(self):
        """No bode point should have freq_hz above the Nyquist frequency."""
        t, sp, meas = white_noise_loop_data(FS_MID, 22.0, delay_ms=0.0, seed=4)
        result = compute_frf(t, sp, meas, FS_MID, axis="yaw", loop="rate",
                             f_nyquist_hz=FS_MID / 2)
        for b in result.bode:
            assert b.freq_hz <= FS_MID / 2 + 0.5


# ═══════════════════════════════════════════════════════════════════════════════
# stability_margins_from_frf
# ═══════════════════════════════════════════════════════════════════════════════

class TestStabilityMargins:
    """Gain margin / phase margin from a synthetic closed-loop FRF."""

    def _stable_frf(self, wn_hz: float = 5.0, zeta: float = 0.7) -> FRFResult:
        freqs = np.geomspace(0.5, 60.0, 120)
        T = analytic_T_2nd_order(freqs, wn_hz, zeta)
        return frf_from_complex(freqs, T, axis="roll", loop="rate",
                                f_nyquist_hz=100.0, confidence="high")

    def test_margins_return_object_with_fields(self):
        m = stability_margins_from_frf(self._stable_frf(), axis="roll")
        assert isinstance(m, StabilityMargins)
        assert hasattr(m, "gain_margin_db")
        assert hasattr(m, "phase_margin_deg")
        assert hasattr(m, "confidence")

    def test_stable_system_positive_pm(self):
        """A well-damped 2nd-order closed loop must have a positive phase margin."""
        m = stability_margins_from_frf(self._stable_frf(wn_hz=5.0, zeta=0.7),
                                       axis="roll")
        assert m.phase_margin_deg is not None
        assert m.phase_margin_deg > 0, (
            f"Expected PM > 0, got {m.phase_margin_deg:.1f} deg")


# ═══════════════════════════════════════════════════════════════════════════════
# estimate_loop_delay
# ═══════════════════════════════════════════════════════════════════════════════

class TestLoopDelay:
    def test_known_delay_recovered(self):
        """Cross-correlation should recover a pure 10 ms transport delay."""
        t, sp, meas = white_noise_loop_data(FS_HIGH, 8.0, delay_ms=10.0, seed=5)
        result = estimate_loop_delay(t, sp, meas, FS_HIGH, axis="roll", frf=None)
        assert isinstance(result, LoopDelay)
        assert abs(result.delay_ms - 10.0) < 3.0, (
            f"Expected ~10 ms, got {result.delay_ms:.2f} ms")

    def test_zero_delay_near_zero(self):
        """No delay between identical signals → delay ~ 0."""
        t, sp, meas = white_noise_loop_data(FS_HIGH, 8.0, delay_ms=0.0, seed=6)
        r = estimate_loop_delay(t, sp, meas, FS_HIGH, axis="pitch", frf=None)
        assert abs(r.delay_ms) < 3.0, f"Expected ~0 ms, got {r.delay_ms:.2f} ms"


# ═══════════════════════════════════════════════════════════════════════════════
# arx_fit
# ═══════════════════════════════════════════════════════════════════════════════

class TestARXFit:
    def _arx_data(self, a1: float = 0.6, b1: float = 0.8, n: int = 4000):
        """Generate ARX(1,1) data: y[k] = a1*y[k-1] + b1*u[k-1] + e[k]."""
        rng = np.random.default_rng(42)
        u = rng.standard_normal(n)
        y = np.zeros(n)
        for k in range(1, n):
            y[k] = a1 * y[k - 1] + b1 * u[k - 1] + 0.01 * rng.standard_normal()
        t = np.arange(n) / FS_MID
        return t, u, y

    def test_arx_returns_vaf_and_coeffs(self):
        t, u, y = self._arx_data()
        result = arx_fit(t, u, y, axis="roll")
        assert isinstance(result, ARXModel)
        assert isinstance(result.vaf_pct, float)
        assert len(result.coeffs_a) == result.na
        assert len(result.coeffs_b) == result.nb

    def test_arx_vaf_high_for_clean_data(self):
        """A genuine ARX process with informative input fits well."""
        t, u, y = self._arx_data()
        result = arx_fit(t, u, y, axis="roll")
        assert result.vaf_pct > 80.0, f"Expected VAF > 80%, got {result.vaf_pct:.1f}%"

    def test_arx_vaf_low_for_random_data(self):
        """Uncorrelated white-noise in/out → the model explains nothing."""
        rng = np.random.default_rng(99)
        t = np.arange(4000) / FS_MID
        u = rng.standard_normal(4000)
        y = rng.standard_normal(4000)
        result = arx_fit(t, u, y, axis="pitch")
        assert result.vaf_pct < 50.0, f"Expected VAF < 50%, got {result.vaf_pct:.1f}%"


# ═══════════════════════════════════════════════════════════════════════════════
# classify_excitation
# ═══════════════════════════════════════════════════════════════════════════════

class TestClassifyExcitation:
    def test_chirp_classified_as_broadband(self):
        t = _make_time(20.0, FS_MID)
        sp = _chirp(t, 1.0, 40.0)
        res = classify_excitation(t, sp, FS_MID)
        assert isinstance(res, ExcitationInfo)
        assert res.excitation_type in ("sweep", "mixed"), (
            f"Chirp should be broadband, got '{res.excitation_type}'")

    def test_slow_steps_classified_as_step(self):
        t = _make_time(20.0, FS_MID)
        sp = _slow_steps(t)
        res = classify_excitation(t, sp, FS_MID)
        assert res.excitation_type == "step", (
            f"Slow steps should be 'step', got '{res.excitation_type}'")

    def test_returns_quality_field(self):
        t = _make_time(20.0, FS_MID)
        sp = _chirp(t, 1.0, 40.0)
        res = classify_excitation(t, sp, FS_MID)
        assert res.quality in ("good", "marginal", "poor")


# ═══════════════════════════════════════════════════════════════════════════════
# detect_actuator_saturation
# ═══════════════════════════════════════════════════════════════════════════════

class TestActuatorSaturation:
    def test_unsaturated_motors(self):
        t = np.arange(500) / FS_MID
        motors = np.full((500, 4), 0.5)  # 50% — well within range
        res = detect_actuator_saturation(t, motors)
        assert isinstance(res, ActuatorSaturation)
        assert res.sat_fraction_pct < 5.0, (
            f"Expected <5% sat, got {res.sat_fraction_pct:.1f}%")

    def test_fully_saturated_motors(self):
        t = np.arange(500) / FS_MID
        motors = np.ones((500, 4))  # always at max
        res = detect_actuator_saturation(t, motors)
        assert res.sat_fraction_pct > 90.0, (
            f"Expected >90% sat, got {res.sat_fraction_pct:.1f}%")

    def test_partial_saturation(self):
        t = np.arange(500) / FS_MID
        motors = np.full((500, 4), 0.5)
        motors[100:200, :] = 1.0  # 100 / 500 = 20%
        res = detect_actuator_saturation(t, motors)
        assert 15.0 < res.sat_fraction_pct < 25.0, (
            f"Expected ~20% sat, got {res.sat_fraction_pct:.1f}%")


# ═══════════════════════════════════════════════════════════════════════════════
# plant_estimator — px4_pid_tf
# ═══════════════════════════════════════════════════════════════════════════════

class TestPx4PidTF:
    def test_returns_complex_array(self):
        f = np.array([1.0, 5.0, 10.0, 20.0])
        C = px4_pid_tf(f, K=1.0, Kp=0.5, Ki=0.1, Kd=0.05, d_filter_hz=30.0)
        assert C.shape == (4,)
        assert np.iscomplexobj(C)

    def test_low_frequency_dominated_by_integrator(self):
        """At very low freq the integrator term Ki/(jw) dominates → large |C|."""
        f_low = np.array([0.01])
        C = px4_pid_tf(f_low, K=1.0, Kp=0.0, Ki=1.0, Kd=0.0, d_filter_hz=100.0)
        assert abs(C[0]) > 10.0, f"Expected |C| >> 1 at low freq, got {abs(C[0]):.3f}"

    def test_d_filter_bounds_high_frequency_gain(self):
        """PX4 filters the D term with a 2nd-order Butterworth (IMU_DGYRO_CUTOFF),
        so far above the cutoff |C_d| must roll OFF (~1/f), not plateau and not
        keep rising like an ideal differentiator |Kd*jw|."""
        cutoff = 10.0  # Hz
        Kd = 1.0
        C_100 = abs(px4_pid_tf(np.array([100.0]), 1.0, 0.0, 0.0, Kd, cutoff)[0])
        C_200 = abs(px4_pid_tf(np.array([200.0]), 1.0, 0.0, 0.0, Kd, cutoff)[0])
        ideal_200 = Kd * 2 * math.pi * 200.0
        # filtered gain << ideal differentiator
        assert C_200 < 0.2 * ideal_200, (
            f"D-filter did not roll off: |C|@200Hz={C_200:.1f}, ideal={ideal_200:.1f}")
        # 2nd-order Butterworth on jw → net −1 slope: doubling f roughly halves |C_d|
        assert C_200 < 0.7 * C_100, (
            f"D gain did not roll off with frequency: |C|@100={C_100:.1f}, "
            f"|C|@200={C_200:.1f}")


# ═══════════════════════════════════════════════════════════════════════════════
# plant_estimator — fit_plant_model
# ═══════════════════════════════════════════════════════════════════════════════

class TestFitPlantModel:
    def _plant_frf(self, wn_hz: float = 15.0, zeta: float = 0.5,
                   K_p: float = 1.0, delay_s: float = 0.006) -> PlantFRF:
        """Build a PlantFRF from a known 2nd-order + delay plant TF."""
        freqs = np.geomspace(1.0, 50.0, 80)
        G = _second_order_tf(freqs, K_p, 2 * math.pi * wn_hz, zeta, delay_s)
        return PlantFRF(
            f=[float(v) for v in freqs],
            G_mag_db=[float(20 * math.log10(abs(g) + 1e-12)) for g in G],
            G_phase_deg=[float(math.degrees(np.angle(g))) for g in G],
            coherence=[1.0] * len(freqs),
            confidence="high",
        )

    def test_fit_recovers_reasonable_wn(self):
        wn_hz = 15.0
        model = fit_plant_model(self._plant_frf(wn_hz=wn_hz, zeta=0.5))
        assert isinstance(model, PlantModel)
        wn_true = 2 * math.pi * wn_hz
        assert abs(model.omega_n - wn_true) < wn_true * 0.4, (
            f"Expected wn~{wn_true:.1f} rad/s, got {model.omega_n:.2f}")

    def test_fit_returns_model(self):
        model = fit_plant_model(self._plant_frf())
        assert isinstance(model, PlantModel)
        assert hasattr(model, "omega_n")
        assert hasattr(model, "fit_vaf")

    def test_fit_on_noisy_plant_frf(self):
        rng = np.random.default_rng(7)
        pf = self._plant_frf(wn_hz=20.0, zeta=0.6)
        pf.G_mag_db = [v + float(rng.normal(scale=3.0)) for v in pf.G_mag_db]
        pf.G_phase_deg = [v + float(rng.normal(scale=10.0)) for v in pf.G_phase_deg]
        model = fit_plant_model(pf)  # must not raise
        assert isinstance(model, PlantModel)
        assert hasattr(model, "fit_vaf")


# ═══════════════════════════════════════════════════════════════════════════════
# Task 2 — strict-xfail correctness suite (fixed by-24)
# ═══════════════════════════════════════════════════════════════════════════════

def test_frf_phase_sign_pure_delay():
    """
    A pure transport delay must give a *negative* phase slope
    (phase ~ -360 * f * tau).  The current compute_frf uses csd(meas, sp),
    which yields conj(H) and hence a positive slope.
    """
    fs = 500.0
    tau_s = 0.020
    t, sp, meas = white_noise_loop_data(fs, 30.0, delay_ms=tau_s * 1e3, seed=11)
    frf = compute_frf(t, sp, meas, fs, axis="roll", loop="rate",
                      f_nyquist_hz=fs / 2, nperseg=1024)
    assert frf is not None
    pts = [b for b in frf.bode if b.valid and 20.0 <= b.freq_hz <= 80.0
           and b.coherence >= 0.9]
    assert len(pts) >= 5
    f = np.array([b.freq_hz for b in pts])
    ph = np.array([b.phase_deg for b in pts])
    slope = float(np.polyfit(f, ph, 1)[0])   # deg / Hz
    expected = -360.0 * tau_s               # ~ -7.2 deg/Hz
    assert slope < 0.0, f"phase slope should be negative, got {slope:+.2f} deg/Hz"
    assert abs(slope - expected) < 2.0, (
        f"expected ~{expected:.1f} deg/Hz, got {slope:+.2f}")


def test_margins_analytic_2nd_order():
    """
    For L = wn^2/(jw(jw+2 zeta wn)) at zeta=0.5 the phase margin is 51.8 deg.
    On a coarse FRF grid whose gain crossover falls between two widely spaced
    bins, the current code reports the phase of the *left* bin instead of
    interpolating at the crossover, biasing PM well beyond the +/-5 deg band.
    """
    zeta = 0.5
    freqs = np.arange(0.5, 60.0, 2.0)   # coarse, straddles the ~3.9 Hz crossover
    T = analytic_T_2nd_order(freqs, wn_hz=5.0, zeta=zeta)
    frf = frf_from_complex(freqs, T, axis="roll", f_nyquist_hz=100.0,
                           confidence="high")
    m = stability_margins_from_frf(frf, axis="roll")
    pm_true = analytic_pm_2nd_order_deg(zeta)   # 51.83 deg
    assert m.phase_margin_deg is not None
    assert abs(m.phase_margin_deg - pm_true) < 5.0, (
        f"expected PM~{pm_true:.1f} deg, got {m.phase_margin_deg:.1f} deg")


def test_plant_recovery_known_G_2dof():
    """
    Plant G(s) = 80/(s(0.025 s + 1)) behind a 2-DOF PX4 controller (D on the
    measurement).  Recovering G from the closed-loop T requires the 2-DOF
    formula G = T/(C_pi(1-T) - T*C_d); the single-DOF G = T/(C(1-T)) is wrong
    when D != 0, biasing |G| by many dB in the 1-20 Hz band.
    """
    freqs = np.geomspace(0.5, 50.0, 120)
    jw = 1j * 2.0 * np.pi * freqs
    G_true = 80.0 / (jw * (0.025 * jw + 1.0))
    C_pi, C_d = px4_controller_frf(K=1.0, P=0.15, I=0.2, D=0.003,
                                   f=freqs, d_cut_hz=30.0)
    T = G_true * C_pi / (1.0 + G_true * (C_pi + C_d))
    frf = frf_from_complex(freqs, T, axis="roll", f_nyquist_hz=100.0,
                           confidence="high")
    params = {
        "MC_ROLLRATE_K": 1.0, "MC_ROLLRATE_P": 0.15,
        "MC_ROLLRATE_I": 0.2, "MC_ROLLRATE_D": 0.003,
        "IMU_DGYRO_CUTOFF": 30.0,
    }
    plant_frf, _ = recover_plant_frf(frf, params, axis="roll")
    assert plant_frf is not None
    f_rec = np.array(plant_frf.f)
    g_rec = np.array(plant_frf.G_mag_db)
    jwr = 1j * 2.0 * np.pi * f_rec
    g_true_db = 20.0 * np.log10(np.abs(80.0 / (jwr * (0.025 * jwr + 1.0))) + 1e-12)
    band = (f_rec >= 1.0) & (f_rec <= 20.0)
    max_err = float(np.max(np.abs(g_rec[band] - g_true_db[band])))
    assert max_err < 2.0, f"recovered |G| off by {max_err:.2f} dB in 1-20 Hz"


def test_margins_2dof_known_GC_D_nonzero():
    """
    Margins from the true open loop L = G·(C_pi + C_d) for a known plant behind a
    2-DOF PX4 controller with D != 0.  The single-DOF L = T/(1-T) misreads the
    phase margin here; the 2-DOF path (fed the gains via params) must land within
    ±5° of the analytic PM computed directly from L.
    """
    freqs = np.geomspace(0.5, 60.0, 200)
    jw = 1j * 2.0 * np.pi * freqs
    G = 80.0 / (jw * (0.025 * jw + 1.0))
    C_pi, C_d = px4_controller_frf(K=1.0, P=0.15, I=0.2, D=0.02,
                                   f=freqs, d_cut_hz=30.0)
    T = G * C_pi / (1.0 + G * (C_pi + C_d))
    frf = frf_from_complex(freqs, T, axis="roll", f_nyquist_hz=100.0,
                           confidence="high")
    params = {
        "MC_ROLLRATE_K": 1.0, "MC_ROLLRATE_P": 0.15,
        "MC_ROLLRATE_I": 0.2, "MC_ROLLRATE_D": 0.02,
        "IMU_DGYRO_CUTOFF": 30.0,
    }

    # Analytic phase margin straight from L = G·(C_pi + C_d).
    L = G * (C_pi + C_d)
    L_mag = np.abs(L)
    L_ph = np.degrees(np.unwrap(np.angle(L)))
    xi = np.where(np.diff(np.sign(L_mag - 1.0)))[0]
    assert len(xi) >= 1
    i = xi[0]
    frac = (1.0 - L_mag[i]) / (L_mag[i + 1] - L_mag[i])
    ph_gc = L_ph[i] + frac * (L_ph[i + 1] - L_ph[i])
    pm_true = ((ph_gc + 180.0) + 180.0) % 360.0 - 180.0

    m = stability_margins_from_frf(frf, axis="roll", params=params)
    assert m.phase_margin_deg is not None
    assert abs(m.phase_margin_deg - pm_true) < 5.0, (
        f"2-DOF PM expected ~{pm_true:.1f} deg, got {m.phase_margin_deg:.1f} deg")

    # And the single-DOF path (no gains) must disagree — proving the 2-DOF
    # correction is doing real work for this D != 0 system.
    m_1dof = stability_margins_from_frf(frf, axis="roll", params=None)
    assert m_1dof.phase_margin_deg is not None
    assert abs(m_1dof.phase_margin_deg - pm_true) > 3.0, (
        "single-DOF PM unexpectedly matched the 2-DOF truth")


def test_simulator_no_derivative_kick():
    """
    PX4 differentiates the *measurement*, so on the first sample after a step
    (measurement still zero) the D term is inert and the response is
    independent of Kd.  The current simulator differentiates the error, which
    injects a derivative kick — making the first post-step sample depend on Kd.
    """
    t_sim = np.linspace(0, 0.5, 1000)
    common = dict(plant_K=1.0, omega_n=2 * math.pi * 10.0, zeta=0.7,
                  delay_s=0.0, pid_K=1.0, pid_Kp=0.15, pid_Ki=0.2,
                  t_sim=t_sim, d_filter_hz=30.0)
    y_nod = _step_response_sim(pid_Kd=0.0, **common)
    y_d = _step_response_sim(pid_Kd=0.05, **common)
    kick = abs(y_d[1] - y_nod[1])
    tol = 0.5 * abs(y_nod[1])   # D must not dominate the P-driven first sample
    assert kick <= tol, (
        f"derivative kick on first sample: |dy|={kick:.2e} > {tol:.2e}")


def _analytic_step_overshoot_pct(plant_K, wn_hz, zeta, K, Kp, Ki, Kd,
                                 d_cut_hz=30.0, t_end=0.5):
    """
    Overshoot (%) of the exact linear PX4 2-DOF closed loop T = G·C_pi/(1+G·(C_pi+C_d))
    via scipy.signal.step, for the same G/gains the simulator integrates (delay=0).
    """
    from scipy import signal as sig
    wn = 2.0 * math.pi * wn_hz
    wd = 2.0 * math.pi * d_cut_hz
    Gn = np.array([plant_K * wn ** 2]);         Gd = np.array([1.0, 2 * zeta * wn, wn ** 2])
    Cpin = np.array([K * Kp, K * Ki]);          Cpid = np.array([1.0, 0.0])
    Cdn = np.array([K * Kd * wd ** 2, 0.0]);    Cdd = np.array([1.0, math.sqrt(2) * wd, wd ** 2])
    # T = G·C_pi·(1+s/wd…) / (Gd·Cpid·Cdd + Gn·(Cpin·Cdd + Cdn·Cpid))
    Tn = np.polymul(np.polymul(Gn, Cpin), Cdd)
    Td = np.polyadd(np.polymul(Gd, np.polymul(Cpid, Cdd)),
                    np.polymul(Gn, np.polyadd(np.polymul(Cpin, Cdd),
                                              np.polymul(Cdn, Cpid))))
    _, y = sig.step(sig.TransferFunction(Tn, Td), T=np.linspace(0, t_end, 8000))
    return max(0.0, (float(np.max(y)) / float(y[-1]) - 1.0) * 100.0)


def test_simulator_matches_analytic_step():
    """
    On a stable loop that stays in the linear (unsaturated) regime, the PX4-structure
    simulator's overshoot must track the exact closed-loop step response of the same
    2-DOF system (scipy.signal.step of T) within ±5 percentage points.
    """
    plant_K, wn_hz, zeta = 4.0, 6.0, 0.7
    K, Kp, Ki, Kd = 1.0, 0.4, 8.0, 0.01     # u0 = K·Kp = 0.4 < 1 → no saturation
    os_analytic = _analytic_step_overshoot_pct(plant_K, wn_hz, zeta, K, Kp, Ki, Kd)

    t_sim = np.linspace(0, 0.5, 2000)
    y = _step_response_sim(plant_K, 2 * math.pi * wn_hz, zeta, 0.0,
                           K, Kp, Ki, Kd, t_sim, d_filter_hz=30.0)
    m = _step_metrics(t_sim, y, r_after=1.0)
    assert m.get("unstable") is False
    assert abs(y[-1] - 1.0) < 0.05, f"loop did not track to r=1 (y[-1]={y[-1]:.3f})"
    assert abs(m["overshoot_pct"] - os_analytic) <= 5.0, (
        f"sim overshoot {m['overshoot_pct']:.1f}% vs analytic {os_analytic:.1f}% "
        "(> 5 pp apart)")


def test_simulator_flags_divergence_and_synth_rejects():
    """
    A diverging closed loop must be flagged ``unstable=True`` (so it is never sold
    as a good tuning), and the synthesizer must never emit runaway gains: with K
    held fixed, every suggested gain passes the PX4 areGainsGood gate.

    Note: the PX4 structure clips torque to ±1 with anti-windup, so a mere loop-gain
    increase can never make a bounded plant's output diverge (higher gain just
    saturates sooner) — divergence here is an explicit-Euler blow-up on a fast,
    lightly-damped plant the integrator cannot track, which the guard catches all
    the same.
    """
    # Fast, very underdamped plant on a coarse grid → explicit-Euler divergence.
    t_sim = np.linspace(0, 0.5, 300)
    y = _step_response_sim(plant_K=1.0, omega_n=2 * math.pi * 120.0, zeta=0.05,
                           delay_s=0.0, pid_K=1.0, pid_Kp=0.15, pid_Ki=8.0,
                           pid_Kd=0.01, t_sim=t_sim, d_filter_hz=30.0)
    m = _step_metrics(t_sim, y, r_after=1.0)
    assert m.get("unstable") is True, "diverging response was not flagged unstable"
    assert m.get("overshoot_pct") is None

    # Synthesis on a healthy integrator+pole plant → gate-passing, K held fixed.
    plant = PlantModel(K_plant=0.05, omega_n=2 * math.pi * 20, zeta=1.0,
                       delay_s=0.005, fit_vaf=90.0, order="integrator_pole",
                       confidence="high", int_gain=6.0, motor_pole=2 * math.pi * 20)
    params = {"MC_ROLLRATE_K": 1.0, "MC_ROLLRATE_P": 0.15, "MC_ROLLRATE_I": 0.2,
              "MC_ROLLRATE_D": 0.003, "IMU_DGYRO_CUTOFF": 30.0}
    g = synthesize_pid_from_plant(plant, "roll", params)
    assert g is not None
    assert g.K == params["MC_ROLLRATE_K"], "K must stay fixed to the current value"
    assert g.Kp > 0 and g.Ki >= 0 and g.Kd >= 0
    assert g.K * g.Kp < GATE_KP_MAX
    assert g.K * g.Ki < GATE_KI_MAX
    assert g.K * g.Kd < GATE_KD_MAX
    # After-response must be simulated and stable.
    assert g.after_response is not None
    assert g.after_response["metrics"].get("unstable") is False


def test_synthesis_degeneracy_free_in_effective_gains():
    """
    The (K,Kp,Ki,Kd) → (λK, Kp/λ, …) degeneracy is removed by holding K fixed.
    Two synthesis runs whose current gains describe the *same* effective loop
    (same K·Kp, K·Ki, K·Kd) but with different K representations must land on the
    same effective products within ±5 %.
    """
    plant = PlantModel(K_plant=0.05, omega_n=2 * math.pi * 20, zeta=1.0,
                       delay_s=0.005, fit_vaf=90.0, order="integrator_pole",
                       confidence="high", int_gain=6.0, motor_pole=2 * math.pi * 20)
    pA = {"MC_ROLLRATE_K": 1.0, "MC_ROLLRATE_P": 0.20, "MC_ROLLRATE_I": 0.10,
          "MC_ROLLRATE_D": 0.004, "IMU_DGYRO_CUTOFF": 30.0}
    pB = {"MC_ROLLRATE_K": 2.0, "MC_ROLLRATE_P": 0.10, "MC_ROLLRATE_I": 0.05,
          "MC_ROLLRATE_D": 0.002, "IMU_DGYRO_CUTOFF": 30.0}
    gA = synthesize_pid_from_plant(plant, "roll", pA)
    gB = synthesize_pid_from_plant(plant, "roll", pB)
    assert gA is not None and gB is not None
    effA = (gA.K * gA.Kp, gA.K * gA.Ki, gA.K * gA.Kd)
    effB = (gB.K * gB.Kp, gB.K * gB.Ki, gB.K * gB.Kd)
    for a, b, name in zip(effA, effB, ("K*Kp", "K*Ki", "K*Kd")):
        rel = abs(a - b) / (abs(a) + 1e-9)
        assert rel < 0.05, f"effective {name} differs by {rel*100:.1f}% (A={a:.4f} B={b:.4f})"


def test_simc_cross_check_integrator_pole():
    """
    SIMC (Skogestad 2003) closed-form second opinion is populated only for the
    integrator+pole plant, returns positive PI gains and Kd == 0, and is attached
    to the synthesis result as ``simc_gains``.
    """
    plant = PlantModel(K_plant=0.05, omega_n=2 * math.pi * 20, zeta=1.0,
                       delay_s=0.005, fit_vaf=90.0, order="integrator_pole",
                       confidence="high", int_gain=6.0, motor_pole=2 * math.pi * 20)
    simc = synthesize_simc(plant, K_fixed=1.0)
    assert isinstance(simc, SimcGains)
    assert simc.Kp > 0 and simc.Ki > 0 and simc.Kd == 0.0
    assert simc.tau_c_s > 0

    # A 2nd-order (non-integrator) plant yields no SIMC opinion.
    plant2 = PlantModel(K_plant=1.0, omega_n=2 * math.pi * 8, zeta=0.6,
                        delay_s=0.005, fit_vaf=90.0, order="2nd_order",
                        confidence="high")
    assert synthesize_simc(plant2, K_fixed=1.0) is None

    # Attached to the synthesis result for the integrator+pole plant.
    params = {"MC_ROLLRATE_K": 1.0, "MC_ROLLRATE_P": 0.15, "MC_ROLLRATE_I": 0.2,
              "MC_ROLLRATE_D": 0.003, "IMU_DGYRO_CUTOFF": 30.0}
    g = synthesize_pid_from_plant(plant, "roll", params)
    assert g is not None and isinstance(g.simc_gains, SimcGains)


def test_arx_freerun_vaf_discriminates():
    """
    A noise-driven 2nd-order oscillator with *no* input is trivially
    predictable one-step-ahead (y[k]~y[k-1]) but not in free-run simulation.
    A free-run VAF must therefore stay well below the 'good' threshold; the
    current one-step-ahead VAF is ~99% and passes the gate spuriously.
    """
    rng = np.random.default_rng(3)
    n = 6000
    y = np.zeros(n)
    for k in range(2, n):
        y[k] = 1.9 * y[k - 1] - 0.95 * y[k - 2] + 0.02 * rng.standard_normal()
    sp = np.zeros(n)   # no exogenous input
    t = np.arange(n) / FS_HIGH
    model = arx_fit(t, sp, y, axis="roll")
    assert model.vaf_pct < MIN_ARX_GOOD_VAF, (
        f"free-run VAF should be low, got {model.vaf_pct:.1f}%")


# ═══════════════════════════════════════════════════════════════════════════════
# vibration_analysis
# ═══════════════════════════════════════════════════════════════════════════════

def _vibe_gyro(fs: float, dur_s: float, tone_hz: float,
               tone_amp: float = 0.5, noise_std: float = 0.02, seed: int = 0):
    """Synthetic gyro (N,3) with a single strong tone on the Z axis over a
    low broadband floor, plus its µs timestamp axis."""
    rng = np.random.default_rng(seed)
    n = int(dur_s * fs)
    t_s = np.arange(n) / fs
    gz = tone_amp * np.sin(2 * math.pi * tone_hz * t_s) + noise_std * rng.standard_normal(n)
    gyro = np.column_stack([noise_std * rng.standard_normal(n),
                            noise_std * rng.standard_normal(n),
                            gz])
    t_us = (t_s * 1e6).astype(np.int64)
    return t_us, gyro


def test_vibration_finds_120hz_peak_suggests_static_notch():
    """A clean 120 Hz resonance well above the noise floor must be found within
    ±2 Hz and draw a *real* PX4 static-notch suggestion (IMU_GYRO_NF0_FRQ)."""
    t_us, gyro = _vibe_gyro(fs=1000.0, dur_s=6.0, tone_hz=120.0, seed=1)
    res = vibration_analysis(t_us, gyro)
    assert isinstance(res, VibrationResult)
    hits = [p for p in res.peaks if abs(p.freq_hz - 120.0) <= 2.0]
    assert hits, f"120 Hz peak not found; peaks={[p.freq_hz for p in res.peaks]}"
    assert "IMU_GYRO_NF0_FRQ" in hits[0].filter_suggestion, (
        f"expected static-notch advice, got '{hits[0].filter_suggestion}'")
    # The obsolete non-existent params must be gone everywhere.
    for p in res.peaks:
        assert "DNF1" not in p.filter_suggestion and "DNF2" not in p.filter_suggestion


def test_vibration_low_freq_peak_not_notched():
    """A 15 Hz peak is inside the control band (<25 Hz) — no notch must be
    suggested; the advice points at mechanics instead."""
    t_us, gyro = _vibe_gyro(fs=1000.0, dur_s=6.0, tone_hz=15.0, tone_amp=0.8, seed=2)
    res = vibration_analysis(t_us, gyro)
    hits = [p for p in res.peaks if abs(p.freq_hz - 15.0) <= 2.0]
    assert hits, f"15 Hz peak not found; peaks={[p.freq_hz for p in res.peaks]}"
    sug = hits[0].filter_suggestion
    assert "IMU_GYRO_NF" not in sug and "IMU_GYRO_DNF" not in sug, (
        f"a notch must NOT be advised below 25 Hz, got '{sug}'")
