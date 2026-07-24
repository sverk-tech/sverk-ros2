"""
Second-order system identification for PID step-response analysis.

No ROS dependency — pure numpy/scipy.

Pipeline
--------
1. resample_uniform()   — project jittered DDS data onto a uniform grid
2. fit_second_order()   — least-squares fit of 2nd-order step-response model
3. log_decrement_zeta() — fallback when model fit is poor
4. coherence_verdict()  — neutral err↔torque coherence (soft confidence only)
5. nyquist_guard()      — clamp or invalidate frequency metrics

All results carry explicit confidence levels.  When data quality is insufficient
the pipeline returns 'low' confidence and omits absolute suggestions — callers
must gate on confidence before quoting numbers to users.

DISCLAIMER: Results are heuristic starting points.
Change one parameter at a time.  Verify in SITL before real flight.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import minimize
from scipy import signal as sig


# ── uniform resampling ────────────────────────────────────────────────────────

def resample_uniform(
    t: np.ndarray, y: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """
    Resample (t, y) onto a uniform grid at the median dt.

    Returns
    -------
    t_uni, y_uni   : uniform-grid arrays
    dt_med         : median inter-sample interval (s)
    jitter_pct     : MAD(dt) / median(dt) * 100  — larger = more jitter
    """
    if len(t) < 4:
        dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.02
        return t.copy(), y.copy(), dt, 0.0

    dts = np.diff(t)
    dt_med = float(np.median(dts))
    if dt_med <= 0:
        dt_med = 1e-3

    jitter_pct = float(np.median(np.abs(dts - dt_med)) / dt_med * 100.0)

    t_uni = np.arange(t[0], t[-1] + dt_med * 0.5, dt_med)
    f_interp = interp1d(t, y, kind="linear", bounds_error=False,
                        fill_value=(y[0], y[-1]))
    y_uni = f_interp(t_uni)
    return t_uni, y_uni, dt_med, jitter_pct


# ── 2nd-order analytical step response ───────────────────────────────────────

def _step_response_2nd(t_rel: np.ndarray, wn: float, zeta: float) -> np.ndarray:
    """
    Analytical unit-step response of G(s) = wn² / (s² + 2·ζ·wn·s + wn²).
    t_rel starts at 0.
    """
    if wn <= 0:
        return np.ones_like(t_rel)
    z = float(np.clip(zeta, 0.0, 3.0))

    if z >= 1.0:
        if abs(z - 1.0) < 1e-6:
            # critically damped
            return 1.0 - np.exp(-wn * t_rel) * (1.0 + wn * t_rel)
        else:
            s1 = -wn * (z - math.sqrt(z * z - 1.0))
            s2 = -wn * (z + math.sqrt(z * z - 1.0))
            if abs(s1 - s2) < 1e-9:
                return np.ones_like(t_rel)
            return 1.0 - (s2 * np.exp(s1 * t_rel) - s1 * np.exp(s2 * t_rel)) / (s2 - s1)
    else:
        wd = wn * math.sqrt(1.0 - z * z)
        env = np.exp(-z * wn * t_rel)
        return 1.0 - env * (np.cos(wd * t_rel) + (z / math.sqrt(1.0 - z * z)) * np.sin(wd * t_rel))


# ── model fit result ──────────────────────────────────────────────────────────

@dataclass
class ModelFit:
    wn: float               # natural frequency (rad/s)
    zeta: float             # damping ratio  0 = undamped, 1 = critical
    freq_hz: float          # wn / (2π)
    r_squared: float        # goodness of fit [0, 1]
    method: str             # "2nd_order" | "log_decrement" | "failed"
    confidence: str         # "high" | "medium" | "low"
    # Nyquist context — set by the caller after nyquist_guard()
    above_nyquist: bool = False
    # Quantitative scaling hint (only when confidence >= medium and not above_nyquist)
    k_scale_for_zeta07: Optional[float] = None   # multiply K by this → ζ ≈ 0.7


def fit_second_order(
    t: np.ndarray,
    meas: np.ndarray,
    sp_before: float,
    sp_after: float,
    f_nyquist_hz: float = 0.0,
) -> ModelFit:
    """
    Fit a 2nd-order model to one step-response window.

    Parameters
    ----------
    t        : time array (s), starting at or before the step
    meas     : measured signal (same units as sp)
    sp_before / sp_after : setpoint before / after step
    f_nyquist_hz : Nyquist limit of the recording; 0 = unknown

    Returns
    -------
    ModelFit with all fields populated.
    """
    step_size = sp_after - sp_before
    if abs(step_size) < 1e-9 or len(t) < 8:
        return ModelFit(wn=0, zeta=1, freq_hz=0, r_squared=0,
                        method="failed", confidence="low")

    # Normalize to [0 → 1] step
    norm = (meas - sp_before) / step_size
    t_rel = t - t[0]

    # Resample to uniform grid
    t_uni, n_uni, dt_med, jitter = resample_uniform(t_rel, norm)
    fs = 1.0 / dt_med if dt_med > 0 else 50.0
    f_nyq = fs / 2.0

    def cost(params):
        wn_, z_ = params
        if wn_ <= 0 or z_ < 0 or z_ > 3.0:
            return 1e6
        pred = _step_response_2nd(t_uni, wn_, z_)
        return float(np.mean((n_uni - pred) ** 2))

    # Grid search for robust initial guess
    best_cost, best_p = 1e9, (2 * math.pi * 3.0, 0.7)
    wn_candidates = np.geomspace(0.5, min(2 * math.pi * f_nyq * 0.8, 2 * math.pi * 80), 20)
    for wn_c in wn_candidates:
        for z_c in (0.2, 0.5, 0.7, 1.0, 1.5):
            c = cost((wn_c, z_c))
            if c < best_cost:
                best_cost, best_p = c, (wn_c, z_c)

    try:
        res = minimize(cost, best_p, method="Nelder-Mead",
                       options={"xatol": 1e-4, "fatol": 1e-7, "maxiter": 1000})
        wn_fit = abs(float(res.x[0]))
        z_fit = float(np.clip(res.x[1], 0.01, 3.0))
    except Exception:
        wn_fit, z_fit = best_p

    pred = _step_response_2nd(t_uni, wn_fit, z_fit)
    ss_tot = np.sum((n_uni - np.mean(n_uni)) ** 2)
    r2 = float(np.clip(1.0 - np.sum((n_uni - pred) ** 2) / (ss_tot + 1e-12), 0.0, 1.0))

    freq_hz = wn_fit / (2 * math.pi)

    # Try log-decrement fallback if fit is poor
    if r2 < 0.55:
        ld = log_decrement_zeta(t_uni, n_uni)
        if ld is not None:
            return ModelFit(wn=round(ld[0] * 2 * math.pi, 2),
                            zeta=round(ld[1], 3),
                            freq_hz=round(ld[0], 2),
                            r_squared=0.0,
                            method="log_decrement",
                            confidence="low")

    # Confidence from R², jitter, n_points
    if r2 > 0.85 and jitter < 15 and len(t_uni) >= 20:
        conf = "high"
    elif r2 > 0.60:
        conf = "medium"
    else:
        conf = "low"

    # Quantitative K-scale hint: K ∝ wn² for rate loop of 2nd-order plant
    # To move from current ζ to 0.7 holding plant constant:
    # ζ_new/ζ_old ≈ sqrt(K_old/K_new) → K_new = K_old * (ζ_old/0.7)²
    k_scale: Optional[float] = None
    if conf in ("high", "medium") and 0.1 < z_fit < 2.5:
        k_scale = round((z_fit / 0.7) ** 2, 2)

    above_nyq = (f_nyquist_hz > 0) and (freq_hz > f_nyquist_hz)

    return ModelFit(
        wn=round(wn_fit, 3),
        zeta=round(z_fit, 3),
        freq_hz=round(freq_hz, 2),
        r_squared=round(r2, 3),
        method="2nd_order",
        confidence=conf,
        above_nyquist=above_nyq,
        k_scale_for_zeta07=k_scale,
    )


# ── log-decrement fallback ────────────────────────────────────────────────────

def log_decrement_zeta(
    t: np.ndarray, y: np.ndarray
) -> Optional[Tuple[float, float]]:
    """
    Estimate (freq_hz, zeta) from log-decrement of oscillation peaks.
    Returns None when there are fewer than 3 peaks.
    """
    if len(y) < 10:
        return None
    try:
        peaks, _ = sig.find_peaks(y - np.mean(y),
                                  height=0.03 * float(np.std(y)), distance=2)
        if len(peaks) < 3:
            return None
        amps = y[peaks] - np.mean(y)
        if np.any(amps <= 0):
            return None
        log_decs = [math.log(amps[i] / amps[i + 1])
                    for i in range(len(amps) - 1) if amps[i + 1] > 0]
        if not log_decs:
            return None
        delta = float(np.median(log_decs))
        if delta <= 0:
            return None
        zeta = delta / math.sqrt(4 * math.pi ** 2 + delta ** 2)
        dts_p = np.diff(t[peaks])
        period = float(np.median(dts_p))
        freq_hz = 1.0 / period if period > 0 else 0.0
        return round(freq_hz, 2), round(zeta, 3)
    except Exception:
        return None


# ── coherence analysis ────────────────────────────────────────────────────────

def coherence_verdict(
    t: np.ndarray,
    rate_err: np.ndarray,
    torque: np.ndarray,
) -> dict:
    """
    Measure ordinary coherence between the rate error and the control torque.

    LIMITATION — read before interpreting the verdict.  In PX4 the torque is
    ``torque = C(err)`` (the controller output), so err and torque are coherent
    *by construction*: high coherence does not prove a "gain problem", and a
    "sensor_noise" verdict is not physically reachable this way (the two signals
    can only decorrelate through non-linearity, saturation or missing data).  The
    verdict is therefore reported in NEUTRAL terms and must be used ONLY as a soft
    confidence modifier downstream — never as evidence of noise vs gain.  The real
    discriminator needs an EXTERNAL reference signal (the setpoint).

    Returns dict:
        verdict          : "high_coherence" | "mixed" | "low_coherence"
                           | "insufficient_data"
        coherence_peak   : float or None
        coherence_freq_hz: float or None
    """
    if len(rate_err) < 32 or len(torque) < 32:
        return {"verdict": "insufficient_data", "coherence_peak": None,
                "coherence_freq_hz": None}
    try:
        _, _, dt_med, _ = resample_uniform(t, rate_err)
        fs = 1.0 / dt_med if dt_med > 0 else 50.0

        t_uni, err_uni, _, _ = resample_uniform(t, rate_err)
        _, tq_uni, _, _ = resample_uniform(t, torque)
        # align lengths
        n = min(len(err_uni), len(tq_uni))
        err_uni, tq_uni = err_uni[:n], tq_uni[:n]

        nperseg = min(64, n // 2)
        if nperseg < 8:
            return {"verdict": "insufficient_data", "coherence_peak": None,
                    "coherence_freq_hz": None}

        f, Cxy = sig.coherence(err_uni, tq_uni, fs=fs, nperseg=nperseg)
        mask = f > 1.0
        if not np.any(mask):
            return {"verdict": "insufficient_data", "coherence_peak": None,
                    "coherence_freq_hz": None}

        peak_coh = float(np.max(Cxy[mask]))
        peak_freq = float(f[mask][np.argmax(Cxy[mask])])

        # Neutral labels only — see the docstring: err↔torque coherence cannot
        # distinguish "noise" from "gain", it just tells how tightly the logged
        # torque follows the error.
        if peak_coh > 0.70:
            verdict = "high_coherence"
        elif peak_coh > 0.40:
            verdict = "mixed"
        else:
            verdict = "low_coherence"

        return {"verdict": verdict,
                "coherence_peak": round(peak_coh, 3),
                "coherence_freq_hz": round(peak_freq, 2)}
    except Exception:
        return {"verdict": "failed", "coherence_peak": None,
                "coherence_freq_hz": None}


# ── Nyquist guard ─────────────────────────────────────────────────────────────

def nyquist_guard(freq_hz: float, f_nyquist_hz: float) -> Tuple[bool, bool]:
    """
    Returns (above_nyquist, low_confidence_near_nyquist).

    above_nyquist            : True if freq_hz > f_nyquist_hz — don't report.
    low_confidence_near_nyquist: True if freq_hz > 0.8 * f_nyquist_hz — reduce confidence.
    """
    if f_nyquist_hz <= 0:
        return False, False
    above = freq_hz > f_nyquist_hz
    near = freq_hz > 0.8 * f_nyquist_hz
    return above, near


# ── step binning ──────────────────────────────────────────────────────────────

def bin_steps_by_amplitude(
    step_sizes: np.ndarray, n_bins: int = 3
) -> np.ndarray:
    """
    Return bin index (0-based) for each step event, based on amplitude quartiles.
    Steps with similar amplitude are comparable; don't aggregate across bins.
    """
    if len(step_sizes) == 0:
        return np.array([], dtype=int)
    percentiles = np.linspace(0, 100, n_bins + 1)
    edges = np.percentile(step_sizes, percentiles)
    bins = np.digitize(step_sizes, edges[1:-1])   # 0..n_bins-1
    return bins
