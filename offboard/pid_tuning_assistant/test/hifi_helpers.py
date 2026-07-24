"""
Shared synthetic-signal generators for the HiFi test suite.

These helpers are reused by test_hifi.py and by the correctness
fixes in-24, so keep their signatures stable.

All generators are deterministic (seeded) and depend only on numpy.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from pid_tuning_assistant.hifi_analyzer import BodePoint, FRFResult


# ── time-domain loop data ─────────────────────────────────────────────────────

def white_noise_loop_data(
    fs: float = 500.0,
    dur_s: float = 30.0,
    delay_ms: float = 0.0,
    amp: float = 1.0,
    seed: int = 0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Return (t, sp, meas) for a closed-loop-style excitation.

    `sp` is band-unlimited white noise; `meas` is `sp` delayed by an integer
    number of samples closest to `delay_ms` (pure transport delay, no gain
    change).  With delay_ms == 0, `meas` is an independent copy of `sp`.

    Duration/rate default to values that satisfy the Welch averaging and
    MIN_RATE_FOR_FRF_HZ gates used by compute_frf.
    """
    t = np.arange(0, dur_s, 1.0 / fs)
    rng = np.random.default_rng(seed)
    sp = amp * rng.standard_normal(len(t))
    d = int(round(delay_ms * 1e-3 * fs))
    if d > 0:
        meas = np.concatenate([np.zeros(d), sp[:-d]])
    else:
        meas = sp.copy()
    return t, sp, meas


# ── frequency-domain analytic transfer functions ─────────────────────────────

def analytic_T_2nd_order(freqs_hz: np.ndarray, wn_hz: float, zeta: float
                         ) -> np.ndarray:
    """
    Closed-loop 2nd-order reference T(jw) = wn^2 / ((jw)^2 + 2 zeta wn jw + wn^2)
    evaluated on `freqs_hz` (Hz).  wn is given in Hz and converted to rad/s.

    For this T the open loop is L = wn^2 / (jw (jw + 2 zeta wn)), whose phase
    margin has the closed form
        PM = atan(2 zeta / sqrt(sqrt(1 + 4 zeta^4) - 2 zeta^2)).
    """
    wn = 2.0 * math.pi * wn_hz
    jw = 1j * 2.0 * math.pi * np.asarray(freqs_hz, dtype=float)
    return wn ** 2 / (jw ** 2 + 2.0 * zeta * wn * jw + wn ** 2)


def analytic_pm_2nd_order_deg(zeta: float) -> float:
    """Closed-form phase margin (deg) of L = wn^2/(jw(jw+2 zeta wn))."""
    return math.degrees(
        math.atan(2.0 * zeta / math.sqrt(math.sqrt(1.0 + 4.0 * zeta ** 4)
                                         - 2.0 * zeta ** 2))
    )


def px4_controller_frf(K: float, P: float, I: float, D: float,
                       f: np.ndarray, d_cut_hz: float = 30.0
                       ) -> Tuple[np.ndarray, np.ndarray]:
    """
    Two-degree-of-freedom PX4 rate controller in the frequency domain.

    Returns (C_pi, C_d) where
        C_pi = K (P + I/(jw))                 acts on the error  (r - y)
        C_d  = K D jw H_d(jw)                 acts on the measurement y only
    and H_d is a 2nd-order Butterworth low-pass at `d_cut_hz` (IMU_DGYRO_CUTOFF).

    The PX4 closed loop is  T = G C_pi / (1 + G (C_pi + C_d)); the open loop is
    L = G (C_pi + C_d).  Note L != T/(1-T) whenever D != 0.
    """
    jw = 1j * 2.0 * math.pi * np.asarray(f, dtype=float)
    wc = 2.0 * math.pi * d_cut_hz
    Hd = wc ** 2 / (jw ** 2 + math.sqrt(2.0) * wc * jw + wc ** 2)
    C_pi = K * (P + I / jw)
    C_d = K * D * jw * Hd
    return C_pi, C_d


# ── FRFResult construction from analytic arrays ──────────────────────────────

def frf_from_complex(
    freqs_hz: np.ndarray,
    H: np.ndarray,
    axis: str = "roll",
    loop: str = "rate",
    f_nyquist_hz: float = 100.0,
    coherence: float = 1.0,
    confidence: str = "high",
) -> FRFResult:
    """
    Build a FRFResult from an analytic complex response `H` on grid `freqs_hz`.
    Every bin is marked valid with the given (uniform) coherence.
    """
    freqs_hz = np.asarray(freqs_hz, dtype=float)
    H = np.asarray(H)
    bode: List[BodePoint] = [
        BodePoint(
            freq_hz=float(f),
            mag_db=float(20.0 * np.log10(np.abs(h) + 1e-12)),
            phase_deg=float(np.degrees(np.angle(h))),
            coherence=float(coherence),
            valid=bool(coherence >= 0.60 and f <= f_nyquist_hz),
        )
        for f, h in zip(freqs_hz, H)
    ]
    n_avg = max(1, len(bode) // 8)
    return FRFResult(axis=axis, loop=loop, bode=bode,
                     f_nyquist_hz=f_nyquist_hz, nperseg=512,
                     n_averages=n_avg, confidence=confidence)
