"""
High-rate analysis algorithms — activated only for ULog source (≥ 200 Hz).

Algorithms
----------
(a) FRF  — Frequency Response Function via cross-spectra (scipy.signal.csd/welch)
           Gated by coherence ≥ COH_THRESHOLD per frequency bin.
(b) Stability margins — GM/PM from open-loop L = T/(1-T).
           Only reported when coherence is sufficient and excitation is rich.
(c) Loop delay — cross-correlation + phase-slope method.
(d) Vibration analysis — raw gyro FFT, motor-harmonic correlation.
(e) Filter characterisation — raw vs filtered gyro phase/group delay.
(f) ARX identification — least-squares ARX with held-out VAF validation.
(g) Actuator saturation — clipping detection at 0/1 boundaries.
(h) Excitation classification — step-like vs sweep-like.

All results carry explicit confidence and data-source notes.
Read-only — no FMU parameter writes.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

import numpy as np
from scipy import signal as sig
from scipy.interpolate import interp1d

from .model_estimator import resample_uniform, nyquist_guard

log = logging.getLogger(__name__)

# ── thresholds ─────────────────────────────────────────────────────────────────
COH_THRESHOLD = 0.60       # below → bin not used in FRF / margins
MIN_EXC_CYCLES = 5.0       # min cycles of the lowest coherent freq in the inject
                           # window for a trustworthy reference-based FRF
MIN_RATE_FOR_FRF_HZ = 100  # minimum primary-topic rate for FRF
MIN_RATE_FOR_VIBE_HZ = 500 # minimum raw-gyro rate for vibration analysis
ARX_VAL_FRACTION = 0.25    # fraction of data held out for ARX validation
MIN_ARX_GOOD_VAF = 60.0    # VAF % threshold for "good" model

# ── data classes ──────────────────────────────────────────────────────────────

@dataclass
class BodePoint:
    freq_hz: float
    mag_db: float
    phase_deg: float
    coherence: float
    valid: bool            # coherence >= COH_THRESHOLD


@dataclass
class FRFResult:
    axis: str
    loop: str              # "rate" | "attitude"
    bode: List[BodePoint]
    f_nyquist_hz: float
    nperseg: int
    n_averages: int
    confidence: str        # "high" | "medium" | "low"
    note: str = ""
    # "setpoint_h1" = biased closed-loop H1=Suy/Suu (internal setpoint correlates
    # with feedback); "reference_indirect" = unbiased Ĝ=Φ_ry/Φ_ru from the external
    # excitation reference. The latter is trusted over the former.
    method: str = "setpoint_h1"


@dataclass
class StabilityMargins:
    axis: str
    gain_margin_db: Optional[float]          # positive = stable
    phase_margin_deg: Optional[float]        # positive = stable
    gain_crossover_hz: Optional[float]       # where |L|=1
    phase_crossover_hz: Optional[float]      # where phase(L)=-180°
    confidence: str                          # "high"|"medium"|"low"|"insufficient"
    note: str = ""


@dataclass
class LoopDelay:
    axis: str
    delay_ms: float
    method: str            # "xcorr" | "phase_slope" | "combined"
    confidence: str
    note: str = ""


@dataclass
class VibrationPeak:
    freq_hz: float
    amplitude_db: float
    motor_harmonic: Optional[str]    # e.g. "M1×2" if matched
    filter_suggestion: str           # e.g. "IMU_GYRO_NF0_FRQ=47, IMU_GYRO_NF0_BW=5"


@dataclass
class VibrationResult:
    peaks: List[VibrationPeak]
    fft_f: List[float]            # downsampled for JSON export
    fft_mag_db: List[float]
    raw_rate_hz: float
    note: str = ""
    confidence: str = "medium"


@dataclass
class FilterCharacteristic:
    axis: str
    delay_ms: float                  # estimated group delay of filter chain
    phase_deg_at_10hz: float
    phase_deg_at_50hz: Optional[float]   # None when 50 Hz is near/above Nyquist
    raw_rate_hz: float
    filtered_rate_hz: float
    confidence: str
    note: str = ""


@dataclass
class ARXModel:
    axis: str
    na: int                          # AR order
    nb: int                          # exogenous order
    coeffs_a: List[float]
    coeffs_b: List[float]
    vaf_pct: float                   # variance accounted for (validation set)
    nrmse: float
    confidence: str
    note: str = ""


@dataclass
class ActuatorSaturation:
    axis: str
    mode: str
    sat_fraction_pct: float          # % samples clipped at 0 or 1
    affected_segments: List[Tuple[float, float]]   # (t_start, t_end) seconds
    note: str = ""


@dataclass
class ExcitationInfo:
    excitation_type: str             # "step" | "sweep" | "mixed" | "unknown"
    quality: str                     # "good" | "marginal" | "poor"
    bandwidth_hz: float              # -3 dB bandwidth of setpoint spectrum
    rms_setpoint: float
    note: str = ""


@dataclass
class HiFiResult:
    """Aggregated result of all high-rate analyses."""
    source_name: str
    duration_s: float
    primary_rate_hz: float
    raw_gyro_rate_hz: float
    frf: List[FRFResult] = field(default_factory=list)
    margins: List[StabilityMargins] = field(default_factory=list)
    delays: List[LoopDelay] = field(default_factory=list)
    vibration: Optional[VibrationResult] = None
    filter_chars: List[FilterCharacteristic] = field(default_factory=list)
    arx_models: List[ARXModel] = field(default_factory=list)
    saturation: List[ActuatorSaturation] = field(default_factory=list)
    excitation: Optional[ExcitationInfo] = None
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        import dataclasses
        return dataclasses.asdict(self)


# ── helpers ───────────────────────────────────────────────────────────────────

def _clean(arr: np.ndarray) -> np.ndarray:
    arr = np.asarray(arr, dtype=float)
    arr = np.where(np.isnan(arr) | np.isinf(arr), 0.0, arr)
    return arr


def _column(frames: List[Any], field: str) -> np.ndarray:
    """Column of ``field`` over frames, missing values left as NaN (no zero-fill)."""
    out = np.full(len(frames), np.nan, dtype=float)
    for i, f in enumerate(frames):
        v = getattr(f, field, None)
        if v is not None:
            out[i] = v
    return out


def _uniform(t: np.ndarray, y: np.ndarray
             ) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Return uniformly resampled ``(t_uni, y_uni, fs)``.

    ``t_uni`` preserves the input's absolute t0 (it starts at ``t[0]``), so two
    streams resampled against a shared time origin keep their true start offset —
    essential for group-delay / alignment estimates.
    """
    t_uni, y_uni, dt_med, _ = resample_uniform(t, y)
    fs = 1.0 / dt_med if dt_med > 0 else 50.0
    return t_uni, y_uni, fs


def _uniform_segments(
    t: np.ndarray, sp: np.ndarray, meas: np.ndarray,
    max_gap_factor: float = 3.0, min_seg: int = 16,
) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], float]:
    """
    Resample (sp, meas) onto a uniform grid at the true sample rate (median dt),
    NaN-aware.  Instead of zero-filling holes (which inject spectral steps):

    * short gaps (< ``max_gap_factor`` missing samples) are linearly
      interpolated inside a segment;
    * longer gaps split the record into independent contiguous segments — their
      periodograms are averaged downstream rather than bridged.

    Returns ``(segments, fs)`` where ``segments`` is a list of
    ``(sp_uni, meas_uni)`` uniform blocks and ``fs`` is 1/dt_med.
    """
    t = np.asarray(t, dtype=float)
    sp = np.asarray(sp, dtype=float)
    meas = np.asarray(meas, dtype=float)
    n = min(len(t), len(sp), len(meas))
    t, sp, meas = t[:n], sp[:n], meas[:n]

    finite = np.isfinite(t) & np.isfinite(sp) & np.isfinite(meas)
    if int(np.sum(finite)) < 4:
        return [], 50.0
    t_f, sp_f, meas_f = t[finite], sp[finite], meas[finite]

    dts = np.diff(t_f)
    dt_med = float(np.median(dts))
    if dt_med <= 0:
        dt_med = 0.02
    fs = 1.0 / dt_med

    # Segment boundaries: a time gap wider than max_gap_factor·dt_med is a hole
    # too long to interpolate (also catches runs of ≥ max_gap_factor NaNs, which
    # drop out of t_f and re-appear as an oversized dt).
    breaks = np.where(dts > max_gap_factor * dt_med)[0]
    bounds = np.split(np.arange(len(t_f)), breaks + 1)

    segments: List[Tuple[np.ndarray, np.ndarray]] = []
    for b in bounds:
        if len(b) < min_seg:
            continue
        t_seg = t_f[b]
        _, sp_u, _, _ = resample_uniform(t_seg, sp_f[b])
        _, meas_u, _, _ = resample_uniform(t_seg, meas_f[b])
        m = min(len(sp_u), len(meas_u))
        if m >= min_seg:
            segments.append((sp_u[:m], meas_u[:m]))
    return segments, fs


def _spectra_from_segments(
    segments: List[Tuple[np.ndarray, np.ndarray]], fs: float, nperseg: int,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]]:
    """
    Welch/CSD H1 estimate averaged over gap-delimited segments.

    Returns ``(f, Pxx, Pyy, Pxy, n_avg)`` — auto/cross spectra averaged (weighted
    by each segment's number of Welch sub-windows).  H = Pxy/Pxx and the ordinary
    coherence |Pxy|²/(Pxx·Pyy) are then formed by the caller.
    """
    max_len = max(len(s[0]) for s in segments)
    nps = int(min(nperseg, max_len))
    if nps < 8:
        return None

    f_ref = None
    Pxx_a = Pyy_a = Pxy_a = None
    w_total = 0.0
    n_avg = 0
    for sp_s, meas_s in segments:
        if len(sp_s) < nps:
            continue
        f, Pxx = sig.welch(sp_s,   fs=fs, nperseg=nps, detrend="linear")
        _, Pyy = sig.welch(meas_s, fs=fs, nperseg=nps, detrend="linear")
        _, Pxy = sig.csd(sp_s, meas_s, fs=fs, nperseg=nps, detrend="linear")
        segs = max(1, 2 * (len(sp_s) // nps) - 1)   # ≈ 50 %-overlap sub-windows
        w = float(segs)
        if Pxx_a is None:
            f_ref = f
            Pxx_a, Pyy_a, Pxy_a = w * Pxx, w * Pyy, w * Pxy
        else:
            Pxx_a += w * Pxx; Pyy_a += w * Pyy; Pxy_a += w * Pxy
        w_total += w
        n_avg += segs

    if Pxx_a is None or w_total <= 0:
        return None
    Pxx_a /= w_total; Pyy_a /= w_total; Pxy_a /= w_total
    return f_ref, Pxx_a, Pyy_a, Pxy_a, max(1, n_avg)


def _wrap180(deg: float) -> float:
    """Wrap an angle in degrees into (−180, 180]."""
    w = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if w == -180.0 else w


def _frequency_bands(f_arr: np.ndarray) -> np.ndarray:
    """
    Label each frequency bin with a contiguous-band index.  A break is inserted
    where the local frequency step jumps well above the surrounding grid spacing
    (a coherence gap in an otherwise uniform FRF grid) — but NOT for the smoothly
    growing steps of a geometric grid.
    """
    band = np.zeros(len(f_arr), dtype=int)
    if len(f_arr) < 3:
        return band
    df = np.diff(f_arr)
    ref = float(np.median(df))
    cur = 0
    for i in range(1, len(df)):
        if df[i] > 1.8 * ref and df[i] > 1.5 * df[i - 1]:
            cur += 1
        band[i + 1] = cur
    return band


# ── (a) FRF ──────────────────────────────────────────────────────────────────

def compute_frf(
    t: np.ndarray, sp: np.ndarray, meas: np.ndarray,
    fs: float, axis: str, loop: str,
    f_nyquist_hz: float, nperseg: int = 512,
) -> Optional[FRFResult]:
    """
    Estimate the H1 FRF H(f) = Sxy / Sxx with coherence gating, x = sp, y = meas.
    Only frequency bins with Cxy ≥ COH_THRESHOLD are marked valid.

    The raw DDS/ULog samples carry jitter and holes; ``t`` is used to resample
    onto a uniform grid at the true rate and to average periodograms over
    gap-delimited segments (no zero-fill).
    """
    # Resample onto a uniform grid at the true sample rate (holes are segmented,
    # not zero-filled) and take fs from the actual median dt.
    segments, fs_uni = _uniform_segments(t, sp, meas)
    total_len = sum(len(s[0]) for s in segments)
    if not segments or total_len < nperseg * 2:
        return None
    fs = fs_uni

    try:
        spec = _spectra_from_segments(segments, fs, nperseg)
    except Exception as e:
        log.warning(f"FRF {axis}/{loop}: scipy error: {e}")
        return None
    if spec is None:
        return None
    f, Pxx, Pyy, Pxy, n_avg = spec

    # H1 estimate — scipy's csd(x, y) = E[conj(X)·Y] = Sxy, so H = Sxy/Sxx.
    H = Pxy / (Pxx + 1e-14)
    Cxy = np.clip(np.abs(Pxy) ** 2 / (Pxx * Pyy + 1e-30), 0.0, 1.0)
    mag_db    = 20.0 * np.log10(np.abs(H) + 1e-12)
    phase_deg = np.degrees(np.unwrap(np.angle(H)))

    n_valid = int(np.sum(Cxy >= COH_THRESHOLD))
    if n_valid < 5:
        conf = "low"
        note = f"Only {n_valid} frequency bins with coherence ≥ {COH_THRESHOLD:.2f}."
    elif n_valid < 20:
        conf = "medium"
        note = ""
    else:
        conf = "high"
        note = ""

    # Downsample Bode for export (max 200 points)
    idx = np.arange(1, len(f))   # skip DC
    step = max(1, len(idx) // 200)
    idx = idx[::step]

    bode = [
        BodePoint(
            freq_hz=round(float(f[i]), 3),
            mag_db=round(float(mag_db[i]), 2),
            phase_deg=round(float(phase_deg[i]), 1),
            coherence=round(float(Cxy[i]), 3),
            valid=bool(Cxy[i] >= COH_THRESHOLD and f[i] <= f_nyquist_hz),
        )
        for i in idx
    ]

    nps_eff = int(min(nperseg, max(len(s[0]) for s in segments)))
    return FRFResult(axis=axis, loop=loop, bode=bode,
                     f_nyquist_hz=f_nyquist_hz, nperseg=nps_eff,
                     n_averages=n_avg, confidence=conf, note=note)


# ── (a′) Reference-based (unbiased) FRF ────────────────────────────────────────

def _uniform_segments3(
    t: np.ndarray, r: np.ndarray, u: np.ndarray, y: np.ndarray,
    max_gap_factor: float = 3.0, min_seg: int = 16,
) -> Tuple[List[Tuple[np.ndarray, np.ndarray, np.ndarray]], float]:
    """3-signal analogue of ``_uniform_segments`` for the (r, u, y) triple.

    Resamples the external reference r, the commanded setpoint u and the measured
    response y onto a shared uniform grid, NaN-aware and gap-segmented. r is NaN
    outside the injection window, so the finite mask naturally restricts the
    segments to the excitation run.
    """
    t = np.asarray(t, dtype=float)
    r = np.asarray(r, dtype=float)
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    n = min(len(t), len(r), len(u), len(y))
    t, r, u, y = t[:n], r[:n], u[:n], y[:n]

    finite = np.isfinite(t) & np.isfinite(r) & np.isfinite(u) & np.isfinite(y)
    if int(np.sum(finite)) < 4:
        return [], 50.0
    t_f, r_f, u_f, y_f = t[finite], r[finite], u[finite], y[finite]

    dts = np.diff(t_f)
    dt_med = float(np.median(dts))
    if dt_med <= 0:
        dt_med = 0.02
    fs = 1.0 / dt_med

    breaks = np.where(dts > max_gap_factor * dt_med)[0]
    bounds = np.split(np.arange(len(t_f)), breaks + 1)

    segments: List[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for b in bounds:
        if len(b) < min_seg:
            continue
        t_seg = t_f[b]
        _, r_u, _, _ = resample_uniform(t_seg, r_f[b])
        _, u_u, _, _ = resample_uniform(t_seg, u_f[b])
        _, y_u, _, _ = resample_uniform(t_seg, y_f[b])
        m = min(len(r_u), len(u_u), len(y_u))
        if m >= min_seg:
            segments.append((r_u[:m], u_u[:m], y_u[:m]))
    return segments, fs


def _spectra_ref_from_segments(
    segments: List[Tuple[np.ndarray, np.ndarray, np.ndarray]], fs: float, nperseg: int,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]]:
    """Segment-averaged auto/cross spectra for the reference-based estimator.

    Returns ``(f, Prr, Pyy, Pry, Pru, n_avg)``. Ĝ = Pry/Pru and the reference
    coherence γ²_ry = |Pry|²/(Prr·Pyy) are formed by the caller.
    """
    max_len = max(len(s[0]) for s in segments)
    nps = int(min(nperseg, max_len))
    if nps < 8:
        return None

    f_ref = None
    Prr_a = Pyy_a = Pry_a = Pru_a = None
    w_total = 0.0
    n_avg = 0
    for r_s, u_s, y_s in segments:
        if len(r_s) < nps:
            continue
        f, Prr = sig.welch(r_s, fs=fs, nperseg=nps, detrend="linear")
        _, Pyy = sig.welch(y_s, fs=fs, nperseg=nps, detrend="linear")
        _, Pry = sig.csd(r_s, y_s, fs=fs, nperseg=nps, detrend="linear")
        _, Pru = sig.csd(r_s, u_s, fs=fs, nperseg=nps, detrend="linear")
        segs = max(1, 2 * (len(r_s) // nps) - 1)
        w = float(segs)
        if Prr_a is None:
            f_ref = f
            Prr_a, Pyy_a, Pry_a, Pru_a = w * Prr, w * Pyy, w * Pry, w * Pru
        else:
            Prr_a += w * Prr; Pyy_a += w * Pyy; Pry_a += w * Pry; Pru_a += w * Pru
        w_total += w
        n_avg += segs

    if Prr_a is None or w_total <= 0:
        return None
    Prr_a /= w_total; Pyy_a /= w_total; Pry_a /= w_total; Pru_a /= w_total
    return f_ref, Prr_a, Pyy_a, Pry_a, Pru_a, max(1, n_avg)


def compute_frf_reference(
    t: np.ndarray, r: np.ndarray, u: np.ndarray, y: np.ndarray,
    fs: float, axis: str, loop: str,
    f_nyquist_hz: float, nperseg: int = 512,
) -> Optional[FRFResult]:
    """Unbiased closed-loop FRF from an external excitation reference.

    ``Ĝ(jω) = Φ_ry(jω) / Φ_ru(jω)`` where r is the *exact injected reference*
    (⟂ measurement/process noise), u the commanded setpoint on the axis and y the
    measured response. Because r is uncorrelated with the closed-loop noise, the
    ratio recovers the same u→y transfer that the setpoint-based H1 = Suy/Suu
    estimates, but WITHOUT the closed-loop (setpoint-based) bias (there the
    internal setpoint was correlated with feedback). Gated by γ²_ry ≥ COH_THRESHOLD.
    """
    segments, fs_uni = _uniform_segments3(t, r, u, y)
    total_len = sum(len(s[0]) for s in segments)
    if not segments or total_len < nperseg * 2:
        return None
    fs = fs_uni

    try:
        spec = _spectra_ref_from_segments(segments, fs, nperseg)
    except Exception as e:
        log.warning(f"ref-FRF {axis}/{loop}: scipy error: {e}")
        return None
    if spec is None:
        return None
    f, Prr, Pyy, Pry, Pru, n_avg = spec

    G = Pry / (Pru + 1e-14)
    coh_ry = np.clip(np.abs(Pry) ** 2 / (Prr * Pyy + 1e-30), 0.0, 1.0)
    mag_db    = 20.0 * np.log10(np.abs(G) + 1e-12)
    phase_deg = np.degrees(np.unwrap(np.angle(G)))

    # Reference-based is unbiased, so we trust it at a slightly lower valid-bin
    # count than the biased setpoint-based path.
    n_valid = int(np.sum(coh_ry >= COH_THRESHOLD))
    valid_mask = coh_ry >= COH_THRESHOLD

    # Excitation-richness gate: the injection window must span enough cycles of
    # the lowest coherent frequency, else the low-frequency FRF is untrustworthy
    # (too few periods for a stable cross-spectral estimate).
    window_dur = total_len / fs if fs > 0 else 0.0
    f_lo = float(np.min(f[valid_mask])) if n_valid > 0 else 0.0
    cycles_lo = window_dur * f_lo
    rich = cycles_lo >= MIN_EXC_CYCLES

    if n_valid < 5:
        conf = "low"
        note = f"Reference-based, but only {n_valid} bins with γ²_ry ≥ {COH_THRESHOLD:.2f}."
    elif not rich:
        conf = "low"
        note = (f"Reference-based, but excitation too short: {cycles_lo:.1f} cycles "
                f"at {f_lo:.2f} Hz (< {MIN_EXC_CYCLES:.0f}); low-freq FRF unreliable.")
    elif n_valid < 15:
        conf = "medium"
        note = "Reference-based (unbiased, closed-loop)."
    else:
        conf = "high"
        note = "Reference-based (unbiased, closed-loop)."

    idx = np.arange(1, len(f))
    step = max(1, len(idx) // 200)
    idx = idx[::step]

    bode = [
        BodePoint(
            freq_hz=round(float(f[i]), 3),
            mag_db=round(float(mag_db[i]), 2),
            phase_deg=round(float(phase_deg[i]), 1),
            coherence=round(float(coh_ry[i]), 3),
            valid=bool(coh_ry[i] >= COH_THRESHOLD and f[i] <= f_nyquist_hz),
        )
        for i in idx
    ]

    nps_eff = int(min(nperseg, max(len(s[0]) for s in segments)))
    return FRFResult(axis=axis, loop=loop, bode=bode,
                     f_nyquist_hz=f_nyquist_hz, nperseg=nps_eff,
                     n_averages=n_avg, confidence=conf, note=note,
                     method="reference_indirect")


def _dominant_exc_run(frames: List[Any], axis: str) -> Tuple[Optional[str], int]:
    """Pick the excitation run for ``axis``: the run_tag with the most inject
    samples. Returns ``(run_tag, n_distinct_runs)`` — never merges two runs into
    one FRF. ``run_tag`` is None when this axis has no excitation-reference data.
    """
    counts: Dict[str, int] = {}
    for f in frames:
        if (getattr(f, "exc_axis", None) == axis
                and getattr(f, "exc_phase", None) == "inject"
                and getattr(f, "exc_ref", None) is not None):
            tag = getattr(f, "exc_run_tag", None)
            if tag:
                counts[tag] = counts.get(tag, 0) + 1
    if not counts:
        return None, 0
    best = max(counts.items(), key=lambda kv: kv[1])[0]
    return best, len(counts)


# ── (b) Stability margins ─────────────────────────────────────────────────────

def stability_margins_from_frf(
    frf: FRFResult, axis: str,
    params: Optional[Dict[str, float]] = None,
) -> StabilityMargins:
    """
    Compute the true open-loop ``L`` from the closed-loop FRF ``T`` and the PX4
    controller gains, then find Gain Margin and Phase Margin.

    PX4 is a 2-DOF controller (D on the measurement), so
        L = T·(C_pi + C_d) / (C_pi·(1 − T) − T·C_d),   L ≠ T/(1 − T) when D ≠ 0.
    ``params`` supplies the axis gains (MC_<AX>RATE_{K,P,I,D}, IMU_DGYRO_CUTOFF).
    When gains are unavailable, or when D = 0, the formula collapses to the
    single-DOF L = T/(1 − T); a missing gain set lowers confidence and is noted.

    Only trusted where coherence is high.
    """
    from .plant_estimator import _c_pi, _c_d   # deferred: avoid circular import

    AX = axis.upper()
    gains_note = ""
    K = Kp = Ki = Kd = d_cut = None
    if params:
        K = params.get(f"MC_{AX}RATE_K")
        if K is not None:
            Kp = params.get(f"MC_{AX}RATE_P", 0.15)
            Ki = params.get(f"MC_{AX}RATE_I", 0.2)
            Kd = params.get(f"MC_{AX}RATE_D", 0.003)
            d_cut = params.get("IMU_DGYRO_CUTOFF", 30.0)
    use_2dof = K is not None and Kd is not None and abs(Kd) > 0.0

    if frf.confidence == "low":
        return StabilityMargins(
            axis=axis, gain_margin_db=None, phase_margin_deg=None,
            gain_crossover_hz=None, phase_crossover_hz=None,
            confidence="insufficient",
            note="FRF confidence too low for margin estimation.",
        )

    valid_pts = [b for b in frf.bode if b.valid and b.freq_hz > 0.3]
    if len(valid_pts) < 10:
        return StabilityMargins(
            axis=axis, gain_margin_db=None, phase_margin_deg=None,
            gain_crossover_hz=None, phase_crossover_hz=None,
            confidence="insufficient",
            note=f"Only {len(valid_pts)} valid coherent bins — insufficient for margins.",
        )

    # Sort by frequency (bode is ascending, but be defensive) and group the
    # coherence-gated bins into contiguous frequency bands: after gating the
    # valid bins are NOT adjacent in frequency, so phase must be unwrapped inside
    # each band independently — a global unwrap across a gap injects a spurious
    # ±π jump that shifts every downstream margin.
    order  = np.argsort([p.freq_hz for p in valid_pts])
    f_arr  = np.array([valid_pts[i].freq_hz   for i in order])
    mag_db = np.array([valid_pts[i].mag_db    for i in order])
    ph_deg = np.array([valid_pts[i].phase_deg for i in order])
    coh    = np.array([valid_pts[i].coherence for i in order])

    band_idx = _frequency_bands(f_arr)

    # Closed-loop complex T (phase periodicity makes unwrap irrelevant here).
    T = 10.0 ** (mag_db / 20.0) * np.exp(1j * np.radians(ph_deg))

    # Open-loop L.  With the PX4 2-DOF controller (D on the measurement):
    #     L = T·(C_pi + C_d) / (C_pi·(1 − T) − T·C_d).
    # With D = 0 or unknown gains this reduces exactly to the single-DOF
    # L = T/(1 − T) (C_pi cancels), so both branches share the tiny-denominator
    # guard (never clamp the phase away — just avoid a divide-by-zero).
    if use_2dof:
        C_pi = _c_pi(f_arr, K, Kp, Ki)
        C_d  = _c_d(f_arr, K, Kd, d_cut)
        denom = C_pi * (1.0 - T) - T * C_d
        tiny = np.abs(C_pi + C_d) * 1e-6 + 1e-12
        denom = np.where(np.abs(denom) < tiny, tiny, denom)
        L = T * (C_pi + C_d) / denom
    else:
        denom = 1.0 - T
        denom = np.where(np.abs(denom) < 1e-6, 1e-6 + 0j, denom)
        L = T / denom
        if params is None or K is None:
            gains_note = ("Controller gains unavailable — D-on-measurement "
                          "correction disabled; L=T/(1−T) is exact only for D=0. ")

    L_mag_db = 20.0 * np.log10(np.abs(L) + 1e-12)
    L_phase  = np.degrees(np.angle(L))
    # Band-wise unwrap — never across a coherence gap.
    for bi in np.unique(band_idx):
        m = band_idx == bi
        L_phase[m] = np.degrees(np.unwrap(np.radians(L_phase[m])))

    # Gain crossover: |L| falling through 0 dB.  Collect ALL falling crossings
    # (rising ones are low-frequency |L| ripple, not a real crossover) and report
    # the WORST (smallest) phase margin; interpolate both gc and the phase at gc.
    gc_hz = pm_deg = coh_at_gc = None
    gc_candidates: List[Tuple[float, float, float]] = []
    for i in range(len(f_arr) - 1):
        if band_idx[i] != band_idx[i + 1]:
            continue
        g0, g1 = float(L_mag_db[i]), float(L_mag_db[i + 1])
        if g0 > 0.0 >= g1:                       # falling through 0 dB
            frac = g0 / (g0 - g1)
            f0, f1 = float(f_arr[i]), float(f_arr[i + 1])
            gc = f0 + (f1 - f0) * frac
            ph_gc = float(L_phase[i]) + frac * float(L_phase[i + 1] - L_phase[i])
            pm = _wrap180(ph_gc + 180.0)
            cg = float(coh[i]) + frac * float(coh[i + 1] - coh[i])
            gc_candidates.append((gc, pm, cg))
    if gc_candidates:
        gc_hz, pm_deg, coh_at_gc = min(gc_candidates, key=lambda c: c[1])

    # Phase crossover: phase(L) = -180°.  Collect all crossings, report the worst
    # (smallest) gain margin.
    pcross_hz, gm_db = None, None
    shifted = L_phase + 180.0
    gm_candidates: List[Tuple[float, float]] = []
    for i in range(len(f_arr) - 1):
        if band_idx[i] != band_idx[i + 1]:
            continue
        s0, s1 = float(shifted[i]), float(shifted[i + 1])
        if s0 == s1 or np.sign(s0) == np.sign(s1):
            continue
        frac = -s0 / (s1 - s0)
        f0, f1 = float(f_arr[i]), float(f_arr[i + 1])
        pc = f0 + (f1 - f0) * frac
        mag_at = float(L_mag_db[i]) + frac * float(L_mag_db[i + 1] - L_mag_db[i])
        gm_candidates.append((pc, -mag_at))
    if gm_candidates:
        pcross_hz, gm_db = min(gm_candidates, key=lambda c: c[1])

    # Confidence: require high coherence at relevant frequencies
    coh_at_gc = coh_at_gc if coh_at_gc is not None else 0.0
    if frf.confidence == "high" and coh_at_gc >= 0.75 and gc_hz is not None:
        conf = "medium"   # never "high" — always depends on excitation richness
        note = (f"Gain crossover: {gc_hz:.2f} Hz. "
                "NOTE: reliable margins require broad-spectrum excitation (chirp sweep).")
    else:
        conf = "low"
        note = ("Margins computed but coherence at crossover is marginal. "
                "Use chirp sweep for reliable stability assessment.")

    # Gains unavailable → the 2-DOF correction could not be applied: cap at "low".
    if gains_note:
        conf = "low"
        note = gains_note + note

    return StabilityMargins(
        axis=axis,
        gain_margin_db=round(gm_db, 1) if gm_db is not None else None,
        phase_margin_deg=round(pm_deg, 1) if pm_deg is not None else None,
        gain_crossover_hz=round(gc_hz, 2) if gc_hz is not None else None,
        phase_crossover_hz=round(pcross_hz, 2) if pcross_hz is not None else None,
        confidence=conf, note=note,
    )


# ── (c) Loop delay ────────────────────────────────────────────────────────────

def estimate_loop_delay(
    t: np.ndarray, sp: np.ndarray, meas: np.ndarray,
    fs: float, axis: str,
    frf: Optional[FRFResult] = None,
) -> LoopDelay:
    # Work on a uniform grid at the true rate — cross-correlation lag → ms is
    # only meaningful on evenly spaced samples.  Use the longest contiguous
    # segment (holes are not bridged for a transport-delay estimate).
    segments, fs_uni = _uniform_segments(t, sp, meas)
    if segments:
        sp, meas = max(segments, key=lambda s: len(s[0]))
        fs = fs_uni
    else:
        sp = _clean(sp); meas = _clean(meas)
        n = min(len(sp), len(meas))
        sp = sp[:n]; meas = meas[:n]

    results: List[float] = []
    method_parts: List[str] = []

    # Method 1: cross-correlation peak
    try:
        corr = np.correlate(meas - meas.mean(), sp - sp.mean(), mode="full")
        lags = np.arange(-(len(sp) - 1), len(sp))
        peak_lag = int(lags[np.argmax(corr)])
        delay_xcorr = max(0.0, peak_lag / fs * 1000.0)   # ms
        results.append(delay_xcorr)
        method_parts.append("xcorr")
    except Exception:
        pass

    # Method 2: phase slope from FRF (linear regression at low freq)
    if frf is not None and len(frf.bode) > 5:
        pts = [b for b in frf.bode if b.valid and 1.0 < b.freq_hz < min(20.0, frf.f_nyquist_hz * 0.5)]
        if len(pts) >= 4:
            freqs = np.array([p.freq_hz for p in pts])
            phases = np.array([p.phase_deg for p in pts])
            # unwrap and fit: phase ≈ -2π * f * delay
            # slope = -2π * delay_s  →  delay_s = -slope / (2π)
            w = np.array([p.coherence for p in pts])
            A = np.column_stack([2.0 * np.pi * freqs, np.ones_like(freqs)])
            try:
                coeffs, _, _, _ = np.linalg.lstsq(
                    A * w[:, None], -np.radians(phases) * w, rcond=None)
                delay_ps = float(coeffs[0]) * 1000.0   # s → ms
                if 0 < delay_ps < 100.0:
                    results.append(delay_ps)
                    method_parts.append("phase_slope")
            except Exception:
                pass

    if not results:
        return LoopDelay(axis=axis, delay_ms=0.0, method="failed",
                         confidence="low", note="Could not estimate delay.")

    delay_ms = float(np.median(results))
    method = "+".join(method_parts) if len(method_parts) > 1 else method_parts[0]
    conf = "medium" if len(results) == 2 else "low"
    return LoopDelay(axis=axis, delay_ms=round(delay_ms, 2),
                     method=method, confidence=conf)


# ── (d) Vibration analysis ────────────────────────────────────────────────────

def vibration_analysis(
    t_us: np.ndarray, gyro_xyz: np.ndarray,
    esc_rpm: Optional[np.ndarray] = None,
    esc_t_us: Optional[np.ndarray] = None,
    n_motors: int = 4,
) -> VibrationResult:
    """
    FFT of raw gyro + motor-harmonic correlation.
    """
    if gyro_xyz is None or len(gyro_xyz) < 256:
        return VibrationResult(peaks=[], fft_f=[], fft_mag_db=[],
                               raw_rate_hz=0.0, confidence="low",
                               note="Insufficient raw gyro data.")

    dur_s = (int(t_us[-1]) - int(t_us[0])) / 1e6
    raw_rate = len(t_us) / max(1.0, dur_s)
    if raw_rate < MIN_RATE_FOR_VIBE_HZ:
        note = (f"Raw gyro at {raw_rate:.0f} Hz — "
                f"need ≥ {MIN_RATE_FOR_VIBE_HZ} Hz for vibration analysis.")
        return VibrationResult(peaks=[], fft_f=[], fft_mag_db=[], raw_rate_hz=raw_rate,
                               confidence="low", note=note)

    # Use the Z-axis gyro (most sensitive to structural vibration for quads)
    t_s = (t_us - t_us[0]).astype(float) / 1e6
    _, gz_uni, fs = _uniform(t_s, _clean(gyro_xyz[:, 2]))

    # Welch PSD
    nperseg = min(4096, len(gz_uni) // 4)
    f, psd = sig.welch(gz_uni, fs=fs, nperseg=nperseg, scaling="density")
    mag_db = 10.0 * np.log10(psd + 1e-30)
    df = float(f[1] - f[0]) if len(f) > 1 else fs / max(1, nperseg)
    f_nyq = fs / 2.0

    # Noise floor = MEDIAN of the spectrum.  The 95th percentile sits *near* the
    # peaks, not the floor, so genuine resonances +15 dB over the floor never got
    # reported; a real peak must stand ≥ 12 dB above the median floor.
    floor_db = float(np.median(mag_db))
    min_height = floor_db + 12.0
    # Minimum peak separation is a PHYSICAL 8 Hz, converted to bins via df — the
    # old int(fs/50) mixed up bins and Hz, so the guard drifted with nperseg/fs.
    min_dist_bins = max(1, int(round(8.0 / df))) if df > 0 else 1
    peak_idx, props = sig.find_peaks(mag_db, height=min_height,
                                      distance=min_dist_bins, prominence=5.0)

    # Average motor RPM → fundamental + harmonics.  The match window widens with
    # the spread of rotor speeds over the log (±max(5 Hz, 2·IQR)); harmonics above
    # Nyquist are dropped — they cannot appear in the PSD anyway.
    motor_harmonics: Dict[str, float] = {}
    match_win_hz = 5.0
    if esc_rpm is not None and esc_t_us is not None and len(esc_rpm) > 0:
        rpm_all = np.asarray(esc_rpm, dtype=float).ravel()
        rpm_all = rpm_all[np.isfinite(rpm_all) & (rpm_all > 100.0)]
        if rpm_all.size > 0:
            hz_all = rpm_all / 60.0
            fund_hz = float(np.median(hz_all))
            q1, q3 = np.percentile(hz_all, [25.0, 75.0])
            match_win_hz = max(5.0, 2.0 * float(q3 - q1))
            n_blade = 2    # typical propeller blade count
            for h in range(1, 5):
                for name, hz in ((f"motor×{h}", fund_hz * h),
                                 (f"prop×{h}",  fund_hz * n_blade * h)):
                    if hz < f_nyq:
                        motor_harmonics[name] = hz

    # Assign static-notch slots to the strongest non-harmonic peaks first (PX4 has
    # two static notches: NF0, NF1).
    peaks: List[VibrationPeak] = []
    static_slot = 0
    for i in sorted(peak_idx, key=lambda j: mag_db[j], reverse=True):
        f0 = float(f[i])
        a  = float(mag_db[i])
        matched_harmonic = None
        for harm_name, harm_hz in motor_harmonics.items():
            if abs(f0 - harm_hz) <= match_win_hz:
                matched_harmonic = f"{harm_name} ({harm_hz:.1f} Hz)"
                break

        if f0 < 25.0:
            # Inside the control bandwidth (PX4 IMU_GYRO_DNF_MIN default = 25 Hz):
            # a notch here fights the controller — fix it mechanically instead.
            filt_sug = (f"{f0:.0f} Hz is within the control band (<25 Hz) — do NOT "
                        "notch. Check propeller balance, frame stiffness and motor mounts.")
        elif matched_harmonic is not None:
            # RPM-tracking peak → the dynamic ESC notch follows it across throttle.
            bw = max(5, int(round(f0 * 0.1)))
            filt_sug = ("RPM harmonic — enable the dynamic ESC notch: "
                        f"IMU_GYRO_DNF_EN=1, IMU_GYRO_DNF_MIN=25, IMU_GYRO_DNF_BW={bw}, "
                        "IMU_GYRO_DNF_HMC=3.")
        elif static_slot < 2:
            slot = static_slot
            static_slot += 1
            bw = max(5, int(round(f0 * 0.1)))
            filt_sug = (f"Static notch: IMU_GYRO_NF{slot}_FRQ={f0:.0f}, "
                        f"IMU_GYRO_NF{slot}_BW={bw}.")
        else:
            filt_sug = (f"Resonance at {f0:.0f} Hz — no free static notch slot left; "
                        "reduce it at the source (motor/frame/prop balance).")
        peaks.append(VibrationPeak(
            freq_hz=round(f0, 1), amplitude_db=round(a, 1),
            motor_harmonic=matched_harmonic, filter_suggestion=filt_sug))

    # Downsample FFT for export (max 500 points)
    step = max(1, len(f) // 500)
    fft_f_export  = [round(float(v), 2) for v in f[1::step]]
    fft_mag_export = [round(float(v), 1) for v in mag_db[1::step]]

    conf = "high" if raw_rate >= 1000 else "medium"
    return VibrationResult(
        peaks=peaks, fft_f=fft_f_export, fft_mag_db=fft_mag_export,
        raw_rate_hz=round(raw_rate, 1), confidence=conf,
    )


# ── (e) Filter characterisation ──────────────────────────────────────────────

def filter_characterization(
    t_us_raw: np.ndarray, gyro_raw_xyz: np.ndarray,
    t_us_filt: np.ndarray, gyro_filt_xyz: np.ndarray,
    axis_idx: int = 2,
    axis: str = "z",
) -> Optional[FilterCharacteristic]:
    """
    Estimate group delay of the filter chain by comparing raw ``sensor_gyro`` and
    filtered ``vehicle_angular_velocity``.

    Both timestamp arrays are absolute µs on the shared PX4 clock.  They are put
    on a COMMON time origin (``min`` of the two first samples) before resampling,
    so the true start offset between the two streams is preserved — otherwise any
    offset between the topics' first timestamps leaks straight into the estimated
    group delay.
    """
    if gyro_raw_xyz is None or len(gyro_raw_xyz) < 512:
        return None
    if t_us_filt is None or gyro_filt_xyz is None or len(t_us_filt) < 100:
        return None

    try:
        # Shared absolute origin — keeps the raw↔filtered start offset intact.
        t0 = min(float(t_us_raw[0]), float(t_us_filt[0]))
        t_raw_s  = (np.asarray(t_us_raw,  dtype=float) - t0) / 1e6
        t_filt_s = (np.asarray(t_us_filt, dtype=float) - t0) / 1e6

        t_raw_uni,  raw_uni,  fs_raw  = _uniform(t_raw_s,  _clean(gyro_raw_xyz[:, axis_idx]))
        t_filt_uni, filt_uni, fs_filt = _uniform(t_filt_s, _clean(gyro_filt_xyz[:, axis_idx]))

        # Interpolate raw onto the filtered stream's (offset-preserving) time axis.
        f_interp = interp1d(t_raw_uni, raw_uni, kind="linear",
                             bounds_error=False,
                             fill_value=(raw_uni[0], raw_uni[-1]))
        raw_on_filt = f_interp(t_filt_uni)
        n = min(len(raw_on_filt), len(filt_uni))
        raw_on_filt = raw_on_filt[:n]; filt_aligned = filt_uni[:n]

        nperseg = min(1024, n // 4)
        if nperseg < 8:
            return None
        f, Pyx = sig.csd(raw_on_filt, filt_aligned, fs=fs_filt, nperseg=nperseg)
        _, Pxx = sig.welch(raw_on_filt, fs=fs_filt, nperseg=nperseg)
        H = Pyx / (Pxx + 1e-14)

        phase_rad = np.unwrap(np.angle(H))
        # Group delay: -d(phase)/d(omega)
        dw = 2.0 * np.pi * np.diff(f)
        dph = np.diff(phase_rad)
        gd_s = -dph / (dw + 1e-12)
        # Median group delay at 1–30 Hz range
        mask = (f[1:] >= 1.0) & (f[1:] <= 30.0)
        delay_ms = float(np.median(gd_s[mask])) * 1000.0 if np.any(mask) else 0.0
        delay_ms = max(0.0, delay_ms)

        ph_10 = float(np.interp(10.0, f, np.degrees(phase_rad)))

        # np.interp clamps silently, so a phase read at 50 Hz is meaningless once
        # 50 Hz approaches either stream's Nyquist — report None instead.
        nyq = 0.5 * min(fs_raw, fs_filt)
        ph_50: Optional[float] = (None if 50.0 > 0.8 * nyq
                                  else round(float(np.interp(50.0, f, np.degrees(phase_rad))), 1))

        conf = "medium" if n >= 2048 else "low"
        return FilterCharacteristic(
            axis=axis, delay_ms=round(delay_ms, 2),
            phase_deg_at_10hz=round(ph_10, 1),
            phase_deg_at_50hz=ph_50,
            raw_rate_hz=round(fs_raw, 1), filtered_rate_hz=round(fs_filt, 1),
            confidence=conf,
        )
    except Exception as e:
        log.warning(f"filter_characterization failed: {e}")
        return None


# ── (f) ARX identification ────────────────────────────────────────────────────

def _arx_freerun_vaf(
    a: np.ndarray, b: np.ndarray, u: np.ndarray, y: np.ndarray,
    start: int, end: int, na: int, nb: int, nk: int,
) -> Tuple[float, float]:
    """
    Free-run (pure-simulation) VAF/NRMSE of an ARX model over ``[start, end)``.

    Unlike one-step-ahead prediction (which feeds the model the *measured* past
    outputs and is trivially ~99 % on any smooth gyro signal), the output lags
    here come from the model's OWN simulated ŷ; only the exogenous input ``u`` is
    measured.  Errors accumulate, so a model that merely tracks y[k]≈y[k-1] scores
    poorly — which is what makes the VAF a real discriminator.
    """
    max_lag = max(na, nb + nk - 1)
    L = end - start
    if L <= max_lag:
        return 0.0, 999.0
    yhat = y[start:end].astype(float).copy()     # seed first max_lag with truth
    for k in range(max_lag, L):
        idx = start + k
        acc = 0.0
        for i in range(1, na + 1):
            acc -= a[i - 1] * yhat[k - i]
        for j in range(nb):
            acc += b[j] * u[idx - nk - j]
        if not math.isfinite(acc) or abs(acc) > 1e12:
            return 0.0, 999.0                    # simulation diverged → useless
        yhat[k] = acc
    y_true = y[start + max_lag: end]
    y_sim  = yhat[max_lag:]
    if not np.all(np.isfinite(y_sim)):
        return 0.0, 999.0
    err = y_true - y_sim
    vaf = float(max(0.0, 1.0 - np.var(err) / (np.var(y_true) + 1e-12)) * 100.0)
    nrmse = float(np.sqrt(np.mean(err ** 2)) / (np.std(y_true) + 1e-12))
    return vaf, nrmse


def arx_fit(
    t: np.ndarray, sp: np.ndarray, meas: np.ndarray,
    axis: str, na: int = 3, nb: int = 3,
) -> ARXModel:
    """
    Fit ARX model: y[k] = -a1*y[k-1] - ... - a_na*y[k-na]
                        + b1*u[k-nk] + ... + b_nb*u[k-nk-nb+1]

    The dead time ``nk`` ∈ {1..5} is selected by **free-run** validation VAF on
    the held-out tail (not one-step-ahead — see ``_arx_freerun_vaf``).
    """
    # ARX assumes a fixed sample step, so resample onto a uniform grid at the
    # true rate and identify on the longest contiguous segment.
    segments, _ = _uniform_segments(t, sp, meas)
    if segments:
        sp, meas = max(segments, key=lambda s: len(s[0]))
    else:
        sp = _clean(sp); meas = _clean(meas)
    n = min(len(sp), len(meas))
    sp = np.asarray(sp[:n], dtype=float)
    meas = np.asarray(meas[:n], dtype=float)

    if n < max(na, nb + 5) + 50:
        return ARXModel(axis=axis, na=na, nb=nb,
                        coeffs_a=[], coeffs_b=[], vaf_pct=0.0, nrmse=999.0,
                        confidence="low", note="Too few samples for ARX.")

    N_fit = int(n * (1 - ARX_VAL_FRACTION))
    N_val = n - N_fit

    def _build_phi(start, end, nk):
        max_lag = max(na, nb + nk - 1)
        rows, tgt = [], []
        for k in range(max_lag, end - start):
            idx = start + k
            row = [-meas[idx - i] for i in range(1, na + 1)]
            row += [sp[idx - nk - j] for j in range(nb)]
            rows.append(row)
            tgt.append(meas[idx])
        return np.array(rows), np.array(tgt)

    # Sweep the dead time and keep the structure with the best free-run VAF.
    best = None   # (vaf, nrmse, theta, nk)
    for nk in range(1, 6):
        Phi_fit, Y_fit = _build_phi(0, N_fit, nk)
        if Phi_fit.size == 0 or len(Y_fit) < na + nb + 5:
            continue
        try:
            theta, _, _, _ = np.linalg.lstsq(Phi_fit, Y_fit, rcond=None)
        except Exception:
            continue
        a = theta[:na]; b = theta[na:na + nb]
        vaf, nrmse = _arx_freerun_vaf(a, b, sp, meas, N_fit, n, na, nb, nk)
        if best is None or vaf > best[0]:
            best = (vaf, nrmse, theta, nk)

    if best is None:
        return ARXModel(axis=axis, na=na, nb=nb,
                        coeffs_a=[], coeffs_b=[], vaf_pct=0.0, nrmse=999.0,
                        confidence="low", note="ARX least-squares failed.")

    vaf, nrmse, theta, nk = best
    a = list(theta[:na])
    b = list(theta[na:na + nb])

    if vaf >= MIN_ARX_GOOD_VAF:
        conf = "high" if vaf >= 80 else "medium"
        note = f"Free-run VAF={vaf:.1f}% on held-out {N_val} samples (nk={nk})."
    else:
        conf = "low"
        note = (f"Free-run VAF={vaf:.1f}% is below {MIN_ARX_GOOD_VAF}% threshold "
                f"(nk={nk}). Poor fit — excitation or noise may be limiting.")

    return ARXModel(axis=axis, na=na, nb=nb,
                    coeffs_a=[round(v, 5) for v in a],
                    coeffs_b=[round(v, 5) for v in b],
                    vaf_pct=round(vaf, 1), nrmse=round(nrmse, 3),
                    confidence=conf, note=note)


# ── (g) Actuator saturation ───────────────────────────────────────────────────

def detect_actuator_saturation(
    t: np.ndarray, motor_outputs: np.ndarray,
    axis: str = "all", mode: str = "all",
    sat_lo: float = 0.02, sat_hi: float = 0.98,
    min_duration_s: float = 0.1,
) -> ActuatorSaturation:
    """
    Find segments where any motor output is clipped at 0 or 1.
    """
    if motor_outputs is None or len(motor_outputs) == 0:
        return ActuatorSaturation(axis=axis, mode=mode, sat_fraction_pct=0.0,
                                   affected_segments=[])
    clipped = np.any(
        (motor_outputs <= sat_lo) | (motor_outputs >= sat_hi), axis=1
    )
    sat_pct = float(np.mean(clipped) * 100.0)

    # Find continuous saturated segments
    segs: List[Tuple[float, float]] = []
    in_seg = False
    t_seg_start = 0.0
    for i, c in enumerate(clipped):
        if c and not in_seg:
            in_seg = True
            t_seg_start = float(t[i])
        elif not c and in_seg:
            in_seg = False
            dur = float(t[i]) - t_seg_start
            if dur >= min_duration_s:
                segs.append((round(t_seg_start, 3), round(float(t[i]), 3)))
    if in_seg and (float(t[-1]) - t_seg_start >= min_duration_s):
        segs.append((round(t_seg_start, 3), round(float(t[-1]), 3)))

    return ActuatorSaturation(axis=axis, mode=mode,
                               sat_fraction_pct=round(sat_pct, 1),
                               affected_segments=segs)


# ── (h) Excitation classification ─────────────────────────────────────────────

def classify_excitation(
    t: np.ndarray, sp: np.ndarray, fs: float,
) -> ExcitationInfo:
    """
    Classify setpoint signal as step-like, sweep-like, or mixed.
    Uses spectral flatness (Wiener entropy) as the primary metric.
    """
    sp = _clean(sp)
    if len(sp) < 64:
        return ExcitationInfo("unknown", "poor", 0.0, 0.0,
                               "Too short to classify excitation.")

    f, psd = sig.welch(sp, fs=fs, nperseg=min(512, len(sp) // 2))
    mask = f > 0.3
    psd_f = psd[mask] + 1e-12

    # Spectral flatness = geometric_mean / arithmetic_mean ∈ [0, 1]
    # 1 = white noise (ideal sweep); 0 = pure sinusoid/step
    log_mean = float(np.mean(np.log(psd_f)))
    arith_mean = float(np.mean(psd_f))
    flatness = float(np.exp(log_mean) / arith_mean)

    # -3 dB bandwidth
    cumulative = np.cumsum(psd)
    total = cumulative[-1]
    f_lo_idx = int(np.searchsorted(cumulative, 0.1 * total))
    f_hi_idx = int(np.searchsorted(cumulative, 0.9 * total))
    bw = float(f[min(f_hi_idx, len(f) - 1)] - f[max(f_lo_idx, 0)])

    rms = float(np.sqrt(np.mean(sp ** 2)))

    # Classification heuristics
    if flatness > 0.20 and bw > 5.0:
        exc_type = "sweep"
        quality = "good" if flatness > 0.35 else "marginal"
        note = (f"Sweep-like excitation detected (flatness={flatness:.3f}). "
                "FRF / stability-margin algorithms are available.")
    elif bw < 2.0 or rms < 0.02:
        exc_type = "step"
        quality = "marginal" if rms > 0.01 else "poor"
        note = (f"Step-like excitation (flatness={flatness:.3f}). "
                "Step-response metrics are reliable. "
                "FRF/margins require chirp sweep — see README for the sweep maneuver.")
    else:
        exc_type = "mixed"
        quality = "marginal"
        note = (f"Mixed excitation (flatness={flatness:.3f}). "
                "Step metrics are reliable; FRF may have limited coherence.")

    return ExcitationInfo(exc_type, quality, round(bw, 2), round(rms, 4), note)


# ── orchestrator ──────────────────────────────────────────────────────────────

class HiFiAnalyzer:
    """
    Runs all high-rate algorithms on data from a UlogParseResult.
    Activated only when capabilities allow; degrades gracefully otherwise.
    """

    def analyse(self, parse_result: "UlogParseResult") -> HiFiResult:
        from .ulog_adapter import UlogParseResult

        meta = parse_result.metadata
        caps = meta.capabilities
        frames = parse_result.frames
        warnings: List[str] = list(parse_result.warnings)

        prim_stats = meta.topic_stats.get("vehicle_angular_velocity")
        rate_hz  = prim_stats.achieved_rate_hz if prim_stats else 0.0
        f_nyq    = prim_stats.f_nyquist_hz if prim_stats else 0.0

        result = HiFiResult(
            source_name=meta.source_name,
            duration_s=meta.duration_s,
            primary_rate_hz=rate_hz,
            raw_gyro_rate_hz=parse_result.raw_gyro_rate_hz,
        )

        if not frames:
            result.warnings.append("No frames — cannot run HiFi analysis.")
            return result

        # Build numpy arrays from frames
        t_arr  = np.array([f.t for f in frames])
        fs_eff = rate_hz if rate_hz > 0 else float(len(t_arr)) / max(1.0, meta.duration_s)

        axes_cfg = [
            ("roll",  "roll_rate_sp",  "roll_rate",  "torque_roll"),
            ("pitch", "pitch_rate_sp", "pitch_rate", "torque_pitch"),
            ("yaw",   "yaw_rate_sp",   "yaw_rate",   "torque_yaw"),
        ]

        # ── Actuator saturation (always if data present) ──────────────────────
        m_arrs = []
        for mn in ("m0", "m1", "m2", "m3"):
            arr = np.array([getattr(f, mn) or np.nan for f in frames])
            if not np.all(np.isnan(arr)):
                m_arrs.append(arr)
        if m_arrs:
            M = np.column_stack([np.nan_to_num(a) for a in m_arrs])
            sat = detect_actuator_saturation(t_arr, M)
            result.saturation.append(sat)
            if sat.sat_fraction_pct > 5.0:
                warnings.append(
                    f"Actuator saturation {sat.sat_fraction_pct:.1f}% of samples — "
                    "segments with clipping have unreliable ID data."
                )

        if not caps.freq_domain_available:
            result.warnings = warnings
            result.warnings.append(
                f"Primary rate {rate_hz:.1f} Hz < {MIN_RATE_FOR_FRF_HZ} Hz — "
                "FRF/margins/delay disabled. Use ULog from Ethernet DDS or native logging."
            )
            # Still do vibration if raw gyro is available
            if caps.vibration_available and parse_result.raw_gyro_xyz is not None:
                result.vibration = vibration_analysis(
                    parse_result.raw_gyro_t, parse_result.raw_gyro_xyz,
                    parse_result.esc_rpm, parse_result.esc_t)
            return result

        # ── Excitation classification ─────────────────────────────────────────
        roll_sp_arr = np.array([f.roll_rate_sp or 0.0 for f in frames])
        exc = classify_excitation(t_arr, roll_sp_arr, fs_eff)
        result.excitation = exc
        meta.capabilities.excitation_type = exc.excitation_type
        meta.capabilities.excitation_quality = exc.quality

        for axis, sp_field, meas_field, _ in axes_cfg:
            # Keep missing samples as NaN (do NOT zero-fill — zeros inject
            # spectral steps).  Downstream resampling interpolates/segments them.
            sp_arr   = _column(frames, sp_field)
            meas_arr = _column(frames, meas_field)

            has_sp = bool(np.any(np.isfinite(sp_arr) & (sp_arr != 0.0)))
            if not has_sp:
                continue

            nperseg = min(1024, len(sp_arr) // 4)
            nperseg = max(64, nperseg)

            # FRF. Setpoint-based H1 by default; if the excitation node logged the
            # exact injected reference r(t) for this axis, prefer the
            # unbiased reference-based Ĝ=Φ_ry/Φ_ru — same u→y transfer without the
            # closed-loop bias. r is masked to this axis' inject window.
            frf = compute_frf(t_arr, sp_arr, meas_arr, fs_eff,
                               axis, "rate", f_nyq, nperseg=nperseg)
            # Segment by run_tag: only the frames of ONE excitation run feed the
            # unbiased FRF (never merge two runs). Pick the largest run for this axis.
            run_tag, n_runs = _dominant_exc_run(frames, axis)
            if run_tag is not None:
                if n_runs > 1:
                    warnings.append(
                        f"{axis}: {n_runs} excitation runs found — using the largest "
                        f"(run_tag={run_tag}); analyze others separately.")
                r_axis = np.array([
                    (f.exc_ref if (getattr(f, "exc_ref", None) is not None
                                   and getattr(f, "exc_axis", None) == axis
                                   and getattr(f, "exc_phase", None) == "inject"
                                   and getattr(f, "exc_run_tag", None) == run_tag)
                     else np.nan)
                    for f in frames
                ], dtype=float)
                frf_ref = compute_frf_reference(
                    t_arr, r_axis, sp_arr, meas_arr, fs_eff,
                    axis, "rate", f_nyq, nperseg=nperseg)
                if frf_ref is not None:
                    frf = frf_ref  # unbiased estimate wins
            if frf:
                result.frf.append(frf)

                # Margins (only for sweep excitation with decent coherence).
                # Pass the logged PX4 gains so the open loop uses the 2-DOF
                # formula (D on the measurement) rather than L = T/(1−T).
                if exc.excitation_type in ("sweep", "mixed"):
                    margins = stability_margins_from_frf(
                        frf, axis, params=getattr(meta, "params", None))
                    result.margins.append(margins)
                else:
                    result.margins.append(StabilityMargins(
                        axis=axis, gain_margin_db=None, phase_margin_deg=None,
                        gain_crossover_hz=None, phase_crossover_hz=None,
                        confidence="insufficient",
                        note="Step excitation — use chirp sweep for stability margins.",
                    ))

                # Loop delay
                delay = estimate_loop_delay(t_arr, sp_arr, meas_arr, fs_eff,
                                             axis, frf)
                result.delays.append(delay)

            # ARX
            if caps.sysid_hifi_available and len(sp_arr) > 200:
                arx = arx_fit(t_arr, sp_arr, meas_arr, axis)
                result.arx_models.append(arx)

        # ── Vibration analysis ─────────────────────────────────────────────────
        if caps.vibration_available and parse_result.raw_gyro_xyz is not None:
            result.vibration = vibration_analysis(
                parse_result.raw_gyro_t, parse_result.raw_gyro_xyz,
                parse_result.esc_rpm, parse_result.esc_t)

            # Filter characterisation: raw sensor_gyro vs the filtered
            # vehicle_angular_velocity, both on their native µs timestamps (shared
            # PX4 clock) so the true start offset is preserved rather than the
            # 50 Hz frame grid.
            if (parse_result.raw_angvel_t is not None
                    and parse_result.raw_angvel_xyz is not None):
                fc = filter_characterization(
                    parse_result.raw_gyro_t, parse_result.raw_gyro_xyz,
                    parse_result.raw_angvel_t, parse_result.raw_angvel_xyz,
                    axis_idx=2, axis="z")
                if fc:
                    result.filter_chars.append(fc)

        result.warnings = warnings
        return result
