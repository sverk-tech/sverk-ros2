"""
Offline reproduction of the PX4 attitude-rate autotune pipeline.

This module runs — offline, on a recorded ULog — the same math that PX4's
``mc_autotune_attitude_control`` module runs onboard when ``MC_AT_EN`` is set:

  1. Closed-loop identification of the torque→rate plant as an ARX(2,2,1)
     model (``system_identification`` library in PX4).
  2. GMVC gain synthesis (Yamamoto/Fujii/Kaneda 1998), ported verbatim from
     PX4 ``pid_design.hpp``.
  3. The same ``areGainsGood()`` sanity gate and the same parameter mapping
     as ``saveGainsToParams()``.

Compared with the onboard autotune this offline path

  * uses a batch least-squares ARX fit instead of RLS (statistically no worse
    for an offline record; an optional RLS is provided for cross-checking);
  * validates the model with a free-run VAF on a held-out third of the data;
  * never writes to the FMU — the result is read-only advice.

It requires a *high-rate* log (``SDLOG_PROFILE=17``): both
``vehicle_angular_velocity`` and ``vehicle_torque_setpoint`` must be logged at
the full rate loop rate (~400 Hz), not the default 50 Hz. The adapter exposes
the two signals on their own timestamp bases (``raw_torque_*`` /
``raw_angvel_*``) so the torque input is not quantised to the 50 Hz frame grid.

Read-only — no FMU parameter writes.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import signal as sig

log = logging.getLogger(__name__)

# ── thresholds / defaults ──────────────────────────────────────────────────────
MIN_RATE_HZ = 100.0          # native angular-velocity rate below this → reject
MODEL_DT_TARGET_S = 0.004    # decimation target (PX4 _model_update_scaler band)
MODEL_DT_MIN_S = 0.002       # 2 ms
MODEL_DT_MAX_S = 0.010       # 10 ms
DEFAULT_GYRO_CUTOFF_HZ = 40.0  # IMU_GYRO_CUTOFF default
HPF_CUTOFF_HZ = 0.5          # PX4 autotune input high-pass
MIN_VAF_PCT = 60.0           # k-step VAF gate (model goodness) — see identify_arx221
KSTEP_HORIZON_S = 0.4        # reseed horizon for the k-step VAF (P1-6)
EXC_ACTIVE_FRAC = 0.20       # |u| above this fraction of the peak counts as excitation
EXC_MARGIN_S = 0.5           # keep this much settle either side of the active span
MIN_INPUT_RMS = 1e-4         # normalised-torque RMS below this → no excitation
GMVC_LAMBDA = 0.7            # control-effort penalty
GMVC_DELTA = 0.0             # 0 = critical damping

# Per-axis PX4 parameter naming + default desired rise time (sigma).
_AXES = {
    "roll":  {"tag": "ROLL",  "tq": 0, "sigma": 0.14},
    "pitch": {"tag": "PITCH", "tq": 1, "sigma": 0.14},
    "yaw":   {"tag": "YAW",   "tq": 2, "sigma": 0.20},
}


# ── result dataclass ────────────────────────────────────────────────────────────

@dataclass
class AutotuneAxisResult:
    axis: str
    accepted: bool = False
    rejected: bool = False
    reason: str = ""                 # why rejected / caveat
    # identification
    model_dt_s: float = 0.0
    coeffs: Dict[str, float] = field(default_factory=dict)      # a1 a2 b0 b1 b2
    coeff_std: Dict[str, float] = field(default_factory=dict)
    vaf_pct: float = 0.0             # k-step (segmented free-run) VAF — the gate metric
    vaf_freerun_pct: float = 0.0     # whole-tail free-run VAF (diagnostic; ~0 on integrators)
    kstep_horizon_s: float = 0.0     # simulation reseed horizon used for vaf_pct
    n_fit: int = 0
    n_val: int = 0
    input_rms: float = 0.0
    data_window_s: Tuple[float, float] = (0.0, 0.0)
    # Excitation window actually used for the fit (P1-5). When the log is mostly
    # idle, cropping to the active input span is what makes the ID trustworthy.
    exc_window_s: Tuple[float, float] = (0.0, 0.0)
    exc_window_src: str = "full"     # "full" | "input-activity"
    # synthesis
    sigma_s: float = 0.0
    kc: Optional[float] = None
    ki: Optional[float] = None
    kd: Optional[float] = None
    gate_pass: bool = False
    gate_reason: str = ""
    # PX4 parameter mapping: {param: {"current": x|None, "suggested": y}}
    params: Dict[str, Dict[str, Optional[float]]] = field(default_factory=dict)

    def to_dict(self) -> dict:
        import dataclasses
        return dataclasses.asdict(self)


@dataclass
class AutotuneResult:
    source_name: str
    method: str = "px4-offline (ARX batch + GMVC)"
    native_rate_hz: float = 0.0
    axes: List[AutotuneAxisResult] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "source_name": self.source_name,
            "method": self.method,
            "native_rate_hz": round(self.native_rate_hz, 1),
            "axes": [a.to_dict() for a in self.axes],
            "warnings": self.warnings,
        }


# ── signal conditioning ─────────────────────────────────────────────────────────

def _interp_uniform(t_us: np.ndarray, y: np.ndarray, t_grid_us: np.ndarray) -> np.ndarray:
    """Linear interpolation of (t_us, y) onto t_grid_us (µs), edge-held."""
    return np.interp(
        t_grid_us.astype(float), t_us.astype(float), np.asarray(y, dtype=float),
        left=float(y[0]), right=float(y[-1]),
    )


def _prefilter(x: np.ndarray, fs: float,
               lpf_hz: float = DEFAULT_GYRO_CUTOFF_HZ,
               hpf_hz: float = HPF_CUTOFF_HZ) -> np.ndarray:
    """
    PX4 autotune input conditioning applied identically to u and y:
    Butterworth-2 low-pass at IMU_GYRO_CUTOFF + 1st-order high-pass at 0.5 Hz.

    Zero-phase (forward-backward) filtering is used offline — because the *same*
    linear filter is applied to both signals, the identified plant G = Y/U is
    unbiased; the prefilter only reshapes the fit's frequency weighting (and
    strips DC/trim via the HPF), exactly as onboard.
    """
    x = np.asarray(x, dtype=float)
    nyq = fs * 0.5
    # Guard cutoffs against the Nyquist limit of the (decimated) grid.
    lpf = min(lpf_hz, 0.95 * nyq)
    out = x
    if lpf > 0 and lpf < nyq:
        sos_lp = sig.butter(2, lpf / nyq, btype="low", output="sos")
        out = sig.sosfiltfilt(sos_lp, out)
    if hpf_hz > 0 and hpf_hz < nyq:
        sos_hp = sig.butter(1, hpf_hz / nyq, btype="high", output="sos")
        out = sig.sosfiltfilt(sos_hp, out)
    return out


def _decimation_factor(dt_native_s: float) -> int:
    """Integer decimation so the model dt lands in [2, 10] ms, near 4 ms."""
    if dt_native_s <= 0:
        return 1
    q = int(round(MODEL_DT_TARGET_S / dt_native_s))
    q = max(1, q)
    # Clamp so dt stays within the allowed band.
    while q > 1 and dt_native_s * q > MODEL_DT_MAX_S:
        q -= 1
    while dt_native_s * (q + 1) <= MODEL_DT_MAX_S and dt_native_s * q < MODEL_DT_MIN_S:
        q += 1
    return max(1, q)


# ── (1) ARX(2,2,1) identification ───────────────────────────────────────────────

def _vaf(err: np.ndarray, y_true: np.ndarray) -> float:
    denom = float(np.var(y_true))
    if denom <= 1e-18:
        return 0.0
    return max(0.0, 1.0 - float(np.var(err)) / denom) * 100.0


def _segmented_freerun(
    u: np.ndarray, y: np.ndarray, theta: np.ndarray,
    start: int, n: int, max_lag: int, horizon: int,
) -> np.ndarray:
    """k-step-ahead (segmented free-run) simulation of the ARX model.

    The output is simulated open-loop within each ``horizon``-sample block, then the
    two-sample history is reseeded from the real signal at the next block boundary.
    Pure whole-tail free-run drifts to VAF≈0 on a near-integrator rate plant (a pole
    at z≈1 integrates any small model mismatch), so it rejected valid models even at
    one-step VAF=100% (P1-6). Bounding the simulation horizon measures genuine model
    fidelity over the excitation timescale without letting the integrator run away.
    """
    a1, a2, b0, b1, b2 = theta
    y_sim = y.copy()
    k = max(start, max_lag)
    while k < n:
        seg_end = min(k + horizon, n)
        # Reseed the two-sample history from ground truth at each block start.
        y_sim[k - 1] = y[k - 1]
        y_sim[k - 2] = y[k - 2]
        for j in range(k, seg_end):
            y_sim[j] = (-a1 * y_sim[j - 1] - a2 * y_sim[j - 2]
                        + b0 * u[j - 1] + b1 * u[j - 2] + b2 * u[j - 3])
        k = seg_end
    return y_sim


def identify_arx221(
    u: np.ndarray, y: np.ndarray, val_fraction: float = 1.0 / 3.0,
    horizon: int = 100,
) -> Tuple[np.ndarray, np.ndarray, float, float, int, int]:
    """
    Batch least-squares ARX(2,2,1):

        y[k] = -a1 y[k-1] - a2 y[k-2] + b0 u[k-1] + b1 u[k-2] + b2 u[k-3]

    theta = [a1, a2, b0, b1, b2].

    Returns ``(theta, theta_std, vaf_kstep, vaf_freerun, n_fit, n_val)``.
    ``theta_std`` are the coefficient standard deviations from (ΦᵀΦ)⁻¹σ².
    ``vaf_kstep`` is a k-step-ahead (segmented free-run, reseed every ``horizon``
    samples) VAF on the held-out tail — the trustworthy gate metric. ``vaf_freerun``
    is the legacy whole-tail free-run VAF, kept only as a diagnostic (it collapses to
    ~0 on the near-integrator rate plant even for a perfect model — the P1-6 bug).
    """
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    n = min(len(u), len(y))
    u, y = u[:n], y[:n]
    max_lag = 3
    if n < max_lag + 40:
        raise ValueError("too few samples for ARX(2,2,1)")

    n_fit = int(n * (1.0 - val_fraction))
    n_fit = max(max_lag + 20, min(n_fit, n - max_lag - 10))

    def _regressors(k: int) -> List[float]:
        return [-y[k - 1], -y[k - 2], u[k - 1], u[k - 2], u[k - 3]]

    # Fit on [max_lag, n_fit)
    Phi = np.array([_regressors(k) for k in range(max_lag, n_fit)])
    Y = y[max_lag:n_fit]
    theta, _, _, _ = np.linalg.lstsq(Phi, Y, rcond=None)

    # Coefficient covariance from residual variance.
    resid = Y - Phi @ theta
    dof = max(1, Phi.shape[0] - Phi.shape[1])
    sigma2 = float(resid @ resid) / dof
    try:
        cov = np.linalg.inv(Phi.T @ Phi) * sigma2
        theta_std = np.sqrt(np.clip(np.diag(cov), 0.0, np.inf))
    except np.linalg.LinAlgError:
        theta_std = np.full(5, np.nan)

    val_start = n_fit
    y_true_v = y[val_start:n]

    # k-step VAF (gate metric) — reseed horizon bounded to the excitation timescale.
    horizon = max(10, int(horizon))
    y_kstep = _segmented_freerun(u, y, theta, val_start, n, max_lag, horizon)
    vaf_kstep = _vaf(y_true_v - y_kstep[val_start:n], y_true_v)

    # Whole-tail free-run VAF (diagnostic only).
    y_free = _segmented_freerun(u, y, theta, val_start, n, max_lag, n)
    vaf_free = _vaf(y_true_v - y_free[val_start:n], y_true_v)

    return (theta, theta_std, float(vaf_kstep), float(vaf_free),
            int(n_fit - max_lag), int(n - val_start))


# ── (2) GMVC synthesis (ported from PX4 pid_design.hpp) ─────────────────────────

def synthesize_gmvc(
    a1: float, a2: float, b0: float, b1: float, b2: float,
    dt: float, sigma: float,
    lbda: float = GMVC_LAMBDA, delta: float = GMVC_DELTA,
) -> Tuple[float, float, float]:
    """
    Generalised Minimum Variance Control gain synthesis, verbatim from PX4
    ``pid_design.hpp`` (getGains). Returns (kc, ki, kd).

    ``sigma`` is the desired closed-loop rise time; ``lbda`` the control-effort
    penalty; ``delta`` the damping selector (0 = critical).
    """
    rho = dt / sigma
    mu = 0.25 * (1.0 - delta) + 0.51 * delta
    p1 = -2.0 * math.exp(-rho / (2.0 * mu)) * math.cos(
        math.sqrt(4.0 * mu - 1.0) * rho / (2.0 * mu))
    p2 = math.exp(-rho / mu)

    e1 = -a1 + p1 + 1.0
    f0 = -a1 * e1 + a1 - a2 + e1 + p2
    f1 = a1 * e1 - a2 * e1 + a2
    f2 = a2 * e1

    nu = lbda + (e1 + 1.0) * (b0 + b1 + b2)
    denom = f1 + 2.0 * f2
    kc = -(f1 + 2.0 * f2) / nu
    ki = -(f0 + f1 + f2) / (dt * denom) / 5.0   # PX4 deliberately weakens I ×5
    kd = -dt * f2 / denom
    return float(kc), float(ki), float(kd)


# ── (3) gate + mapping (ported from areGainsGood / saveGainsToParams) ───────────

def _att_p(kc: float) -> float:
    return float(np.clip(1.0 / (math.radians(60.0) * kc), 2.0, 6.5)) if kc > 0 else 12.0


def are_gains_good(kc: float, ki: float, kd: float) -> Tuple[bool, str]:
    """
    PX4 ``areGainsGood`` sanity gate. the magnitude bounds are
    applied to the mapped rate params (kc·ki, kc·kd), not to the raw kid vector.
    """
    vals = [kc, ki, kd]
    if not all(math.isfinite(v) for v in vals):
        return False, "non-finite gains"
    if not all(v > 0.0 for v in vals):
        return False, "gains must all be positive"
    att_p = _att_p(kc)
    checks = [
        (kc < 0.5, f"kc={kc:.3f} ≥ 0.5"),
        (kc * ki < 10.0, f"kc·ki={kc*ki:.3f} ≥ 10"),
        (kc * kd < 0.1, f"kc·kd={kc*kd:.4f} ≥ 0.1"),
        (att_p < 12.0, f"att_p={att_p:.2f} ≥ 12"),
    ]
    for ok, msg in checks:
        if not ok:
            return False, msg
    return True, ""


def map_to_px4_params(
    axis: str, kc: float, ki: float, kd: float,
    current: Optional[Dict[str, float]] = None,
) -> Dict[str, Dict[str, Optional[float]]]:
    """PX4 ``saveGainsToParams`` mapping → {param: {current, suggested}}."""
    current = current or {}
    tag = _AXES[axis]["tag"]
    suggested = {
        f"MC_{tag}RATE_P": kc,
        f"MC_{tag}RATE_K": 1.0,
        f"MC_{tag}RATE_I": kc * ki,
        f"MC_{tag}RATE_D": kc * kd,
        f"MC_{tag}_P": _att_p(kc),
    }
    out: Dict[str, Dict[str, Optional[float]]] = {}
    for p, val in suggested.items():
        cur = current.get(p)
        out[p] = {
            "current": round(float(cur), 5) if cur is not None else None,
            "suggested": round(float(val), 5),
        }
    return out


def _active_window(u: np.ndarray, frac: float, margin: int) -> Tuple[int, int]:
    """Index span [lo, hi) that contains the excitation, cropping idle segments.

    Offline ULogs carry no exc_ref (the excitation reference is a ROS topic, not a
    uORB message), so the excitation window is recovered from input activity: the
    contiguous span where |u| exceeds ``frac`` of its peak, padded by ``margin``
    samples of settle either side. Averaging the ARX fit / FRF over a mostly-idle
    log sinks VAF and coherence (P1-5); cropping to this span is what makes the
    identification trustworthy. Returns the full range when no clear burst exists.
    """
    a = np.abs(np.asarray(u, dtype=float))
    n = len(a)
    peak = float(np.max(a)) if n else 0.0
    if peak <= 0.0:
        return 0, n
    active = np.where(a > frac * peak)[0]
    if len(active) == 0:
        return 0, n
    lo = max(0, int(active[0]) - margin)
    hi = min(n, int(active[-1]) + 1 + margin)
    return lo, hi


# ── orchestrator ────────────────────────────────────────────────────────────────

class OfflineAutotune:
    """Runs the offline PX4 autotune pipeline per axis on a UlogParseResult."""

    def run(self, parse_result) -> AutotuneResult:
        meta = parse_result.metadata
        params = dict(meta.params or {})
        res = AutotuneResult(source_name=meta.source_name)

        t_tq = parse_result.raw_torque_t
        u_tq = parse_result.raw_torque_xyz
        t_av = parse_result.raw_angvel_t
        y_av = parse_result.raw_angvel_xyz
        res.native_rate_hz = float(parse_result.raw_angvel_rate_hz or 0.0)

        if t_tq is None or u_tq is None or t_av is None or y_av is None:
            res.warnings.append(
                "Missing vehicle_torque_setpoint or vehicle_angular_velocity "
                "high-rate arrays — cannot run offline autotune.")
            return res

        if res.native_rate_hz < MIN_RATE_HZ:
            res.warnings.append(
                f"Angular-velocity rate {res.native_rate_hz:.0f} Hz < {MIN_RATE_HZ:.0f} Hz. "
                "Offline autotune needs a high-rate log — set SDLOG_PROFILE=17 "
                "(default | high-rate) and reboot.")
            return res

        if not params:
            res.warnings.append(
                "No PID parameters loaded — cannot normalise the torque input "
                "(u = torque / (P·K)). Upload a .params file or fetch live params.")
            return res

        for axis, cfg in _AXES.items():
            res.axes.append(self._run_axis(axis, cfg, t_tq, u_tq, t_av, y_av, params))
        return res

    def _run_axis(self, axis, cfg, t_tq, u_tq, t_av, y_av, params) -> AutotuneAxisResult:
        out = AutotuneAxisResult(axis=axis, sigma_s=float(cfg["sigma"]))
        tag = cfg["tag"]
        tq_idx = cfg["tq"]

        P = params.get(f"MC_{tag}RATE_P")
        K = params.get(f"MC_{tag}RATE_K")
        if P is None or K is None or (P * K) == 0.0:
            out.rejected = True
            out.reason = f"MC_{tag}RATE_P / MC_{tag}RATE_K missing or zero — cannot normalise input."
            return out

        # ── build the common uniform grid on the angular-velocity timebase ────
        y_raw = np.asarray(y_av[:, tq_idx], dtype=float)
        t0 = max(int(t_tq[0]), int(t_av[0]))
        t1 = min(int(t_tq[-1]), int(t_av[-1]))
        if t1 - t0 < 2_000_000:   # < 2 s of overlap
            out.rejected = True
            out.reason = "less than 2 s of overlapping torque/rate data."
            return out

        mask = (t_av >= t0) & (t_av <= t1)
        t_grid = t_av[mask]
        if len(t_grid) < 200:
            out.rejected = True
            out.reason = "too few overlapping samples."
            return out
        dt_native = float(np.median(np.diff(t_grid))) / 1e6
        if dt_native <= 0:
            out.rejected = True
            out.reason = "non-monotonic timestamps."
            return out

        y = y_raw[mask]
        # Torque interpolated onto the rate grid (linear, not forward-fill).
        u_raw = _interp_uniform(t_tq, u_tq[:, tq_idx], t_grid)

        # ── (1) normalise as PX4: u = torque / (P·K) ──────────────────────────
        u = u_raw / (P * K)

        # ── (2) prefilter both identically at the native rate ────────────────
        fs_native = 1.0 / dt_native
        gyro_cut = float(params.get("IMU_GYRO_CUTOFF", DEFAULT_GYRO_CUTOFF_HZ) or DEFAULT_GYRO_CUTOFF_HZ)
        try:
            u_f = _prefilter(u, fs_native, lpf_hz=gyro_cut)
            y_f = _prefilter(y, fs_native, lpf_hz=gyro_cut)
        except Exception as e:
            out.rejected = True
            out.reason = f"prefilter failed: {e}"
            return out

        # ── (3) decimate to model dt ──────────────────────────────────────────
        q = _decimation_factor(dt_native)
        u_d = u_f[::q]
        y_d = y_f[::q]
        model_dt = dt_native * q
        out.model_dt_s = round(model_dt, 5)
        out.data_window_s = (round((t0 - int(t_av[0])) / 1e6, 2),
                             round((t1 - int(t_av[0])) / 1e6, 2))

        # ── (3b) crop to the excitation window (P1-5) ─────────────────────────
        # Restrict the fit to where the axis is actually excited; idle hover before
        # and after the burst only dilutes conditioning and sinks VAF.
        margin = max(5, int(round(EXC_MARGIN_S / model_dt)))
        n_full = len(u_d)
        lo, hi = _active_window(u_d, EXC_ACTIVE_FRAC, margin)
        if (hi - lo) >= 43 and (lo > 0 or hi < n_full):   # ARX(2,2,1) needs ≥ max_lag+40
            u_d = u_d[lo:hi]
            y_d = y_d[lo:hi]
            out.exc_window_src = "input-activity"
            w0 = out.data_window_s[0] + lo * model_dt
            out.exc_window_s = (round(w0, 2), round(w0 + (hi - lo) * model_dt, 2))
        else:
            out.exc_window_s = out.data_window_s
            out.exc_window_src = "full"

        # excitation energy check (on the cropped, normalised, decimated input)
        input_rms = float(np.sqrt(np.mean(u_d ** 2)))
        out.input_rms = round(input_rms, 6)
        if input_rms < MIN_INPUT_RMS:
            out.rejected = True
            out.reason = (
                f"input excitation too weak (RMS={input_rms:.2e}). Fly step/sweep "
                "commands on this axis (≥ 20 s with active excitation).")
            return out

        # ── (4) ARX identification + k-step VAF ───────────────────────────────
        horizon = max(10, int(round(KSTEP_HORIZON_S / model_dt)))
        out.kstep_horizon_s = round(horizon * model_dt, 3)
        try:
            theta, theta_std, vaf, vaf_free, n_fit, n_val = identify_arx221(
                u_d, y_d, horizon=horizon)
        except Exception as e:
            out.rejected = True
            out.reason = f"ARX identification failed: {e}"
            return out
        out.vaf_freerun_pct = round(vaf_free, 1)

        a1, a2, b0, b1, b2 = [float(v) for v in theta]
        names = ["a1", "a2", "b0", "b1", "b2"]
        out.coeffs = {k: round(v, 6) for k, v in zip(names, theta)}
        out.coeff_std = {k: (round(float(v), 6) if math.isfinite(v) else None)
                         for k, v in zip(names, theta_std)}
        out.vaf_pct = round(vaf, 1)
        out.n_fit = n_fit
        out.n_val = n_val

        if vaf < MIN_VAF_PCT:
            out.rejected = True
            out.reason = (
                f"model {out.kstep_horizon_s:.2f}s-horizon VAF={vaf:.1f}% "
                f"< {MIN_VAF_PCT:.0f}% — identification not trustworthy. Need richer "
                "excitation (steps/sweep) on this axis.")
            return out

        # ── (5) GMVC synthesis + gate + mapping ──────────────────────────────
        try:
            kc, ki, kd = synthesize_gmvc(a1, a2, b0, b1, b2, model_dt, float(cfg["sigma"]))
        except (ValueError, ZeroDivisionError) as e:
            out.rejected = True
            out.reason = f"GMVC synthesis failed: {e}"
            return out
        out.kc, out.ki, out.kd = round(kc, 6), round(ki, 6), round(kd, 6)

        good, gate_reason = are_gains_good(kc, ki, kd)
        out.gate_pass = good
        out.gate_reason = gate_reason
        out.params = map_to_px4_params(axis, kc, ki, kd, current=params)

        if not good:
            out.rejected = True
            out.reason = f"synthesized gains rejected by sanity gate: {gate_reason}"
            return out

        out.accepted = True
        return out


def run_offline_autotune(parse_result) -> AutotuneResult:
    """Top-level convenience entry point for the web server."""
    return OfflineAutotune().run(parse_result)


# ── gains from a native-autotune ARX coeff vector ───────────────────

def gains_from_autotune_coeffs(
    axis: str,
    coeff: List[float],
    model_dt: Optional[float],
    sigma: Optional[float] = None,
    current: Optional[Dict[str, float]] = None,
) -> Optional[Dict]:
    """Reproduce the gains PX4 would compute from a converged autotune coeff vector.

    ``coeff = [a1, a2, b0, b1, b2]`` is the ARX(2,2,1) estimate published on
    ``autotune_attitude_control_status``.  This runs the SAME GMVC
    synthesis + ``areGainsGood`` gate + parameter mapping the onboard module runs
    in ``saveGainsToParams()``, so the log's identified plant yields the same
    MC_xRATE_* / MC_x_P values the firmware would have applied.

    Returns ``{"kc","ki","kd","gate_pass","gate_reason","params":{...}}`` or None
    when ``model_dt`` is unknown (GMVC needs the sample time) or synthesis fails.
    ``sigma`` defaults to the PX4 desired rise time (0.14 s roll/pitch, 0.20 s yaw).
    """
    if axis not in _AXES:
        return None
    if model_dt is None or not (model_dt > 0):
        return None
    if coeff is None or len(coeff) < 5:
        return None
    a1, a2, b0, b1, b2 = [float(v) for v in coeff[:5]]
    if not all(math.isfinite(v) for v in (a1, a2, b0, b1, b2)):
        return None
    sig_s = float(sigma) if (sigma and sigma > 0) else float(_AXES[axis]["sigma"])
    try:
        kc, ki, kd = synthesize_gmvc(a1, a2, b0, b1, b2, float(model_dt), sig_s)
    except (ValueError, ZeroDivisionError, FloatingPointError):
        return None
    if not all(math.isfinite(v) for v in (kc, ki, kd)):
        return None
    good, reason = are_gains_good(kc, ki, kd)
    return {
        "kc": round(kc, 6),
        "ki": round(ki, 6),
        "kd": round(kd, 6),
        "sigma_s": round(sig_s, 4),
        "gate_pass": good,
        "gate_reason": reason,
        "params": map_to_px4_params(axis, kc, ki, kd, current=current),
    }
