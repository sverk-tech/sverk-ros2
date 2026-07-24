"""
Plant (airframe) recovery and model-based PID synthesis.

Pipeline
--------
1. Build the 2-DOF PX4 controller: C_pi(jw) on the error, C_d(jw) on the
   measurement (D-on-measurement, 2nd-order Butterworth filter).
2. From closed-loop FRF T(jw), recover the open-loop plant with the 2-DOF
   formula:
     G(jw) = T(jw) / (C_pi(jw) * (1 - T(jw)) - T(jw) * C_d(jw))
3. Fit a parametric model to G(jw): 2nd-order+delay or integrator+pole+delay.
4. From G model, synthesize new rate PID gains targeting desired PM and bandwidth.
5. Simulate closed-loop response with current and suggested gains
   → before/after step-response predictions.

All suggested gains are heuristic starting points.  Apply one at a time.
Read-only — no FMU parameter writes.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize
from scipy import signal as sig

from .hifi_analyzer import FRFResult, COH_THRESHOLD

log = logging.getLogger(__name__)


# ── PX4 rate controller TF (2-DOF) ────────────────────────────────────────────
#
# PX4 is a two-degree-of-freedom controller: u = C_pi·(r − y) − C_d·y, where the
# proportional + integral part acts on the error and the derivative part acts on
# the MEASUREMENT only (angular_accel), filtered by a 2nd-order Butterworth at
# IMU_DGYRO_CUTOFF.  The consequences (see 01_CONTEXT.md):
#     T = G·C_pi / (1 + G·(C_pi + C_d))          closed loop r → y
#     L = G·(C_pi + C_d)                          open loop
#     G = T / (C_pi·(1 − T) − T·C_d)              plant recovery
# so L ≠ T/(1−T) whenever D ≠ 0.

D_CUTOFF_DEFAULT_HZ = 30.0   # PX4 IMU_DGYRO_CUTOFF default
INT_LIM_DEFAULT = 0.30       # PX4 MC_RR_INT_LIM default (rate integrator clamp)

# PX4 autotune sanity gate (areGainsGood): effective K·gain limits.
GATE_KP_MAX = 0.5
GATE_KI_MAX = 10.0
GATE_KD_MAX = 0.1
# Loop-shaping safety limits used in synthesis.
MS_MAX = 1.7        # peak sensitivity max|1/(1+L)|
HF_DGAIN_MAX = 0.5  # high-frequency D-path gain K·Kd·wd (motor-noise guard)
GAIN_CLAMP_FRAC = 0.30  # each suggested gain within ±30 % of the current one


def _c_pi(f: np.ndarray, K: float, Kp: float, Ki: float) -> np.ndarray:
    """
    Error path of the PX4 rate controller: C_pi(jw) = K·(Kp + Ki/(jw)).

    Acts on the tracking error (r − y).  ``f`` is frequency in Hz (no 0).
    """
    jw = 1j * 2.0 * np.pi * np.asarray(f, dtype=float)
    return K * (Kp + Ki / jw)


def _c_d(f: np.ndarray, K: float, Kd: float,
         d_cutoff_hz: float = D_CUTOFF_DEFAULT_HZ) -> np.ndarray:
    """
    Measurement path of the PX4 rate controller: C_d(jw) = K·Kd·jw·H_d(jw).

    Acts on the measurement y only.  H_d is a 2nd-order Butterworth low-pass at
    ``d_cutoff_hz`` (IMU_DGYRO_CUTOFF), matching the PX4 derivative filter:
        H_d(jw) = 1 / (1 + j·√2·(w/wd) − (w/wd)²),   wd = 2π·d_cutoff_hz.
    """
    w = 2.0 * np.pi * np.asarray(f, dtype=float)
    wd = 2.0 * np.pi * max(d_cutoff_hz, 1e-6)
    H_d = 1.0 / (1.0 + 1j * np.sqrt(2.0) * (w / wd) - (w / wd) ** 2)
    return K * Kd * (1j * w) * H_d


def px4_pid_tf(f: np.ndarray, K: float, Kp: float, Ki: float, Kd: float,
               d_filter_hz: float = D_CUTOFF_DEFAULT_HZ) -> np.ndarray:
    """
    Total PX4 rate controller gain C_pi + C_d = K·(Kp + Ki/(jw)) + K·Kd·jw·H_d(jw).

    This sum equals the open-loop controller gain that multiplies the plant in
    L = G·(C_pi + C_d); it is NOT the single-DOF "PID on the error" transfer.
    Retained for the loop-shaping synthesis path (which shapes |L|) and for tests.

    Parameters
    ----------
    f            : frequency array (Hz), must not contain 0
    K, Kp, Ki, Kd: PX4 rate-loop params
    d_filter_hz  : derivative low-pass cutoff (IMU_DGYRO_CUTOFF, default 30 Hz)
    """
    return _c_pi(f, K, Kp, Ki) + _c_d(f, K, Kd, d_filter_hz)


# ── plant recovery ────────────────────────────────────────────────────────────

@dataclass
class PlantFRF:
    """Frequency-domain plant estimate G(jw)."""
    f: List[float]                 # Hz
    G_mag_db: List[float]
    G_phase_deg: List[float]
    coherence: List[float]
    confidence: str
    note: str = ""


@dataclass
class PlantModel:
    """
    Parametric model of the plant.

    Two structures are supported (selected by held-out validation VAF):
      * ``order == "2nd_order"`` — G = K·wn²/((jw)²+2ζwn·jw+wn²)·e^(−jw·td),
        described by (K_plant, omega_n, zeta, delay_s);
      * ``order == "integrator_pole"`` — G = b·e^(−jw·td)/(jw·(1+jw/p)), the
        multicopter torque→rate physics (rigid-body integrator + motor pole),
        described by (int_gain=b, motor_pole=p, delay_s); the 2nd-order fields
        then hold a low-frequency-equivalent approximation for legacy consumers.
    """
    K_plant: float                 # plant DC gain (2nd-order) / equiv. low-freq gain
    omega_n: float                 # natural frequency (rad/s)
    zeta: float                    # damping ratio
    delay_s: float                 # effective loop delay (s)
    fit_vaf: float                 # validation VAF of model vs FRF points (%)
    order: str                     # "2nd_order" | "integrator_pole" | "1st_order" | "failed"
    confidence: str                # "high" | "medium" | "low"
    note: str = ""
    # Integrator+pole model parameters (populated when order == "integrator_pole")
    int_gain: Optional[float] = None       # b  (rad/s per unit torque, ×s)
    motor_pole: Optional[float] = None     # p  (rad/s)


@dataclass
class SimcGains:
    """
    Analytical SIMC cross-check (Skogestad 2003) for an integrating process.

    A closed-form second opinion on (Kp, Ki) from the integrator+pole+delay plant,
    mapped onto the PX4 rate loop at the same fixed loop gain K.  Not applied — it
    is shown alongside the numerically synthesized gains for comparison.
    """
    Kp: float
    Ki: float
    Kd: float
    tau_c_s: float          # closed-loop tuning time constant used
    note: str = ""


@dataclass
class SynthesizedGains:
    """PID gains synthesized from plant model."""
    axis: str
    K: float
    Kp: float
    Ki: float
    Kd: float
    # Predictions for the synthesized gains
    predicted_pm_deg: float
    predicted_bandwidth_hz: float
    predicted_overshoot_pct: float
    predicted_rise_time_s: float
    confidence: str
    note: str = ""
    # Original (in-log) gains for comparison
    orig_K: Optional[float] = None
    orig_Kp: Optional[float] = None
    orig_Ki: Optional[float] = None
    orig_Kd: Optional[float] = None
    # Cross-check with PX4 autotune if available
    autotune_divergence_pct: Optional[float] = None
    # Analytical SIMC second opinion (integrator+pole plant only)
    simc_gains: Optional[SimcGains] = None
    # Simulated before/after step responses {"t":[...], "y":[...], "metrics":{...}}
    before_response: Optional[Dict] = None
    after_response: Optional[Dict] = None


@dataclass
class PlantEstimation:
    axis: str
    plant_frf: Optional[PlantFRF]
    plant_model: Optional[PlantModel]
    synthesized: Optional[SynthesizedGains]
    before_response: Optional[Dict]   # {"t": [...], "y": [...], "metrics": {...}}
    after_response: Optional[Dict]
    warnings: List[str] = field(default_factory=list)


# ── helpers ───────────────────────────────────────────────────────────────────

def _butter2_lowpass_coeffs(cutoff_hz: float, dt: float):
    """
    Discrete 2nd-order Butterworth low-pass (b, a) at ``cutoff_hz`` for a sampler
    running at 1/dt.  Returns (None, None) when the filter is disabled (cutoff ≤ 0)
    or would exceed Nyquist.  Matches the PX4 IMU_DGYRO_CUTOFF derivative filter.
    """
    if not cutoff_hz or cutoff_hz <= 0 or dt <= 0:
        return None, None
    nyq = 0.5 / dt
    wn = float(np.clip(cutoff_hz / nyq, 1e-4, 0.99))
    b, a = sig.butter(2, wn, btype="low")
    return b, a


def _step_response_sim(plant_K: float, omega_n: float, zeta: float, delay_s: float,
                       pid_K: float, pid_Kp: float, pid_Ki: float, pid_Kd: float,
                       t_sim: np.ndarray,
                       d_filter_hz: float = D_CUTOFF_DEFAULT_HZ,
                       r_after: float = 1.0,
                       int_lim: float = INT_LIM_DEFAULT) -> np.ndarray:
    """
    Simulate the closed-loop step response in the PX4 rate-controller structure.

    PX4 differentiates the *measurement* (angular_accel), not the error, so there
    is no derivative kick on the step, and the normalized torque is clipped to
    ±1.0 with per-direction anti-windup on the integrator (MC_RR_INT_LIM):

        e      = r − y
        y_f    = Butterworth-2(y)                      # D-filter @ IMU_DGYRO_CUTOFF
        dgyro  = (y_f − y_f_prev) / dt
        u      = K·Kp·e + integ − K·Kd·dgyro
        u      = clip(u, −1, +1)
        integ += K·Ki·e·dt   (only if not saturating in the same direction)
        integ  = clip(integ, −int_lim, +int_lim)

    Plant: 2nd-order + transport delay, integrated with forward Euler.
    """
    dt = float(t_sim[1] - t_sim[0])
    delay_steps = max(0, int(round(delay_s / dt)))
    N = len(t_sim)
    sp = np.full(N, float(r_after))

    y = np.zeros(N)
    u_hist = np.zeros(N + delay_steps + 1)   # applied torque history (for delay)

    integ = 0.0
    dy = 0.0    # ẏ  (plant state)

    # 2nd-order Butterworth on the measurement, Direct-Form-II-transposed state.
    b_d, a_d = _butter2_lowpass_coeffs(d_filter_hz, dt)
    z1 = z2 = 0.0
    yf_prev = 0.0

    for k in range(1, N):
        y_prev = y[k - 1]
        e = sp[k] - y_prev

        # D on measurement: filter y, differentiate the filtered signal.
        if b_d is not None:
            yf = b_d[0] * y_prev + z1
            z1 = b_d[1] * y_prev - a_d[1] * yf + z2
            z2 = b_d[2] * y_prev - a_d[2] * yf
        else:
            yf = y_prev
        dgyro = (yf - yf_prev) / dt
        yf_prev = yf

        u = pid_K * pid_Kp * e + integ - pid_K * pid_Kd * dgyro
        u_sat = float(np.clip(u, -1.0, 1.0))

        # Conditional integration: freeze when saturated and e drives u further out.
        worsening = (u_sat >= 1.0 and e > 0.0) or (u_sat <= -1.0 and e < 0.0)
        if not worsening:
            integ += pid_K * pid_Ki * e * dt
            integ = float(np.clip(integ, -int_lim, int_lim))

        # Plant: torque(u) → rate(y), 2nd-order ODE + transport delay.
        if delay_steps > 0:
            idx = k - delay_steps
            u_eff = u_hist[idx] if idx >= 1 else 0.0
        else:
            u_eff = u_sat
        ddy = omega_n ** 2 * (plant_K * u_eff - y_prev) - 2.0 * zeta * omega_n * dy
        dy += ddy * dt
        y[k] = y_prev + dy * dt
        u_hist[k] = u_sat

    return y


def _step_metrics(t: np.ndarray, y: np.ndarray, r_after: float = 1.0) -> dict:
    """
    Overshoot %, rise time and settling time from a step response toward ``r_after``.

    Normalization is against the *commanded* step target (r_after), not the last
    sample — a still-oscillating loop's y[-1] would otherwise blow the metrics up.
    Crossings use ``argmax(norm >= x)`` with a reachability check (the old
    ``searchsorted`` assumed a monotone signal).  A diverging or non-finite
    response returns ``{"unstable": True}`` so synthesis can reject those gains.
    """
    if len(y) < 4:
        return {}
    y = np.asarray(y, dtype=float)
    target = float(r_after)
    if abs(target) < 1e-12:
        target = 1e-12 if target >= 0 else -1e-12

    # Divergence / non-finite guard.
    if not np.all(np.isfinite(y)) or float(np.max(np.abs(y))) > 5.0 * abs(target):
        return {"overshoot_pct": None, "rise_time_s": None,
                "settling_time_s": None, "unstable": True}

    norm = y / target
    peak = float(np.max(norm))
    overshoot_pct = max(0.0, (peak - 1.0) * 100.0)

    # Rise time 10→90 %: first reachable crossings (argmax on a boolean mask).
    rise_s = float(t[-1])
    if np.any(norm >= 0.10) and np.any(norm >= 0.90):
        i10 = int(np.argmax(norm >= 0.10))
        i90 = int(np.argmax(norm >= 0.90))
        if i90 > i10:
            rise_s = float(t[i90] - t[i10])

    # Settling time ±5 % of target: last sample outside the band.
    settled = np.abs(norm - 1.0) < 0.05
    if np.any(settled):
        bad = np.where(~settled)[0]
        settling_s = float(t[bad[-1]]) if len(bad) else float(t[0])
    else:
        settling_s = float(t[-1])

    return {"overshoot_pct": round(overshoot_pct, 1),
            "rise_time_s": round(rise_s, 3),
            "settling_time_s": round(settling_s, 3),
            "unstable": False}


# ── plant recovery from FRF ───────────────────────────────────────────────────

def recover_plant_frf(
    frf: FRFResult,
    params: Dict[str, float],
    axis: str,
) -> Tuple[Optional[PlantFRF], List[str]]:
    """
    Recover the open-loop plant from the closed-loop FRF T using the 2-DOF
    formula (PX4 puts D on the measurement):

        G(jw) = T / (C_pi·(1 − T) − T·C_d)

    Only coherent bins are used.  The complex denominator ``C_pi·(1−T) − T·C_d``
    is NOT clamped to a real constant when small (that would destroy the phase);
    instead a bin is dropped when |denom| falls below 3× its propagated
    uncertainty (from the FRF variance ε), and non-finite G is dropped too.
    """
    warnings: List[str] = []
    AX = axis.upper()

    K  = params.get(f"MC_{AX}RATE_K",  0.0)
    Kp = params.get(f"MC_{AX}RATE_P",  0.15)
    Ki = params.get(f"MC_{AX}RATE_I",  0.0)
    Kd = params.get(f"MC_{AX}RATE_D",  0.0)
    d_filt = params.get("IMU_DGYRO_CUTOFF", D_CUTOFF_DEFAULT_HZ)

    if K == 0:
        warnings.append(
            f"MC_{AX}RATE_K = 0 (not in log). Cannot recover plant — C(jw) undefined."
        )
        return None, warnings

    valid_pts = [b for b in frf.bode if b.valid and 0.5 < b.freq_hz < frf.f_nyquist_hz * 0.8]
    if len(valid_pts) < 8:
        warnings.append(f"Only {len(valid_pts)} coherent FRF bins — insufficient for plant recovery.")
        return None, warnings

    f_arr  = np.array([p.freq_hz for p in valid_pts])
    T_mag  = 10.0 ** (np.array([p.mag_db for p in valid_pts]) / 20.0)
    T_ph   = np.radians(np.array([p.phase_deg for p in valid_pts]))
    coh    = np.array([p.coherence for p in valid_pts])

    T = T_mag * np.exp(1j * T_ph)
    C_pi = _c_pi(f_arr, K, Kp, Ki)
    C_d  = _c_d(f_arr, K, Kd, d_filt)

    denom = C_pi * (1.0 - T) - T * C_d

    # FRF random error of the H1 estimate (Bendat & Piersol): relative std of T
    #     ε_r = sqrt(1 − γ²) / (|γ|·sqrt(2·n_avg)),   γ² = coherence.
    # Propagate onto the denominator: d(denom)/dT = −(C_pi + C_d), so the absolute
    # 1σ uncertainty of |denom| is |C_pi + C_d|·ε_r·|T|.  A bin whose |denom| is
    # within 3σ of zero is masked (division by a value indistinguishable from 0).
    n_avg = max(1, int(getattr(frf, "n_averages", 1) or 1))
    coh_c = np.clip(coh, 1e-6, 1.0)
    eps_r = np.sqrt(np.clip(1.0 - coh_c, 0.0, 1.0)) / (np.sqrt(coh_c) * np.sqrt(2.0 * n_avg))
    sigma_denom = np.abs(C_pi + C_d) * eps_r * np.abs(T)

    with np.errstate(divide="ignore", invalid="ignore"):
        G = T / denom

    keep = np.isfinite(G) & (np.abs(denom) >= 3.0 * sigma_denom)
    n_dropped = int(np.sum(~keep))
    if n_dropped:
        warnings.append(
            f"Dropped {n_dropped} FRF bin(s) where the 2-DOF denominator was "
            "indistinguishable from zero (within 3σ of the FRF variance)."
        )
    if int(np.sum(keep)) < 8:
        warnings.append(
            f"Only {int(np.sum(keep))} bins survived denominator masking — "
            "insufficient for plant recovery."
        )
        return None, warnings

    f_arr = f_arr[keep]
    coh   = coh[keep]
    G     = G[keep]

    G_mag_db  = 20.0 * np.log10(np.abs(G) + 1e-12)
    G_phase   = np.degrees(np.unwrap(np.angle(G)))

    conf = frf.confidence   # inherits from FRF
    if Ki == 0:
        conf = "low"
        warnings.append(f"MC_{AX}RATE_I = 0 — integrator absent, C(jw) approximate.")

    return PlantFRF(
        f=[round(float(v), 3) for v in f_arr],
        G_mag_db=[round(float(v), 2) for v in G_mag_db],
        G_phase_deg=[round(float(v), 1) for v in G_phase],
        coherence=[round(float(v), 3) for v in coh],
        confidence=conf,
    ), warnings


# ── parametric model fit ──────────────────────────────────────────────────────

def _second_order_tf(f: np.ndarray, K_p: float, omega_n: float,
                     zeta: float, delay_s: float) -> np.ndarray:
    """
    G(jw) = K_p * omega_n^2 / [(jw)^2 + 2*zeta*omega_n*(jw) + omega_n^2] * exp(-jw*delay)
    """
    jw = 1j * 2.0 * np.pi * f
    G = K_p * omega_n**2 / (jw**2 + 2*zeta*omega_n*jw + omega_n**2)
    G *= np.exp(-jw * delay_s)
    return G


def _integrator_pole_tf(f: np.ndarray, b: float, p: float,
                        delay_s: float) -> np.ndarray:
    """
    Multicopter torque→rate plant: G(jw) = b·e^(−jw·delay) / (jw·(1 + jw/p)).

    A rigid-body integrator (torque → angular acceleration → rate) in series
    with a single motor/ESC pole ``p`` (rad/s) and an effective delay.
    """
    jw = 1j * 2.0 * np.pi * f
    G = b / (jw * (1.0 + jw / p))
    G *= np.exp(-jw * delay_s)
    return G


def _weighted_vaf(G_meas: np.ndarray, G_pred: np.ndarray,
                  w: np.ndarray) -> float:
    """Complex, weighted variance-accounted-for (%) of a model vs measured FRF."""
    num = float(np.sum(w * np.abs(G_meas - G_pred) ** 2))
    den = float(np.sum(w * np.abs(G_meas - np.average(G_meas, weights=w)) ** 2))
    return max(0.0, 1.0 - num / (den + 1e-12)) * 100.0


def fit_plant_model(plant_frf: PlantFRF) -> PlantModel:
    """
    Fit two candidate plant models to the recovered FRF and keep the one with the
    higher held-out validation VAF:

      * 2nd-order + delay (broad, no structural prior);
      * integrator + motor-pole + delay (multicopter torque→rate physics).

    Bin weights come from the FRF variance (inverse-variance, w ∝ γ²/(1−γ²), so
    coherent bins dominate) rather than γ⁴, and the fit cost is the RELATIVE
    complex error |ΔG/G|² so that high-|G| low-frequency bins do not swamp the
    resonance region.
    """
    f  = np.array(plant_frf.f)
    Gm = 10.0 ** (np.array(plant_frf.G_mag_db) / 20.0)
    Gp = np.radians(np.array(plant_frf.G_phase_deg))
    coh = np.array(plant_frf.coherence)
    G_meas_all = Gm * np.exp(1j * Gp)

    sel = f > 0.3
    f_filt = f[sel]
    G_filt = G_meas_all[sel]
    Gm_filt = Gm[sel]
    coh_filt = np.clip(coh[sel], 1e-6, 1.0 - 1e-3)
    # Inverse-variance weight: 1/ε_r² ∝ γ²/(1−γ²)  (n_avg constant → drops out).
    w_filt = coh_filt / (1.0 - coh_filt)

    if len(f_filt) < 8:
        return PlantModel(K_plant=1.0, omega_n=2*math.pi*5, zeta=0.7, delay_s=0.005,
                          fit_vaf=0.0, order="failed", confidence="low",
                          note="Too few coherent bins to fit plant model.")

    # Relative complex error weight (denominator floored to avoid blow-up where
    # |G| is tiny): cost weight = w / |G|².
    g_floor = float(np.median(Gm_filt)) * 1e-3 + 1e-12
    rel_w = w_filt / np.maximum(Gm_filt, g_floor) ** 2

    # Held-out validation split: every 4th coherent bin is validation-only, so
    # model selection is not judged on the bins the fit already minimised.
    val_mask = np.zeros(len(f_filt), dtype=bool)
    val_mask[::4] = True
    fit_mask = ~val_mask
    if int(np.sum(fit_mask)) < 6 or int(np.sum(val_mask)) < 3:
        fit_mask = np.ones(len(f_filt), dtype=bool)
        val_mask = fit_mask

    def _rel_cost(G_pred_all, m):
        return float(np.sum(rel_w[m] * np.abs(G_filt[m] - G_pred_all[m]) ** 2))

    # ── Candidate A: 2nd-order + delay ────────────────────────────────────────
    K_est = float(np.median(Gm_filt[f_filt < 3.0])) if np.any(f_filt < 3.0) \
        else float(np.median(Gm_filt))
    if K_est <= 0:
        K_est = 1.0

    def cost2(params):
        Kp_, wn_, z_, td_ = params
        if Kp_ <= 0 or wn_ <= 0 or z_ <= 0 or td_ < 0:
            return 1e8
        return _rel_cost(_second_order_tf(f_filt, Kp_, wn_, z_, td_), fit_mask)

    best2_cost, best2_p = 1e18, (K_est, 2*math.pi*5, 0.7, 0.005)
    for wn_c in np.geomspace(2*math.pi*1, 2*math.pi*50, 12):
        for z_c in (0.3, 0.7, 1.2):
            for td_c in (0.003, 0.01, 0.02):
                c = cost2((K_est, wn_c, z_c, td_c))
                if c < best2_cost:
                    best2_cost, best2_p = c, (K_est, wn_c, z_c, td_c)
    try:
        r2 = minimize(cost2, best2_p, method="Nelder-Mead",
                      options={"xatol": 1e-5, "fatol": 1e-10, "maxiter": 2000})
        K2, wn2, z2, td2 = (abs(float(r2.x[0])), abs(float(r2.x[1])),
                            float(np.clip(r2.x[2], 0.01, 3.0)),
                            float(np.clip(r2.x[3], 0.0, 0.1)))
    except Exception:
        K2, wn2, z2, td2 = best2_p
    vaf2 = _weighted_vaf(G_filt[val_mask],
                         _second_order_tf(f_filt, K2, wn2, z2, td2)[val_mask],
                         w_filt[val_mask])

    # ── Candidate B: integrator + motor-pole + delay ──────────────────────────
    # b ≈ |G·jw| over a coherent mid-band (2–10 Hz): for G = b/(jw·(1+jw/p)),
    # |G·jw| → b below the motor pole, a robust integrator-gain estimate.
    mid = (f_filt >= 2.0) & (f_filt <= 10.0)
    if not np.any(mid):
        mid = f_filt <= float(np.median(f_filt))
    b_est = float(np.median(Gm_filt[mid] * 2.0 * np.pi * f_filt[mid]))
    if b_est <= 0:
        b_est = 1.0

    def costI(params):
        b_, p_, td_ = params
        if b_ <= 0 or p_ <= 0 or td_ < 0:
            return 1e8
        return _rel_cost(_integrator_pole_tf(f_filt, b_, p_, td_), fit_mask)

    bestI_cost, bestI_p = 1e18, (b_est, 2*math.pi*20, 0.005)
    for p_c in np.geomspace(2*math.pi*3, 2*math.pi*80, 12):
        for td_c in (0.003, 0.01, 0.02):
            c = costI((b_est, p_c, td_c))
            if c < bestI_cost:
                bestI_cost, bestI_p = c, (b_est, p_c, td_c)
    try:
        rI = minimize(costI, bestI_p, method="Nelder-Mead",
                      options={"xatol": 1e-6, "fatol": 1e-10, "maxiter": 2000})
        bI, pI, tdI = (abs(float(rI.x[0])), abs(float(rI.x[1])),
                       float(np.clip(rI.x[2], 0.0, 0.1)))
    except Exception:
        bI, pI, tdI = bestI_p
    vafI = _weighted_vaf(G_filt[val_mask],
                         _integrator_pole_tf(f_filt, bI, pI, tdI)[val_mask],
                         w_filt[val_mask])

    def _conf(v):
        return "high" if v > 75 else ("medium" if v > 50 else "low")

    # ── Select by validation VAF ──────────────────────────────────────────────
    if vafI > vaf2:
        # Low-frequency-equivalent 2nd-order fields for legacy consumers: match
        # the motor pole and integrator roll-off near the pole.
        eq_wn = pI
        eq_zeta = 1.0
        eq_K = bI / max(pI, 1e-6)
        note = (f"integrator+pole+delay fit (multicopter physics): VAF={vafI:.1f}% "
                f"(val), b={bI:.1f}, motor pole={pI/(2*math.pi):.1f} Hz, "
                f"td={tdI*1000:.1f} ms. 2nd-order alt VAF={vaf2:.1f}%.")
        return PlantModel(
            K_plant=round(eq_K, 4), omega_n=round(eq_wn, 3),
            zeta=round(eq_zeta, 3), delay_s=round(tdI, 5),
            fit_vaf=round(vafI, 1), order="integrator_pole",
            confidence=_conf(vafI), note=note,
            int_gain=round(bI, 4), motor_pole=round(pI, 3))

    note = (f"2nd-order+delay fit: VAF={vaf2:.1f}% (val), "
            f"wn={wn2/(2*math.pi):.1f} Hz, ζ={z2:.2f}, td={td2*1000:.1f} ms. "
            f"integrator+pole alt VAF={vafI:.1f}%.")
    return PlantModel(K_plant=round(K2, 4), omega_n=round(wn2, 3),
                      zeta=round(z2, 3), delay_s=round(td2, 5),
                      fit_vaf=round(vaf2, 1), order="2nd_order",
                      confidence=_conf(vaf2), note=note)


# ── analytical SIMC cross-check ───────────────────────────────────────────────

def synthesize_simc(plant: PlantModel, tau_c: Optional[float] = None,
                    K_fixed: float = 1.0) -> Optional[SimcGains]:
    """
    SIMC (Skogestad 2003) closed-form gains for the integrator+pole+delay plant
    G = b·e^(−sτ)/(s·(1 + s/p)), mapped onto the PX4 rate loop at fixed K.

    The fast motor pole (time constant 1/p) is folded into the effective delay by
    Skogestad's half rule (half of 1/p added to θ).  For the resulting integrating
    process g = b·e^(−sθ)/s the SIMC-PI rules are closed form:

        Kc = 1 / (b·(τc + θ)),   Ti = 4·(τc + θ),   τc = θ by default.

    Mapped to PX4 (C_pi = K·(Kp + Ki/s), K held fixed):
        Kp = Kc / K,   Ki = Kc / (Ti·K),   Kd = 0.

    Returns None when the candidate model is not the integrator+pole structure.
    """
    if plant.order != "integrator_pole" or not plant.int_gain or not plant.motor_pole:
        return None
    b = float(plant.int_gain)
    p = float(plant.motor_pole)
    if b <= 0 or p <= 0:
        return None

    theta = float(plant.delay_s) + 0.5 / p   # half-rule: half of motor lag → delay
    if tau_c is None:
        tau_c = theta
    tau_c = max(float(tau_c), 1e-4)

    Kc = 1.0 / (b * (tau_c + theta))
    Ti = 4.0 * (tau_c + theta)
    K = K_fixed if (K_fixed and K_fixed > 0) else 1.0
    Kp = Kc / K
    Ki = Kc / (Ti * K)

    return SimcGains(
        Kp=round(Kp, 4), Ki=round(Ki, 4), Kd=0.0, tau_c_s=round(tau_c, 5),
        note=(f"SIMC integrating-process second opinion (Skogestad 2003): "
              f"τc={tau_c * 1000:.1f} ms, θ={theta * 1000:.1f} ms, "
              f"Kc={Kc:.3f}, Ti={Ti:.3f} s."))


# ── PID synthesis ─────────────────────────────────────────────────────────────

def synthesize_pid_from_plant(
    plant: PlantModel,
    axis: str,
    params: Dict[str, float],
    target_pm_deg: float = 50.0,
    target_bw_hz: float = 15.0,
    autotune_params: Optional[Dict] = None,
) -> Optional[SynthesizedGains]:
    """
    Synthesize rate PID gains from the plant model targeting desired PM and bandwidth.

    Loop shaping via Nelder-Mead on a PM + BW objective, but with the loop gain
    **K held fixed** (to the current MC_xRATE_K, or 1.0 if params are absent) so
    only (Kp, Ki, Kd) are optimized.  This removes the (K,Kp,Ki,Kd) → (λK, Kp/λ,
    Ki/λ, Kd/λ) degeneracy — L, T and every closed-loop metric depend only on the
    products K·Kp, K·Ki, K·Kd, so a free K just slides the solution along a null
    direction.  Fixing K matches PX4 practice (K is the user's overall knob; the
    synthesis writes P/I/D).

    Safety guards folded into the cost (soft penalties) and enforced as final
    caps: peak sensitivity Ms ≤ 1.7, high-frequency D-path gain K·Kd·wd ≤ 0.5,
    the PX4 areGainsGood gate (K·Kp<0.5, K·Ki<10, K·Kd<0.1, all >0) and a ±30 %
    clamp of each gain around its current value (iterative safe tuning).
    """
    if plant.confidence == "low":
        return None
    if plant.K_plant == 0 or plant.omega_n == 0:
        return None

    AX = axis.upper()
    orig_K  = params.get(f"MC_{AX}RATE_K")
    orig_Kp = params.get(f"MC_{AX}RATE_P")
    orig_Ki = params.get(f"MC_{AX}RATE_I")
    orig_Kd = params.get(f"MC_{AX}RATE_D")
    d_filt  = params.get("IMU_DGYRO_CUTOFF", D_CUTOFF_DEFAULT_HZ)

    # Fixed loop gain: current K, or 1.0 when params are not loaded.
    K_fixed = float(orig_K) if (orig_K and orig_K > 0) else 1.0
    wd = 2.0 * np.pi * max(d_filt, 1e-6)

    # Frequency array for design
    f_des = np.geomspace(0.1, min(200.0, target_bw_hz * 10), 300)

    def plant_tf(f_arr):
        if plant.order == "integrator_pole" and plant.int_gain and plant.motor_pole:
            return _integrator_pole_tf(f_arr, plant.int_gain, plant.motor_pole,
                                       plant.delay_s)
        jw_ = 1j * 2.0 * np.pi * f_arr
        G = (plant.K_plant * plant.omega_n**2
             / (jw_**2 + 2*plant.zeta*plant.omega_n*jw_ + plant.omega_n**2)
             * np.exp(-jw_ * plant.delay_s))
        return G

    G_des = plant_tf(f_des)

    def loop_tf(Kp_, Ki_, Kd_):
        C = px4_pid_tf(f_des, K_fixed, Kp_, Ki_, Kd_, d_filt)
        return C * G_des

    def pm_bw(L):
        """Phase margin (deg) and gain-crossover freq (Hz) from an open loop L."""
        L_mag_db = 20.0 * np.log10(np.abs(L) + 1e-12)
        L_phase  = np.degrees(np.unwrap(np.angle(L)))
        sign_c = np.where(np.diff(np.sign(L_mag_db)))[0]
        if len(sign_c) == 0:
            return None, None
        i = sign_c[0]
        gc_hz = f_des[i] + (f_des[i+1] - f_des[i]) * (-L_mag_db[i]) / \
            (L_mag_db[i+1] - L_mag_db[i] + 1e-9)
        pm = float(np.interp(gc_hz, f_des, L_phase)) + 180.0
        return pm, float(gc_hz)

    def gate_ok(Kp_, Ki_, Kd_):
        return (Kp_ > 0 and Ki_ >= 0 and Kd_ >= 0
                and K_fixed * Kp_ < GATE_KP_MAX
                and K_fixed * Ki_ < GATE_KI_MAX
                and K_fixed * Kd_ < GATE_KD_MAX)

    def objective(p):
        Kp_, Ki_, Kd_ = p
        if Kp_ <= 0 or Ki_ < 0 or Kd_ < 0:
            return 1e8
        L = loop_tf(Kp_, Ki_, Kd_)
        pm, gc_hz = pm_bw(L)
        if pm is None:
            return 1e6   # no crossover
        cost = (pm - target_pm_deg) ** 2 + 50.0 * (gc_hz - target_bw_hz) ** 2 / target_bw_hz**2
        # Peak-sensitivity penalty: Ms = max|1/(1+L)| ≤ 1.7.
        Ms = float(np.max(1.0 / (np.abs(1.0 + L) + 1e-12)))
        if Ms > MS_MAX:
            cost += 1.0e3 * (Ms - MS_MAX) ** 2
        # High-frequency D-path gain penalty: K·Kd·wd ≤ 0.5.
        hf = K_fixed * Kd_ * wd
        if hf > HF_DGAIN_MAX:
            cost += 1.0e3 * (hf - HF_DGAIN_MAX) ** 2
        # PX4 areGainsGood gate.
        if not gate_ok(Kp_, Ki_, Kd_):
            cost += 1.0e4
        return cost

    # Initial guess: current (Kp, Ki, Kd) if known, else PX4 defaults.
    x0 = [orig_Kp if (orig_Kp and orig_Kp > 0) else 0.15,
          orig_Ki if (orig_Ki and orig_Ki > 0) else 0.05,
          orig_Kd if (orig_Kd and orig_Kd >= 0) else 0.003]

    try:
        res = minimize(objective, x0, method="Nelder-Mead",
                       options={"xatol": 1e-6, "fatol": 1e-6, "maxiter": 3000})
        Kp_s, Ki_s, Kd_s = (abs(float(res.x[0])), abs(float(res.x[1])),
                            abs(float(res.x[2])))
    except Exception as e:
        log.warning(f"PID synthesis failed: {e}")
        return None

    # Safe-tuning clamp: each gain within ±30 % of its current value (if known).
    def _clamp(val, cur):
        if cur and cur > 0:
            lo, hi = (1.0 - GAIN_CLAMP_FRAC) * cur, (1.0 + GAIN_CLAMP_FRAC) * cur
            return float(min(max(val, lo), hi))
        return float(val)
    Kp_s = _clamp(Kp_s, orig_Kp)
    Ki_s = _clamp(Ki_s, orig_Ki)
    Kd_s = _clamp(Kd_s, orig_Kd)

    # Hard-cap against the areGainsGood gate after clamping.
    Kp_s = min(Kp_s, (GATE_KP_MAX - 1e-6) / K_fixed)
    Ki_s = min(Ki_s, (GATE_KI_MAX - 1e-6) / K_fixed)
    Kd_s = min(Kd_s, (GATE_KD_MAX - 1e-6) / K_fixed)

    # Predicted PM and BW at the final gains.
    L_s = loop_tf(Kp_s, Ki_s, Kd_s)
    pred_pm, pred_bw = pm_bw(L_s)
    pred_pm = pred_pm if pred_pm is not None else 0.0
    pred_bw = pred_bw if pred_bw is not None else 0.0

    # Simulate AFTER (synthesized gains).
    t_sim = np.linspace(0, 0.5, 500)
    y_sim = _step_response_sim(plant.K_plant, plant.omega_n, plant.zeta, plant.delay_s,
                                K_fixed, Kp_s, Ki_s, Kd_s, t_sim, d_filter_hz=d_filt)
    metrics_after = _step_metrics(t_sim, y_sim)
    if metrics_after.get("unstable"):
        log.warning(f"Synthesized gains for {axis} simulate as unstable — rejected.")
        return None

    after_resp = {
        "t": [round(v, 4) for v in t_sim[::5].tolist()],
        "y": [round(v, 4) for v in y_sim[::5].tolist()],
        "metrics": metrics_after,
    }

    # Simulate BEFORE (original gains) when they are known.
    before_resp: Optional[Dict] = None
    if orig_K and orig_K > 0:
        y_before = _step_response_sim(
            plant.K_plant, plant.omega_n, plant.zeta, plant.delay_s,
            K_fixed, orig_Kp if orig_Kp else 0.15,
            orig_Ki if orig_Ki else 0.05, orig_Kd if orig_Kd else 0.003,
            t_sim, d_filter_hz=d_filt)
        before_resp = {
            "t": [round(v, 4) for v in t_sim[::5].tolist()],
            "y": [round(v, 4) for v in y_before[::5].tolist()],
            "metrics": _step_metrics(t_sim, y_before),
        }

    # Analytical SIMC second opinion (integrator+pole plant only).
    simc = synthesize_simc(plant, K_fixed=K_fixed)

    # Cross-check with PX4 autotune.
    divergence: Optional[float] = None
    if autotune_params:
        at_kc = autotune_params.get("roll_gain") or autotune_params.get("pitch_gain")
        if at_kc and at_kc > 0:
            # PX4 autotune maps kc → MC_xRATE_P at K=1; compare effective K·Kp.
            divergence = round(abs(K_fixed * Kp_s - at_kc) / at_kc * 100.0, 1)

    conf = "medium" if plant.confidence in ("high", "medium") else "low"
    note = (f"K held fixed at {K_fixed:.3f}. Target: PM={target_pm_deg:.0f}°, "
            f"BW={target_bw_hz:.0f} Hz. Predicted: PM≈{pred_pm:.1f}°, "
            f"BW≈{pred_bw:.1f} Hz. Validate in SITL before real flight.")
    if divergence is not None:
        note += f" PX4 autotune K·Kp divergence: {divergence:.1f}%."

    return SynthesizedGains(
        axis=axis,
        K=round(K_fixed, 4), Kp=round(Kp_s, 4), Ki=round(Ki_s, 4), Kd=round(Kd_s, 4),
        predicted_pm_deg=round(pred_pm, 1),
        predicted_bandwidth_hz=round(pred_bw, 2),
        predicted_overshoot_pct=metrics_after.get("overshoot_pct") or 0.0,
        predicted_rise_time_s=metrics_after.get("rise_time_s") or 0.0,
        confidence=conf, note=note,
        orig_K=orig_K, orig_Kp=orig_Kp, orig_Ki=orig_Ki, orig_Kd=orig_Kd,
        autotune_divergence_pct=divergence,
        simc_gains=simc,
        before_response=before_resp,
        after_response=after_resp,
    )


# ── orchestrator ──────────────────────────────────────────────────────────────

class PlantEstimator:

    def __init__(self, target_pm_deg: float = 50.0, target_bw_hz: float = 15.0):
        self.target_pm_deg = target_pm_deg
        self.target_bw_hz  = target_bw_hz

    def estimate(
        self,
        frf_results: List[FRFResult],
        params: Dict[str, float],
        autotune_params: Optional[Dict] = None,
    ) -> List[PlantEstimation]:
        estimations: List[PlantEstimation] = []
        for frf in frf_results:
            if frf.loop != "rate":
                continue
            axis = frf.axis
            warns: List[str] = []

            # 1. Plant FRF recovery
            plant_frf, w = recover_plant_frf(frf, params, axis)
            warns.extend(w)
            if plant_frf is None:
                estimations.append(PlantEstimation(
                    axis=axis, plant_frf=None, plant_model=None,
                    synthesized=None, before_response=None, after_response=None,
                    warnings=warns,
                ))
                continue

            # 2. Parametric model fit
            plant_model = fit_plant_model(plant_frf)
            warns.append(plant_model.note)

            # 3. PID synthesis
            synth_result = None
            before_resp = None
            after_resp = None
            if plant_model.confidence != "low":
                synth_result = synthesize_pid_from_plant(
                    plant_model, axis, params,
                    target_pm_deg=self.target_pm_deg,
                    target_bw_hz=self.target_bw_hz,
                    autotune_params=autotune_params,
                )
                if synth_result is not None:
                    before_resp = synth_result.before_response
                    after_resp = synth_result.after_response

            estimations.append(PlantEstimation(
                axis=axis,
                plant_frf=plant_frf,
                plant_model=plant_model,
                synthesized=synth_result,
                before_response=before_resp,
                after_response=after_resp,
                warnings=[w for w in warns if w],
            ))
        return estimations
