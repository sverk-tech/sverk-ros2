"""
Offline PID analysis engine.
No ROS dependency — operates on List[TelemetryFrame] from the buffer.

Key improvements over v1
------------------------
* PX4 timestamps (frame.t) used instead of wall-clock — no ROS scheduler jitter.
* Nyquist guard: spectral metrics above f_nyq are not reported.
* Uniform resampling before spectral/model analysis (handles DDS jitter).
* 2nd-order model fitting (scipy) replaces peak-picking for damping/wn.
* Per-bin step aggregation: steps binned by amplitude — only comparable
  events are averaged together.
* Multi-hypothesis overshoot attribution (K / D / I-windup evidence).
* Data-quality gating: low quality → direction only, no suggested value.
* Vehicle profile: racer / cinematic / default shifts thresholds.
* Indoor/OF confounder detection via eph/epv spikes during POSCTL.
* I-rule only fires on steady-hold segments, not on transient steps.
* DataQuality dataclass included in AnalysisResult and JSON export.

DISCLAIMER: All recommendations are heuristic starting points only.
Change one parameter at a time.  Test in SITL before real flight.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any, Tuple

import numpy as np
from scipy import signal as sig

from .data_buffer import TelemetryFrame, TopicStats
from .model_estimator import (
    fit_second_order, coherence_verdict, nyquist_guard,
    bin_steps_by_amplitude, resample_uniform, ModelFit,
)
from .param_parser import describe_params

# ── nav_state → mode name ─────────────────────────────────────────────────────
NAV_STATE_MAP = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    4: "AUTO_LOITER", 5: "AUTO_RTL", 10: "ACRO", 14: "OFFBOARD",
    15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
}

AXES = ("roll", "pitch", "yaw")

# ── recommendation magnitude cap ──────────────────────────────────────────────
# The model-based K-scale rule (r1) is derived for an ideal 2nd-order loop with no
# delay; the real oscillation is driven by phase lag, so its magnitude cannot be
# trusted (ζ_fit=0.1 → Δ=−98%, an un-flyable "fix").  Cap every recommendation's
# delta at ±30% per iteration and ask the pilot to re-measure between changes.
MAX_DELTA_PCT = 30.0

# ── vehicle profile defaults (overridden by config/params.yaml) ───────────────
# yaw_rise_scale multiplies the slow-rise threshold on the yaw axis: yaw dynamics
# are inherently slower than roll/pitch, so a shared rise threshold fires false
# "yaw slow" recommendations without it.
_PROFILES: Dict[str, Dict[str, float]] = {
    "default":   dict(overshoot_warn_pct=20.0, overshoot_high_pct=40.0,
                      rise_time_slow_s=0.35, settling_slow_s=1.5,
                      noise_warn=0.08, noise_high=0.15, ss_error_rad_s=0.05,
                      yaw_rise_scale=2.0),
    "racer":     dict(overshoot_warn_pct=30.0, overshoot_high_pct=55.0,
                      rise_time_slow_s=0.20, settling_slow_s=0.80,
                      noise_warn=0.12, noise_high=0.22, ss_error_rad_s=0.08,
                      yaw_rise_scale=2.0),
    "cinematic": dict(overshoot_warn_pct=10.0, overshoot_high_pct=20.0,
                      rise_time_slow_s=0.50, settling_slow_s=2.0,
                      noise_warn=0.05, noise_high=0.10, ss_error_rad_s=0.03,
                      yaw_rise_scale=2.0),
}

# ── recommendation arbitration ────────────────────────────────────────────────
# Lower number = higher priority.  On a direction conflict for the same
# (axis, param) the highest-priority rule wins and the rest are dropped.
_REC_PRIORITY: Dict[str, int] = {
    "r1_oscillation": 0,
    "r1_oscillation_no_model": 0,
    "r2_overshoot_K": 1,
    "r6_noise_D": 2,
    "a1_att_overshoot": 3,
    "a2_att_oscillation": 3,
    "r4_slow_rise": 4,
    "r5_steady_state_err": 5,
    "a3_att_slow_rise": 6,
    "r2_overshoot_D": 7,
}
_DEFAULT_PRIORITY = 99
_CONF_RANK = {"high": 0, "medium": 1, "low": 2}

# ── quality gates (overridden by config) ──────────────────────────────────────
_QUALITY_DEFAULTS = dict(
    min_rate_hz_for_spectral=10.0,
    min_events_for_high_conf=3,
    min_r2_for_model_suggestion=0.60,
    max_jitter_pct_for_high_conf=20.0,
    max_drop_pct_for_high_conf=15.0,
)

# ── reason translation templates ──────────────────────────────────────────────
REASON_TR: Dict[str, Dict[str, str]] = {
    "en": {
        "r1_oscillation":
            "Sustained oscillation at {0:.1f} Hz (ζ={1:.2f}, R²={2:.2f}). "
            "Reduce rate gain K ≈ ×{3:.2f} to reach ζ≈0.7.",
        "r1_oscillation_no_model":
            "Sustained oscillation near {0:.1f} Hz (damping low). "
            "Reduce rate gain K in steps of ~0.02.",
        "r2_overshoot_K":
            "Overshoot {0:.0f}% > {1:.0f}% threshold. Most likely cause: K too high. "
            "Evidence: {2}. Reduce K first; adding D can also help if the response "
            "is oscillatory rather than pure overshoot.",
        "r2_overshoot_D":
            "High overshoot and low D — increasing D can add damping without slowing rise.",
        "r2_overshoot_I":
            "I-windup contribution suspected (integrator {0:.3f} is large relative to torque).",
        "clamp_note":
            " (capped at {0:+.0f}% per iteration; re-measure after applying).",
        "r3_low_freq_osc":
            "Low-frequency oscillation at {0:.2f} Hz in {1} — attitude P likely too high "
            "relative to rate gains.",
        "r4_slow_rise":
            "Rise time {0:.3f} s is slow (>{1:.2f} s threshold). Increase K/P for crisper response.",
        "r5_steady_state_err":
            "Steady-state error {0:.3f} rad/s on hold segment. "
            "Raise I to eliminate offset under hover/wind.",
        "r6_noise_D":
            "Control-signal noise RMS {0:.4f} > {1:.3f}. "
            "err↔torque coherence: {2} (informational). Lower D or check IMU_GYRO_CUTOFF.",
        "r6_noise_cutoff":
            "High gyro noise — consider lowering IMU_GYRO_CUTOFF. "
            "Reduces D amplification but may slow response.",
        "r7_cross_mode":
            "[{0}] Rate loop acceptable in Stabilized but degrades in Position. "
            "Inner loop is marginal under outer-loop setpoints. Check rate_K and D noise.",
        "a1_att_overshoot":
            "Attitude overshoot {0:.0f}% > 25% on {1} loop. "
            "Reduce MC_{2}_P. Verify rate loop is well-tuned first.",
        "a2_att_oscillation":
            "Slow oscillation at {0:.2f} Hz in attitude loop (ζ={1:.2f}). "
            "MC_{2}_P likely too high relative to rate gains.",
        "a3_att_slow_rise":
            "Attitude rise time {0:.3f} s is slow (>0.5 s). "
            "Increase MC_{1}_P for snappier attitude response.",
        "no_step_events":
            "No step events detected. Perform deliberate quick stick jabs in Stabilized mode "
            "while recording for useful analysis.",
        "indoor_confounder":
            "[{0}] POSCTL oscillation coincides with eph spike ({1:.2f} m) — "
            "may be estimator/optical-flow noise, not a PID problem. "
            "Verify in Stabilized mode first.",
        "low_sample_rate":
            "Achieved sample rate {0:.1f} Hz — spectral analysis unreliable below Nyquist "
            "{1:.1f} Hz.  Increase DDS link speed (Ethernet > UART) for better diagnostics.",
        "warn_no_frames": "No frames recorded.",
        "warn_topic_missing":
            "Topic {0} never received data. "
            "Add it to dds_topics.yaml and rebuild PX4 firmware. "
            "Step-response analysis for rate/attitude loop unavailable.",
        "ev_rate":
            "mode={0}, n={1}, overshoot={2:.1f}%, rise={3:.3f}s, "
            "settle={4:.3f}s, freq={5:.2f}Hz, noise={6:.5f}",
        "ev_attitude":
            "mode={0}, n={1}, overshoot={2:.1f}%, rise={3:.3f}s, "
            "settle={4:.3f}s, freq={5:.2f}Hz [attitude loop]",
        "ev_no_events": "n_events=0 across all modes/axes",
        "json_disclaimer":
            "These recommendations are heuristic starting points only. "
            "Change one parameter at a time. Test in SITL before real flight.",
    },
    "ru": {
        "r1_oscillation":
            "Устойчивая осцилляция на {0:.1f} Гц (ζ={1:.2f}, R²={2:.2f}). "
            "Уменьшите K ≈ ×{3:.2f} для достижения ζ≈0.7.",
        "r1_oscillation_no_model":
            "Устойчивая осцилляция около {0:.1f} Гц (демпфирование низкое). "
            "Уменьшайте K шагами ~0.02.",
        "r2_overshoot_K":
            "Перерегулирование {0:.0f}% > порога {1:.0f}%. Наиболее вероятная причина: K завышен. "
            "Доказательства: {2}. Сначала уменьшите K; добавление D помогает, если отклик "
            "колебательный, а не чистое перерегулирование.",
        "r2_overshoot_D":
            "Большое перерегулирование и малый D — увеличение D добавит демпфирования.",
        "r2_overshoot_I":
            "Возможен вклад I-windup (интегратор {0:.3f} велик относительно момента).",
        "clamp_note":
            " (ограничено {0:+.0f}% за итерацию; повторите замер после применения).",
        "r3_low_freq_osc":
            "Низкочастотная осцилляция на {0:.2f} Гц в {1} — "
            "P углового контура, вероятно, завышен относительно K скорости.",
        "r4_slow_rise":
            "Время нарастания {0:.3f}с — медленно (порог >{1:.2f}с). "
            "Увеличьте K/P для более быстрого отклика.",
        "r5_steady_state_err":
            "Установившаяся ошибка {0:.3f} рад/с на удержании. "
            "Увеличьте I для устранения смещения при висении/ветре.",
        "r6_noise_D":
            "Шум управления RMS {0:.4f} > {1:.3f}. "
            "Когерентность err↔torque: {2} (справочно). Уменьшите D или проверьте IMU_GYRO_CUTOFF.",
        "r6_noise_cutoff":
            "Высокий шум гироскопа — уменьшите IMU_GYRO_CUTOFF. "
            "Снижает усиление D, но может замедлить отклик.",
        "r7_cross_mode":
            "[{0}] Контур скорости приемлем в Stabilized, но ухудшается в Position. "
            "Внутренний контур на грани при уставках внешнего. Проверьте rate_K и шум D.",
        "a1_att_overshoot":
            "Перерегулирование угла {0:.0f}% > 25% по оси {1}. "
            "Уменьшите MC_{2}_P. Сначала убедитесь, что контур скорости настроен.",
        "a2_att_oscillation":
            "Медленная осцилляция на {0:.2f} Гц в угловом контуре (ζ={1:.2f}). "
            "MC_{2}_P, вероятно, завышен относительно K скорости.",
        "a3_att_slow_rise":
            "Время нарастания угла {0:.3f}с — медленно (>0.5с). "
            "Увеличьте MC_{1}_P для более резвого отклика.",
        "no_step_events":
            "Ступенчатых событий не обнаружено. Выполните быстрые тычки стиком "
            "в режиме Stabilized во время записи.",
        "indoor_confounder":
            "[{0}] Осцилляция в POSCTL совпадает со скачком eph ({1:.2f} м) — "
            "возможно шум оценщика/оптопотока, а не PID. "
            "Проверьте в режиме Stabilized.",
        "low_sample_rate":
            "Частота выборки {0:.1f} Гц — спектральный анализ ненадёжен ниже Найквиста "
            "{1:.1f} Гц. Увеличьте скорость DDS-линка (Ethernet > UART).",
        "warn_no_frames": "Нет записанных кадров.",
        "warn_topic_missing":
            "Топик {0} не получил данных. "
            "Добавьте в dds_topics.yaml и пересоберите прошивку PX4. "
            "Анализ отклика недоступен.",
        "ev_rate":
            "режим={0}, n={1}, перерегул.={2:.1f}%, нараст.={3:.3f}с, "
            "устан.={4:.3f}с, частота={5:.2f}Гц, шум={6:.5f}",
        "ev_attitude":
            "режим={0}, n={1}, перерегул.={2:.1f}%, нараст.={3:.3f}с, "
            "устан.={4:.3f}с, частота={5:.2f}Гц [угловой]",
        "ev_no_events": "событий=0 по всем режимам/осям",
        "json_disclaimer":
            "Это эвристические оценки. Меняйте по одному параметру. "
            "Тестируйте в SITL перед реальным полётом.",
    },
}


# ── data classes ──────────────────────────────────────────────────────────────

@dataclass
class StepEvent:
    axis: str
    t_start: float
    t_end: float
    sp_before: float
    sp_after: float
    step_size: float
    overshoot_pct: float
    rise_time_s: float
    settling_time_s: float
    steady_state_error: float
    rms_error: float
    dominant_freq_hz: float
    damping_ratio: float
    noise_rms: float
    control_effort: float
    model_r2: float = 0.0
    model_method: str = "none"
    amplitude_bin: int = 0        # which amplitude bin this step belongs to


@dataclass
class SegmentMetrics:
    mode: str
    axis: str
    loop: str
    n_events: int
    overshoot_pct_mean: float
    overshoot_pct_max: float
    rise_time_mean_s: float
    settling_time_mean_s: float
    rms_error_mean: float
    dominant_freq_hz: float
    noise_rms: float
    control_effort: float
    coherence_verdict: str = "unknown"
    model_r2_mean: float = 0.0
    integ_max: float = 0.0        # max |int_{axis}| over valid frames (I-windup)
    eph_max: float = 0.0          # max eph over frames (indoor/OF confounder)
    ss_error_signed: float = 0.0  # signed mean tracking error (DC offset, not RMS)
    events: List[StepEvent] = field(default_factory=list)


@dataclass
class DataQuality:
    """Measured recording quality — gate recommendations on these values."""
    primary_topic: str = "VehicleAngularVelocity"
    achieved_rate_hz: float = 0.0
    jitter_ms: float = 0.0
    drop_pct: float = 0.0
    f_nyquist_hz: float = 0.0
    overall_confidence: str = "low"     # "high" | "medium" | "low"
    spectral_ok: bool = False           # rate >= min_rate_hz_for_spectral
    n_events_by_axis: Dict[str, int] = field(default_factory=dict)
    topic_stats: Dict[str, dict] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)


@dataclass
class Recommendation:
    axis: str
    param: str
    current: Optional[float]
    suggested: Optional[float]
    delta_pct: float
    direction: str
    reason: str
    reason_key: str = ""
    reason_kwargs: list = field(default_factory=list)
    evidence: str = ""
    confidence: str = "medium"
    data_quality_gate: str = ""    # non-empty if suggestion was suppressed

    def to_dict(self) -> dict:
        d = asdict(self)
        del d["reason_key"]
        del d["reason_kwargs"]
        return d


@dataclass
class AnalysisResult:
    run_name: str
    timestamp: str
    params_loaded: bool
    topic_status: Dict[str, str]
    segments: List[SegmentMetrics]
    recommendations: List[Recommendation]
    warnings: List[str]
    data_quality: DataQuality = field(default_factory=DataQuality)
    vehicle_profile: str = "default"
    warning_meta: List[tuple] = field(default_factory=list)

    def to_dict(self) -> dict:
        d = asdict(self)
        del d["warning_meta"]
        return d


# ── internal helpers ──────────────────────────────────────────────────────────

def _col(frames: List[TelemetryFrame], name: str) -> np.ndarray:
    """Extract a float column: missing / None → NaN.

    Unlike ``getattr(...) or np.nan`` this preserves a valid 0.0 (e.g. a hover
    setpoint) instead of turning it into "no data", and it marks real dropouts
    (None) as NaN so they can be masked rather than back-filled with a hard 0.0.
    """
    return np.array([np.nan if (v := getattr(f, name, None)) is None else float(v)
                     for f in frames])


def _noise_rms(arr: np.ndarray, fs: float) -> float:
    if len(arr) < 20 or fs <= 0:
        return 0.0
    try:
        cutoff = min(0.4, 10.0 / (fs / 2.0))
        b, a = sig.butter(2, cutoff, btype="high")
        hp = sig.filtfilt(b, a, arr)
        return float(np.sqrt(np.mean(hp ** 2)))
    except Exception:
        return float(np.std(np.diff(arr)))


def _dominant_freq_guarded(
    t: np.ndarray, err: np.ndarray, f_nyquist_hz: float,
    sp: Optional[np.ndarray] = None,
) -> Tuple[float, float, bool]:
    """
    Returns (freq_hz, damping_estimate, is_above_nyquist).
    Frequency above Nyquist returns (0, 1, True).  No prominent peak → (0, 1, False).

    A bare ``argmax`` finds a peak on *every* spectrum, including the spectrum of
    the commanded stick jabs themselves — which fabricates a "gain oscillation"
    on a perfectly tuned drone.  Two guards:

    * ``find_peaks`` with a prominence floor of 4×median PSD — no salient peak
      means no oscillation.
    * if ``sp`` is given, bins where the setpoint carries comparable energy are
      excluded: a gain-induced oscillation lives in the error but NOT in the
      command.
    """
    if len(err) < 16:
        return 0.0, 1.0, False
    try:
        _, err_uni, dt_med, _ = resample_uniform(t, err)
        fs = 1.0 / dt_med if dt_med > 0 else 50.0
        f_nyq_local = fs / 2.0

        nperseg = min(256, len(err_uni))
        freqs, psd = sig.welch(err_uni, fs=fs, nperseg=nperseg)
        mask = freqs > 0.3
        if not np.any(mask):
            return 0.0, 1.0, False

        med = float(np.median(psd[mask]))
        if not (med > 0.0):
            return 0.0, 1.0, False

        psd_masked = psd.copy()
        psd_masked[~mask] = 0.0

        # Exclude the setpoint's energy band — an oscillation caused by gains is
        # energy in the error that is NOT present in the command.
        if sp is not None and len(sp) == len(err):
            _, sp_uni, _, _ = resample_uniform(t, sp)
            n = min(len(sp_uni), len(err_uni))
            _, psd_sp = sig.welch(sp_uni[:n], fs=fs, nperseg=min(nperseg, n))
            m = min(len(psd_sp), len(psd_masked))
            contaminated = np.zeros_like(psd_masked, dtype=bool)
            contaminated[:m] = psd_sp[:m] > 0.25 * psd_masked[:m]
            psd_masked[contaminated] = 0.0

        prominence = 4.0 * med
        peaks, _props = sig.find_peaks(psd_masked, prominence=prominence)
        peaks = peaks[freqs[peaks] > 0.3]
        if len(peaks) == 0:
            return 0.0, 1.0, False

        peak_idx = int(peaks[int(np.argmax(psd_masked[peaks]))])
        f0 = float(freqs[peak_idx])
        if f0 <= 0.3:
            return 0.0, 1.0, False

        above, _ = nyquist_guard(f0, f_nyquist_hz if f_nyquist_hz > 0 else f_nyq_local)
        if above:
            return 0.0, 1.0, True

        # Half-power bandwidth over the *connected* band around the peak (walk
        # left/right until PSD drops below half-power), not a global count of
        # every in-mask bin above the threshold.
        half_power = psd_masked[peak_idx] / 2.0
        lo = peak_idx
        while lo - 1 >= 0 and psd_masked[lo - 1] > half_power:
            lo -= 1
        hi = peak_idx
        while hi + 1 < len(psd_masked) and psd_masked[hi + 1] > half_power:
            hi += 1
        df = float(freqs[1] - freqs[0])
        bw = float((hi - lo) * df)
        zeta = min(1.0, bw / (2.0 * f0)) if f0 > 0 else 1.0
        return f0, zeta, False
    except Exception:
        return 0.0, 1.0, False


def _topic_is_active(topic_status: Dict, ros_key: str, ulog_key: str) -> bool:
    """True if a setpoint topic carried data, tolerant of both topic_status shapes.

    Live DDS:  topic_status[ros_key] == "active" (a status string).
    ULog:      topic_status[ulog_key] is a TopicStats with msg_count > 0.
    """
    live = topic_status.get(ros_key)
    if live == "active":
        return True
    stat = topic_status.get(ulog_key)
    if stat is None:
        return False
    if isinstance(stat, str):
        return stat == "active"
    # TopicStats (or any object exposing msg_count) — present with samples = active.
    return bool(getattr(stat, "msg_count", 0))


def _segment_by_mode(
    frames: List[TelemetryFrame],
) -> List[Tuple[str, List[TelemetryFrame]]]:
    """Split frames into contiguous runs — a new run starts on every nav_state
    change.  Returning a dict keyed by mode name would splice non-adjacent
    intervals (STAB → POSCTL → STAB) into one list, fabricating a "step" and a
    wrong ``fs_eff = len/duration`` across the time gap.  Mode names may repeat
    in the returned sequence — that is expected."""
    segments: List[Tuple[str, List[TelemetryFrame]]] = []
    current_state: Optional[int] = None
    current_key = "UNKNOWN"
    current_frames: List[TelemetryFrame] = []
    started = False
    for f in frames:
        ns = f.nav_state
        if not started or ns != current_state:
            if current_frames:
                segments.append((current_key, current_frames))
            current_state = ns
            current_key = NAV_STATE_MAP.get(ns, f"STATE_{ns}") if ns is not None else "UNKNOWN"
            current_frames = []
            started = True
        current_frames.append(f)
    if current_frames:
        segments.append((current_key, current_frames))
    return segments


def _downgrade_confidence(conf: str, steps: int = 1) -> str:
    order = ["high", "medium", "low"]
    idx = order.index(conf) if conf in order else 2
    return order[min(idx + steps, 2)]


def _build_data_quality(
    topic_stats: Dict[str, TopicStats],
    frames: List[TelemetryFrame],
    quality_cfg: dict,
) -> DataQuality:
    dq = DataQuality()
    if not topic_stats:
        return dq

    prim = topic_stats.get("VehicleAngularVelocity")
    if prim and prim.msg_count > 1:
        dq.achieved_rate_hz = prim.achieved_rate_hz
        dq.jitter_ms = prim.jitter_ms
        dq.drop_pct = prim.drop_pct
        dq.f_nyquist_hz = prim.f_nyquist_hz

    dq.topic_stats = {k: asdict(v) for k, v in topic_stats.items()}

    min_rate = quality_cfg.get("min_rate_hz_for_spectral", 10.0)
    max_jit = quality_cfg.get("max_jitter_pct_for_high_conf", 20.0)
    max_drop = quality_cfg.get("max_drop_pct_for_high_conf", 15.0)

    dq.spectral_ok = dq.achieved_rate_hz >= min_rate

    jitter_pct = (dq.jitter_ms / (1000.0 / dq.achieved_rate_hz) * 100.0
                  if dq.achieved_rate_hz > 0 else 100.0)

    if (dq.achieved_rate_hz >= min_rate
            and jitter_pct <= max_jit
            and dq.drop_pct <= max_drop):
        dq.overall_confidence = "high"
    elif dq.achieved_rate_hz >= min_rate / 2.0:
        dq.overall_confidence = "medium"
    else:
        dq.overall_confidence = "low"

    if not dq.spectral_ok:
        dq.notes.append(
            f"Sample rate {dq.achieved_rate_hz:.1f} Hz < {min_rate:.1f} Hz — "
            f"spectral analysis disabled (Nyquist {dq.f_nyquist_hz:.1f} Hz). "
            f"Use Ethernet DDS link for higher rates."
        )
    return dq


# ── step detection ────────────────────────────────────────────────────────────

WINDOW_AFTER_S = 2.5
MIN_STEP_GAP_S = 1.0


def _detect_and_analyse_steps(
    t: np.ndarray, sp: np.ndarray, meas: np.ndarray,
    ctrl: Optional[np.ndarray], threshold: float, fs: float,
    f_nyquist_hz: float,
) -> List[StepEvent]:
    events: List[StepEvent] = []
    if len(t) < 10:
        return events

    dsp = np.abs(np.diff(sp))
    candidates = np.where(dsp > threshold)[0]
    last_t = -999.0

    for j, idx in enumerate(candidates):
        t_step = float(t[idx])
        if t_step - last_t < MIN_STEP_GAP_S:
            continue
        last_t = t_step

        # Truncate this step's analysis window at the NEXT raw candidate — even
        # ones MIN_STEP_GAP_S will later drop still move the setpoint and would
        # otherwise pollute overshoot/settling (e.g. the stick returning).
        next_cand_idx = int(candidates[j + 1]) if j + 1 < len(candidates) else len(sp)
        next_cand_t = float(t[candidates[j + 1]]) if j + 1 < len(candidates) else np.inf

        sp_before = float(np.median(sp[max(0, idx - 10): max(1, idx)]))
        sp_hi = max(idx + 2, min(idx + 15, next_cand_idx))
        sp_after = float(np.median(sp[idx + 1: sp_hi]))
        step_size = sp_after - sp_before

        # Skip near-zero steps (medians can shrink a candidate below threshold)
        if abs(step_size) < threshold:
            continue

        win_end = int(np.searchsorted(t, min(t_step + WINDOW_AFTER_S, next_cand_t)))
        win_end = min(win_end, len(t) - 1)
        if win_end - idx < 5:
            continue

        tw  = t[idx: win_end]
        spw = sp[idx: win_end]
        mw  = meas[idx: win_end]
        err = spw - mw

        # Normalised response [0 → 1], sign-preserving so it rises for both
        # positive and negative steps.
        denom = step_size + math.copysign(1e-9, step_size)
        norm = (mw - mw[0]) / denom

        # Overshoot — norm rises toward 1 for both signs, so the peak is the max.
        peak = float(np.max(norm))
        overshoot_pct = max(0.0, (peak - 1.0) * 100.0)

        # Rise time 10→90% via first threshold crossing (response is not
        # monotone, so searchsorted would return garbage indices).
        i10 = int(np.argmax(norm >= 0.10)) if np.any(norm >= 0.10) else -1
        i90 = int(np.argmax(norm >= 0.90)) if np.any(norm >= 0.90) else -1
        rise_time = float(tw[i90] - tw[i10]) if 0 <= i10 < i90 else float("nan")

        # Settling time ±5%
        settled = np.abs(norm - 1.0) < 0.05
        if np.any(settled):
            try:
                i_last_unsettled = int(np.where(~settled)[0][-1])
                settling_time = float(tw[i_last_unsettled] - tw[0])
            except Exception:
                settling_time = float(tw[np.where(settled)[0][0]] - tw[0])
        else:
            # Window may be truncated by the next step — report its real length,
            # not the nominal WINDOW_AFTER_S constant.
            settling_time = float(tw[-1] - tw[0])

        ss_error = float(np.mean(err[-max(1, len(err) // 5):]))
        rms_error = float(np.sqrt(np.mean(err ** 2)))

        dom_freq, damping, above_nyq = _dominant_freq_guarded(tw, err, f_nyquist_hz)
        if above_nyq:
            dom_freq, damping = 0.0, 1.0

        noise = _noise_rms(mw, fs)
        # ctrl may carry NaN at dropped samples (kept synchronous, not zero-filled) —
        # use a NaN-aware RMS so a single dropout doesn't null the effort estimate.
        ctrl_effort = 0.0
        if ctrl is not None:
            cslice = ctrl[idx: win_end]
            if np.any(np.isfinite(cslice)):
                ctrl_effort = float(np.sqrt(np.nanmean(cslice ** 2)))

        # Model fit for this step window
        fit: ModelFit = fit_second_order(tw, mw, sp_before, sp_after, f_nyquist_hz)

        events.append(StepEvent(
            axis="", t_start=t_step, t_end=float(t[win_end]),
            sp_before=sp_before, sp_after=sp_after,
            step_size=abs(step_size),
            overshoot_pct=round(overshoot_pct, 1),
            rise_time_s=round(rise_time, 3),
            settling_time_s=round(settling_time, 3),
            steady_state_error=round(ss_error, 4),
            rms_error=round(rms_error, 4),
            dominant_freq_hz=round(dom_freq, 2),
            damping_ratio=round(fit.zeta if fit.r_squared > 0.55 else damping, 3),
            noise_rms=round(noise, 5),
            control_effort=round(ctrl_effort, 4),
            model_r2=round(fit.r_squared, 3),
            model_method=fit.method,
        ))
    return events


# ── main analyser ─────────────────────────────────────────────────────────────

class PIDAnalyzer:
    def __init__(
        self,
        params: Optional[Dict[str, float]] = None,
        lang: str = "en",
        vehicle_profile: str = "default",
        quality_cfg: Optional[dict] = None,
    ):
        self.params = params or {}
        self.lang = lang if lang in REASON_TR else "en"
        self.profile = _PROFILES.get(vehicle_profile, _PROFILES["default"])
        self.vehicle_profile = vehicle_profile
        self.quality_cfg = quality_cfg or _QUALITY_DEFAULTS

    def _t(self, key: str, *args) -> str:
        templates = REASON_TR.get(self.lang, REASON_TR["en"])
        fmt = templates.get(key) or REASON_TR["en"].get(key, key)
        return fmt.format(*args) if args and "{" in fmt else fmt

    # ── main entry point ──────────────────────────────────────────────────────

    def analyse(
        self,
        frames: List[TelemetryFrame],
        run_name: str,
        topic_status: Dict[str, str],
        topic_stats: Optional[Dict[str, TopicStats]] = None,
    ) -> AnalysisResult:
        warnings_out: List[str] = []
        warning_meta_out: List[tuple] = []
        seg_metrics: List[SegmentMetrics] = []

        dq = _build_data_quality(topic_stats or {}, frames,
                                  self.quality_cfg)

        if not frames:
            return AnalysisResult(
                run_name=run_name,
                timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
                params_loaded=bool(self.params),
                topic_status=topic_status,
                segments=[], recommendations=[],
                warnings=[self._t("warn_no_frames")],
                data_quality=dq,
                vehicle_profile=self.vehicle_profile,
                warning_meta=[("warn_no_frames", [])],
            )

        # Low-rate warning
        if not dq.spectral_ok and dq.achieved_rate_hz > 0:
            min_r = self.quality_cfg.get("min_rate_hz_for_spectral", 10.0)
            msg = self._t("low_sample_rate", dq.achieved_rate_hz, dq.f_nyquist_hz)
            warnings_out.append(msg)
            warning_meta_out.append(("low_sample_rate",
                                     [dq.achieved_rate_hz, dq.f_nyquist_hz]))

        # Missing setpoint topics. The two data paths key topic_status differently:
        # the live DDS node uses ROS class names ("VehicleRatesSetpoint") mapped to a
        # status string ("active"), while the ULog path passes topic_stats keyed by the
        # snake_case log dataset name ("vehicle_rates_setpoint") to TopicStats objects.
        # Accept either representation, otherwise every ULog load raised a bogus
        # "topic never received data" warning (P1-7).
        _setpoint_topics = {
            "VehicleRatesSetpoint": "vehicle_rates_setpoint",
            "VehicleAttitudeSetpoint": "vehicle_attitude_setpoint",
        }
        for needed, ulog_key in _setpoint_topics.items():
            if _topic_is_active(topic_status, needed, ulog_key):
                continue
            warnings_out.append(self._t("warn_topic_missing", needed))
            warning_meta_out.append(("warn_topic_missing", [needed]))

        # Use PX4 timestamps: t field is already seconds from recording start
        mode_segments = _segment_by_mode(frames)
        f_nyq = dq.f_nyquist_hz

        n_events_by_axis: Dict[str, int] = {a: 0 for a in AXES}

        for mode, mode_frames in mode_segments:
            if len(mode_frames) < 10:
                continue

            t_arr  = np.array([f.t for f in mode_frames])
            dur    = float(t_arr[-1] - t_arr[0])
            fs_eff = float(len(t_arr)) / max(1.0, dur)

            # Step-detection setpoint threshold (rad/s for rate loop).  Racers fly
            # larger, faster stick jabs, so a higher threshold avoids treating
            # ordinary control activity as a step; other profiles use 0.15.
            rate_step_thresh = 0.25 if self.vehicle_profile == "racer" else 0.15
            loop_specs = [
                ("rate",     "{ax}_rate_sp", "{ax}_rate",  "torque_{ax}",
                 rate_step_thresh),
                ("attitude", "{ax}_sp",      "{ax}",       "torque_{ax}", 0.05),
            ]

            for axis in AXES:
                for loop_name, sp_tmpl, meas_tmpl, ctrl_tmpl, threshold in loop_specs:
                    sp_f   = sp_tmpl.replace("{ax}", axis)
                    meas_f = meas_tmpl.replace("{ax}", axis)
                    ctrl_f = ctrl_tmpl.replace("{ax}", axis)

                    # NaN-safe columns: None → NaN, valid 0.0 preserved.
                    sp_arr   = _col(mode_frames, sp_f)
                    meas_arr = _col(mode_frames, meas_f)
                    ctrl_arr = _col(mode_frames, ctrl_f)

                    sp_ok   = ~np.isnan(sp_arr)
                    meas_ok = ~np.isnan(meas_arr)
                    ctrl_ok = ~np.isnan(ctrl_arr)
                    # "have data" = at least 20 valid samples (not merely "not all NaN").
                    has_sp   = int(np.count_nonzero(sp_ok))   >= 20
                    has_meas = int(np.count_nonzero(meas_ok)) >= 20

                    if loop_name == "attitude" and not has_sp:
                        continue

                    # Mask to samples where BOTH sp and meas are present — never
                    # back-fill dropouts with 0.0 (that fabricates step edges in
                    # np.diff(sp) and poisons the error signal).
                    valid = sp_ok & meas_ok
                    n_valid = int(np.count_nonzero(valid))

                    t_valid    = t_arr[valid]
                    sp_valid   = sp_arr[valid]
                    meas_valid = meas_arr[valid]
                    # ctrl kept synchronous with the valid mask; may still hold NaN
                    # where only ctrl dropped — RMS/effort use NaN-aware means.
                    ctrl_valid = ctrl_arr[valid] if np.any(ctrl_ok) else None

                    events: List[StepEvent] = []
                    if has_sp and has_meas and n_valid >= 20:
                        events = _detect_and_analyse_steps(
                            t_valid, sp_valid, meas_valid, ctrl_valid,
                            threshold, fs_eff, f_nyq,
                        )
                        for ev in events:
                            ev.axis = axis
                            n_events_by_axis[axis] += 1

                    # Assign amplitude bins so callers can group comparable events.
                    # Segment means are aggregated over the DOMINANT bin only
                    # (the bin holding the most events) — mixing incomparable
                    # step amplitudes was the docstring's stated anti-goal.  All
                    # events are still stored on the segment.
                    dom_events = events
                    if events:
                        sizes = np.array([ev.step_size for ev in events])
                        bins  = bin_steps_by_amplitude(sizes)
                        for ev, b in zip(events, bins):
                            ev.amplitude_bin = int(b)
                        bin_ids, counts = np.unique(
                            [ev.amplitude_bin for ev in events], return_counts=True)
                        dom_bin = int(bin_ids[int(np.argmax(counts))])
                        dom_events = [ev for ev in events
                                      if ev.amplitude_bin == dom_bin]

                    noise  = _noise_rms(meas_arr[meas_ok], fs_eff) if has_meas else 0.0
                    effort = (float(np.sqrt(np.mean(ctrl_arr[ctrl_ok] ** 2)))
                              if np.any(ctrl_ok) else 0.0)

                    # Per-axis integrator peak (RateCtrlStatus int_{axis}) and
                    # position-estimate error peak (eph) — feed the honest
                    # I-windup and indoor-confounder checks.  NaN-safe.
                    int_arr = _col(mode_frames, f"int_{axis}")
                    int_ok  = ~np.isnan(int_arr)
                    integ_max = (float(np.nanmax(np.abs(int_arr[int_ok])))
                                 if np.any(int_ok) else 0.0)
                    eph_arr = _col(mode_frames, "eph")
                    eph_ok  = ~np.isnan(eph_arr)
                    eph_max = (float(np.nanmax(eph_arr[eph_ok]))
                               if np.any(eph_ok) else 0.0)

                    err = (meas_valid - sp_valid) if (has_sp and has_meas) else np.zeros(0)
                    # Signed mean error = the DC steady-state offset (RMS cannot
                    # tell a real offset from symmetric noise — see rule r5).
                    ss_err_signed = float(np.mean(err)) if len(err) else 0.0
                    dom_f = 0.0
                    if has_sp and has_meas and len(err) >= 16:
                        dom_f, _, _ = _dominant_freq_guarded(
                            t_valid, err, f_nyq, sp=sp_valid)

                    # Coherence for the whole segment (only where ctrl is present)
                    coh = {"verdict": "insufficient_data"}
                    if has_sp and has_meas and np.any(ctrl_ok):
                        cmask = valid & ctrl_ok
                        if int(np.count_nonzero(cmask)) >= 32:
                            coh = coherence_verdict(
                                t_arr[cmask],
                                meas_arr[cmask] - sp_arr[cmask],
                                ctrl_arr[cmask],
                            )

                    if events:
                        r2s = [ev.model_r2 for ev in events]
                        # The three step means aggregate over the dominant bin;
                        # overshoot_max / rms / R² stay over all events.
                        # NaN rise times (no clean 10→90% crossing) must not
                        # drag the segment mean to NaN — average finite ones only.
                        rise_finite = [e.rise_time_s for e in dom_events
                                       if math.isfinite(e.rise_time_s)]
                        rise_mean = float(np.mean(rise_finite)) if rise_finite else 0.0
                        sm = SegmentMetrics(
                            mode=mode, axis=axis, loop=loop_name,
                            n_events=len(events),
                            overshoot_pct_mean=float(np.mean([e.overshoot_pct for e in dom_events])),
                            overshoot_pct_max=float(np.max([e.overshoot_pct for e in events])),
                            rise_time_mean_s=rise_mean,
                            settling_time_mean_s=float(np.mean([e.settling_time_s for e in dom_events])),
                            rms_error_mean=float(np.mean([e.rms_error for e in events])),
                            dominant_freq_hz=dom_f,
                            noise_rms=noise,
                            control_effort=effort,
                            coherence_verdict=coh.get("verdict", "unknown"),
                            model_r2_mean=float(np.mean(r2s)),
                            integ_max=integ_max,
                            eph_max=eph_max,
                            ss_error_signed=ss_err_signed,
                            events=events,
                        )
                    else:
                        sm = SegmentMetrics(
                            mode=mode, axis=axis, loop=loop_name,
                            n_events=0,
                            overshoot_pct_mean=0.0, overshoot_pct_max=0.0,
                            rise_time_mean_s=0.0, settling_time_mean_s=0.0,
                            rms_error_mean=(float(np.sqrt(np.mean(err ** 2)))
                                            if len(err) else 0.0),
                            dominant_freq_hz=dom_f,
                            noise_rms=noise,
                            control_effort=effort,
                            coherence_verdict=coh.get("verdict", "unknown"),
                            model_r2_mean=0.0,
                            integ_max=integ_max,
                            eph_max=eph_max,
                            ss_error_signed=ss_err_signed,
                        )
                    seg_metrics.append(sm)

        dq.n_events_by_axis = n_events_by_axis
        recommendations = self._generate_recommendations(seg_metrics, dq)

        return AnalysisResult(
            run_name=run_name,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            params_loaded=bool(self.params),
            topic_status=topic_status,
            segments=seg_metrics,
            recommendations=recommendations,
            warnings=warnings_out,
            data_quality=dq,
            vehicle_profile=self.vehicle_profile,
            warning_meta=warning_meta_out,
        )

    # ── recommendation engine ─────────────────────────────────────────────────

    def _generate_recommendations(
        self,
        segments: List[SegmentMetrics],
        dq: DataQuality,
    ) -> List[Recommendation]:
        recs: List[Recommendation] = []
        p = self.profile
        q = self.quality_cfg
        min_ev = q.get("min_events_for_high_conf", 3)
        min_r2 = q.get("min_r2_for_model_suggestion", 0.60)

        by_axis:     Dict[str, List[SegmentMetrics]] = {a: [] for a in AXES}
        by_axis_att: Dict[str, List[SegmentMetrics]] = {a: [] for a in AXES}
        for sm in segments:
            if sm.loop == "rate":
                by_axis[sm.axis].append(sm)
            elif sm.loop == "attitude":
                by_axis_att[sm.axis].append(sm)

        for axis in AXES:
            segs = by_axis[axis]
            if not segs:
                continue
            AX = axis.upper()

            priority_modes = ("STAB", "ACRO", "MANUAL", "ALTCTL", "POSCTL")
            best = next(
                (s for m in priority_modes for s in segs if s.mode == m and s.n_events > 0),
                next((s for s in segs if s.n_events > 0), segs[0]),
            )

            # Pick best model fit across events in best segment
            best_fit: Optional[StepEvent] = None
            if best.events:
                best_fit = max(best.events, key=lambda e: e.model_r2)

            evidence = self._t(
                "ev_rate", best.mode, best.n_events,
                best.overshoot_pct_mean, best.rise_time_mean_s,
                best.settling_time_mean_s, best.dominant_freq_hz, best.noise_rms,
            )

            def cur(name: str) -> Optional[float]:
                return self.params.get(name)

            def make_rec(
                param: str, delta_pct: float,
                reason_key: str, *reason_args,
                confidence: str,
            ) -> Recommendation:
                conf = confidence
                # Downgrade if data quality is poor
                if best.n_events < min_ev:
                    conf = _downgrade_confidence(conf)
                if dq.overall_confidence == "low":
                    conf = _downgrade_confidence(conf)
                # Clamp the magnitude (sign-preserving) — no single iteration may
                # move a gain more than MAX_DELTA_PCT.
                clamped = float(np.clip(delta_pct, -MAX_DELTA_PCT, MAX_DELTA_PCT))
                was_clamped = abs(clamped - delta_pct) > 1e-9
                delta_pct = clamped
                # Suppress suggested value if model quality is insufficient
                curr_val = cur(param)
                gate = ""
                if curr_val is not None and (
                    best_fit is None or best_fit.model_r2 < min_r2
                ):
                    suggested = None
                    gate = (f"model R²={best_fit.model_r2:.2f}" if best_fit
                            else "no model fit")
                else:
                    suggested = (round(curr_val * (1 + delta_pct / 100.0), 4)
                                 if curr_val is not None else None)
                reason = self._t(reason_key, *reason_args)
                if was_clamped and reason_key in (
                    "r1_oscillation", "r1_oscillation_no_model"
                ):
                    reason += self._t("clamp_note", delta_pct)
                return Recommendation(
                    axis=axis, param=param,
                    current=curr_val, suggested=suggested,
                    delta_pct=round(delta_pct, 1),
                    direction="increase" if delta_pct > 0 else "decrease",
                    reason=reason,
                    reason_key=reason_key,
                    reason_kwargs=list(reason_args),
                    evidence=evidence,
                    confidence=conf,
                    data_quality_gate=gate,
                )

            ovs  = best.overshoot_pct_mean
            rt   = best.rise_time_mean_s
            freq = best.dominant_freq_hz
            damp = (best_fit.damping_ratio if best_fit and best_fit.model_r2 > 0.55
                    else (best.events[0].damping_ratio if best.events else 1.0))
            r2   = best_fit.model_r2 if best_fit else 0.0
            noise = best.noise_rms
            coh = best.coherence_verdict

            # ── Rule 1: oscillation ───────────────────────────────────────────
            if freq > 1.0 and damp < 0.5 and best.n_events > 0:
                if r2 >= min_r2 and best_fit and best_fit.damping_ratio > 0.01:
                    # Model-based: compute scale for ζ → 0.7
                    k_scale = round((best_fit.damping_ratio / 0.7) ** 2, 2)
                    delta = round((k_scale - 1.0) * 100.0, 1)
                    recs.append(make_rec(
                        f"MC_{AX}RATE_K", delta,
                        "r1_oscillation", freq, damp, r2, k_scale,
                        confidence="high" if damp < 0.3 else "medium",
                    ))
                else:
                    recs.append(make_rec(
                        f"MC_{AX}RATE_K", -15.0,
                        "r1_oscillation_no_model", freq,
                        confidence="medium",
                    ))

            # ── Rule 2: high overshoot (multi-hypothesis) ─────────────────────
            elif ovs > p["overshoot_warn_pct"]:
                conf = "high" if ovs > p["overshoot_high_pct"] else "medium"

                # Hypothesis A: K too high (primary)
                recs.append(make_rec(
                    f"MC_{AX}RATE_K", -20.0,
                    "r2_overshoot_K", ovs, p["overshoot_warn_pct"],
                    "coherence=" + coh,
                    confidence=conf,
                ))
                # Hypothesis B: D too low.  coherence is only a soft gate here —
                # if err and torque have decorrelated (low_coherence) the loop is
                # not in a clean gain regime, so a "raise D" tip is unreliable.
                if coh != "low_coherence" and noise < p["noise_warn"]:
                    recs.append(make_rec(
                        f"MC_{AX}RATE_D", +15.0,
                        "r2_overshoot_D",
                        confidence="medium",
                    ))
                # Hypothesis C: I-windup — the integrator (RateCtrlStatus
                # int_{axis}) is riding near its clamp MC_RR_INT_LIM (default
                # 0.30) while the loop overshoots.
                int_lim = float(self.params.get("MC_RR_INT_LIM", 0.30))
                if best.integ_max > 0.5 * int_lim:
                    recs.append(make_rec(
                        f"MC_{AX}RATE_I", -15.0,
                        "r2_overshoot_I", best.integ_max,
                        confidence="low",
                    ))
                if noise > p["noise_warn"]:
                    recs.append(make_rec(
                        f"MC_{AX}RATE_D", -15.0,
                        "r6_noise_D", noise, p["noise_warn"], coh,
                        confidence="medium",
                    ))

            # ── Rule 3: low-frequency oscillation ────────────────────────────
            if 0.2 < freq < 1.0 and damp < 0.6:
                recs.append(make_rec(
                    f"MC_{AX}_P", -15.0,
                    "r3_low_freq_osc", freq, best.mode,
                    confidence="medium",
                ))

            # ── Rule 4: slow rise ─────────────────────────────────────────────
            # Yaw is inherently slower than roll/pitch — scale its rise threshold
            # so a normally-lagging yaw axis is not flagged as "slow".
            rise_thresh = p["rise_time_slow_s"]
            if axis == "yaw":
                rise_thresh *= p.get("yaw_rise_scale", 1.0)
            if rt > rise_thresh and best.n_events > 0:
                recs.append(make_rec(
                    f"MC_{AX}RATE_K", +15.0,
                    "r4_slow_rise", rt, rise_thresh,
                    confidence="medium" if rt > rise_thresh * 1.5 else "low",
                ))

            # ── Rule 5: steady-state error (ONLY on hold segments) ───────────
            # Measure ss_error on the tail of the recording in stable modes,
            # not on transient step windows.
            hold_segs = [s for s in segs
                         if s.mode in ("ALTCTL", "POSCTL", "AUTO_LOITER")]
            if hold_segs:
                hold = hold_segs[0]
                # DC offset (signed mean), not RMS: RMS is inflated by symmetric
                # noise and would fire the I-up rule on a well-centred hold.
                hold_err = abs(hold.ss_error_signed)
                if hold_err > p["ss_error_rad_s"]:
                    recs.append(make_rec(
                        f"MC_{AX}RATE_I", +20.0,
                        "r5_steady_state_err", hold_err,
                        confidence="medium",
                    ))

            # ── Rule 6: sensor noise (D or filter) ───────────────────────────
            if noise > p["noise_warn"]:
                recs.append(make_rec(
                    f"MC_{AX}RATE_D", -25.0,
                    "r6_noise_D", noise, p["noise_warn"], coh,
                    confidence="high" if noise > p["noise_high"] else "medium",
                ))
                if noise > p["noise_high"]:
                    recs.append(make_rec(
                        "IMU_GYRO_CUTOFF", -15.0,
                        "r6_noise_cutoff",
                        confidence="medium",
                    ))

            # ── Rule 7: cross-mode divergence ─────────────────────────────────
            stab_segs = [s for s in segs if s.mode in ("STAB", "MANUAL", "ACRO")]
            pos_segs  = [s for s in segs if s.mode == "POSCTL"]
            if stab_segs and pos_segs:
                stab_ok = all(s.overshoot_pct_mean < p["overshoot_warn_pct"]
                              for s in stab_segs)
                pos_bad = any(s.dominant_freq_hz > 1.0
                              or s.overshoot_pct_mean > p["overshoot_warn_pct"] * 1.5
                              for s in pos_segs)
                if stab_ok and pos_bad:
                    recs.append(Recommendation(
                        axis=axis, param="NOTE",
                        current=None, suggested=None,
                        delta_pct=0.0, direction="info",
                        reason=self._t("r7_cross_mode", axis),
                        reason_key="r7_cross_mode", reason_kwargs=[axis],
                        evidence=evidence, confidence="medium",
                    ))

            # ── Indoor/OF confounder check ─────────────────────────────────────
            # Only flag when a POSCTL segment actually oscillated AND its position
            # estimate degraded (eph spike > 0.5 m) — otherwise the note was a
            # fabricated "eph 0.00 m" that fired on every position hold.
            for pos_seg in pos_segs:
                if pos_seg.dominant_freq_hz > 1.0 and pos_seg.eph_max > 0.5:
                    recs.append(Recommendation(
                        axis=axis, param="NOTE",
                        current=None, suggested=None,
                        delta_pct=0.0, direction="info",
                        reason=self._t("indoor_confounder", axis, pos_seg.eph_max),
                        reason_key="indoor_confounder",
                        reason_kwargs=[axis, pos_seg.eph_max],
                        evidence=evidence, confidence="low",
                    ))
                    break

        # ── Attitude-loop recommendations ─────────────────────────────────────
        for axis in AXES:
            att_segs = by_axis_att[axis]
            if not att_segs:
                continue
            AX = axis.upper()
            best_att = next((s for s in att_segs if s.n_events > 0), att_segs[0])
            att_ev = self._t(
                "ev_attitude", best_att.mode, best_att.n_events,
                best_att.overshoot_pct_mean, best_att.rise_time_mean_s,
                best_att.settling_time_mean_s, best_att.dominant_freq_hz,
            )
            best_att_fit = (max(best_att.events, key=lambda e: e.model_r2)
                            if best_att.events else None)

            def cur_a(name: str) -> Optional[float]:
                return self.params.get(name)

            def att_rec(
                param: str, delta_pct: float,
                reason_key: str, *reason_args,
                confidence: str,
            ) -> Recommendation:
                conf = _downgrade_confidence(confidence) if best_att.n_events < min_ev else confidence
                # Sign-preserving magnitude clamp — same cap as the rate loop.
                delta_pct = float(np.clip(delta_pct, -MAX_DELTA_PCT, MAX_DELTA_PCT))
                cv = cur_a(param)
                gate = ""
                if cv is not None and (
                    best_att_fit is None or best_att_fit.model_r2 < min_r2
                ):
                    sug = None
                    gate = (f"model R²={best_att_fit.model_r2:.2f}" if best_att_fit
                            else "no model fit")
                else:
                    sug = round(cv * (1 + delta_pct / 100.0), 4) if cv is not None else None
                return Recommendation(
                    axis=axis, param=param, current=cv, suggested=sug,
                    delta_pct=round(delta_pct, 1),
                    direction="increase" if delta_pct > 0 else "decrease",
                    reason=self._t(reason_key, *reason_args),
                    reason_key=reason_key, reason_kwargs=list(reason_args),
                    evidence=att_ev, confidence=conf,
                    data_quality_gate=gate,
                )

            ovs_att  = best_att.overshoot_pct_mean
            rt_att   = best_att.rise_time_mean_s
            freq_att = best_att.dominant_freq_hz
            damp_att = (best_att_fit.damping_ratio
                        if best_att_fit and best_att_fit.model_r2 > 0.55
                        else (best_att.events[0].damping_ratio if best_att.events else 1.0))

            if ovs_att > 25.0 and best_att.n_events > 0:
                recs.append(att_rec(
                    f"MC_{AX}_P", -15.0,
                    "a1_att_overshoot", ovs_att, axis, AX,
                    confidence="high" if ovs_att > 40 else "medium",
                ))
            if 0.2 < freq_att < 1.0 and damp_att < 0.6 and best_att.n_events > 0:
                recs.append(att_rec(
                    f"MC_{AX}_P", -20.0,
                    "a2_att_oscillation", freq_att, damp_att, AX,
                    confidence="high",
                ))
            if rt_att > 0.5 and best_att.n_events > 0:
                recs.append(att_rec(
                    f"MC_{AX}_P", +15.0,
                    "a3_att_slow_rise", rt_att, AX,
                    confidence="low" if rt_att < 0.8 else "medium",
                ))

        recs = self._arbitrate(recs)

        if not recs:
            recs.append(Recommendation(
                axis="all", param="NOTE",
                current=None, suggested=None,
                delta_pct=0.0, direction="info",
                reason=self._t("no_step_events"),
                reason_key="no_step_events", reason_kwargs=[],
                evidence=self._t("ev_no_events"),
                confidence="high",
            ))

        return recs

    # ── recommendation arbitration ────────────────────────────────────────────

    @staticmethod
    def _arbitrate(recs: List[Recommendation]) -> List[Recommendation]:
        """Resolve contradictory / duplicate suggestions before returning.

        The rules are independent, so the same (axis, param) can collect opposing
        deltas (e.g. r2 "K −20%" and r4 "K +15%").  Collapse each group:

        1. If r1 or r2_overshoot_K fired for an axis (K down), drop r4 (K up) on
           the same axis outright.
        2. Deduplicate identical (axis, param, direction) → keep the strongest
           (larger |Δ|, then higher confidence).
        3. On a remaining direction conflict, keep the highest-priority rule and
           drop the rest.

        NOTE recommendations pass through untouched.
        """
        notes = [r for r in recs if r.param == "NOTE"]
        actionable = [r for r in recs if r.param != "NOTE"]

        k_down_axes = {
            r.axis for r in actionable
            if r.reason_key in ("r1_oscillation", "r1_oscillation_no_model",
                                "r2_overshoot_K")
        }
        actionable = [
            r for r in actionable
            if not (r.reason_key == "r4_slow_rise" and r.axis in k_down_axes)
        ]

        def _stronger(a: Recommendation, b: Recommendation) -> bool:
            if abs(a.delta_pct) != abs(b.delta_pct):
                return abs(a.delta_pct) > abs(b.delta_pct)
            return (_CONF_RANK.get(a.confidence, 2)
                    < _CONF_RANK.get(b.confidence, 2))

        groups: Dict[Tuple[str, str], List[Recommendation]] = {}
        order: List[Tuple[str, str]] = []
        for r in actionable:
            key = (r.axis, r.param)
            if key not in groups:
                groups[key] = []
                order.append(key)
            groups[key].append(r)

        resolved: List[Recommendation] = []
        for key in order:
            group = groups[key]
            # Deduplicate within each direction.
            by_dir: Dict[str, Recommendation] = {}
            for r in group:
                cur = by_dir.get(r.direction)
                if cur is None or _stronger(r, cur):
                    by_dir[r.direction] = r
            kept = list(by_dir.values())
            # Direction conflict → keep the highest-priority rule(s) only.
            if len({r.direction for r in kept}) > 1:
                best_prio = min(_REC_PRIORITY.get(r.reason_key, _DEFAULT_PRIORITY)
                                for r in kept)
                kept = [r for r in kept
                        if _REC_PRIORITY.get(r.reason_key, _DEFAULT_PRIORITY)
                        == best_prio]
            resolved.extend(kept)

        return resolved + notes

    # ── export helpers ────────────────────────────────────────────────────────

    def export_json(self, result: AnalysisResult, frames: List[TelemetryFrame]) -> str:
        doc = {
            "schema_version": "2.0",
            "meta": {
                "run_name": result.run_name,
                "timestamp": result.timestamp,
                "params_loaded": result.params_loaded,
                "params": self.params,
                "topic_status": result.topic_status,
                "n_frames": len(frames),
                "vehicle_profile": result.vehicle_profile,
            },
            "data_quality": asdict(result.data_quality),
            "warnings": result.warnings,
            "segments": [
                {
                    "mode": sm.mode, "axis": sm.axis, "loop": sm.loop,
                    "n_events": sm.n_events,
                    "coherence_verdict": sm.coherence_verdict,
                    "model_r2_mean": sm.model_r2_mean,
                    "metrics": {
                        "overshoot_pct_mean": sm.overshoot_pct_mean,
                        "overshoot_pct_max": sm.overshoot_pct_max,
                        "rise_time_mean_s": sm.rise_time_mean_s,
                        "settling_time_mean_s": sm.settling_time_mean_s,
                        "rms_error_mean": sm.rms_error_mean,
                        "dominant_freq_hz": sm.dominant_freq_hz,
                        "noise_rms": sm.noise_rms,
                        "control_effort": sm.control_effort,
                    },
                    "events": [asdict(e) for e in sm.events],
                }
                for sm in result.segments
            ],
            "recommendations": [r.to_dict() for r in result.recommendations],
            "disclaimer": self._t("json_disclaimer"),
        }
        return json.dumps(doc, indent=2, ensure_ascii=False)

    def export_csv(self, frames: List[TelemetryFrame]) -> str:
        import csv, io
        step = max(1, len(frames) // 5000)
        sampled = frames[::step]
        fieldnames = [
            "t", "wall_t", "nav_state",
            "roll_rate_sp", "roll_rate", "torque_roll",
            "pitch_rate_sp", "pitch_rate", "torque_pitch",
            "yaw_rate_sp", "yaw_rate", "torque_yaw",
            "roll_sp", "roll", "pitch_sp", "pitch", "yaw_sp", "yaw",
            "m0", "m1", "m2", "m3", "v_bat", "bat_remaining",
            "eph", "epv",
        ]
        buf = io.StringIO()
        w = csv.DictWriter(buf, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for f in sampled:
            w.writerow({fn: getattr(f, fn, None) for fn in fieldnames})
        return buf.getvalue()

    def export_parquet(self, frames: List[TelemetryFrame]) -> bytes:
        import pandas as pd
        import pyarrow as pa
        import pyarrow.parquet as pq
        import io
        records = [f.to_dict() for f in frames]
        df = pd.DataFrame(records)
        buf = io.BytesIO()
        pq.write_table(pa.Table.from_pandas(df), buf)
        return buf.getvalue()

    @staticmethod
    def _format_autotune_section(autotune) -> str:
        """
        Render the offline PX4-autotune result as a prompt section.
        ``autotune`` is an AutotuneResult or None; returns "" when absent.
        """
        if autotune is None or not getattr(autotune, "axes", None):
            return ""
        lines = [f"\n## Offline PX4 autotune ({autotune.method})",
                 f"  Native rate: {autotune.native_rate_hz:.0f} Hz"]
        for ax in autotune.axes:
            if ax.accepted:
                pieces = []
                for p, v in ax.params.items():
                    cur = "?" if v.get("current") is None else f"{v['current']:g}"
                    pieces.append(f"{p}: {cur}→{v['suggested']:g}")
                lines.append(
                    f"  [{ax.axis}] ACCEPTED  VAF={ax.vaf_pct:.1f}%  "
                    f"dt={ax.model_dt_s*1000:.1f}ms  kc={ax.kc:g} ki={ax.ki:g} kd={ax.kd:g}")
                lines.append("      " + "  ".join(pieces))
            else:
                lines.append(f"  [{ax.axis}] REJECTED — {ax.reason}")
        for w in autotune.warnings:
            lines.append(f"  ! {w}")
        return "\n".join(lines)

    def export_llm_prompt(self, result: AnalysisResult, autotune=None) -> str:
        params_text = describe_params(self.params) if self.params else "(no params loaded)"
        dq = result.data_quality
        quality_text = (
            f"  Sample rate: {dq.achieved_rate_hz:.1f} Hz "
            f"(f_nyq={dq.f_nyquist_hz:.1f} Hz, jitter={dq.jitter_ms:.1f} ms, "
            f"drops={dq.drop_pct:.1f}%, confidence={dq.overall_confidence})"
        )
        metrics_lines = []
        for sm in result.segments:
            if sm.n_events > 0:
                metrics_lines.append(
                    f"  [{sm.mode}][{sm.axis}][{sm.loop}] n={sm.n_events}  "
                    f"overshoot={sm.overshoot_pct_mean:.1f}%  "
                    f"rise={sm.rise_time_mean_s:.3f}s  "
                    f"settle={sm.settling_time_mean_s:.3f}s  "
                    f"rms={sm.rms_error_mean:.4f}  "
                    f"freq={sm.dominant_freq_hz:.2f}Hz  "
                    f"noise={sm.noise_rms:.5f}  "
                    f"coh={sm.coherence_verdict}  "
                    f"R²={sm.model_r2_mean:.2f}"
                )
        rec_lines = []
        for r in result.recommendations:
            if r.param == "NOTE":
                rec_lines.append(f"  NOTE [{r.axis}]: {r.reason}")
            else:
                delta = f"{r.delta_pct:+.0f}%"
                curr  = f"{r.current}" if r.current is not None else "unknown"
                sugg  = f"{r.suggested}" if r.suggested is not None else "direction-only"
                gate  = f" [gated: {r.data_quality_gate}]" if r.data_quality_gate else ""
                rec_lines.append(
                    f"  [{r.axis}] {r.param}: {curr} → {sugg} ({delta}) | "
                    f"{r.confidence} conf{gate} | {r.reason}"
                )
        autotune_text = self._format_autotune_section(autotune)

        prompt = f"""# PX4 Multicopter PID Tuning Analysis — {result.run_name}
Date: {result.timestamp}
Vehicle profile: {result.vehicle_profile}

## Data Quality
{quality_text}

## Current PID parameters
{params_text}

## Recorded metrics (per mode × axis × loop)
{chr(10).join(metrics_lines) if metrics_lines else "  (no step events detected)"}

## Heuristic recommendations (confidence-gated)
{chr(10).join(rec_lines)}
{autotune_text}

## Warnings
{chr(10).join("  - " + w for w in result.warnings) if result.warnings else "  none"}

## Question for the LLM
Given the metrics and data-quality information above, which PID gains should be
adjusted, by how much, and in which order?  Cross-reference model R², coherence,
and Nyquist limit.  Flag anything where sample rate prevents a reliable conclusion.
Remind the user: change one parameter at a time; test in SITL before real flight.

DISCLAIMER: This analysis is heuristic and must be validated by a qualified pilot/engineer.
"""
        return prompt
