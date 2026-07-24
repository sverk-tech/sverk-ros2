"""
VIO / EV (external-vision) analysis — EKF2 external-vision fuse diagnostics.

Consumes the raw ``EstimatorSeries`` extracted by ``ulog_adapter`` and produces
tuning insight for whatever EKF2 fuses as external vision.  It is SOURCE-AGNOSTIC:
ArUco/VPE, 3D-lidar odometry (LIO), 2D scan-match, camera VIO and mocap all land
in the same PX4 EV topics (``estimator_status_flags.cs_ev_*``,
``estimator_innovations.ev_*``, ``estimator_ev_pos_bias``,
``vehicle_visual_odometry``) and are analysed identically here.

No ROS dependency (mirrors ``hifi_analyzer.py``).  READ-ONLY — never writes FMU
parameters; produces recommendations only.

Domain reference (thresholds, EKF2_EV_* meaning, height strategy, EV_DELAY tuning
method):
``sverk_ws/src/sverk_drone/odometry/vpe/px4_local_pose_publisher/PX4_EKF2_TUNING_AND_DEBUG.md``

PX4 v1.15.4 facts used:
* ``test_ratio > 1.0`` ⇒ observation rejected by the innovation gate.
* EV-yaw innovation rides the shared ``heading`` field (there is no ``ev_yaw``).
* ``EKF2_EV_DELAY`` realistic range 0…50 ms (``EKF2_DELAY_MAX`` = 200).
* ``EKF2_EVA_GATE`` does NOT exist on 1.15.4 — never suggested.
"""
from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

log = logging.getLogger(__name__)

# ── thresholds ────────────────────────────────────────────────────────────────
TR_REJECT = 1.0                 # test_ratio > 1 ⇒ rejected observation (VERIFIED)
BIAS_CORR_THRESHOLD = 0.5       # |corr(|innov|, speed)| above this ⇒ grows with speed
BIAS_MEAN_FLOOR_M = 0.02        # 2 cm — a constant offset below this is not flagged
MIN_DELAY_RATE_HZ = 20.0        # EV source below this ⇒ EV_DELAY not estimable
DELAY_SIGNIFICANT_MS = 5.0      # |Δ| below this ⇒ no EKF2_EV_DELAY recommendation
DELAY_CLAMP_MS = (0.0, 50.0)    # realistic EKF2_EV_DELAY search window
NEES_OVER = 2.0                 # mean NIS > OVER·DOF ⇒ EV over-confident (noise low)
NEES_UNDER = 0.3                # mean NIS < UNDER·DOF ⇒ EV under-confident (noise high)
DROPOUT_MIN_S = 0.20            # ignore fusion gaps shorter than this
COASTING_WARN_PCT = 5.0         # dead-reckoning/const-pos fraction above this ⇒ note
HGT_TR_WARN = 0.5               # mean hgt_test_ratio above this ⇒ height-conflict note

EVP_NOISE_TIGHT = 0.05          # floor when tightening EV position noise
SOL_CONST_POS_BIT = 7           # solution_status_flags bit 7 = const-position mode


# ── data classes ──────────────────────────────────────────────────────────────

@dataclass
class AxisTestRatio:
    axis: str                    # "ev_hpos" | "ev_vpos" | "heading" | "ev_hvel" | "ev_vvel"
    mean: float
    p95: float
    pct_over_1: float            # % of samples with test_ratio > 1.0
    n: int
    source: str = "test_ratios"


@dataclass
class AxisBias:
    axis: str                    # "north" | "east" | "down"
    mean_innov: float
    speed_corr: float            # correlation of |innovation| with EKF speed
    classification: str          # "delay" | "extrinsic" | "none"
    confidence: str
    note: str = ""


@dataclass
class DropoutInterval:
    t_start: float
    t_end: float
    duration_s: float
    cause: str                   # "cs_ev_pos_false"|"reject"|"dead_reckoning"|"const_pos"


@dataclass
class DropoutReport:
    n_intervals: int = 0
    total_duration_s: float = 0.0
    max_duration_s: float = 0.0
    coasting_fraction_pct: float = 0.0
    intervals: List[DropoutInterval] = field(default_factory=list)


@dataclass
class NeesAxis:
    axis: str
    mean_nis: float
    dof: int
    verdict: str                 # "ok" | "overconfident" | "underconfident"
    source: str                  # "aid_src" | "ev_pos_bias"
    note: str = ""


@dataclass
class RateLatency:
    rate_hz: float
    expected_rate_hz: float
    latency_ms_median: float
    latency_ms_p95: float
    confidence: str
    note: str = ""


@dataclass
class ResetReport:
    vio_reset_count: int = 0
    ekf_reset_count_pos_ne: int = 0
    ekf_reset_count_pos_d: int = 0
    note: str = ""


@dataclass
class DelayEstimate:
    estimable: bool
    delay_ms: Optional[float]
    confidence: str
    r_squared: float             # cross-correlation peak (0..1)
    dynamics_score: float        # EKF speed std — how much motion the log carries
    method: str
    note: str = ""


@dataclass
class VioCapabilities:
    vio_basic_available: bool = False
    vio_aidsrc_available: bool = False
    vio_delay_estimable: bool = False
    topics_found: List[str] = field(default_factory=list)
    ev_velocity_fused: bool = False


@dataclass
class VioRecommendation:
    param: str                   # "EKF2_EV_DELAY" | "EKF2_EVP_NOISE" | "NOTE" | ...
    direction: str               # "increase" | "decrease" | "info"
    current: Optional[float]
    suggested: Optional[float]
    reason: str
    reason_key: str = ""
    reason_kwargs: list = field(default_factory=list)
    confidence: str = "medium"
    source_tag: str = ""         # "" | "ArUco/camera" | "lidar"

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d.pop("reason_key", None)
        d.pop("reason_kwargs", None)
        return d


@dataclass
class VioAnalysisResult:
    source_name: str
    duration_s: float
    capabilities: VioCapabilities
    test_ratios: List[AxisTestRatio] = field(default_factory=list)
    biases: List[AxisBias] = field(default_factory=list)
    dropouts: DropoutReport = field(default_factory=DropoutReport)
    nees: List[NeesAxis] = field(default_factory=list)
    rate_latency: Optional[RateLatency] = None
    resets: ResetReport = field(default_factory=ResetReport)
    delay: Optional[DelayEstimate] = None
    recommendations: List[VioRecommendation] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        # recommendations carry translation metadata that asdict leaves in —
        # keep reason_key/kwargs for the client-side re-translation path.
        return d


# ── translations (EN + RU), positional {0} like analyzer.REASON_TR ────────────
VIO_REASON_TR: Dict[str, Dict[str, str]] = {
    "en": {
        "rec_ev_delay":
            "EV position lags the EKF velocity by ≈{0:.0f} ms (our cross-correlation "
            "estimate, R²={1:.2f}). Move EKF2_EV_DELAY toward {2:.0f} ms "
            "(range 0…50). NOTE: this is our estimate, not PX4's official method "
            "(overlay EV rates on IMU rates and sweep to minimum innovation).",
        "rec_extrinsic":
            "Innovation offset is roughly constant and does not grow with speed "
            "(mean {0:.3f} m). That points to an extrinsic / frame-alignment error, "
            "not delay — check EKF2_EV_POS_X/Y/Z and the source→body alignment "
            "rather than EKF2_EV_DELAY.",
        "rec_evp_noise_up":
            "EV innovations are over-confident (mean NIS {0:.1f} ≫ DOF {1}). "
            "Raise the EV position noise floor EKF2_EVP_NOISE (the filter trusts "
            "the vision too much).",
        "rec_evp_noise_down":
            "EV innovations are conservative (mean NIS {0:.2f} ≪ DOF {1}) and hold "
            "is soft. EKF2_EVP_NOISE can be tightened toward {2:.2f} for crisper "
            "position hold (noisier).",
        "rec_eva_noise_up":
            "EV-yaw (heading) is over-confident (mean test-ratio {0:.2f}). "
            "Raise the EV yaw noise floor EKF2_EVA_NOISE.",
        "rec_dropout":
            "EV fusion drops out {0} time(s), {1:.1f} s total (max {2:.1f} s). "
            "Coasting/const-position {3:.0f}% of the log. The odometry is being "
            "lost — inspect the intervals below.",
        "rec_dropout_camera":
            "[camera] If the source is a camera/ArUco: reduce MPC_TILTMAX_AIR — a "
            "fisheye loses the map from FOV on tilt — and verify marker detection "
            "over the dropout intervals.",
        "rec_dropout_lidar":
            "[lidar] If the source is a lidar SLAM/scan-match: check for occlusion, "
            "reflective/absorbing surfaces and out-of-range geometry over the "
            "dropout intervals.",
        "rec_hgt_conflict":
            "Height test-ratio is elevated (mean {0:.2f}) with EV height fused — "
            "EV-Z is fighting the range finder. Set EKF2_HGT_REF=2 (Range) and "
            "EKF2_EV_CTRL=9 (h-pos+yaw, drop EV-Z). See domain doc §3.3.",
        "rec_qmin":
            "EV 'quality' varies and fusion dropouts are present. If EKF2_EV_QMIN>0, "
            "low-quality frames are being rejected — either publish a real quality "
            "from the source or keep EKF2_EV_QMIN=0.",
        "rec_disclaimer":
            "Heuristic starting points. Change one EKF2_EV_* parameter at a time and "
            "re-verify on the stand (listener estimator_aid_src_ev_*).",
        "note_no_ev": "No EV fusion detected in this log (cs_ev_* never true).",
    },
    "ru": {
        "rec_ev_delay":
            "EV-позиция отстаёт от EKF-скорости на ≈{0:.0f} мс (оценка кросс-"
            "корреляцией, R²={1:.2f}). Двигайте EKF2_EV_DELAY к {2:.0f} мс "
            "(диапазон 0…50). ВНИМАНИЕ: это наша оценка, не официальный метод PX4 "
            "(наложить EV-темпы на IMU-темпы и свипнуть до минимума инноваций).",
        "rec_extrinsic":
            "Смещение инноваций примерно постоянно и не растёт со скоростью "
            "(среднее {0:.3f} м). Это указывает на ошибку экстринсика/выравнивания "
            "фреймов, а не задержки — проверьте EKF2_EV_POS_X/Y/Z и выравнивание "
            "источник→тело, а не EKF2_EV_DELAY.",
        "rec_evp_noise_up":
            "EV-инновации переуверены (среднее NIS {0:.1f} ≫ DOF {1}). "
            "Поднимите флор шума EV-позиции EKF2_EVP_NOISE (фильтр слишком доверяет "
            "зрению).",
        "rec_evp_noise_down":
            "EV-инновации консервативны (среднее NIS {0:.2f} ≪ DOF {1}), удержание "
            "вялое. EKF2_EVP_NOISE можно зажать к {2:.2f} для более цепкого "
            "удержания (шумнее).",
        "rec_eva_noise_up":
            "EV-курс (heading) переуверен (среднее test-ratio {0:.2f}). "
            "Поднимите флор шума EV-угла EKF2_EVA_NOISE.",
        "rec_dropout":
            "Фьюз EV пропадает {0} раз, суммарно {1:.1f} с (макс {2:.1f} с). "
            "Коастинг/const-position {3:.0f}% лога. Одометрия теряется — "
            "разберите интервалы ниже.",
        "rec_dropout_camera":
            "[камера] Если источник — камера/ArUco: уменьшите MPC_TILTMAX_AIR "
            "(fisheye теряет карту из FOV при наклоне) и проверьте детекцию маркеров "
            "на интервалах дропаута.",
        "rec_dropout_lidar":
            "[лидар] Если источник — лидарный SLAM/scan-match: проверьте перекрытие, "
            "отражающие/поглощающие поверхности и выход геометрии за пределы "
            "дальности на интервалах дропаута.",
        "rec_hgt_conflict":
            "Test-ratio высоты повышен (среднее {0:.2f}) при фьюзе EV-высоты — "
            "EV-Z дерётся с дальномером. Поставьте EKF2_HGT_REF=2 (Range) и "
            "EKF2_EV_CTRL=9 (h-pos+yaw, без EV-Z). См. доменный док §3.3.",
        "rec_qmin":
            "EV 'quality' меняется и есть дропауты фьюза. Если EKF2_EV_QMIN>0, "
            "кадры низкого качества отбраковываются — шлите реальный quality из "
            "источника либо держите EKF2_EV_QMIN=0.",
        "rec_disclaimer":
            "Эвристические оценки. Меняйте по одному параметру EKF2_EV_* и "
            "перепроверяйте на стенде (listener estimator_aid_src_ev_*).",
        "note_no_ev": "Фьюз EV в этом логе не обнаружен (cs_ev_* всегда false).",
    },
}


def translate(key: str, lang: str, args: list) -> str:
    templates = VIO_REASON_TR.get(lang, VIO_REASON_TR["en"])
    fmt = templates.get(key) or VIO_REASON_TR["en"].get(key, key)
    try:
        return fmt.format(*args) if args and "{" in fmt else fmt
    except (IndexError, KeyError):
        return fmt


# ── numeric helpers ──────────────────────────────────────────────────────────

def _finite(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    return a[np.isfinite(a)]


def _pearson(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    m = np.isfinite(a) & np.isfinite(b)
    if int(np.sum(m)) < 4:
        return 0.0
    a, b = a[m], b[m]
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def _interp(t_src: np.ndarray, y_src: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    m = np.isfinite(t_src) & np.isfinite(y_src)
    if int(np.sum(m)) < 2:
        return np.full(len(t_grid), np.nan)
    ts, ys = t_src[m], y_src[m]
    order = np.argsort(ts)
    return np.interp(t_grid, ts[order], ys[order])


def _rate_from_us(ts_us: np.ndarray) -> float:
    ts = np.asarray(ts_us, dtype=float)
    if len(ts) < 3:
        return 0.0
    d = np.diff(np.sort(ts))
    d = d[d > 0]
    if len(d) == 0:
        return 0.0
    med = float(np.median(d))
    return 1e6 / med if med > 0 else 0.0


def _true_intervals(t: np.ndarray, mask: np.ndarray) -> List[Tuple[float, float]]:
    """Contiguous runs where ``mask`` is True, treating each sample as a state
    held until the next timestamp (correct for on-change estimator_status_flags).
    A run that reaches the final sample is closed at the last timestamp."""
    t = np.asarray(t, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    n = min(len(t), len(mask))
    t, mask = t[:n], mask[:n]
    out: List[Tuple[float, float]] = []
    i = 0
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            t_start = float(t[i])
            t_end = float(t[j + 1]) if j + 1 < n else float(t[j])
            out.append((t_start, t_end))
            i = j + 1
        else:
            i += 1
    return out


def _horizontal_speed(series) -> Tuple[np.ndarray, np.ndarray]:
    """(t_lpos, horizontal speed) or empty arrays."""
    if series.t_lpos is None or series.lpos_vel is None:
        return np.empty(0), np.empty(0)
    v = np.asarray(series.lpos_vel, dtype=float)
    sp = np.sqrt(v[:, 0] ** 2 + v[:, 1] ** 2)
    return np.asarray(series.t_lpos, dtype=float), sp


# ── (1) test-ratio statistics ─────────────────────────────────────────────────

def axis_test_ratio_stats(series) -> List[AxisTestRatio]:
    """Per-axis test-ratio mean / p95 / % over 1.0 (rejected).

    (Named ``axis_test_ratio_stats`` rather than ``test_ratio_stats`` so pytest
    does not mis-collect it as a test when it is imported into a test module.)
    """
    out: List[AxisTestRatio] = []

    def _add(axis: str, arr: Optional[np.ndarray]):
        if arr is None:
            return
        a = _finite(arr)
        if len(a) == 0:
            return
        out.append(AxisTestRatio(
            axis=axis,
            mean=round(float(np.mean(a)), 4),
            p95=round(float(np.percentile(a, 95)), 4),
            pct_over_1=round(float(np.mean(a > TR_REJECT) * 100.0), 1),
            n=int(len(a)),
        ))

    def _col0(arr):
        if arr is None:
            return None
        arr = np.asarray(arr)
        return arr[:, 0] if arr.ndim == 2 else arr

    _add("ev_hpos", _col0(series.ev_hpos_tr))
    _add("ev_vpos", series.ev_vpos_tr)
    _add("heading", series.heading_tr)
    # EV-velocity axes only when actually fused (non-trivial ratios present).
    for axis, arr in (("ev_hvel", _col0(series.ev_hvel_tr)),
                      ("ev_vvel", series.ev_vvel_tr)):
        if arr is not None and len(_finite(arr)) and np.nanmax(np.abs(arr)) > 1e-6:
            _add(axis, arr)
    return out


# ── (2) innovation bias: delay vs extrinsic ───────────────────────────────────

def innovation_bias(series) -> List[AxisBias]:
    """Classify each horizontal innovation axis as delay- or extrinsic-driven.

    * Non-zero mean growing with speed  → EKF2_EV_DELAY error.
    * Constant offset, low speed-corr   → extrinsic / frame-alignment error.
    """
    if series.t_innov is None or series.ev_hpos is None:
        return []
    t_sp, speed = _horizontal_speed(series)
    have_speed = len(t_sp) >= 4 and float(np.nanstd(speed)) > 1e-3
    speed_on_innov = (_interp(t_sp, speed, series.t_innov)
                      if have_speed else np.full(len(series.t_innov), np.nan))

    out: List[AxisBias] = []
    hpos = np.asarray(series.ev_hpos, dtype=float)
    comps = [("north", hpos[:, 0]), ("east", hpos[:, 1])]
    if series.ev_vpos is not None:
        comps.append(("down", np.asarray(series.ev_vpos, dtype=float)))

    for axis, innov in comps:
        f = _finite(innov)
        if len(f) < 8:
            continue
        mean = float(np.mean(f))
        std = float(np.std(f))
        corr = _pearson(np.abs(innov), speed_on_innov) if have_speed else 0.0
        significant = abs(mean) > max(BIAS_MEAN_FLOOR_M, 0.5 * std)
        if have_speed and abs(corr) >= BIAS_CORR_THRESHOLD:
            cls = "delay"
            conf = "medium" if abs(corr) >= 0.7 and len(f) >= 20 else "low"
            note = "Innovation grows with speed → delay-like."
        elif significant:
            cls = "extrinsic"
            conf = "medium" if abs(mean) > 3 * BIAS_MEAN_FLOOR_M else "low"
            note = "Constant offset, weak speed correlation → extrinsic-like."
        else:
            cls = "none"
            conf = "low"
            note = "No significant bias."
        out.append(AxisBias(
            axis=axis, mean_innov=round(mean, 4),
            speed_corr=round(corr, 3),
            classification=cls, confidence=conf, note=note))
    return out


# ── (3) fusion dropouts / coasting ────────────────────────────────────────────

def fusion_dropouts(series) -> DropoutReport:
    rep = DropoutReport()
    intervals: List[DropoutInterval] = []

    def _collect(t, mask, cause):
        if t is None or mask is None:
            return
        for a, b in _true_intervals(t, mask):
            dur = b - a
            if dur >= DROPOUT_MIN_S:
                intervals.append(DropoutInterval(
                    t_start=round(a, 3), t_end=round(b, 3),
                    duration_s=round(dur, 3), cause=cause))

    # EV position/vel/yaw not fused while (implicitly) airborne — use the flag
    # directly; on-change logging means each False sample holds until the next.
    if series.cs_ev_pos is not None:
        _collect(series.t_flags, ~np.asarray(series.cs_ev_pos, dtype=bool),
                 "cs_ev_pos_false")
    for rej in (series.reject_hor_pos, series.reject_ver_pos, series.reject_yaw):
        if rej is not None:
            _collect(series.t_flags, np.asarray(rej, dtype=bool), "reject")

    # Coasting: cs_inertial_dead_reckoning and const-position mode (bit 7).
    coast_intervals: List[Tuple[float, float]] = []
    if series.cs_inertial_dead_reckoning is not None:
        dr = _true_intervals(series.t_flags,
                             np.asarray(series.cs_inertial_dead_reckoning, dtype=bool))
        coast_intervals += dr
        for a, b in dr:
            if b - a >= DROPOUT_MIN_S:
                intervals.append(DropoutInterval(
                    round(a, 3), round(b, 3), round(b - a, 3), "dead_reckoning"))
    if series.solution_status_flags is not None and series.t_status is not None:
        sol = np.asarray(series.solution_status_flags, dtype=np.int64)
        const_pos = ((sol >> SOL_CONST_POS_BIT) & 1) != 0
        cp = _true_intervals(series.t_status, const_pos)
        coast_intervals += cp
        for a, b in cp:
            if b - a >= DROPOUT_MIN_S:
                intervals.append(DropoutInterval(
                    round(a, 3), round(b, 3), round(b - a, 3), "const_pos"))

    intervals.sort(key=lambda iv: iv.t_start)
    rep.intervals = intervals
    rep.n_intervals = len(intervals)
    if intervals:
        durs = [iv.duration_s for iv in intervals]
        rep.total_duration_s = round(float(sum(durs)), 3)
        rep.max_duration_s = round(float(max(durs)), 3)

    # Coasting fraction over the log span (union not computed — approximate).
    span = _series_span(series)
    if span > 0 and coast_intervals:
        coast_total = sum(max(0.0, b - a) for a, b in coast_intervals)
        rep.coasting_fraction_pct = round(min(100.0, coast_total / span * 100.0), 1)
    return rep


def _series_span(series) -> float:
    ends = []
    for t in (series.t_flags, series.t_innov, series.t_status,
              series.t_lpos, series.t_bias, series.t_vio):
        if t is not None and len(t) > 1:
            ends.append(float(t[-1]) - float(t[0]))
    return max(ends) if ends else 0.0


# ── (4) NEES / consistency ────────────────────────────────────────────────────

def nees_consistency(series) -> List[NeesAxis]:
    out: List[NeesAxis] = []

    def _verdict(mean_nis, dof):
        if mean_nis > NEES_OVER * dof:
            return "overconfident"
        if mean_nis < NEES_UNDER * dof:
            return "underconfident"
        return "ok"

    # Preferred: real NIS from aid_src (innovation² / innovation_variance).
    aid = series.aid_ev_pos
    used_aid = False
    if aid is not None and "innovation" in aid and "innovation_variance" in aid:
        innov = np.asarray(aid["innovation"], dtype=float)
        var = np.asarray(aid["innovation_variance"], dtype=float)
        if innov.ndim == 1:
            innov = innov[:, None]
            var = var[:, None] if var.ndim == 1 else var
        dof = innov.shape[1]
        nis = np.nansum(innov ** 2 / np.where(var > 1e-12, var, np.nan), axis=1)
        nis = _finite(nis)
        if len(nis):
            m = float(np.mean(nis))
            out.append(NeesAxis("ev_pos", round(m, 3), dof,
                                _verdict(m, dof), "aid_src"))
            used_aid = True

    # Fallback proxy present in the default profile: ev_pos_bias.innov_test_ratio.
    if not used_aid and series.ev_pos_bias_tr is not None:
        tr = np.asarray(series.ev_pos_bias_tr, dtype=float)
        # Each axis' test-ratio ~ normalized consistency; DOF 1 per axis.
        mean_tr = _finite(tr).mean() if len(_finite(tr)) else float("nan")
        if np.isfinite(mean_tr):
            verdict = "overconfident" if mean_tr > 1.0 else "ok"
            out.append(NeesAxis("ev_pos_bias", round(float(mean_tr), 3), 1,
                                verdict, "ev_pos_bias",
                                note="Proxy: mean innov_test_ratio (default-log)."))
    return out


# ── (5) rate + latency ────────────────────────────────────────────────────────

def rate_latency(series, expected_rate_hz: float) -> Optional[RateLatency]:
    if not series.has_vio_input or series.vio_timestamp_sample is None:
        return None
    rate = _rate_from_us(series.vio_timestamp_sample)
    lat = (_finite(series.vio_latency_ms)
           if series.vio_latency_ms is not None else np.empty(0))
    lat_med = float(np.median(lat)) if len(lat) else 0.0
    lat_p95 = float(np.percentile(lat, 95)) if len(lat) else 0.0
    conf = "high" if len(lat) > 200 else "medium" if len(lat) > 20 else "low"
    note = ""
    if expected_rate_hz > 0 and rate > 0 and rate < 0.7 * expected_rate_hz:
        note = (f"Measured EV rate {rate:.1f} Hz is well below the expected "
                f"{expected_rate_hz:.0f} Hz — source may be dropping frames.")
    return RateLatency(
        rate_hz=round(rate, 1), expected_rate_hz=round(expected_rate_hz, 1),
        latency_ms_median=round(lat_med, 1), latency_ms_p95=round(lat_p95, 1),
        confidence=conf, note=note)


# ── (6) resets ────────────────────────────────────────────────────────────────

def reset_report(series) -> ResetReport:
    rep = ResetReport()
    if series.vio_reset_counter is not None:
        rc = np.asarray(series.vio_reset_counter, dtype=float)
        rc = rc[np.isfinite(rc)]
        if len(rc) > 1:
            rep.vio_reset_count = int(np.sum(np.diff(rc) > 0))
    for attr, field_name in (("reset_count_pos_ne", "ekf_reset_count_pos_ne"),
                             ("reset_count_pos_d", "ekf_reset_count_pos_d")):
        arr = getattr(series, attr, None)
        if arr is not None:
            a = np.asarray(arr, dtype=float)
            a = a[np.isfinite(a)]
            if len(a) > 1:
                # Count monotonic bumps (same method as the VIO path above), not
                # the net delta — robust to a non-monotonic / wrapping counter.
                setattr(rep, field_name, int(np.sum(np.diff(a) > 0)))
    return rep


# ── (7) EV_DELAY estimate ─────────────────────────────────────────────────────

def estimate_ev_delay(series, expected_rate_hz: float,
                      max_lag_ms: float = 120.0) -> DelayEstimate:
    """Differentiate EV position → EV velocity, cross-correlate against EKF
    velocity, argmax lag → delay.  Gated on a high-rate EV source."""
    hint = ("Add estimator_aid_src_ev_* / vehicle_visual_odometry via "
            "logger_topics.txt (or SDLOG_PROFILE replay bit) — the 2 Hz "
            "estimator_innovations stream is too slow to estimate EV_DELAY.")

    # EV position source: prefer vehicle_visual_odometry, else aid_src observation.
    t_ev = ev_pos = None
    if series.has_vio_input and series.vio_pos is not None:
        t_ev, ev_pos = np.asarray(series.t_vio, dtype=float), np.asarray(series.vio_pos, dtype=float)
    elif series.aid_ev_pos is not None and "observation" in series.aid_ev_pos:
        obs = np.asarray(series.aid_ev_pos["observation"], dtype=float)
        if obs.ndim == 2 and obs.shape[1] >= 2:
            t_ev, ev_pos = np.asarray(series.aid_ev_pos["t"], dtype=float), obs

    if t_ev is None or ev_pos is None or series.lpos_vel is None:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated", hint)

    rate = _rate_from_us((t_ev * 1e6))
    if rate < MIN_DELAY_RATE_HZ:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated",
                             f"EV source rate {rate:.1f} Hz < {MIN_DELAY_RATE_HZ:.0f} Hz. " + hint)

    t_v = np.asarray(series.t_lpos, dtype=float)
    ekf_vel = np.asarray(series.lpos_vel, dtype=float)

    # Uniform grid over the overlap at the EV rate.
    t_lo = max(float(t_ev[0]), float(t_v[0]))
    t_hi = min(float(t_ev[-1]), float(t_v[-1]))
    if t_hi - t_lo < 1.0:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated",
                             "EV and EKF-velocity records do not overlap enough.")
    fs = rate
    tg = np.arange(t_lo, t_hi, 1.0 / fs)
    if len(tg) < 32:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated", hint)

    max_lag = int(round(max_lag_ms / 1000.0 * fs))
    dyn = float(np.nanstd(np.sqrt(ekf_vel[:, 0] ** 2 + ekf_vel[:, 1] ** 2)))

    best_corr = 0.0
    lag_num = 0.0
    lag_w = 0.0
    for ax in (0, 1):     # north, east
        ev_p = _interp(t_ev, ev_pos[:, ax], tg)
        ekf_v = _interp(t_v, ekf_vel[:, ax], tg)
        ev_v = np.gradient(ev_p, 1.0 / fs)
        a = ev_v - np.nanmean(ev_v)
        b = ekf_v - np.nanmean(ekf_v)
        a = np.nan_to_num(a)
        b = np.nan_to_num(b)
        if np.std(a) < 1e-9 or np.std(b) < 1e-9:
            continue
        lag, corr = _xcorr_lag(a, b, max_lag)
        # Weight each axis by its correlation strength.
        w = max(0.0, corr)
        lag_num += lag * w
        lag_w += w
        best_corr = max(best_corr, corr)

    if lag_w <= 0:
        return DelayEstimate(True, None, "low", round(best_corr, 3), round(dyn, 3),
                             "xcorr", "No coherent EV↔EKF velocity correlation.")
    lag_samples = lag_num / lag_w
    delay_ms = lag_samples / fs * 1000.0

    if dyn < 0.05:
        conf = "low"
        note = "Very little motion in the log — delay estimate is unreliable."
    elif best_corr >= 0.6:
        conf = "medium"
        note = ""
    else:
        conf = "low"
        note = "Weak correlation — treat the delay estimate as indicative only."
    return DelayEstimate(True, round(delay_ms, 1), conf, round(best_corr, 3),
                         round(dyn, 3), "xcorr", note)


def _xcorr_lag(a: np.ndarray, b: np.ndarray, max_lag: int) -> Tuple[int, float]:
    """Return (lag, normalized peak) where positive lag ⇒ a lags b.
    a[n] ≈ b[n − lag] at the peak."""
    n = len(a)
    norm = (np.std(a) * np.std(b) * n)
    if norm <= 0:
        return 0, 0.0
    from scipy import signal as _sig
    full = _sig.correlate(a, b, mode="full")
    lags = _sig.correlation_lags(n, len(b), mode="full")
    sel = np.abs(lags) <= max_lag
    full = full[sel] / norm
    lags = lags[sel]
    k = int(np.argmax(full))
    return int(lags[k]), float(full[k])


# ── (8) recommendations ───────────────────────────────────────────────────────

def build_recommendations(
    result: VioAnalysisResult, params: Dict[str, Any], lang: str = "en",
) -> List[VioRecommendation]:
    recs: List[VioRecommendation] = []

    def _cur(name):
        v = params.get(name)
        try:
            return float(v) if v is not None else None
        except (TypeError, ValueError):
            return None

    def _mk(param, direction, current, suggested, key, args, conf, tag=""):
        recs.append(VioRecommendation(
            param=param, direction=direction, current=current, suggested=suggested,
            reason=translate(key, lang, args), reason_key=key,
            reason_kwargs=list(args), confidence=conf, source_tag=tag))

    d = result.delay
    biases = result.biases
    delay_axes = [b for b in biases if b.classification == "delay"]
    extr_axes = [b for b in biases if b.classification == "extrinsic"]

    # ── EKF2_EV_DELAY ─────────────────────────────────────────────────────────
    if (d and d.estimable and d.delay_ms is not None
            and d.confidence in ("medium", "high")):
        cur = _cur("EKF2_EV_DELAY")
        base = cur if cur is not None else 0.0
        if abs(d.delay_ms - base) > DELAY_SIGNIFICANT_MS:
            suggested = float(np.clip(d.delay_ms, *DELAY_CLAMP_MS))
            direction = "increase" if suggested > base else "decrease"
            _mk("EKF2_EV_DELAY", direction, cur, round(suggested, 1),
                "rec_ev_delay", [d.delay_ms, d.r_squared, suggested],
                d.confidence)
    elif extr_axes and not delay_axes:
        # Constant offset that does not grow with speed ⇒ extrinsic, not delay.
        worst = max(extr_axes, key=lambda b: abs(b.mean_innov))
        _mk("EKF2_EV_POS", "info", None, None,
            "rec_extrinsic", [worst.mean_innov], worst.confidence)

    # ── EKF2_EVP_NOISE / EVA_NOISE from NEES ─────────────────────────────────
    for nx in result.nees:
        if nx.verdict == "overconfident":
            _mk("EKF2_EVP_NOISE", "increase", _cur("EKF2_EVP_NOISE"), None,
                "rec_evp_noise_up", [nx.mean_nis, nx.dof],
                "medium" if nx.source == "aid_src" else "low")
        elif nx.verdict == "underconfident":
            _mk("EKF2_EVP_NOISE", "decrease", _cur("EKF2_EVP_NOISE"),
                EVP_NOISE_TIGHT, "rec_evp_noise_down",
                [nx.mean_nis, nx.dof, EVP_NOISE_TIGHT], "low")
    # EV-yaw over-confidence from heading test-ratio.
    heading_tr = next((t for t in result.test_ratios if t.axis == "heading"), None)
    if heading_tr and heading_tr.pct_over_1 > 10.0:
        _mk("EKF2_EVA_NOISE", "increase", _cur("EKF2_EVA_NOISE"), None,
            "rec_eva_noise_up", [heading_tr.mean], "low")

    # ── dropout / coasting ────────────────────────────────────────────────────
    dr = result.dropouts
    if dr.n_intervals > 0 or dr.coasting_fraction_pct > COASTING_WARN_PCT:
        _mk("NOTE", "info", None, None, "rec_dropout",
            [dr.n_intervals, dr.total_duration_s, dr.max_duration_s,
             dr.coasting_fraction_pct], "medium")
        # Source-specific hints, explicitly tagged so they don't mislead.
        _mk("NOTE", "info", None, None, "rec_dropout_camera", [], "low",
            tag="ArUco/camera")
        _mk("NOTE", "info", None, None, "rec_dropout_lidar", [], "low",
            tag="lidar")

    # ── height conflict (EV-Z vs range) ──────────────────────────────────────
    hgt_active = _series_flag_ever_true(result, "cs_ev_hgt")
    if result._hgt_tr_mean is not None and result._hgt_tr_mean > HGT_TR_WARN and hgt_active:
        _mk("EKF2_HGT_REF", "info", _cur("EKF2_HGT_REF"), 2.0,
            "rec_hgt_conflict", [result._hgt_tr_mean], "low")

    # ── EV_QMIN vs quality ────────────────────────────────────────────────────
    if result._quality_varies and dr.n_intervals > 0:
        qmin = _cur("EKF2_EV_QMIN")
        if qmin is None or qmin > 0:
            _mk("EKF2_EV_QMIN", "info", qmin, None, "rec_qmin", [], "low")

    # ── disclaimer NOTE always last ──────────────────────────────────────────
    _mk("NOTE", "info", None, None, "rec_disclaimer", [], "high")
    return recs


def _series_flag_ever_true(result: VioAnalysisResult, name: str) -> bool:
    return bool(getattr(result, "_" + name + "_ever", False))


# ── orchestrator ──────────────────────────────────────────────────────────────

class VioAnalyzer:
    """Runs all VIO/EV analyses on a UlogParseResult (mirrors HiFiAnalyzer)."""

    def __init__(self, expected_rate_hz: float = 30.0, lang: str = "en"):
        self.expected_rate_hz = float(expected_rate_hz)
        self.lang = lang if lang in VIO_REASON_TR else "en"

    def analyse(self, parse_result: "Any") -> VioAnalysisResult:
        meta = parse_result.metadata
        series = getattr(parse_result, "estimator_series", None)
        caps_src = meta.capabilities

        caps = VioCapabilities(
            vio_basic_available=getattr(caps_src, "vio_basic_available", False),
            vio_aidsrc_available=getattr(caps_src, "vio_aidsrc_available", False),
            vio_delay_estimable=getattr(caps_src, "vio_delay_estimable", False),
            topics_found=self._topics_found(series),
        )

        result = VioAnalysisResult(
            source_name=meta.source_name,
            duration_s=meta.duration_s,
            capabilities=caps,
        )
        # scratch attributes for the recommendation pass
        result._hgt_tr_mean = None
        result._quality_varies = False
        result._cs_ev_hgt_ever = False

        if series is None or not (series.has_flags or series.has_innov):
            result.warnings.append("No estimator/EV topics in this log — VIO tab unavailable.")
            return result

        caps.ev_velocity_fused = bool(
            series.cs_ev_vel is not None and np.any(np.asarray(series.cs_ev_vel, dtype=bool)))
        result._cs_ev_hgt_ever = bool(
            series.cs_ev_hgt is not None and np.any(np.asarray(series.cs_ev_hgt, dtype=bool)))
        if series.hgt_test_ratio is not None:
            h = _finite(series.hgt_test_ratio)
            result._hgt_tr_mean = round(float(np.mean(h)), 3) if len(h) else None
        if series.vio_quality is not None:
            q = _finite(series.vio_quality)
            result._quality_varies = bool(len(q) and float(np.std(q)) > 1e-6)

        result.test_ratios = axis_test_ratio_stats(series)
        result.biases = innovation_bias(series)
        result.dropouts = fusion_dropouts(series)
        result.nees = nees_consistency(series)
        result.rate_latency = rate_latency(series, self.expected_rate_hz)
        result.resets = reset_report(series)
        result.delay = estimate_ev_delay(series, self.expected_rate_hz)

        if not caps.vio_delay_estimable and result.delay is not None:
            result.delay.estimable = False

        # No EV fusion at all → clear note.
        ev_ever = bool(series.cs_ev_pos is not None
                       and np.any(np.asarray(series.cs_ev_pos, dtype=bool)))
        if not ev_ever and series.cs_ev_pos is not None:
            result.warnings.append(translate("note_no_ev", self.lang, []))

        result.recommendations = build_recommendations(
            result, meta.params_all if hasattr(meta, "params_all") else {}, self.lang)
        return result

    @staticmethod
    def _topics_found(series) -> List[str]:
        if series is None:
            return []
        found = []
        if series.has_flags:      found.append("estimator_status_flags")
        if series.has_innov:      found.append("estimator_innovations")
        if series.ev_hpos_tr is not None: found.append("estimator_innovation_test_ratios")
        if series.has_status:     found.append("estimator_status")
        if series.has_bias:       found.append("estimator_ev_pos_bias")
        if series.has_lpos:       found.append("vehicle_local_position")
        if series.has_aidsrc_pos: found.append("estimator_aid_src_ev_pos")
        if series.has_aidsrc_yaw: found.append("estimator_aid_src_ev_yaw")
        if series.has_aidsrc_hgt: found.append("estimator_aid_src_ev_hgt")
        if series.has_vio_input:  found.append("vehicle_visual_odometry")
        return found
