"""
Live VIO / EV (external-vision) monitor — DDS watchdog for EKF2 EV fusion.

READ-ONLY: consumes ``/fmu/out/estimator_status_flags``,
``/fmu/out/vehicle_odometry`` and the EV inputs
``/fmu/in/vehicle_visual_odometry`` / ``/fmu/in/vehicle_mocap_odometry``; it
never publishes to the FMU.

Scope (deliberately narrow):
    What DDS exposes live is only enough for a *watchdog*: are we fusing EV right
    now, at what rate / latency, and is the filter coasting?  The numeric
    innovation consistency and the EKF2_EV_DELAY estimate need
    ``estimator_innovations`` / ``estimator_aid_src_ev_*`` which are ULog-only
    (not bridged over DDS) — those live in the offline VIO/EV tab (,
    :mod:`vio_analyzer`).  The UI must surface this split with a banner.

Source-agnostic: works identically for ArUco/VPE, 3D-lidar odometry, 2D
scan-match, camera VIO and mocap — everything EKF2 fuses as external vision
lands in the same EV interface.  The expected publish rate is a *parameter*
(``vio_expected_rate_hz``), never hard-coded to the 30 Hz ArUco cadence.

The verdict logic is a pure function (:func:`compute_verdict`) with no ROS /
socket dependency so it can be unit-tested directly (``test/test_vio_live.py``).

Thresholds come from the domain reference
``odometry/vpe/px4_local_pose_publisher/PX4_EKF2_TUNING_AND_DEBUG.md``
(``EKF2_DELAY_MAX`` = 200 ms latency ceiling; EV-pos/yaw fusion flags).

PX4 v1.15.4 facts used:
* ``EstimatorStatusFlags`` exposes named booleans — ``cs_ev_pos``, ``cs_ev_yaw``,
  ``cs_ev_hgt``, ``cs_ev_vel``, ``cs_inertial_dead_reckoning``,
  ``reject_hor_pos`` / ``reject_ver_pos`` / ``reject_yaw`` — read them directly,
  no bitmask decode.
* ``VehicleOdometry``: ``pose_frame`` (1 = NED), ``position[3]``, ``q[4]``,
  ``timestamp``, ``timestamp_sample``, ``reset_counter``, ``quality``.  On the EV
  *input* topic the ROS-side ``timestamp`` is 0 and ``timestamp_sample`` carries
  the capture time (ROS system clock, µs), so latency ≈ now − timestamp_sample.
"""
from __future__ import annotations

import dataclasses
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Sequence, Tuple

from .data_buffer import compute_stats_from_arrivals

# ── thresholds (domain doc PX4_EKF2_TUNING_AND_DEBUG.md) ──────────────────────
LATENCY_OK_MS = 100.0            # median EV latency below this ⇒ healthy
LATENCY_MAX_MS = 200.0           # EKF2_DELAY_MAX — hard latency ceiling
RATE_LOW_FACTOR = 0.5            # rate < 0.5×expected ⇒ WARN (too slow)
RATE_HIGH_FACTOR = 1.5           # rate > 1.5×expected ⇒ WARN (out of band)
DEFAULT_DATA_TIMEOUT_S = 0.5     # EV input silent longer than this ⇒ CRIT

# timestamp_sample above this (µs) is a wall-clock stamp (≈ system time); below
# it is a PX4 boot-relative time we cannot subtract from the host clock.  1e12 µs
# ≈ 11.5 days of uptime — safely above any boot time, far below a wall-clock now.
WALL_CLOCK_US_MIN = 1e12
LATENCY_SANE_MAX_MS = 10_000.0   # computed latency beyond this ⇒ treat as unknown

_VERDICT_RANK = {"UNAVAILABLE": -1, "OK": 0, "WARN": 1, "CRIT": 2}


# ── verdict (pure) ────────────────────────────────────────────────────────────

def compute_verdict(
    *,
    available: bool,
    ev_input_available: bool,
    cs_ev_pos: Optional[bool],
    cs_ev_yaw: Optional[bool],
    cs_inertial_dead_reckoning: Optional[bool],
    reject_hor_pos: Optional[bool],
    reject_ver_pos: Optional[bool],
    reject_yaw: Optional[bool],
    ev_rate_hz: float,
    ev_latency_ms: Optional[float],
    ev_input_age_s: Optional[float],
    expected_rate_hz: float,
    data_timeout_s: float = DEFAULT_DATA_TIMEOUT_S,
    in_air: bool = False,
) -> Tuple[str, List[str]]:
    """Summarize live EV health into OK / WARN / CRIT plus machine-readable
    reason keys (the UI maps keys → localized text).

    Precedence CRIT > WARN > OK.  A ``None`` flag means the topic is absent /
    not yet seen and never escalates on its own — only an explicit ``False``
    (fusion actually off) or ``True`` (reject actually active) does.

    CRIT: coasting (``cs_inertial_dead_reckoning``), EV input silent longer than
        ``data_timeout_s``, or EV position not fused while airborne.
    WARN: rate out of ``[0.5, 1.5]×expected``, latency above 100 ms, an active
        innovation reject, or EV pos/yaw not fused on the ground.
    """
    if not available:
        return "UNAVAILABLE", ["no_vio_topics"]

    verdict = "OK"
    reasons: List[str] = []

    def escalate(level: str, key: str) -> None:
        nonlocal verdict
        if key not in reasons:
            reasons.append(key)
        if _VERDICT_RANK[level] > _VERDICT_RANK[verdict]:
            verdict = level

    # ── CRIT ──────────────────────────────────────────────────────────────────
    if cs_inertial_dead_reckoning is True:
        escalate("CRIT", "coasting")
    if (ev_input_available and ev_input_age_s is not None
            and ev_input_age_s > data_timeout_s):
        escalate("CRIT", "ev_input_silent")
    if in_air and cs_ev_pos is False:
        escalate("CRIT", "ev_pos_not_fused_airborne")

    # ── WARN ──────────────────────────────────────────────────────────────────
    if expected_rate_hz > 0 and ev_rate_hz > 0:
        if ev_rate_hz < RATE_LOW_FACTOR * expected_rate_hz:
            escalate("WARN", "low_rate")
        elif ev_rate_hz > RATE_HIGH_FACTOR * expected_rate_hz:
            escalate("WARN", "high_rate")
    if ev_latency_ms is not None and ev_latency_ms > LATENCY_OK_MS:
        escalate("WARN", "high_latency")
    if any(r is True for r in (reject_hor_pos, reject_ver_pos, reject_yaw)):
        escalate("WARN", "reject_active")
    if not in_air and cs_ev_pos is False:
        escalate("WARN", "ev_pos_not_fused")
    if in_air and cs_ev_yaw is False:
        escalate("WARN", "ev_yaw_not_fused")

    return verdict, reasons


# ── live-state snapshot ───────────────────────────────────────────────────────

@dataclass
class VioLiveState:
    """Immutable snapshot of the live EV monitor at one instant (WS/REST payload)."""
    available: bool = False
    source: str = "none"                 # "visual" | "mocap" | "none"

    # fusion flags (estimator_status_flags)
    flags_available: bool = False
    cs_ev_pos: Optional[bool] = None
    cs_ev_yaw: Optional[bool] = None
    cs_ev_hgt: Optional[bool] = None
    cs_ev_vel: Optional[bool] = None
    cs_inertial_dead_reckoning: Optional[bool] = None
    reject_hor_pos: Optional[bool] = None
    reject_ver_pos: Optional[bool] = None
    reject_yaw: Optional[bool] = None
    reject_hor_pos_count: int = 0
    reject_ver_pos_count: int = 0
    reject_yaw_count: int = 0

    # EV input (vehicle_visual_odometry / vehicle_mocap_odometry)
    ev_input_available: bool = False
    ev_rate_hz: float = 0.0
    ev_latency_ms: Optional[float] = None
    ev_input_age_s: Optional[float] = None
    ev_reset_counter: Optional[int] = None
    ev_quality: Optional[int] = None
    ev_position_finite: Optional[bool] = None
    ev_q_finite: Optional[bool] = None

    # EKF output (vehicle_odometry) and EV-input vs EKF-output divergence
    ekf_output_available: bool = False
    ev_ekf_divergence_m: Optional[float] = None

    # context
    expected_rate_hz: float = 0.0
    in_air: bool = False

    # verdict
    verdict: str = "UNAVAILABLE"
    verdict_reasons: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def _all_finite(vals: Sequence[float]) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in vals)
    except (TypeError, ValueError):
        return False


def _latency_ms(now: float, timestamp_sample_us: Optional[int]) -> Optional[float]:
    """Age of an EV sample in ms, or ``None`` when it can't be trusted.

    Returns ``None`` for a PX4 boot-relative ``timestamp_sample`` (below
    ``WALL_CLOCK_US_MIN``) — subtracting it from the host wall clock would be
    meaningless — and for physically impossible (negative / huge) results.
    """
    if not timestamp_sample_us or timestamp_sample_us <= 0:
        return None
    if timestamp_sample_us < WALL_CLOCK_US_MIN:
        return None
    lat = (now - timestamp_sample_us / 1e6) * 1000.0
    if lat < 0.0 or lat > LATENCY_SANE_MAX_MS:
        return None
    return lat


class VioLiveMonitor:
    """Accumulates EV/EKF DDS messages and produces :class:`VioLiveState`
    snapshots on demand.

    Thread-safe (own lock): ROS callbacks feed it (``update_*``) while the web
    thread reads it (``snapshot``).  Pure Python — no ROS import — so tests
    drive it directly.  ``snapshot`` is non-destructive: it never clears the
    arrival deque or the last-value cache, so repeated polling (WS at ~10 Hz)
    keeps returning a consistent live picture.
    """

    def __init__(self, data_timeout_s: float = DEFAULT_DATA_TIMEOUT_S,
                 arrivals_maxlen: int = 600):
        self._lock = threading.Lock()
        self.data_timeout_s = float(data_timeout_s)
        self._arrivals: Deque[float] = deque(maxlen=arrivals_maxlen)

        self._seen_flags = False
        self._seen_ev = False

        # flags
        self._cs_ev_pos: Optional[bool] = None
        self._cs_ev_yaw: Optional[bool] = None
        self._cs_ev_hgt: Optional[bool] = None
        self._cs_ev_vel: Optional[bool] = None
        self._cs_dr: Optional[bool] = None
        self._reject: dict = {"hor_pos": None, "ver_pos": None, "yaw": None}
        self._reject_prev: dict = {"hor_pos": False, "ver_pos": False, "yaw": False}
        self._reject_count: dict = {"hor_pos": 0, "ver_pos": 0, "yaw": 0}

        # EV input
        self._source = "none"
        self._ev_latency_ms: Optional[float] = None
        self._ev_reset: Optional[int] = None
        self._ev_quality: Optional[int] = None
        self._ev_pos: Optional[List[float]] = None
        self._ev_pos_finite: Optional[bool] = None
        self._ev_q_finite: Optional[bool] = None
        self._last_ev_wall: Optional[float] = None

        # EKF output
        self._ekf_pos: Optional[List[float]] = None

    # ── ingest ────────────────────────────────────────────────────────────────

    def update_flags(self, *, cs_ev_pos: Optional[bool], cs_ev_yaw: Optional[bool],
                     cs_ev_hgt: Optional[bool], cs_ev_vel: Optional[bool],
                     cs_inertial_dead_reckoning: Optional[bool],
                     reject_hor_pos: bool, reject_ver_pos: bool,
                     reject_yaw: bool) -> None:
        with self._lock:
            self._seen_flags = True
            self._cs_ev_pos = cs_ev_pos
            self._cs_ev_yaw = cs_ev_yaw
            self._cs_ev_hgt = cs_ev_hgt
            self._cs_ev_vel = cs_ev_vel
            self._cs_dr = cs_inertial_dead_reckoning
            # Edge-triggered reject counters: count each False→True transition.
            for key, val in (("hor_pos", bool(reject_hor_pos)),
                             ("ver_pos", bool(reject_ver_pos)),
                             ("yaw", bool(reject_yaw))):
                if val and not self._reject_prev[key]:
                    self._reject_count[key] += 1
                self._reject_prev[key] = val
                self._reject[key] = val

    def update_ev_input(self, *, now: float, source: str,
                        timestamp_sample_us: Optional[int],
                        reset_counter: Optional[int], quality: Optional[int],
                        position: Sequence[float], q: Sequence[float]) -> None:
        with self._lock:
            self._seen_ev = True
            self._source = source
            self._arrivals.append(now)
            self._last_ev_wall = now
            self._ev_latency_ms = _latency_ms(now, timestamp_sample_us)
            self._ev_reset = int(reset_counter) if reset_counter is not None else None
            self._ev_quality = int(quality) if quality is not None else None
            pos = [float(x) for x in position[:3]] if position is not None else None
            self._ev_pos = pos
            self._ev_pos_finite = _all_finite(pos) if pos is not None else None
            self._ev_q_finite = (_all_finite(q) if q is not None and len(q) else None)

    def update_ekf_output(self, *, position: Sequence[float]) -> None:
        with self._lock:
            if position is not None and len(position) >= 3:
                self._ekf_pos = [float(x) for x in position[:3]]

    # ── read ──────────────────────────────────────────────────────────────────

    def snapshot(self, *, now: float, expected_rate_hz: float,
                 in_air: bool = False) -> VioLiveState:
        with self._lock:
            available = self._seen_flags or self._seen_ev

            rate = 0.0
            if self._arrivals:
                rate = compute_stats_from_arrivals(
                    list(self._arrivals), "ev_input").achieved_rate_hz

            age = (now - self._last_ev_wall) if self._last_ev_wall is not None else None

            divergence = None
            if self._ev_pos is not None and self._ekf_pos is not None:
                d = [self._ev_pos[i] - self._ekf_pos[i] for i in range(3)]
                if _all_finite(d):
                    divergence = math.sqrt(sum(c * c for c in d))

            verdict, reasons = compute_verdict(
                available=available,
                ev_input_available=self._seen_ev,
                cs_ev_pos=self._cs_ev_pos,
                cs_ev_yaw=self._cs_ev_yaw,
                cs_inertial_dead_reckoning=self._cs_dr,
                reject_hor_pos=self._reject["hor_pos"],
                reject_ver_pos=self._reject["ver_pos"],
                reject_yaw=self._reject["yaw"],
                ev_rate_hz=rate,
                ev_latency_ms=self._ev_latency_ms,
                ev_input_age_s=age,
                expected_rate_hz=expected_rate_hz,
                data_timeout_s=self.data_timeout_s,
                in_air=in_air,
            )

            return VioLiveState(
                available=available,
                source=self._source,
                flags_available=self._seen_flags,
                cs_ev_pos=self._cs_ev_pos,
                cs_ev_yaw=self._cs_ev_yaw,
                cs_ev_hgt=self._cs_ev_hgt,
                cs_ev_vel=self._cs_ev_vel,
                cs_inertial_dead_reckoning=self._cs_dr,
                reject_hor_pos=self._reject["hor_pos"],
                reject_ver_pos=self._reject["ver_pos"],
                reject_yaw=self._reject["yaw"],
                reject_hor_pos_count=self._reject_count["hor_pos"],
                reject_ver_pos_count=self._reject_count["ver_pos"],
                reject_yaw_count=self._reject_count["yaw"],
                ev_input_available=self._seen_ev,
                ev_rate_hz=round(float(rate), 1),
                ev_latency_ms=(round(self._ev_latency_ms, 1)
                               if self._ev_latency_ms is not None else None),
                ev_input_age_s=(round(age, 3) if age is not None else None),
                ev_reset_counter=self._ev_reset,
                ev_quality=self._ev_quality,
                ev_position_finite=self._ev_pos_finite,
                ev_q_finite=self._ev_q_finite,
                ekf_output_available=self._ekf_pos is not None,
                ev_ekf_divergence_m=(round(divergence, 3)
                                     if divergence is not None else None),
                expected_rate_hz=round(float(expected_rate_hz), 1),
                in_air=bool(in_air),
                verdict=verdict,
                verdict_reasons=reasons,
            )
