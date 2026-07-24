"""
EKF2 aid-source analysis — registry-driven generalisation of the EV analyzer.

Prompt 60 built an *external-vision-specific* analyzer (``vio_analyzer.py``).
This module generalises it into a **registry of EKF2 aid sources**, where
external vision is simply one entry.  The SAME metric machine (per-axis
test-ratio stats, innovation bias, fusion dropouts, NEES, rate, delay estimate,
per-source recommendations) runs over every registered source, driven entirely
by the registry — no source is hard-coded in the shared machine.

The stack this tool targets fuses **EV + optical flow (MTF-02) + range-height
(lidar)** as first-class sources; GPS / baro / mag are in the registry too but
typically show up as *inactive* for an indoor no-GPS/no-mag/no-baro build.

No ROS dependency (mirrors ``hifi_analyzer.py`` / ``vio_analyzer.py``).
READ-ONLY — never writes FMU parameters; produces recommendations only.

Domain reference:
``sverk_ws/src/sverk_drone/odometry/vpe/px4_local_pose_publisher/PX4_EKF2_TUNING_AND_DEBUG.md``

── Verified PX4 v1.15.4 facts baked into the registry (checked vs msg/ + module.yaml)

* Prefix trap: aid topics use ``gnss_*`` (``estimator_aid_src_gnss_pos/vel/hgt/
  yaw``) while ``estimator_innovations`` uses ``gps_*`` and flags use ``cs_gps*``.
  ``estimator_aid_src_gps_*`` does NOT exist.
* ``estimator_innovations`` field layout (order): gps_hvel[2], gps_vvel,
  gps_hpos[2], gps_vpos, ev_hvel[2], ev_vvel, ev_hpos[2], ev_vpos, rng_vpos,
  baro_vpos, aux_hvel[2], flow[2], terr_flow[2], heading, mag_field[3],
  gravity[3], drag[2], airspeed, beta, hagl, hagl_rate.  No ``aux_vpos``, no
  ``ev_hgt`` (vertical EV innovation = ``ev_vpos``), no ``rng_hgt`` field
  (range-height = ``rng_vpos``); EV/GNSS/mag-yaw share ``heading``.
* ``reject_*`` = 10 shared flags (hor_vel/ver_vel/hor_pos/ver_pos/yaw/airspeed/
  sideslip/hagl/optflow_x/optflow_y).  There is no per-source reject_baro/rng/
  gps/ev — take per-source rejection from ``aid_src.innovation_rejected``.
* Only ``estimator_aid_src_aux_global_position`` is logged by default
  (SDLOG_PROFILE=1); every other ``estimator_aid_src_*`` needs
  logger_topics.txt.  Delay/NEES for any source therefore require its aid_src
  topic in the log — gate on it.
* Parameters that do NOT exist on 1.15.4 (never suggested): EKF2_EV_GATE,
  EKF2_EVA_GATE, EKF2_REQ_FIX, EKF2_MAG_DECL, EKF2_MAG_YAWLIM.
"""
from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# Reuse the numeric primitives from the EV analyzer (single source of truth).
from .vio_analyzer import (
    TR_REJECT, BIAS_CORR_THRESHOLD, BIAS_MEAN_FLOOR_M, MIN_DELAY_RATE_HZ,
    DELAY_SIGNIFICANT_MS, NEES_OVER, NEES_UNDER, DROPOUT_MIN_S,
    COASTING_WARN_PCT, SOL_CONST_POS_BIT,
    _finite, _pearson, _interp, _rate_from_us, _true_intervals, _xcorr_lag,
)

log = logging.getLogger(__name__)

# Parameters that are NOT present on PX4 v1.15.4 — the recommendation builder
# asserts against this set so a stale name can never leak into the UI.
NONEXISTENT_PARAMS = frozenset({
    "EKF2_EV_GATE", "EKF2_EVA_GATE", "EKF2_REQ_FIX",
    "EKF2_MAG_DECL", "EKF2_MAG_YAWLIM",
})

# Height source-reference note threshold (mean hgt_test_ratio).
HGT_TR_WARN = 0.5


# ── registry ─────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class AidSource:
    """Descriptor of one EKF2 aid source.  Drives extraction + every metric.

    ``innov_fields`` are ``(role, field_base, dim)`` triples naming the
    ``estimator_innovations`` / ``_test_ratios`` / ``_variances`` fields (the
    three topics share a layout).  ``aid_src_topics`` map a role to the optional
    rich ``estimator_aid_src_*`` topic (full uORB name — mind the gnss prefix
    trap) and its dimensionality.
    """
    key: str
    display_name: str
    cs_flags: Tuple[str, ...]            # estimator_status_flags booleans (fusion active)
    primary_cs: str                      # main fusion-active flag for dropout detection
    innov_fields: Tuple[Tuple[str, str, int], ...]
    aid_src_topics: Tuple[Tuple[str, str, int], ...]   # (role, uORB name, dim)
    ctrl_param: str                      # EKF2_*_CTRL — 0 ⇒ disabled
    ctrl_disabled_value: float           # value of ctrl_param meaning "off"
    params: Dict[str, float]             # name → default (v1.15.4)
    gate_params: Tuple[str, ...]         # gate params (all exist on 1.15.4)
    noise_params: Tuple[str, ...]        # noise-floor params for the NEES rec
    delay_param: Optional[str]           # EKF2_*_DELAY or None
    pos_offset_params: Tuple[str, ...]   # *_POS_X/Y/Z (extrinsic)
    dof: int                             # NEES DOF (h-pos=2, scalar=1, 3d=3)
    reject_fields: Tuple[str, ...]       # relevant reject_* flags
    delay_estimable: bool                # source carries a position observation
    source_tag: str = ""

    def primary_aid_topic(self) -> Optional[Tuple[str, str, int]]:
        """First (position-like) aid_src topic — used for NEES / delay."""
        for role, name, dim in self.aid_src_topics:
            if role in ("pos", "hgt", "flow", "mag", "yaw"):
                return (role, name, dim)
        return self.aid_src_topics[0] if self.aid_src_topics else None


AID_SOURCES: List[AidSource] = [
    AidSource(
        key="ev", display_name="External vision (EV)",
        cs_flags=("cs_ev_pos", "cs_ev_vel", "cs_ev_yaw", "cs_ev_hgt"),
        primary_cs="cs_ev_pos",
        innov_fields=(("hpos", "ev_hpos", 2), ("vpos", "ev_vpos", 1),
                      ("hvel", "ev_hvel", 2), ("vvel", "ev_vvel", 1),
                      ("yaw", "heading", 1)),
        aid_src_topics=(("pos", "estimator_aid_src_ev_pos", 2),
                        ("vel", "estimator_aid_src_ev_vel", 3),
                        ("yaw", "estimator_aid_src_ev_yaw", 1),
                        ("hgt", "estimator_aid_src_ev_hgt", 1)),
        ctrl_param="EKF2_EV_CTRL", ctrl_disabled_value=0,
        params={"EKF2_EV_CTRL": 15, "EKF2_EV_DELAY": 0, "EKF2_EV_NOISE_MD": 0,
                "EKF2_EVP_NOISE": 0.1, "EKF2_EVV_NOISE": 0.1, "EKF2_EVA_NOISE": 0.1,
                "EKF2_EVP_GATE": 5.0, "EKF2_EVV_GATE": 3.0,
                "EKF2_EV_POS_X": 0, "EKF2_EV_POS_Y": 0, "EKF2_EV_POS_Z": 0,
                "EKF2_EV_QMIN": 0},
        gate_params=("EKF2_EVP_GATE", "EKF2_EVV_GATE"),
        noise_params=("EKF2_EVP_NOISE", "EKF2_EVA_NOISE"),
        delay_param="EKF2_EV_DELAY",
        pos_offset_params=("EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z"),
        dof=2, reject_fields=("reject_hor_pos", "reject_ver_pos", "reject_yaw"),
        delay_estimable=True, source_tag="ArUco/camera/lidar-odom",
    ),
    AidSource(
        key="optical_flow", display_name="Optical flow (MTF-02)",
        cs_flags=("cs_opt_flow",), primary_cs="cs_opt_flow",
        innov_fields=(("flow", "flow", 2),),
        aid_src_topics=(("flow", "estimator_aid_src_optical_flow", 2),),
        ctrl_param="EKF2_OF_CTRL", ctrl_disabled_value=0,
        params={"EKF2_OF_CTRL": 0, "EKF2_OF_DELAY": 20, "EKF2_OF_QMIN": 1,
                "EKF2_OF_QMIN_GND": 0, "EKF2_OF_N_MIN": 0.15, "EKF2_OF_N_MAX": 0.5,
                "EKF2_OF_GATE": 3.0,
                "EKF2_OF_POS_X": 0, "EKF2_OF_POS_Y": 0, "EKF2_OF_POS_Z": 0,
                "EKF2_TERR_MASK": 3, "EKF2_TERR_NOISE": 5.0, "EKF2_TERR_GRAD": 0.5},
        gate_params=("EKF2_OF_GATE",),
        noise_params=("EKF2_OF_N_MIN", "EKF2_OF_N_MAX"),
        delay_param="EKF2_OF_DELAY",
        pos_offset_params=("EKF2_OF_POS_X", "EKF2_OF_POS_Y", "EKF2_OF_POS_Z"),
        dof=2, reject_fields=("reject_optflow_x", "reject_optflow_y"),
        delay_estimable=False, source_tag="MTF-02",
    ),
    AidSource(
        key="rng_hgt", display_name="Range height (lidar)",
        cs_flags=("cs_rng_hgt", "cs_rng_stuck", "cs_rng_fault",
                  "cs_rng_kin_consistent"),
        primary_cs="cs_rng_hgt",
        innov_fields=(("vpos", "rng_vpos", 1), ("hagl", "hagl", 1),
                      ("hagl_rate", "hagl_rate", 1)),
        aid_src_topics=(("hgt", "estimator_aid_src_rng_hgt", 1),
                        ("terrain", "estimator_aid_src_terrain_range_finder", 1)),
        ctrl_param="EKF2_RNG_CTRL", ctrl_disabled_value=0,
        params={"EKF2_RNG_CTRL": 1, "EKF2_RNG_DELAY": 5, "EKF2_RNG_NOISE": 0.1,
                "EKF2_RNG_SFE": 0.05, "EKF2_RNG_GATE": 5.0, "EKF2_RNG_A_HMAX": 5.0,
                "EKF2_RNG_PITCH": 0,
                "EKF2_RNG_POS_X": 0, "EKF2_RNG_POS_Y": 0, "EKF2_RNG_POS_Z": 0,
                "EKF2_MIN_RNG": 0.1},
        gate_params=("EKF2_RNG_GATE",),
        noise_params=("EKF2_RNG_NOISE",),
        delay_param="EKF2_RNG_DELAY",
        pos_offset_params=("EKF2_RNG_POS_X", "EKF2_RNG_POS_Y", "EKF2_RNG_POS_Z"),
        dof=1, reject_fields=("reject_ver_pos", "reject_hagl"),
        delay_estimable=False, source_tag="MTF-02",
    ),
    AidSource(
        key="gps", display_name="GNSS (GPS)",
        cs_flags=("cs_gps", "cs_gps_hgt", "cs_gps_yaw"), primary_cs="cs_gps",
        innov_fields=(("hpos", "gps_hpos", 2), ("vpos", "gps_vpos", 1),
                      ("hvel", "gps_hvel", 2), ("vvel", "gps_vvel", 1)),
        # PREFIX TRAP: aid topics are gnss_*, innovations are gps_*.
        aid_src_topics=(("pos", "estimator_aid_src_gnss_pos", 2),
                        ("vel", "estimator_aid_src_gnss_vel", 3),
                        ("hgt", "estimator_aid_src_gnss_hgt", 1),
                        ("yaw", "estimator_aid_src_gnss_yaw", 1)),
        ctrl_param="EKF2_GPS_CTRL", ctrl_disabled_value=0,
        params={"EKF2_GPS_CTRL": 7, "EKF2_GPS_DELAY": 110, "EKF2_GPS_P_NOISE": 0.5,
                "EKF2_GPS_V_NOISE": 0.3, "EKF2_GPS_P_GATE": 5.0, "EKF2_GPS_V_GATE": 5.0,
                "EKF2_GPS_POS_X": 0, "EKF2_GPS_POS_Y": 0, "EKF2_GPS_POS_Z": 0},
        gate_params=("EKF2_GPS_P_GATE", "EKF2_GPS_V_GATE"),
        noise_params=("EKF2_GPS_P_NOISE", "EKF2_GPS_V_NOISE"),
        delay_param="EKF2_GPS_DELAY",
        pos_offset_params=("EKF2_GPS_POS_X", "EKF2_GPS_POS_Y", "EKF2_GPS_POS_Z"),
        dof=2, reject_fields=("reject_hor_pos", "reject_ver_pos",
                              "reject_hor_vel", "reject_ver_vel"),
        delay_estimable=True, source_tag="GNSS",
    ),
    AidSource(
        key="baro", display_name="Barometer height",
        cs_flags=("cs_baro_hgt", "cs_gnd_effect"), primary_cs="cs_baro_hgt",
        innov_fields=(("vpos", "baro_vpos", 1),),
        aid_src_topics=(("hgt", "estimator_aid_src_baro_hgt", 1),),
        ctrl_param="EKF2_BARO_CTRL", ctrl_disabled_value=0,
        params={"EKF2_BARO_CTRL": 1, "EKF2_BARO_DELAY": 0, "EKF2_BARO_NOISE": 3.5,
                "EKF2_BARO_GATE": 5.0},
        gate_params=("EKF2_BARO_GATE",),
        noise_params=("EKF2_BARO_NOISE",),
        delay_param="EKF2_BARO_DELAY",
        pos_offset_params=(),
        dof=1, reject_fields=("reject_ver_pos",),
        delay_estimable=False, source_tag="",
    ),
    AidSource(
        key="mag", display_name="Magnetometer",
        cs_flags=("cs_mag", "cs_mag_hdg", "cs_mag_3d", "cs_mag_dec"),
        primary_cs="cs_mag",
        innov_fields=(("mag", "mag_field", 3), ("yaw", "heading", 1)),
        aid_src_topics=(("mag", "estimator_aid_src_mag", 3),),
        # MAG_TYPE is not a bitmask CTRL; on this indoor build 5 == "not used".
        ctrl_param="EKF2_MAG_TYPE", ctrl_disabled_value=5,
        params={"EKF2_MAG_TYPE": 0, "EKF2_MAG_DELAY": 0, "EKF2_MAG_NOISE": 0.05,
                "EKF2_HEAD_NOISE": 0.3, "EKF2_MAG_GATE": 3.0, "EKF2_HDG_GATE": 2.6},
        gate_params=("EKF2_MAG_GATE", "EKF2_HDG_GATE"),
        noise_params=("EKF2_MAG_NOISE", "EKF2_HEAD_NOISE"),
        delay_param="EKF2_MAG_DELAY",
        pos_offset_params=(),
        dof=3, reject_fields=("reject_yaw",),
        delay_estimable=False, source_tag="",
    ),
]

AID_SOURCE_BY_KEY: Dict[str, AidSource] = {s.key: s for s in AID_SOURCES}


def get_source(key: str) -> Optional[AidSource]:
    return AID_SOURCE_BY_KEY.get(key)


# ── per-source series + shared context ───────────────────────────────────────

@dataclass
class AidSourceSeries:
    """Raw per-source arrays pulled from a ULog (each on its own time axis)."""
    key: str
    t_innov: Optional[np.ndarray] = None
    innov: Dict[str, np.ndarray] = field(default_factory=dict)      # role → (N,) | (N,dim)
    innov_tr: Dict[str, np.ndarray] = field(default_factory=dict)
    innov_var: Dict[str, np.ndarray] = field(default_factory=dict)
    t_flags: Optional[np.ndarray] = None
    cs: Dict[str, np.ndarray] = field(default_factory=dict)         # flag name → bool arr
    reject: Dict[str, np.ndarray] = field(default_factory=dict)     # reject name → bool arr
    aid_src: Optional[Dict[str, np.ndarray]] = None                 # rich aid_src topic dict
    aid_src_dim: int = 1
    aid_src_name: str = ""

    def has_innov(self) -> bool:
        return any(v is not None and len(_finite(np.ravel(v))) for v in self.innov.values())

    def fused_ever(self) -> bool:
        arr = self.cs.get(_primary_cs_of(self.key))
        return bool(arr is not None and np.any(np.asarray(arr, dtype=bool)))


def _primary_cs_of(key: str) -> str:
    src = AID_SOURCE_BY_KEY.get(key)
    return src.primary_cs if src else ""


@dataclass
class SharedContext:
    """Cross-source EKF state (velocity, top-level coasting flags, duration)."""
    duration_s: float = 0.0
    t_lpos: Optional[np.ndarray] = None
    lpos_vel: Optional[np.ndarray] = None
    t_status: Optional[np.ndarray] = None
    hgt_test_ratio: Optional[np.ndarray] = None
    solution_status_flags: Optional[np.ndarray] = None
    t_flags: Optional[np.ndarray] = None
    cs_fake_pos: Optional[np.ndarray] = None
    cs_inertial_dead_reckoning: Optional[np.ndarray] = None

    def horizontal_speed(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.t_lpos is None or self.lpos_vel is None:
            return np.empty(0), np.empty(0)
        v = np.asarray(self.lpos_vel, dtype=float)
        return np.asarray(self.t_lpos, dtype=float), np.sqrt(v[:, 0] ** 2 + v[:, 1] ** 2)


@dataclass
class AidSourceBundle:
    """Everything the analyzer needs: shared context + per-source series."""
    shared: SharedContext = field(default_factory=SharedContext)
    sources: Dict[str, AidSourceSeries] = field(default_factory=dict)
    topics_found: List[str] = field(default_factory=list)


# ── extractor ────────────────────────────────────────────────────────────────

_AID_SRC_FIELDS = [
    "observation", "observation_variance", "innovation",
    "innovation_variance", "test_ratio", "innovation_rejected", "fused",
]


def parse_aid_source_series(datasets: list,
                            sources: Optional[List[AidSource]] = None
                            ) -> Optional[AidSourceBundle]:
    """Registry-driven extraction of every aid source from ULog ``datasets``.

    Generalises ``ulog_adapter.parse_estimator_series``: pulls each
    source's ``estimator_innovations`` / ``_test_ratios`` / ``_variances``
    fields, its ``cs_*`` / ``reject_*`` flags from ``estimator_status_flags``,
    and its optional ``estimator_aid_src_*`` topic if present.  Field resolution
    is dynamic (matches the rest of the adapter).  Returns ``None`` when the log
    carries no estimator topic at all.
    """
    from .ulog_adapter import _find_topic, _resolve_field, _vec_field, _ds_ts_us
    sources = sources if sources is not None else AID_SOURCES

    innov_ds  = _find_topic(datasets, ["estimator_innovations"])
    tr_ds     = _find_topic(datasets, ["estimator_innovation_test_ratios"])
    var_ds    = _find_topic(datasets, ["estimator_innovation_variances"])
    flags_ds  = _find_topic(datasets, ["estimator_status_flags"])
    status_ds = _find_topic(datasets, ["estimator_status"])
    lpos_ds   = _find_topic(datasets, ["vehicle_local_position"])

    # Resolve each source's aid_src topic once (primary topic per source).
    aid_ds_by_key: Dict[str, Tuple[str, int, Any]] = {}
    for src in sources:
        prim = src.primary_aid_topic()
        if prim is None:
            continue
        _role, name, dim = prim
        ds = _find_topic(datasets, [name])
        if ds is not None:
            aid_ds_by_key[src.key] = (name, dim, ds)

    present = [d for d in (innov_ds, tr_ds, var_ds, flags_ds, status_ds, lpos_ds)
               if d is not None]
    present += [ds for (_n, _d, ds) in aid_ds_by_key.values()]
    if not present:
        return None

    t0 = min(int(_ds_ts_us(d)[0]) for d in present if len(_ds_ts_us(d)) > 0)

    def _t(ds) -> np.ndarray:
        return (_ds_ts_us(ds).astype(np.float64) - t0) / 1e6

    def _field(ds, base, dim):
        if ds is None:
            return None
        if dim > 1:
            return _vec_field(ds.data, base, dim)
        return _resolve_field(ds.data, [base])

    bundle = AidSourceBundle()
    sh = bundle.shared

    # ── shared estimator_status_flags (top-level coasting + per-source cs) ────
    fake_pos = dead_reck = None
    if flags_ds is not None:
        sh.t_flags = _t(flags_ds)
        def _bool(name):
            arr = _resolve_field(flags_ds.data, [name])
            return None if arr is None else (np.asarray(arr) != 0)
        fake_pos = _bool("cs_fake_pos")
        dead_reck = _bool("cs_inertial_dead_reckoning")
        sh.cs_fake_pos = fake_pos
        sh.cs_inertial_dead_reckoning = dead_reck

    # ── shared estimator_status ──────────────────────────────────────────────
    if status_ds is not None:
        sh.t_status = _t(status_ds)
        sh.hgt_test_ratio = _resolve_field(status_ds.data, ["hgt_test_ratio"])
        sh.solution_status_flags = _resolve_field(status_ds.data, ["solution_status_flags"])

    # ── shared vehicle_local_position (velocity for bias / delay) ────────────
    if lpos_ds is not None:
        sh.t_lpos = _t(lpos_ds)
        vx = _resolve_field(lpos_ds.data, ["vx"])
        vy = _resolve_field(lpos_ds.data, ["vy"])
        vz = _resolve_field(lpos_ds.data, ["vz"])
        if vx is not None and vy is not None and vz is not None:
            sh.lpos_vel = np.column_stack([vx, vy, vz])

    ends = [float(t[-1]) - float(t[0])
            for t in (sh.t_flags, sh.t_status, sh.t_lpos)
            if t is not None and len(t) > 1]
    sh.duration_s = max(ends) if ends else 0.0

    def _aid(ds) -> Optional[Dict[str, np.ndarray]]:
        if ds is None:
            return None
        out: Dict[str, np.ndarray] = {"t": _t(ds)}
        found = False
        for fld in _AID_SRC_FIELDS:
            vec = None
            for dim in (3, 2, 1):
                vec = _vec_field(ds.data, fld, dim)
                if vec is not None:
                    break
            if vec is not None:
                out[fld] = vec if vec.shape[1] > 1 else vec[:, 0]
                found = True
                continue
            scal = _resolve_field(ds.data, [fld])
            if scal is not None:
                out[fld] = np.asarray(scal)
                found = True
        return out if found else None

    # ── per-source extraction ────────────────────────────────────────────────
    t_innov = _t(innov_ds) if innov_ds is not None else None
    t_flags = sh.t_flags
    for src in sources:
        s = AidSourceSeries(key=src.key, t_innov=t_innov, t_flags=t_flags)
        for role, base, dim in src.innov_fields:
            iv = _field(innov_ds, base, dim)
            if iv is not None:
                s.innov[role] = iv
            tv = _field(tr_ds, base, dim)
            if tv is not None:
                s.innov_tr[role] = tv
            vv = _field(var_ds, base, dim)
            if vv is not None:
                s.innov_var[role] = vv
        if flags_ds is not None:
            for flag in src.cs_flags:
                arr = _resolve_field(flags_ds.data, [flag])
                if arr is not None:
                    s.cs[flag] = np.asarray(arr) != 0
            for rej in src.reject_fields:
                arr = _resolve_field(flags_ds.data, [rej])
                if arr is not None:
                    s.reject[rej] = np.asarray(arr) != 0
        if src.key in aid_ds_by_key:
            name, dim, ds = aid_ds_by_key[src.key]
            s.aid_src = _aid(ds)
            s.aid_src_dim = dim
            s.aid_src_name = name
        bundle.sources[src.key] = s

    # ── topics found (for capability badges) ─────────────────────────────────
    tf = bundle.topics_found
    if flags_ds is not None:  tf.append("estimator_status_flags")
    if innov_ds is not None:  tf.append("estimator_innovations")
    if tr_ds is not None:     tf.append("estimator_innovation_test_ratios")
    if var_ds is not None:    tf.append("estimator_innovation_variances")
    if status_ds is not None: tf.append("estimator_status")
    if lpos_ds is not None:   tf.append("vehicle_local_position")
    for name, _dim, _ds in aid_ds_by_key.values():
        tf.append(name)
    return bundle


# ── result dataclasses ───────────────────────────────────────────────────────

@dataclass
class AxisTestRatio:
    axis: str
    mean: float
    p95: float
    pct_over_1: float
    n: int


@dataclass
class AxisBias:
    axis: str
    mean_innov: float
    speed_corr: float
    classification: str          # "delay" | "extrinsic" | "none"
    confidence: str
    note: str = ""


@dataclass
class DropoutInterval:
    t_start: float
    t_end: float
    duration_s: float
    cause: str


@dataclass
class DropoutReport:
    n_intervals: int = 0
    total_duration_s: float = 0.0
    max_duration_s: float = 0.0
    intervals: List[DropoutInterval] = field(default_factory=list)


@dataclass
class NeesAxis:
    axis: str
    mean_nis: float
    dof: int
    verdict: str                 # "ok" | "overconfident" | "underconfident"
    source: str                  # "aid_src"
    note: str = ""


@dataclass
class RateInfo:
    rate_hz: float
    n_samples: int
    note: str = ""


@dataclass
class DelayEstimate:
    estimable: bool
    delay_ms: Optional[float]
    confidence: str
    r_squared: float
    dynamics_score: float
    method: str
    note: str = ""


@dataclass
class AidRecommendation:
    param: str
    direction: str               # "increase" | "decrease" | "info"
    current: Optional[float]
    suggested: Optional[float]
    reason: str
    reason_key: str = ""
    reason_kwargs: list = field(default_factory=list)
    confidence: str = "medium"

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


@dataclass
class SourceReport:
    key: str
    display_name: str
    active: bool                 # fused in the log OR has innovation data + CTRL≠0
    inactive_reason: str = ""
    ctrl_param: str = ""
    ctrl_value: Optional[float] = None
    test_ratios: List[AxisTestRatio] = field(default_factory=list)
    biases: List[AxisBias] = field(default_factory=list)
    dropouts: DropoutReport = field(default_factory=DropoutReport)
    nees: List[NeesAxis] = field(default_factory=list)
    rate: Optional[RateInfo] = None
    delay: Optional[DelayEstimate] = None
    recommendations: List[AidRecommendation] = field(default_factory=list)
    aid_src_present: bool = False
    params: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


@dataclass
class TopLevelAlarm:
    coasting: bool = False
    fake_pos_intervals: List[DropoutInterval] = field(default_factory=list)
    dead_reckoning_intervals: List[DropoutInterval] = field(default_factory=list)
    coasting_fraction_pct: float = 0.0


@dataclass
class AidSourceAnalysisResult:
    source_name: str
    duration_s: float
    available: bool
    top_level: TopLevelAlarm = field(default_factory=TopLevelAlarm)
    sources: List[SourceReport] = field(default_factory=list)
    topics_found: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


# ── translations (EN + RU) ───────────────────────────────────────────────────

AID_REASON_TR: Dict[str, Dict[str, str]] = {
    "en": {
        "rec_delay":
            "{src}: observation lags the EKF velocity by ≈{0:.0f} ms (our cross-"
            "correlation estimate, R²={1:.2f}). Move {2} toward {3:.0f} ms. NOTE: "
            "our estimate, not PX4's official sweep-to-minimum-innovation method.",
        "rec_extrinsic":
            "{src}: innovation offset is roughly constant and does not grow with "
            "speed (mean {0:.3f} m) → extrinsic / frame-alignment, not delay. "
            "Check {1} and the source→body alignment.",
        "rec_noise_up":
            "{src}: innovations are over-confident (mean NIS {0:.1f} ≫ DOF {1}). "
            "Raise the noise floor {2} — the filter trusts this source too much.",
        "rec_noise_down":
            "{src}: innovations are conservative (mean NIS {0:.2f} ≪ DOF {1}) and "
            "hold is soft. {2} can be tightened for crisper hold (noisier).",
        "rec_gate":
            "{src}: {0:.0f}% of {1} innovations sit at/over the gate. If genuine "
            "rejections, widen {2}; if outliers, leave it and fix the source.",
        "rec_dropout":
            "{src}: fusion drops out {0} time(s), {1:.1f} s total (max {2:.1f} s). "
            "Inspect the intervals below.",
        "rec_flow_quality":
            "[flow] Correlate dropouts with flow quality — EKF2_OF_QMIN / "
            "EKF2_OF_QMIN_GND gate low-quality frames; flow is only valid with a "
            "healthy terrain/HAGL estimate.",
        "rec_rng_health":
            "[range] Check cs_rng_stuck / cs_rng_fault over the dropout intervals "
            "and EKF2_RNG_A_HMAX (max height for range aiding).",
        "rec_mag_health":
            "[mag] Check cs_mag_field_disturbed / cs_mag_fault — magnetic "
            "disturbance rejects yaw fusion.",
        "rec_gps_inactive":
            "[gps] GPS is inactive for this build (indoor). Confirm cs_gps=false "
            "and EKF2_GPS_CTRL=0 — nothing to tune.",
        "rec_inactive":
            "{src}: inactive ({0}=0, no fusion in the log). No recommendations.",
        "rec_disclaimer":
            "Heuristic starting points. Change one EKF2 parameter at a time and "
            "re-verify on the stand (listener estimator_aid_src_*).",
        "rec_delay_hint":
            "{src}: delay/NEES need estimator_aid_src_{key} in the log — add it to "
            "logger_topics.txt (the 2 Hz estimator_innovations stream is too slow).",
        "top_coasting":
            "ALL odometry lost: fake_position / inertial dead-reckoning active "
            "{0:.0f}% of the log. The EKF is coasting on inertial only.",
    },
    "ru": {
        "rec_delay":
            "{src}: измерение отстаёт от EKF-скорости на ≈{0:.0f} мс (оценка кросс-"
            "корреляцией, R²={1:.2f}). Двигайте {2} к {3:.0f} мс. ВНИМАНИЕ: это "
            "наша оценка, не официальный свип-до-минимума-инноваций PX4.",
        "rec_extrinsic":
            "{src}: смещение инноваций примерно постоянно и не растёт со скоростью "
            "(среднее {0:.3f} м) → экстринсик/выравнивание фреймов, не задержка. "
            "Проверьте {1} и выравнивание источник→тело.",
        "rec_noise_up":
            "{src}: инновации переуверены (среднее NIS {0:.1f} ≫ DOF {1}). "
            "Поднимите флор шума {2} — фильтр слишком доверяет источнику.",
        "rec_noise_down":
            "{src}: инновации консервативны (среднее NIS {0:.2f} ≪ DOF {1}), "
            "удержание вялое. {2} можно зажать для более цепкого удержания (шумнее).",
        "rec_gate":
            "{src}: {0:.0f}% инноваций {1} у/за гейтом. Если отказы реальные — "
            "расширьте {2}; если выбросы — не трогайте, чините источник.",
        "rec_dropout":
            "{src}: фьюз пропадает {0} раз, суммарно {1:.1f} с (макс {2:.1f} с). "
            "Разберите интервалы ниже.",
        "rec_flow_quality":
            "[flow] Сопоставьте дропауты с качеством потока — EKF2_OF_QMIN / "
            "EKF2_OF_QMIN_GND гейтят кадры низкого качества; поток валиден только "
            "при здоровой оценке terrain/HAGL.",
        "rec_rng_health":
            "[range] Проверьте cs_rng_stuck / cs_rng_fault на интервалах дропаута "
            "и EKF2_RNG_A_HMAX (макс. высота приёма дальномера).",
        "rec_mag_health":
            "[mag] Проверьте cs_mag_field_disturbed / cs_mag_fault — магнитная "
            "помеха отбраковывает фьюз курса.",
        "rec_gps_inactive":
            "[gps] GPS неактивен для этой сборки (indoor). Подтвердите cs_gps=false "
            "и EKF2_GPS_CTRL=0 — тюнить нечего.",
        "rec_inactive":
            "{src}: неактивен ({0}=0, фьюза в логе нет). Рекомендаций нет.",
        "rec_disclaimer":
            "Эвристические оценки. Меняйте по одному параметру EKF2 и "
            "перепроверяйте на стенде (listener estimator_aid_src_*).",
        "rec_delay_hint":
            "{src}: delay/NEES нужен estimator_aid_src_{key} в логе — добавьте его "
            "в logger_topics.txt (поток estimator_innovations 2 Гц слишком медленный).",
        "top_coasting":
            "ВСЯ одометрия потеряна: fake_position / инерциальный dead-reckoning "
            "активен {0:.0f}% лога. EKF коастит только на инерции.",
    },
}


def translate(key: str, lang: str, args: list, src: str = "", src_key: str = "") -> str:
    templates = AID_REASON_TR.get(lang, AID_REASON_TR["en"])
    fmt = templates.get(key) or AID_REASON_TR["en"].get(key, key)
    try:
        return fmt.format(*args, src=src, key=src_key)
    except (IndexError, KeyError):
        return fmt


# ── metrics ──────────────────────────────────────────────────────────────────

def axis_test_ratio_stats(source: AidSource,
                          series: AidSourceSeries) -> List[AxisTestRatio]:
    """Per-role test-ratio mean / p95 / % over 1.0 (rejected)."""
    out: List[AxisTestRatio] = []
    for role, _base, _dim in source.innov_fields:
        arr = series.innov_tr.get(role)
        if arr is None:
            continue
        a = _finite(np.ravel(np.asarray(arr, dtype=float)))
        if len(a) == 0 or float(np.nanmax(np.abs(a))) <= 1e-9:
            continue
        out.append(AxisTestRatio(
            axis=f"{source.key}_{role}",
            mean=round(float(np.mean(a)), 4),
            p95=round(float(np.percentile(a, 95)), 4),
            pct_over_1=round(float(np.mean(a > TR_REJECT) * 100.0), 1),
            n=int(len(a)),
        ))
    return out


def innovation_bias(source: AidSource, series: AidSourceSeries,
                    ctx: SharedContext) -> List[AxisBias]:
    """Delay-vs-extrinsic classification for the source's POSITION innovations.

    Only position roles (hpos / vpos) are classified: a non-zero mean growing
    with speed → delay; a constant offset with weak speed-corr → extrinsic.
    """
    if series.t_innov is None:
        return []
    t_sp, speed = ctx.horizontal_speed()
    have_speed = len(t_sp) >= 4 and float(np.nanstd(speed)) > 1e-3
    speed_on_innov = (_interp(t_sp, speed, series.t_innov)
                      if have_speed else np.full(len(series.t_innov), np.nan))

    comps: List[Tuple[str, np.ndarray]] = []
    hpos = series.innov.get("hpos")
    if hpos is not None:
        hpos = np.asarray(hpos, dtype=float)
        comps.append((f"{source.key}_north", hpos[:, 0]))
        comps.append((f"{source.key}_east", hpos[:, 1]))
    vpos = series.innov.get("vpos")
    if vpos is not None:
        comps.append((f"{source.key}_down", np.asarray(vpos, dtype=float).ravel()))

    out: List[AxisBias] = []
    for axis, innov in comps:
        f = _finite(innov)
        if len(f) < 8:
            continue
        mean = float(np.mean(f))
        std = float(np.std(f))
        corr = _pearson(np.abs(innov), speed_on_innov) if have_speed else 0.0
        significant = abs(mean) > max(BIAS_MEAN_FLOOR_M, 0.5 * std)
        if have_speed and abs(corr) >= BIAS_CORR_THRESHOLD:
            cls, note = "delay", "Innovation grows with speed → delay-like."
            conf = "medium" if abs(corr) >= 0.7 and len(f) >= 20 else "low"
        elif significant:
            cls, note = "extrinsic", "Constant offset, weak speed corr → extrinsic-like."
            conf = "medium" if abs(mean) > 3 * BIAS_MEAN_FLOOR_M else "low"
        else:
            cls, note, conf = "none", "No significant bias.", "low"
        out.append(AxisBias(axis=axis, mean_innov=round(mean, 4),
                            speed_corr=round(corr, 3), classification=cls,
                            confidence=conf, note=note))
    return out


def fusion_dropouts(source: AidSource, series: AidSourceSeries,
                    ctx: SharedContext) -> DropoutReport:
    """Fusion-inactive + rejection intervals for this source.

    Primary-cs false runs (source not fused) + its reject_* runs + per-source
    innovation_rejected from the aid_src topic when present.
    """
    rep = DropoutReport()
    intervals: List[DropoutInterval] = []

    def _collect(t, mask, cause):
        if t is None or mask is None:
            return
        for a, b in _true_intervals(t, mask):
            if b - a >= DROPOUT_MIN_S:
                intervals.append(DropoutInterval(round(a, 3), round(b, 3),
                                                 round(b - a, 3), cause))

    prim = series.cs.get(source.primary_cs)
    if prim is not None:
        _collect(series.t_flags, ~np.asarray(prim, dtype=bool),
                 f"{source.primary_cs}_false")
    for rej in source.reject_fields:
        arr = series.reject.get(rej)
        if arr is not None:
            _collect(series.t_flags, np.asarray(arr, dtype=bool), rej)

    # Per-source rejection straight from the aid_src topic (unambiguous).
    if series.aid_src is not None and "innovation_rejected" in series.aid_src:
        t_a = series.aid_src.get("t")
        rej = np.asarray(series.aid_src["innovation_rejected"]).astype(bool)
        _collect(t_a, rej, "aid_src_rejected")

    intervals.sort(key=lambda iv: iv.t_start)
    rep.intervals = intervals
    rep.n_intervals = len(intervals)
    if intervals:
        durs = [iv.duration_s for iv in intervals]
        rep.total_duration_s = round(float(sum(durs)), 3)
        rep.max_duration_s = round(float(max(durs)), 3)
    return rep


def nees_consistency(source: AidSource, series: AidSourceSeries) -> List[NeesAxis]:
    """NEES/NIS from the aid_src topic (innovation² / innovation_variance).

    Only computable when the rich aid_src topic is in the log.
    """
    out: List[NeesAxis] = []
    aid = series.aid_src
    if aid is None or "innovation" not in aid or "innovation_variance" not in aid:
        return out
    innov = np.asarray(aid["innovation"], dtype=float)
    var = np.asarray(aid["innovation_variance"], dtype=float)
    if innov.ndim == 1:
        innov = innov[:, None]
    if var.ndim == 1:
        var = var[:, None]
    dof = innov.shape[1]
    nis = np.nansum(innov ** 2 / np.where(var > 1e-12, var, np.nan), axis=1)
    nis = _finite(nis)
    if not len(nis):
        return out
    m = float(np.mean(nis))
    if m > NEES_OVER * dof:
        verdict = "overconfident"
    elif m < NEES_UNDER * dof:
        verdict = "underconfident"
    else:
        verdict = "ok"
    out.append(NeesAxis(f"{source.key}_pos", round(m, 3), dof, verdict, "aid_src"))
    return out


def rate_info(series: AidSourceSeries) -> Optional[RateInfo]:
    if series.aid_src is None or "t" not in series.aid_src:
        return None
    t = np.asarray(series.aid_src["t"], dtype=float)
    rate = _rate_from_us(t * 1e6)
    return RateInfo(rate_hz=round(rate, 1), n_samples=int(len(t)))


def estimate_delay(source: AidSource, series: AidSourceSeries,
                   ctx: SharedContext, max_lag_ms: float = 150.0) -> DelayEstimate:
    """Cross-correlate d(observation)/dt against EKF velocity → delay.

    Only meaningful for horizontal-position sources with a rich aid_src topic
    (ev, gps).  Gated otherwise with a logger_topics.txt hint.
    """
    hint = (f"Add {source.primary_aid_topic()[1] if source.primary_aid_topic() else 'the aid_src topic'} "
            "via logger_topics.txt — the 2 Hz estimator_innovations stream is too "
            "slow to estimate the delay.")
    if not source.delay_estimable:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "n/a",
                             "Delay is not estimable for this source type "
                             "(no horizontal position observation).")
    aid = series.aid_src
    if aid is None or "observation" not in aid or ctx.lpos_vel is None:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated", hint)
    obs = np.asarray(aid["observation"], dtype=float)
    if obs.ndim != 2 or obs.shape[1] < 2:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated", hint)
    t_ev = np.asarray(aid["t"], dtype=float)

    rate = _rate_from_us(t_ev * 1e6)
    if rate < MIN_DELAY_RATE_HZ:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated",
                             f"aid_src rate {rate:.1f} Hz < {MIN_DELAY_RATE_HZ:.0f} Hz. " + hint)

    t_v = np.asarray(ctx.t_lpos, dtype=float)
    ekf_vel = np.asarray(ctx.lpos_vel, dtype=float)
    t_lo = max(float(t_ev[0]), float(t_v[0]))
    t_hi = min(float(t_ev[-1]), float(t_v[-1]))
    if t_hi - t_lo < 1.0:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated",
                             "Observation and EKF-velocity records do not overlap enough.")
    fs = rate
    tg = np.arange(t_lo, t_hi, 1.0 / fs)
    if len(tg) < 32:
        return DelayEstimate(False, None, "low", 0.0, 0.0, "gated", hint)

    max_lag = int(round(max_lag_ms / 1000.0 * fs))
    dyn = float(np.nanstd(np.sqrt(ekf_vel[:, 0] ** 2 + ekf_vel[:, 1] ** 2)))
    best_corr = lag_num = lag_w = 0.0
    for ax in (0, 1):
        ev_p = _interp(t_ev, obs[:, ax], tg)
        ekf_v = _interp(t_v, ekf_vel[:, ax], tg)
        ev_v = np.gradient(ev_p, 1.0 / fs)
        a = np.nan_to_num(ev_v - np.nanmean(ev_v))
        b = np.nan_to_num(ekf_v - np.nanmean(ekf_v))
        if np.std(a) < 1e-9 or np.std(b) < 1e-9:
            continue
        lag, corr = _xcorr_lag(a, b, max_lag)
        w = max(0.0, corr)
        lag_num += lag * w
        lag_w += w
        best_corr = max(best_corr, corr)

    if lag_w <= 0:
        return DelayEstimate(True, None, "low", round(best_corr, 3), round(dyn, 3),
                             "xcorr", "No coherent observation↔EKF velocity correlation.")
    delay_ms = (lag_num / lag_w) / fs * 1000.0
    if dyn < 0.05:
        conf, note = "low", "Very little motion in the log — delay unreliable."
    elif best_corr >= 0.6:
        conf, note = "medium", ""
    else:
        conf, note = "low", "Weak correlation — treat as indicative only."
    return DelayEstimate(True, round(delay_ms, 1), conf, round(best_corr, 3),
                         round(dyn, 3), "xcorr", note)


# ── recommendations ──────────────────────────────────────────────────────────

def build_recommendations(source: AidSource, report: SourceReport,
                          params: Dict[str, Any], lang: str = "en"
                          ) -> List[AidRecommendation]:
    recs: List[AidRecommendation] = []
    name = source.display_name

    def _cur(pname):
        v = params.get(pname)
        try:
            return float(v) if v is not None else None
        except (TypeError, ValueError):
            return None

    def _mk(param, direction, current, suggested, key, args, conf):
        # Hard guard: never emit a parameter that does not exist on v1.15.4.
        assert param not in NONEXISTENT_PARAMS, f"nonexistent param leaked: {param}"
        recs.append(AidRecommendation(
            param=param, direction=direction, current=current, suggested=suggested,
            reason=translate(key, lang, args, src=name, src_key=source.key),
            reason_key=key, reason_kwargs=list(args), confidence=conf))

    if not report.active:
        _mk("NOTE", "info", _cur(source.ctrl_param), None, "rec_inactive",
            [source.ctrl_param], "high")
        if source.key == "gps":
            _mk("NOTE", "info", None, None, "rec_gps_inactive", [], "low")
        return recs

    # ── delay (only when the estimate is trustworthy) ────────────────────────
    d = report.delay
    delay_axes = [b for b in report.biases if b.classification == "delay"]
    extr_axes = [b for b in report.biases if b.classification == "extrinsic"]
    if (source.delay_param and d and d.estimable and d.delay_ms is not None
            and d.confidence in ("medium", "high")):
        cur = _cur(source.delay_param)
        base = cur if cur is not None else float(source.params.get(source.delay_param, 0.0))
        if abs(d.delay_ms - base) > DELAY_SIGNIFICANT_MS:
            suggested = round(max(0.0, d.delay_ms), 1)
            direction = "increase" if suggested > base else "decrease"
            _mk(source.delay_param, direction, cur, suggested, "rec_delay",
                [d.delay_ms, d.r_squared, source.delay_param, suggested], d.confidence)
    elif extr_axes and not delay_axes and source.pos_offset_params:
        worst = max(extr_axes, key=lambda b: abs(b.mean_innov))
        _mk(source.pos_offset_params[0].rsplit("_", 1)[0], "info", None, None,
            "rec_extrinsic", [worst.mean_innov, "/".join(source.pos_offset_params)],
            worst.confidence)

    # ── noise floor from NEES ────────────────────────────────────────────────
    noise_param = source.noise_params[0] if source.noise_params else None
    if noise_param:
        for nx in report.nees:
            if nx.verdict == "overconfident":
                _mk(noise_param, "increase", _cur(noise_param), None,
                    "rec_noise_up", [nx.mean_nis, nx.dof, noise_param], "medium")
            elif nx.verdict == "underconfident":
                _mk(noise_param, "decrease", _cur(noise_param), None,
                    "rec_noise_down", [nx.mean_nis, nx.dof, noise_param], "low")

    # ── gate (many rejections at the boundary) ───────────────────────────────
    gate_param = source.gate_params[0] if source.gate_params else None
    worst_tr = max((t for t in report.test_ratios), key=lambda t: t.pct_over_1,
                   default=None)
    if gate_param and worst_tr and worst_tr.pct_over_1 > 10.0:
        _mk(gate_param, "increase", _cur(gate_param), None, "rec_gate",
            [worst_tr.pct_over_1, worst_tr.axis, gate_param], "low")

    # ── dropouts + source-specific health note ───────────────────────────────
    dr = report.dropouts
    if dr.n_intervals > 0:
        _mk("NOTE", "info", None, None, "rec_dropout",
            [dr.n_intervals, dr.total_duration_s, dr.max_duration_s], "medium")
        health_key = {"optical_flow": "rec_flow_quality", "rng_hgt": "rec_rng_health",
                      "mag": "rec_mag_health"}.get(source.key)
        if health_key:
            _mk("NOTE", "info", None, None, health_key, [], "low")

    # ── logger_topics hint when delay/NEES unavailable ───────────────────────
    if not report.aid_src_present and source.delay_estimable:
        _mk("NOTE", "info", None, None, "rec_delay_hint", [], "low")

    if recs:
        _mk("NOTE", "info", None, None, "rec_disclaimer", [], "high")
    return recs


# ── orchestrator ─────────────────────────────────────────────────────────────

def _top_level_alarm(ctx: SharedContext) -> TopLevelAlarm:
    alarm = TopLevelAlarm()
    coast_intervals: List[Tuple[float, float]] = []

    def _ivs(t, mask, cause):
        out = []
        if t is None or mask is None:
            return out
        for a, b in _true_intervals(t, np.asarray(mask, dtype=bool)):
            coast_intervals.append((a, b))
            if b - a >= DROPOUT_MIN_S:
                out.append(DropoutInterval(round(a, 3), round(b, 3),
                                           round(b - a, 3), cause))
        return out

    alarm.fake_pos_intervals = _ivs(ctx.t_flags, ctx.cs_fake_pos, "fake_pos")
    alarm.dead_reckoning_intervals = _ivs(
        ctx.t_flags, ctx.cs_inertial_dead_reckoning, "dead_reckoning")
    if ctx.solution_status_flags is not None and ctx.t_status is not None:
        sol = np.asarray(ctx.solution_status_flags, dtype=np.int64)
        const_pos = ((sol >> SOL_CONST_POS_BIT) & 1) != 0
        for a, b in _true_intervals(ctx.t_status, const_pos):
            coast_intervals.append((a, b))

    alarm.coasting = bool(alarm.fake_pos_intervals or alarm.dead_reckoning_intervals)
    if ctx.duration_s > 0 and coast_intervals:
        total = sum(max(0.0, b - a) for a, b in coast_intervals)
        alarm.coasting_fraction_pct = round(
            min(100.0, total / ctx.duration_s * 100.0), 1)
    return alarm


class AidSourceAnalyzer:
    """Registry-driven analysis over every EKF2 aid source (ULog offline)."""

    def __init__(self, lang: str = "en"):
        self.lang = lang if lang in AID_REASON_TR else "en"

    def analyse(self, parse_result: Any) -> AidSourceAnalysisResult:
        meta = parse_result.metadata
        bundle = getattr(parse_result, "aid_source_bundle", None)
        params = getattr(meta, "params_all", None) or getattr(meta, "params", {}) or {}
        result = AidSourceAnalysisResult(
            source_name=getattr(meta, "source_name", "?"),
            duration_s=getattr(meta, "duration_s", 0.0),
            available=bundle is not None,
        )
        if bundle is None:
            result.warnings.append(
                "No estimator/EV topics in this log — aid-sources tab unavailable.")
            return result
        return self.analyse_bundle(bundle, params, result)

    def analyse_bundle(self, bundle: AidSourceBundle, params: Dict[str, Any],
                       result: Optional[AidSourceAnalysisResult] = None
                       ) -> AidSourceAnalysisResult:
        if result is None:
            result = AidSourceAnalysisResult(source_name="?",
                                             duration_s=bundle.shared.duration_s,
                                             available=True)
        result.available = True
        result.topics_found = list(bundle.topics_found)
        result.top_level = _top_level_alarm(bundle.shared)

        for src in AID_SOURCES:
            series = bundle.sources.get(src.key) or AidSourceSeries(key=src.key)
            report = self._analyse_source(src, series, bundle.shared, params)
            result.sources.append(report)
        return result

    def _analyse_source(self, src: AidSource, series: AidSourceSeries,
                        ctx: SharedContext, params: Dict[str, Any]) -> SourceReport:
        ctrl_val = None
        try:
            cv = params.get(src.ctrl_param)
            ctrl_val = float(cv) if cv is not None else None
        except (TypeError, ValueError):
            ctrl_val = None

        fused = series.fused_ever()
        has_data = series.has_innov()
        ctrl_off = (ctrl_val is not None and ctrl_val == src.ctrl_disabled_value)
        active = bool(fused or (has_data and not ctrl_off))

        report = SourceReport(
            key=src.key, display_name=src.display_name, active=active,
            ctrl_param=src.ctrl_param, ctrl_value=ctrl_val,
            aid_src_present=series.aid_src is not None,
            params=dict(src.params),
        )
        if not active:
            report.inactive_reason = (
                "CTRL=0 / not fused in this log" if ctrl_off or not has_data
                else "no fusion detected")
            report.recommendations = build_recommendations(
                src, report, params, self.lang)
            return report

        report.test_ratios = axis_test_ratio_stats(src, series)
        report.biases = innovation_bias(src, series, ctx)
        report.dropouts = fusion_dropouts(src, series, ctx)
        report.nees = nees_consistency(src, series)
        report.rate = rate_info(series)
        report.delay = estimate_delay(src, series, ctx)
        report.recommendations = build_recommendations(src, report, params, self.lang)
        return report
