"""
ULog (.ulg) → TelemetryFrame adapter.

pyulog is an OPTIONAL dependency — this module guards every import and
raises ImportError with a human-readable install hint when missing.

Design
------
* Field names are resolved DYNAMICALLY from ULog.message_formats — never
  hardcoded, because PX4 firmware versions differ.
* All datasets are aligned to the vehicle_angular_velocity timestamp axis
  using forward-fill (last-known-value).
* Parameters are extracted from initial_parameters (snapshot at log start)
  plus changed_parameters (in-flight changes, timestamped).
* Multi-instance datasets (sensor_gyro[0], sensor_gyro[1], esc_status[i])
  are handled via multi_id.

Reference-based ID note
-----------------------------------
The excitation reference r(t) (``exc_ref`` etc.) is a COMPANION ROS 2 topic
(``offboard_interfaces/ExcitationSample``), not a PX4 uORB topic, so it is
never present in an FMU ULog. Offline ULog runs therefore leave the ``exc_*``
frame fields as None and the analyzer falls back to the setpoint-based (biased)
FRF. To get unbiased reference-based ID offline you must record the reference
separately (e.g. a companion rosbag) and merge — the default .ulg cannot carry
it. Reference-based ID is primarily a LIVE (DDS) feature.
"""
from __future__ import annotations

import io
import logging
import math
import tempfile
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

log = logging.getLogger(__name__)

# ── conditional import ────────────────────────────────────────────────────────

_PYULOG_AVAILABLE = False
try:
    from pyulog import ULog  # type: ignore
    _PYULOG_AVAILABLE = True
except ImportError:
    pass


def _require_pyulog():
    if not _PYULOG_AVAILABLE:
        raise ImportError(
            "pyulog is not installed. "
            "Install with: pip install pyulog\n"
            "Then rebuild the package: colcon build --packages-select pid_tuning_assistant"
        )


# ── candidate field lists (resolved at parse time) ───────────────────────────

_ANG_VEL_XYZ = ["xyz[0]", "xyz[1]", "xyz[2]"]
_RATE_FIELDS = {
    "roll_rate":  ["xyz[0]", "rollspeed",  "x"],
    "pitch_rate": ["xyz[1]", "pitchspeed", "y"],
    "yaw_rate":   ["xyz[2]", "yawspeed",   "z"],
}
_RATE_SP_FIELDS = {
    "roll_rate_sp":  ["roll",  "rollspeed",  "xyz[0]"],
    "pitch_rate_sp": ["pitch", "pitchspeed", "xyz[1]"],
    "yaw_rate_sp":   ["yaw",   "yawspeed",   "xyz[2]"],
}
_ATT_Q = ["q[0]", "q[1]", "q[2]", "q[3]"]
_ATT_SP_Q = ["q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"]
_TORQUE_FIELDS = {
    "torque_roll":  ["xyz[0]", "roll",  "x"],
    "torque_pitch": ["xyz[1]", "pitch", "y"],
    "torque_yaw":   ["xyz[2]", "yaw",   "z"],
}
_RATE_CTRL_INTEG = {
    "int_roll":  ["rollspeed_integ",  "roll_integ"],
    "int_pitch": ["pitchspeed_integ", "pitch_integ"],
    "int_yaw":   ["yawspeed_integ",   "yaw_integ"],
}
_MOTOR_FIELDS = {
    "m0": ["control[0]", "output[0]"],
    "m1": ["control[1]", "output[1]"],
    "m2": ["control[2]", "output[2]"],
    "m3": ["control[3]", "output[3]"],
}
_GYRO_FIELDS = {
    "gx": ["x", "gyro_rad[0]"],
    "gy": ["y", "gyro_rad[1]"],
    "gz": ["z", "gyro_rad[2]"],
}

_TOPIC_CANDIDATES = {
    "angular_velocity":    ["vehicle_angular_velocity"],
    "rates_setpoint":      ["vehicle_rates_setpoint"],
    "attitude":            ["vehicle_attitude"],
    "attitude_setpoint":   ["vehicle_attitude_setpoint"],
    "torque_setpoint":     ["vehicle_torque_setpoint"],
    "rate_ctrl_status":    ["rate_ctrl_status"],
    "actuator_motors":     ["actuator_motors"],
    "actuator_controls":   ["actuator_controls_0", "actuator_controls"],
    "actuator_outputs":    ["actuator_outputs"],
    "sensor_combined":     ["sensor_combined"],
    "sensor_gyro":         ["sensor_gyro"],
    "sensor_gyro_fft":     ["sensor_gyro_fft"],
    "local_position":      ["vehicle_local_position"],
    "esc_status":          ["esc_status"],
    "vehicle_status":      ["vehicle_status"],
    "battery_status":      ["battery_status"],
    "autotune_status":     ["autotune_attitude_control_status"],
    # ── estimator / EV (external-vision) topics — VIO/EV tab ──────
    # Source-agnostic PX4 EV interface: any external odometry EKF2 fuses as EV
    # (ArUco/VPE, 3D LIO, 2D scan-match, camera VIO, mocap) lands here.
    "estimator_status":        ["estimator_status"],
    "estimator_status_flags":  ["estimator_status_flags"],
    "estimator_innovations":   ["estimator_innovations"],
    "estimator_innov_tr":      ["estimator_innovation_test_ratios"],
    "estimator_innov_var":     ["estimator_innovation_variances"],
    "estimator_ev_pos_bias":   ["estimator_ev_pos_bias"],
    "aid_src_ev_pos":          ["estimator_aid_src_ev_pos"],
    "aid_src_ev_yaw":          ["estimator_aid_src_ev_yaw"],
    "aid_src_ev_hgt":          ["estimator_aid_src_ev_hgt"],
    "aid_src_ev_vel":          ["estimator_aid_src_ev_vel"],
    "visual_odometry":         ["vehicle_visual_odometry"],
}

_PID_PARAM_NAMES = {
    "MC_ROLLRATE_K",  "MC_ROLLRATE_P",  "MC_ROLLRATE_I",  "MC_ROLLRATE_D",
    "MC_PITCHRATE_K", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D",
    "MC_YAWRATE_K",   "MC_YAWRATE_P",   "MC_YAWRATE_I",   "MC_YAWRATE_D",
    "MC_ROLL_P",      "MC_PITCH_P",     "MC_YAW_P",
    "MC_RR_INT_LIM",  "MC_PR_INT_LIM",  "MC_YR_INT_LIM",
    "IMU_GYRO_CUTOFF", "IMU_DGYRO_CUTOFF",
    # Real PX4 v1.15 gyro notches: two static (NF0/NF1) + dynamic ESC notch (DNF).
    "IMU_GYRO_NF0_FRQ",   "IMU_GYRO_NF0_BW",
    "IMU_GYRO_NF1_FRQ",   "IMU_GYRO_NF1_BW",
    "IMU_GYRO_DNF_EN",    "IMU_GYRO_DNF_MIN",
    "IMU_GYRO_DNF_BW",    "IMU_GYRO_DNF_HMC",
    "MC_ROLLRATE_FF", "MC_PITCHRATE_FF",
    "ATT_W_GYRO_BIAS",
}


# ── helpers ───────────────────────────────────────────────────────────────────

def _quat_to_euler(q_w: float, q_x: float, q_y: float, q_z: float):
    roll  = math.atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x*q_x + q_y*q_y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(q_w*q_y - q_z*q_x))))
    yaw   = math.atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y*q_y + q_z*q_z))
    return roll, pitch, yaw


def _resolve_field(data_dict: dict, candidates: List[str]) -> Optional[np.ndarray]:
    """Return the first candidate field that exists in data_dict."""
    for c in candidates:
        if c in data_dict:
            return np.asarray(data_dict[c], dtype=float)
    return None


def _find_topic(datasets: list, candidates: List[str], multi_id: int = 0) -> Optional[Any]:
    """Return first dataset whose name matches any candidate."""
    for name in candidates:
        for ds in datasets:
            if ds.name == name and ds.multi_id == multi_id:
                return ds
    return None


def _forward_fill(t_primary: np.ndarray, t_src: np.ndarray,
                  values: np.ndarray) -> np.ndarray:
    """
    Align values from (t_src, values) onto t_primary using forward-fill.
    Values before the first t_src sample are nan.
    """
    out = np.full(len(t_primary), np.nan)
    src_idx = 0
    n_src = len(t_src)
    for i, tp in enumerate(t_primary):
        # Advance src_idx while next source timestamp <= tp
        while src_idx + 1 < n_src and t_src[src_idx + 1] <= tp:
            src_idx += 1
        if t_src[src_idx] <= tp:
            out[i] = values[src_idx]
    return out


def _ff_multi(t_primary: np.ndarray, t_src: np.ndarray,
              cols: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """Forward-fill multiple columns from the same source to t_primary."""
    return {k: _forward_fill(t_primary, t_src, v) for k, v in cols.items()}


def _resolve_string_field(data: dict, base: str) -> Optional[List[str]]:
    """Reconstruct per-sample strings from a ULog string field.

    ULog encodes strings as char arrays exposed as ``base[0]``, ``base[1]``, …
    (int code per position). Returns one string per sample, or None if absent. A
    directly-exposed list of bytes/str (some loggers) is also handled.
    """
    if base in data:
        try:
            return [c.decode() if isinstance(c, (bytes, bytearray)) else str(c)
                    for c in data[base]]
        except Exception:
            return None
    cols = []
    idx = 0
    while f"{base}[{idx}]" in data:
        cols.append(np.asarray(data[f"{base}[{idx}]"]))
        idx += 1
    if not cols:
        return None
    n = min(len(c) for c in cols)
    out: List[str] = []
    for i in range(n):
        chars = []
        for c in cols:
            code = int(c[i])
            if code == 0:
                break
            chars.append(chr(code))
        out.append("".join(chars))
    return out


def _ffill_str(t_primary: np.ndarray, t_src: np.ndarray,
               values: List[str]) -> List[Optional[str]]:
    """Forward-fill (last-known) string values from (t_src, values) onto t_primary."""
    if len(t_src) == 0 or not values:
        return [None] * len(t_primary)
    idx = np.searchsorted(t_src, t_primary, side="right") - 1
    m = len(values)
    return [values[int(j)] if 0 <= int(j) < m else None for j in idx]


def _extract_excitation(datasets: list, t_us_primary: np.ndarray,
                        warnings: List[str]):
    """Extract the excitation reference from a ULog if present.

    ``exc_ref`` lives in a companion ROS 2 topic (``ExcitationSample``), NOT a PX4
    uORB topic, so a normal FMU ULog does not contain it — then this warns and
    returns empty maps, and the analyzer falls back to setpoint-based FRF. If a
    merged log DOES carry ``excitation_sample``, numeric (ref/baseline/loop) and
    string (axis/phase/run_tag) fields are forward-filled onto the primary axis.
    """
    ds = _find_topic(datasets, ["excitation_sample", "ExcitationSample"])
    if ds is None:
        warnings.append(
            "No excitation_sample in ULog — reference-based (unbiased) ID "
            "unavailable offline (companion topic; record a rosbag of "
            "/offboard_control/excitation/reference and merge). Using setpoint-based FRF.")
        return {}, {}
    t_src = np.asarray(ds.data["timestamp"], dtype=np.int64).astype(float)
    tp = t_us_primary.astype(float)
    num_cols: Dict[str, np.ndarray] = {}
    for out_name, cands in (("exc_ref", ["ref"]), ("exc_baseline", ["baseline"]),
                            ("exc_loop", ["loop"])):
        arr = _resolve_field(ds.data, cands)
        if arr is not None:
            num_cols[out_name] = _forward_fill(tp, t_src, arr)
    str_cols: Dict[str, List[Optional[str]]] = {}
    for out_name, base in (("exc_axis", "axis"), ("exc_phase", "phase"),
                           ("exc_run_tag", "run_tag")):
        vals = _resolve_string_field(ds.data, base)
        if vals is not None:
            str_cols[out_name] = _ffill_str(tp, t_src, vals)
    if not str_cols.get("exc_axis") or not str_cols.get("exc_phase"):
        warnings.append(
            "excitation_sample found but axis/phase strings unreadable — "
            "reference-based ID may not trigger (needs exc_axis + exc_phase).")
    return num_cols, str_cols


# ── main result types ─────────────────────────────────────────────────────────

@dataclass
class UlogParseResult:
    """All outputs of a single ULog parse."""
    from .data_buffer import TelemetryFrame, TopicStats
    from .input_source import SourceMetadata

    frames: List["TelemetryFrame"]
    metadata: "SourceMetadata"
    # High-rate raw arrays (for HiFi analysis) — None if topic missing
    raw_gyro_t: Optional[np.ndarray] = None      # µs timestamps
    raw_gyro_xyz: Optional[np.ndarray] = None    # (N,3) rad/s
    raw_gyro_rate_hz: float = 0.0
    # Native-rate closed-loop ID pair (for offline PX4 autotune).
    # These are NOT forward-filled onto the frame axis — each keeps its own
    # timestamp base so the torque signal stays full-rate (SDLOG_PROFILE=17)
    # instead of the 50 Hz frame grid.
    raw_torque_t: Optional[np.ndarray] = None    # µs timestamps
    raw_torque_xyz: Optional[np.ndarray] = None  # (N,3) normalised torque sp
    raw_torque_rate_hz: float = 0.0
    raw_angvel_t: Optional[np.ndarray] = None    # µs timestamps (filtered gyro)
    raw_angvel_xyz: Optional[np.ndarray] = None  # (N,3) rad/s
    raw_angvel_rate_hz: float = 0.0
    # ESC RPM (M motors × T samples) if available
    esc_rpm: Optional[np.ndarray] = None         # (T, M)
    esc_t: Optional[np.ndarray] = None           # µs
    # Autotune result (if in log)
    autotune_params: Optional[Dict[str, float]] = None
    # Param snapshot
    params_all: Dict[str, Any] = field(default_factory=dict)
    params_pid: Dict[str, float] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    # Estimator / EV raw series for the VIO/EV tab — None if no
    # estimator topics are present in the log.
    estimator_series: Optional["EstimatorSeries"] = None
    # Registry-driven per-source aid bundle for the EKF2 aid-sources tab
    # — EV/flow/rng/gps/baro/mag. None if no estimator topics.
    aid_source_bundle: Optional[Any] = None
    # PX4 native attitude-autotune status series — None if the log
    # carries no autotune_attitude_control_status topic. Holds the per-sample
    # RLS-identified ARX(2,2,1) coeffs + fitness and the per-axis converged model.
    autotune_series: Optional["AutotuneStatusSeries"] = None


# ── ULog adapter ──────────────────────────────────────────────────────────────

class UlogAdapter:
    """
    Converts a PX4 ULog file to TelemetryFrame list + SourceMetadata.

    Usage:
        adapter = UlogAdapter("/path/to/log.ulg")
        result = adapter.parse()
        frames = result.frames
        metadata = result.metadata
    """

    def __init__(self, path_or_bytes):
        _require_pyulog()
        self._path_or_bytes = path_or_bytes
        self._ulog: Optional[ULog] = None

    def _open(self) -> ULog:
        if self._ulog is not None:
            return self._ulog
        if isinstance(self._path_or_bytes, (str, os.PathLike)):
            self._ulog = ULog(str(self._path_or_bytes))
        else:
            # bytes → temp file
            with tempfile.NamedTemporaryFile(suffix=".ulg", delete=False) as f:
                f.write(self._path_or_bytes)
                tmp = f.name
            # try/finally so a parse failure in ULog(tmp) does not leak the temp file
            try:
                self._ulog = ULog(tmp)
            finally:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
        return self._ulog

    # ── public entry point ────────────────────────────────────────────────────

    def parse(self) -> UlogParseResult:
        """
        Thin wrapper: open the ULog file, extract parameters + firmware info
        (which require the ULog object itself), then delegate all dataset →
        frames/metadata conversion to the pure helper `_datasets_to_frames`.
        """
        ul = self._open()
        warnings: List[str] = []

        datasets = ul.data_list if hasattr(ul, "data_list") else []

        # ── Parameter extraction (from the ULog object, not the datasets) ─────
        params_all = dict(ul.initial_parameters) if ul.initial_parameters else {}
        # Apply changed_parameters at last timestamp (end-of-flight snapshot)
        for ts_us, name, val in (ul.changed_parameters or []):
            params_all[name] = val
        params_pid = {k: float(v) for k, v in params_all.items()
                      if k in _PID_PARAM_NAMES and isinstance(v, (int, float))}
        if not params_pid:
            warnings.append(
                "No PID parameters found in ULog — "
                "using direction-only recommendations. "
                "PX4 v1.10+ logs should contain initial_parameters."
            )

        # Firmware + hw info
        fw_ver = ""
        sys_id = ""
        if hasattr(ul, "msg_info_dict"):
            fw_ver = str(ul.msg_info_dict.get("ver_sw", ""))
            sys_id = str(ul.msg_info_dict.get("sys_hw", ""))

        fname_str = (str(self._path_or_bytes)
                     if isinstance(self._path_or_bytes, (str, os.PathLike))
                     else "<uploaded>")

        return _datasets_to_frames(
            datasets, warnings,
            params_all=params_all,
            params_pid=params_pid,
            fw_ver=fw_ver,
            sys_id=sys_id,
            source_name=os.path.basename(fname_str),
            ulog_path=fname_str,
        )


# ── pure dataset → frames/metadata conversion (testable without a file) ───────

def _datasets_to_frames(
    datasets: list,
    warnings: List[str],
    params_all: Optional[Dict[str, Any]] = None,
    params_pid: Optional[Dict[str, float]] = None,
    fw_ver: str = "",
    sys_id: str = "",
    source_name: str = "<uploaded>",
    ulog_path: str = "",
) -> "UlogParseResult":
    """
    Convert already-opened ULog datasets into a full ``UlogParseResult``.

    Contains all logic that follows opening the ULog file (primary-axis
    selection, forward-fill of secondary topics, quaternion→Euler, raw
    gyro / ESC / autotune extraction, TopicStats and capabilities), so it can
    be unit-tested with hand-built fake datasets and no real ``.ulg`` file.

    ``warnings`` is appended to in place.  Parameters and firmware info are
    extracted from the ULog object by ``parse()`` and passed in; unit tests
    may omit them.
    """
    from .data_buffer import TelemetryFrame, TopicStats
    from .input_source import SourceCapabilities, SourceMetadata

    params_all = params_all if params_all is not None else {}
    params_pid = params_pid if params_pid is not None else {}

    # ── 2. Primary time axis: vehicle_angular_velocity ───────────────────
    ang_ds = _find_topic(datasets, _TOPIC_CANDIDATES["angular_velocity"])
    if ang_ds is None:
        warnings.append("vehicle_angular_velocity not found — cannot build frames.")
        meta = SourceMetadata(
            source_type="ulog", source_name=source_name, duration_s=0.0,
            n_frames=0, topic_stats={}, capabilities=SourceCapabilities(),
            params=params_pid, params_all=params_all,
            firmware_version=fw_ver, hw_system_id=sys_id, ulog_path=ulog_path,
        )
        return UlogParseResult(frames=[], metadata=meta, warnings=warnings,
                               params_all=params_all, params_pid=params_pid)

    t_us_primary = np.asarray(ang_ds.data["timestamp"], dtype=np.int64)
    t0_us = int(t_us_primary[0])
    t_primary_s = (t_us_primary - t0_us) / 1e6   # seconds from log start
    N = len(t_primary_s)

    # Rate fields
    roll_rate  = _resolve_field(ang_ds.data, _RATE_FIELDS["roll_rate"])
    pitch_rate = _resolve_field(ang_ds.data, _RATE_FIELDS["pitch_rate"])
    yaw_rate   = _resolve_field(ang_ds.data, _RATE_FIELDS["yaw_rate"])

    # ── 3. Forward-fill all secondary topics ─────────────────────────────
    ff: Dict[str, np.ndarray] = {}

    def _ff_topic(key: str, field_map: Dict[str, List[str]],
                  multi_id: int = 0):
        ds = _find_topic(datasets, _TOPIC_CANDIDATES.get(key, [key]), multi_id)
        if ds is None:
            warnings.append(f"Topic '{key}' not found in log — field(s) will be NaN.")
            for fname in field_map:
                ff[fname] = np.full(N, np.nan)
            return
        t_src = np.asarray(ds.data["timestamp"], dtype=np.int64)
        cols = {}
        for out_name, cands in field_map.items():
            arr = _resolve_field(ds.data, cands)
            if arr is None:
                warnings.append(
                    f"  [{key}] field {cands[0]} not found. "
                    f"Available: {list(ds.data.keys())}"
                )
                arr = np.zeros(len(t_src))
            cols[out_name] = arr
        aligned = _ff_multi(t_us_primary.astype(float), t_src.astype(float), cols)
        ff.update(aligned)

    _ff_topic("rates_setpoint", _RATE_SP_FIELDS)
    _ff_topic("torque_setpoint", _TORQUE_FIELDS)
    _ff_topic("rate_ctrl_status", _RATE_CTRL_INTEG)
    _ff_topic("actuator_motors", _MOTOR_FIELDS)

    # Attitude (quaternion → Euler)
    att_ds = _find_topic(datasets, _TOPIC_CANDIDATES["attitude"])
    if att_ds:
        t_att = np.asarray(att_ds.data["timestamp"], dtype=np.int64)
        # NB: `_resolve_field(...) or default` is WRONG for ndarrays — bool() on a
        # multi-element array raises ValueError. Use explicit None checks instead.
        q_w = _resolve_field(att_ds.data, ["q[0]"])
        if q_w is None:
            q_w = np.ones(len(t_att))
        q_x = _resolve_field(att_ds.data, ["q[1]"])
        if q_x is None:
            q_x = np.zeros(len(t_att))
        q_y = _resolve_field(att_ds.data, ["q[2]"])
        if q_y is None:
            q_y = np.zeros(len(t_att))
        q_z = _resolve_field(att_ds.data, ["q[3]"])
        if q_z is None:
            q_z = np.zeros(len(t_att))
        roll_a  = np.array([_quat_to_euler(w, x, y, z)[0]
                             for w, x, y, z in zip(q_w, q_x, q_y, q_z)])
        pitch_a = np.array([_quat_to_euler(w, x, y, z)[1]
                             for w, x, y, z in zip(q_w, q_x, q_y, q_z)])
        yaw_a   = np.array([_quat_to_euler(w, x, y, z)[2]
                             for w, x, y, z in zip(q_w, q_x, q_y, q_z)])
        for fname, src_arr in [("roll", roll_a), ("pitch", pitch_a), ("yaw", yaw_a)]:
            ff[fname] = _forward_fill(
                t_us_primary.astype(float), t_att.astype(float), src_arr)
    else:
        warnings.append("vehicle_attitude not found — attitude fields will be NaN.")
        for fname in ("roll", "pitch", "yaw"):
            ff[fname] = np.full(N, np.nan)

    # Attitude setpoint
    att_sp_ds = _find_topic(datasets, _TOPIC_CANDIDATES["attitude_setpoint"])
    if att_sp_ds:
        t_sp = np.asarray(att_sp_ds.data["timestamp"], dtype=np.int64)
        q_w = _resolve_field(att_sp_ds.data, ["q_d[0]"])
        if q_w is None:
            q_w = np.ones(len(t_sp))
        q_x = _resolve_field(att_sp_ds.data, ["q_d[1]"])
        if q_x is None:
            q_x = np.zeros(len(t_sp))
        q_y = _resolve_field(att_sp_ds.data, ["q_d[2]"])
        if q_y is None:
            q_y = np.zeros(len(t_sp))
        q_z = _resolve_field(att_sp_ds.data, ["q_d[3]"])
        if q_z is None:
            q_z = np.zeros(len(t_sp))
        roll_asp  = np.array([_quat_to_euler(w, x, y, z)[0]
                               for w, x, y, z in zip(q_w, q_x, q_y, q_z)])
        pitch_asp = np.array([_quat_to_euler(w, x, y, z)[1]
                               for w, x, y, z in zip(q_w, q_x, q_y, q_z)])
        yaw_asp   = np.array([_quat_to_euler(w, x, y, z)[2]
                               for w, x, y, z in zip(q_w, q_x, q_y, q_z)])
        for fname, src_arr in [("roll_sp", roll_asp),
                                ("pitch_sp", pitch_asp),
                                ("yaw_sp", yaw_asp)]:
            ff[fname] = _forward_fill(
                t_us_primary.astype(float), t_sp.astype(float), src_arr)
    else:
        for fname in ("roll_sp", "pitch_sp", "yaw_sp"):
            ff[fname] = np.full(N, np.nan)

    # Vehicle status → nav_state
    status_ds = _find_topic(datasets, _TOPIC_CANDIDATES["vehicle_status"])
    if status_ds:
        t_st = np.asarray(status_ds.data["timestamp"], dtype=np.int64)
        ns_arr = _resolve_field(status_ds.data, ["nav_state", "nav_state_user_intention"])
        if ns_arr is not None:
            ff["nav_state"] = _forward_fill(
                t_us_primary.astype(float), t_st.astype(float), ns_arr)
        else:
            ff["nav_state"] = np.full(N, np.nan)
    else:
        ff["nav_state"] = np.full(N, np.nan)

    # Battery
    bat_ds = _find_topic(datasets, _TOPIC_CANDIDATES["battery_status"])
    if bat_ds:
        t_bat = np.asarray(bat_ds.data["timestamp"], dtype=np.int64)
        v_bat = _resolve_field(bat_ds.data, ["voltage_v", "voltage_filtered_v"])
        if v_bat is None:
            v_bat = np.zeros(len(t_bat))
        bat_rem = _resolve_field(bat_ds.data, ["remaining"])
        if bat_rem is None:
            bat_rem = np.zeros(len(t_bat))
        ff["v_bat"]         = _forward_fill(t_us_primary.astype(float),
                                             t_bat.astype(float), v_bat)
        ff["bat_remaining"] = _forward_fill(t_us_primary.astype(float),
                                             t_bat.astype(float), bat_rem)

    # Local position (eph/epv for indoor confounder)
    lp_ds = _find_topic(datasets, _TOPIC_CANDIDATES["local_position"])
    if lp_ds:
        t_lp = np.asarray(lp_ds.data["timestamp"], dtype=np.int64)
        for fname, cands in [("eph", ["eph"]), ("epv", ["epv"]),
                               ("px", ["x"]), ("py", ["y"]), ("pz", ["z"])]:
            arr = _resolve_field(lp_ds.data, cands)
            if arr is not None:
                ff[fname] = _forward_fill(t_us_primary.astype(float),
                                          t_lp.astype(float), arr)

    # ── 3b. Excitation reference — companion topic, usually absent ─
    exc_num, exc_str = _extract_excitation(datasets, t_us_primary, warnings)
    ff.update(exc_num)
    exc_axis_l    = exc_str.get("exc_axis")    or [None] * N
    exc_phase_l   = exc_str.get("exc_phase")   or [None] * N
    exc_run_tag_l = exc_str.get("exc_run_tag") or [None] * N

    # ── 4a. Native-rate closed-loop ID pair (torque_sp, angular_velocity) ─
    # The filtered angular velocity IS the primary axis, so it is already at
    # full rate. The torque setpoint is otherwise forward-filled onto the
    # frame grid; here we keep its OWN timestamps so the offline autotune
    # sees the true high-rate signal (SDLOG_PROFILE=17), not a
    # 50 Hz-quantised copy.
    raw_angvel_t: Optional[np.ndarray] = None
    raw_angvel_xyz: Optional[np.ndarray] = None
    raw_angvel_rate_hz = 0.0
    if roll_rate is not None and pitch_rate is not None and yaw_rate is not None:
        raw_angvel_t = t_us_primary.copy()
        raw_angvel_xyz = np.column_stack([roll_rate, pitch_rate, yaw_rate])
        dur_av = (int(raw_angvel_t[-1]) - int(raw_angvel_t[0])) / 1e6
        raw_angvel_rate_hz = len(raw_angvel_t) / max(1.0, dur_av)

    raw_torque_t: Optional[np.ndarray] = None
    raw_torque_xyz: Optional[np.ndarray] = None
    raw_torque_rate_hz = 0.0
    tq_ds = _find_topic(datasets, _TOPIC_CANDIDATES["torque_setpoint"])
    if tq_ds is not None:
        raw_torque_t = np.asarray(tq_ds.data["timestamp"], dtype=np.int64)
        tqx = _resolve_field(tq_ds.data, _TORQUE_FIELDS["torque_roll"])
        tqy = _resolve_field(tq_ds.data, _TORQUE_FIELDS["torque_pitch"])
        tqz = _resolve_field(tq_ds.data, _TORQUE_FIELDS["torque_yaw"])
        if tqx is not None and tqy is not None and tqz is not None:
            raw_torque_xyz = np.column_stack([tqx, tqy, tqz])
            dur_tq = (int(raw_torque_t[-1]) - int(raw_torque_t[0])) / 1e6
            raw_torque_rate_hz = len(raw_torque_t) / max(1.0, dur_tq)
        else:
            raw_torque_t = None

    # ── 4. Raw gyro (for vibration analysis / filter characterization) ────
    raw_gyro_t: Optional[np.ndarray] = None
    raw_gyro_xyz: Optional[np.ndarray] = None
    raw_gyro_rate_hz = 0.0

    sc_ds = _find_topic(datasets, _TOPIC_CANDIDATES["sensor_combined"])
    sg_ds = _find_topic(datasets, _TOPIC_CANDIDATES["sensor_gyro"])
    gyro_ds = sc_ds or sg_ds
    if gyro_ds:
        raw_gyro_t = np.asarray(gyro_ds.data["timestamp"], dtype=np.int64)
        gx = _resolve_field(gyro_ds.data, _GYRO_FIELDS["gx"])
        gy = _resolve_field(gyro_ds.data, _GYRO_FIELDS["gy"])
        gz = _resolve_field(gyro_ds.data, _GYRO_FIELDS["gz"])
        # sensor_combined may have gyro_rad[i]
        if gx is None:
            gx = _resolve_field(gyro_ds.data, ["gyro_rad[0]"])
        if gy is None:
            gy = _resolve_field(gyro_ds.data, ["gyro_rad[1]"])
        if gz is None:
            gz = _resolve_field(gyro_ds.data, ["gyro_rad[2]"])
        if gx is not None and gy is not None and gz is not None:
            raw_gyro_xyz = np.column_stack([gx, gy, gz])
            dur_raw = (int(raw_gyro_t[-1]) - int(raw_gyro_t[0])) / 1e6
            raw_gyro_rate_hz = len(raw_gyro_t) / max(1.0, dur_raw)

    # ── 5. ESC RPM ────────────────────────────────────────────────────────
    esc_ds = _find_topic(datasets, _TOPIC_CANDIDATES["esc_status"])
    esc_rpm: Optional[np.ndarray] = None
    esc_t: Optional[np.ndarray] = None
    if esc_ds:
        esc_t = np.asarray(esc_ds.data["timestamp"], dtype=np.int64)
        rpm_cols = []
        for i in range(8):
            col = _resolve_field(esc_ds.data, [f"esc[{i}].esc_rpm", f"rpm[{i}]"])
            if col is not None:
                rpm_cols.append(col)
            else:
                break
        if rpm_cols:
            esc_rpm = np.column_stack(rpm_cols)

    # ── 6. Autotune result ────────────────────────────────────────────────
    autotune_params: Optional[Dict[str, float]] = None
    at_ds = _find_topic(datasets, _TOPIC_CANDIDATES["autotune_status"])
    if at_ds:
        at_fields = ["roll_gain", "pitch_gain", "yaw_gain",
                     "roll_d_gain", "pitch_d_gain"]
        autotune_params = {}
        for fn in at_fields:
            arr = _resolve_field(at_ds.data, [fn])
            if arr is not None and len(arr) > 0:
                autotune_params[fn] = float(arr[-1])

    # ── 7. Build TelemetryFrame list ──────────────────────────────────────
    frames: List[TelemetryFrame] = []
    for i in range(N):
        def _g(name: str, default=None):
            arr = ff.get(name)
            if arr is None:
                return default
            v = arr[i]
            return None if np.isnan(v) else float(v)

        frames.append(TelemetryFrame(
            t=float(t_primary_s[i]),
            wall_t=float(t_primary_s[i]),
            px4_t=int(t_us_primary[i]),
            nav_state=int(_g("nav_state", 0) or 0),
            roll_rate=float(roll_rate[i]) if roll_rate is not None else None,
            pitch_rate=float(pitch_rate[i]) if pitch_rate is not None else None,
            yaw_rate=float(yaw_rate[i]) if yaw_rate is not None else None,
            roll_rate_sp=_g("roll_rate_sp"),
            pitch_rate_sp=_g("pitch_rate_sp"),
            yaw_rate_sp=_g("yaw_rate_sp"),
            roll=_g("roll"), pitch=_g("pitch"), yaw=_g("yaw"),
            roll_sp=_g("roll_sp"), pitch_sp=_g("pitch_sp"), yaw_sp=_g("yaw_sp"),
            torque_roll=_g("torque_roll"),
            torque_pitch=_g("torque_pitch"),
            torque_yaw=_g("torque_yaw"),
            int_roll=_g("int_roll"), int_pitch=_g("int_pitch"), int_yaw=_g("int_yaw"),
            m0=_g("m0"), m1=_g("m1"), m2=_g("m2"), m3=_g("m3"),
            v_bat=_g("v_bat"), bat_remaining=_g("bat_remaining"),
            px=_g("px"), py=_g("py"), pz=_g("pz"),
            eph=_g("eph"), epv=_g("epv"),
            exc_ref=_g("exc_ref"),
            exc_baseline=_g("exc_baseline"),
            exc_loop=(int(_g("exc_loop")) if _g("exc_loop") is not None else None),
            exc_axis=exc_axis_l[i],
            exc_phase=exc_phase_l[i],
            exc_run_tag=exc_run_tag_l[i],
        ))

    # ── 8. TopicStats from actual timestamp arrays ────────────────────────
    duration_s = float(t_primary_s[-1] - t_primary_s[0]) if N > 1 else 0.0
    topic_stats = _compute_topic_stats_from_datasets(
        datasets, _TOPIC_CANDIDATES, duration_s)

    # ── 9. SourceMetadata + capabilities ─────────────────────────────────
    prim_stats = topic_stats.get("vehicle_angular_velocity",
                                  TopicStats("vehicle_angular_velocity"))
    achieved_rate = prim_stats.achieved_rate_hz

    # ── PX4 native autotune status series ────────────────────────
    autotune_series = parse_autotune_series(datasets)

    # ── Estimator / EV series for the VIO/EV tab ─────────────────
    estimator_series = parse_estimator_series(datasets)
    # ── Registry-driven aid-source bundle for the EKF2 aid-sources tab:
    # EV/flow/rng/gps/baro/mag, one metric machine over all of them.
    try:
        from .aid_source_analyzer import parse_aid_source_series
        aid_source_bundle = parse_aid_source_series(datasets)
    except Exception:
        log.debug("aid-source bundle extraction failed", exc_info=True)
        aid_source_bundle = None
    vio_basic = bool(estimator_series and estimator_series.has_flags
                     and estimator_series.has_innov)
    vio_aidsrc = bool(estimator_series and (
        estimator_series.has_aidsrc_pos or estimator_series.has_aidsrc_yaw
        or estimator_series.has_aidsrc_hgt))
    # EV_DELAY estimate needs a high-rate EV source (aid_src full-rate, or a
    # vehicle_visual_odometry input topic present).  A bare 2 Hz innovations
    # stream is NOT enough — gate honestly.
    vio_delay_estimable = bool(estimator_series and (
        estimator_series.has_aidsrc_pos or estimator_series.has_vio_input))

    caps = SourceCapabilities(
        high_rate_available=achieved_rate >= 200.0,
        freq_domain_available=achieved_rate >= 50.0,
        vibration_available=raw_gyro_xyz is not None,
        raw_sensor_available=raw_gyro_xyz is not None,
        esc_available=esc_rpm is not None,
        actuator_data_available=bool(
            topic_stats.get("actuator_motors",
                            TopicStats("actuator_motors")).msg_count > 0),
        params_available=bool(params_pid),
        sysid_hifi_available=bool(params_pid) and achieved_rate >= 200.0,
        vio_basic_available=vio_basic,
        vio_aidsrc_available=vio_aidsrc,
        vio_delay_estimable=vio_delay_estimable,
    )

    metadata = SourceMetadata(
        source_type="ulog",
        source_name=source_name,
        duration_s=duration_s,
        n_frames=N,
        topic_stats=topic_stats,
        capabilities=caps,
        params=params_pid,
        params_all=params_all,
        firmware_version=fw_ver,
        hw_system_id=sys_id,
        ulog_path=ulog_path,
    )

    return UlogParseResult(
        frames=frames,
        metadata=metadata,
        raw_gyro_t=raw_gyro_t,
        raw_gyro_xyz=raw_gyro_xyz,
        raw_gyro_rate_hz=raw_gyro_rate_hz,
        raw_torque_t=raw_torque_t,
        raw_torque_xyz=raw_torque_xyz,
        raw_torque_rate_hz=raw_torque_rate_hz,
        raw_angvel_t=raw_angvel_t,
        raw_angvel_xyz=raw_angvel_xyz,
        raw_angvel_rate_hz=raw_angvel_rate_hz,
        esc_rpm=esc_rpm,
        esc_t=esc_t,
        autotune_params=autotune_params,
        params_all=params_all,
        params_pid=params_pid,
        warnings=warnings,
        estimator_series=estimator_series,
        aid_source_bundle=aid_source_bundle,
        autotune_series=autotune_series,
    )


# ── TopicStats from ULog datasets ────────────────────────────────────────────

def _compute_topic_stats_from_datasets(
    datasets: list,
    topic_candidates: Dict[str, List[str]],
    duration_s: float,
) -> Dict[str, "TopicStats"]:
    from .data_buffer import TopicStats
    stats: Dict[str, TopicStats] = {}
    for logical_name, candidates in topic_candidates.items():
        for ds_name in candidates:
            ds = next((d for d in datasets
                       if d.name == ds_name and d.multi_id == 0), None)
            if ds is None:
                continue
            t_us = np.asarray(ds.data["timestamp"], dtype=np.int64)
            n = len(t_us)
            if n < 2:
                stats[ds_name] = TopicStats(topic=ds_name, msg_count=n,
                                            duration_s=duration_s)
                break
            rate = n / max(1.0, duration_s)
            dts_ms = np.diff(t_us) / 1e3          # µs → ms
            dt_med = float(np.median(dts_ms))
            jitter  = float(np.median(np.abs(dts_ms - dt_med)))
            drop_pct = float(np.sum(dts_ms > 3.0 * dt_med) / len(dts_ms) * 100.0)
            stats[ds_name] = TopicStats(
                topic=ds_name,
                msg_count=n,
                achieved_rate_hz=round(rate, 1),
                jitter_ms=round(jitter / 1000.0, 2),   # µs → ms → ms
                drop_pct=round(drop_pct, 1),
                f_nyquist_hz=round(rate / 2.0, 1),
                duration_s=round(duration_s, 1),
            )
            break   # found first matching candidate
    return stats


# ── EstimatorSeries: raw EV / estimator time series ──────────────
#
# Sibling of `_datasets_to_frames`, but deliberately NOT forward-filled onto the
# vehicle_angular_velocity axis: estimator/EV topics have their own rates and
# semantics (flags are on-change, innovations 2 Hz, local_position 10 Hz).  Each
# series keeps its own timestamp.  All timestamps share the PX4 clock, so they
# are stored as seconds from a common t0 to make cross-series correlation
# (innovation vs. speed) possible downstream without alignment here.


@dataclass
class EstimatorSeries:
    """Raw estimator/EV series pulled from a ULog, each on its own time axis.

    Everything is optional: a default `SDLOG_PROFILE=1` real-board log carries
    the flags/innovations/status/local_position/ev_pos_bias groups but NOT the
    rich `estimator_aid_src_ev_*` or `vehicle_visual_odometry` (those need
    logger_topics.txt / replay / SITL).  Availability flags drive capability
    badges and gate the EV_DELAY estimate.
    """
    # estimator_status_flags (on-change) — named booleans
    t_flags: Optional[np.ndarray] = None
    cs_ev_pos: Optional[np.ndarray] = None
    cs_ev_yaw: Optional[np.ndarray] = None
    cs_ev_hgt: Optional[np.ndarray] = None
    cs_ev_vel: Optional[np.ndarray] = None
    cs_inertial_dead_reckoning: Optional[np.ndarray] = None
    reject_hor_pos: Optional[np.ndarray] = None
    reject_ver_pos: Optional[np.ndarray] = None
    reject_yaw: Optional[np.ndarray] = None

    # estimator_innovations (2 Hz) + variances + test_ratios
    t_innov: Optional[np.ndarray] = None
    ev_hpos: Optional[np.ndarray] = None          # (N,2)
    ev_vpos: Optional[np.ndarray] = None          # (N,)
    ev_hvel: Optional[np.ndarray] = None          # (N,2)
    ev_vvel: Optional[np.ndarray] = None          # (N,)
    heading: Optional[np.ndarray] = None          # (N,)  EV-yaw rides the shared heading field
    ev_hpos_tr: Optional[np.ndarray] = None
    ev_vpos_tr: Optional[np.ndarray] = None
    ev_hvel_tr: Optional[np.ndarray] = None
    ev_vvel_tr: Optional[np.ndarray] = None
    heading_tr: Optional[np.ndarray] = None
    ev_hpos_var: Optional[np.ndarray] = None
    ev_vpos_var: Optional[np.ndarray] = None

    # estimator_ev_pos_bias (2 Hz) — EKF estimate of EV position offset (NED)
    t_bias: Optional[np.ndarray] = None
    ev_pos_bias: Optional[np.ndarray] = None            # (N,3)
    ev_pos_bias_tr: Optional[np.ndarray] = None         # (N,3) innov_test_ratio (NEES proxy)

    # estimator_status (5 Hz)
    t_status: Optional[np.ndarray] = None
    pos_test_ratio: Optional[np.ndarray] = None
    hgt_test_ratio: Optional[np.ndarray] = None
    reset_count_pos_ne: Optional[np.ndarray] = None
    reset_count_pos_d: Optional[np.ndarray] = None
    solution_status_flags: Optional[np.ndarray] = None
    control_mode_flags: Optional[np.ndarray] = None

    # vehicle_local_position (10 Hz) — EKF output
    t_lpos: Optional[np.ndarray] = None
    lpos_xyz: Optional[np.ndarray] = None          # (N,3)
    lpos_vel: Optional[np.ndarray] = None          # (N,3)
    lpos_dead_reckoning: Optional[np.ndarray] = None
    lpos_eph: Optional[np.ndarray] = None
    lpos_epv: Optional[np.ndarray] = None

    # estimator_aid_src_ev_pos/yaw/hgt (optional, extended) — dicts of arrays
    aid_ev_pos: Optional[Dict[str, np.ndarray]] = None
    aid_ev_yaw: Optional[Dict[str, np.ndarray]] = None
    aid_ev_hgt: Optional[Dict[str, np.ndarray]] = None

    # vehicle_visual_odometry (optional) — EV input
    t_vio: Optional[np.ndarray] = None
    vio_pos: Optional[np.ndarray] = None           # (N,3)
    vio_q: Optional[np.ndarray] = None             # (N,4)
    vio_timestamp_sample: Optional[np.ndarray] = None   # µs
    vio_latency_ms: Optional[np.ndarray] = None         # timestamp − timestamp_sample
    vio_reset_counter: Optional[np.ndarray] = None
    vio_quality: Optional[np.ndarray] = None

    # capability flags
    has_flags: bool = False
    has_innov: bool = False
    has_bias: bool = False
    has_status: bool = False
    has_lpos: bool = False
    has_aidsrc_pos: bool = False
    has_aidsrc_yaw: bool = False
    has_aidsrc_hgt: bool = False
    has_vio_input: bool = False


_AID_SRC_FIELDS = [
    "observation", "observation_variance", "innovation",
    "innovation_variance", "test_ratio", "innovation_rejected", "fused",
]


def _ds_ts_us(ds) -> np.ndarray:
    return np.asarray(ds.data["timestamp"], dtype=np.int64)


def _vec_field(data: dict, base: str, dim: int) -> Optional[np.ndarray]:
    """Resolve an array field `base[0..dim-1]` → (N,dim) or None if absent."""
    cols = []
    for i in range(dim):
        arr = _resolve_field(data, [f"{base}[{i}]"])
        if arr is None:
            return None
        cols.append(arr)
    return np.column_stack(cols)


def parse_estimator_series(datasets: list) -> Optional[EstimatorSeries]:
    """Extract raw estimator/EV series from ULog datasets.

    Returns None if no estimator/EV topic is present at all.  Field names are
    resolved dynamically through candidate lists, matching the rest of the
    adapter.  Timestamps are converted to seconds from a shared t0 (the earliest
    first-timestamp across the present estimator/output topics).
    """
    tc = _TOPIC_CANDIDATES
    flags_ds  = _find_topic(datasets, tc["estimator_status_flags"])
    innov_ds  = _find_topic(datasets, tc["estimator_innovations"])
    tr_ds     = _find_topic(datasets, tc["estimator_innov_tr"])
    var_ds    = _find_topic(datasets, tc["estimator_innov_var"])
    bias_ds   = _find_topic(datasets, tc["estimator_ev_pos_bias"])
    status_ds = _find_topic(datasets, tc["estimator_status"])
    lpos_ds   = _find_topic(datasets, tc["local_position"])
    aidp_ds   = _find_topic(datasets, tc["aid_src_ev_pos"])
    aidy_ds   = _find_topic(datasets, tc["aid_src_ev_yaw"])
    aidh_ds   = _find_topic(datasets, tc["aid_src_ev_hgt"])
    vio_ds    = _find_topic(datasets, tc["visual_odometry"])

    present = [d for d in (flags_ds, innov_ds, tr_ds, var_ds, bias_ds,
                           status_ds, lpos_ds, aidp_ds, aidy_ds, aidh_ds, vio_ds)
               if d is not None]
    if not present:
        return None

    t0 = min(int(_ds_ts_us(d)[0]) for d in present if len(_ds_ts_us(d)) > 0)

    def _t(ds) -> np.ndarray:
        return (_ds_ts_us(ds).astype(np.float64) - t0) / 1e6

    s = EstimatorSeries()

    # ── estimator_status_flags ───────────────────────────────────────────────
    if flags_ds is not None:
        s.t_flags = _t(flags_ds)
        def _bool(name):
            arr = _resolve_field(flags_ds.data, [name])
            return None if arr is None else (np.asarray(arr) != 0)
        s.cs_ev_pos = _bool("cs_ev_pos")
        s.cs_ev_yaw = _bool("cs_ev_yaw")
        s.cs_ev_hgt = _bool("cs_ev_hgt")
        s.cs_ev_vel = _bool("cs_ev_vel")
        s.cs_inertial_dead_reckoning = _bool("cs_inertial_dead_reckoning")
        s.reject_hor_pos = _bool("reject_hor_pos")
        s.reject_ver_pos = _bool("reject_ver_pos")
        s.reject_yaw = _bool("reject_yaw")
        s.has_flags = s.cs_ev_pos is not None or s.cs_ev_yaw is not None

    # ── estimator_innovations (+ test_ratios / variances) ────────────────────
    if innov_ds is not None:
        s.t_innov = _t(innov_ds)
        s.ev_hpos = _vec_field(innov_ds.data, "ev_hpos", 2)
        s.ev_vpos = _resolve_field(innov_ds.data, ["ev_vpos"])
        s.ev_hvel = _vec_field(innov_ds.data, "ev_hvel", 2)
        s.ev_vvel = _resolve_field(innov_ds.data, ["ev_vvel"])
        s.heading = _resolve_field(innov_ds.data, ["heading"])
        s.has_innov = s.ev_hpos is not None or s.ev_vpos is not None
    if tr_ds is not None:
        s.ev_hpos_tr = _vec_field(tr_ds.data, "ev_hpos", 2)
        s.ev_vpos_tr = _resolve_field(tr_ds.data, ["ev_vpos"])
        s.ev_hvel_tr = _vec_field(tr_ds.data, "ev_hvel", 2)
        s.ev_vvel_tr = _resolve_field(tr_ds.data, ["ev_vvel"])
        s.heading_tr = _resolve_field(tr_ds.data, ["heading"])
    if var_ds is not None:
        s.ev_hpos_var = _vec_field(var_ds.data, "ev_hpos", 2)
        s.ev_vpos_var = _resolve_field(var_ds.data, ["ev_vpos"])

    # ── estimator_ev_pos_bias ────────────────────────────────────────────────
    if bias_ds is not None:
        s.t_bias = _t(bias_ds)
        s.ev_pos_bias = _vec_field(bias_ds.data, "bias", 3)
        s.ev_pos_bias_tr = _vec_field(bias_ds.data, "innov_test_ratio", 3)
        s.has_bias = s.ev_pos_bias is not None

    # ── estimator_status ─────────────────────────────────────────────────────
    if status_ds is not None:
        s.t_status = _t(status_ds)
        s.pos_test_ratio = _resolve_field(status_ds.data, ["pos_test_ratio"])
        s.hgt_test_ratio = _resolve_field(status_ds.data, ["hgt_test_ratio"])
        s.reset_count_pos_ne = _resolve_field(status_ds.data, ["reset_count_pos_ne"])
        s.reset_count_pos_d = _resolve_field(status_ds.data, ["reset_count_pos_d"])
        s.solution_status_flags = _resolve_field(status_ds.data, ["solution_status_flags"])
        s.control_mode_flags = _resolve_field(status_ds.data, ["control_mode_flags"])
        s.has_status = s.pos_test_ratio is not None

    # ── vehicle_local_position (EKF output; velocity for delay/bias checks) ───
    if lpos_ds is not None:
        s.t_lpos = _t(lpos_ds)
        n_lp = len(s.t_lpos)
        def _lp(name):
            arr = _resolve_field(lpos_ds.data, [name])
            return arr if arr is not None else np.full(n_lp, np.nan)
        s.lpos_xyz = np.column_stack([_lp("x"), _lp("y"), _lp("z")])
        vx = _resolve_field(lpos_ds.data, ["vx"])
        vy = _resolve_field(lpos_ds.data, ["vy"])
        vz = _resolve_field(lpos_ds.data, ["vz"])
        if vx is not None and vy is not None and vz is not None:
            s.lpos_vel = np.column_stack([vx, vy, vz])
        dr = _resolve_field(lpos_ds.data, ["dead_reckoning"])
        s.lpos_dead_reckoning = None if dr is None else (np.asarray(dr) != 0)
        s.lpos_eph = _resolve_field(lpos_ds.data, ["eph"])
        s.lpos_epv = _resolve_field(lpos_ds.data, ["epv"])
        s.has_lpos = True

    # ── estimator_aid_src_ev_* (optional, extended) ──────────────────────────
    def _aid(ds) -> Optional[Dict[str, np.ndarray]]:
        if ds is None:
            return None
        out: Dict[str, np.ndarray] = {"t": _t(ds)}
        found = False
        for fld in _AID_SRC_FIELDS:
            # vector fields (observation/innovation/...[i]) — try 3, then 2, then 1.
            # NB: never `ndarray or ndarray` (ambiguous-truth ValueError) — use
            # explicit None checks.
            vec = None
            for dim in (3, 2, 1):
                vec = _vec_field(ds.data, fld, dim)
                if vec is not None:
                    break
            if vec is not None:
                out[fld] = vec
                found = True
                continue
            scal = _resolve_field(ds.data, [fld])
            if scal is not None:
                out[fld] = np.asarray(scal)
                found = True
        return out if found else None

    s.aid_ev_pos = _aid(aidp_ds)
    s.aid_ev_yaw = _aid(aidy_ds)
    s.aid_ev_hgt = _aid(aidh_ds)
    s.has_aidsrc_pos = s.aid_ev_pos is not None
    s.has_aidsrc_yaw = s.aid_ev_yaw is not None
    s.has_aidsrc_hgt = s.aid_ev_hgt is not None

    # ── vehicle_visual_odometry (optional) — EV input ────────────────────────
    if vio_ds is not None:
        s.t_vio = _t(vio_ds)
        s.vio_pos = _vec_field(vio_ds.data, "position", 3)
        s.vio_q = _vec_field(vio_ds.data, "q", 4)
        tss = _resolve_field(vio_ds.data, ["timestamp_sample"])
        s.vio_timestamp_sample = None if tss is None else np.asarray(tss, dtype=np.float64)
        # Transport latency = timestamp − timestamp_sample (µs → ms), computed on
        # absolute timestamps before the t0 shift.
        if s.vio_timestamp_sample is not None:
            ts_abs = _ds_ts_us(vio_ds).astype(np.float64)
            s.vio_latency_ms = (ts_abs - s.vio_timestamp_sample) / 1e3
        s.vio_reset_counter = _resolve_field(vio_ds.data, ["reset_counter"])
        s.vio_quality = _resolve_field(vio_ds.data, ["quality"])
        s.has_vio_input = s.vio_pos is not None

    return s


# ── PX4 native autotune status series ────────────────────────────
#
# The onboard ``mc_autotune_attitude_control`` module (Bresciani, Auterion) runs
# a closed-loop RLS identification of an ARX(2,2,1) plant on ONE axis at a time
# and publishes ``autotune_attitude_control_status`` throughout.  Its state field
# says which axis is being excited; the ``coeff`` array carries the running RLS
# estimate ``[a1, a2, b0, b1, b2]``.  This is the SAME model structure the offline
# path (autotune_offline.py) identifies, so the two can be compared directly.
#
# Unlike the rate/torque topics, this status stream is NOT forward-filled onto the
# angular_velocity axis — it keeps its own timestamps.  Per-axis converged coeffs
# are the last sample recorded while the state machine sat in that axis's ID state.

# State-machine constants (mc_autotune_attitude_control, PX4 v1.15.4).  The
# per-axis identification runs in ROLL/PITCH/YAW; the surrounding pause/verify
# states carry stale coeffs and are excluded from convergence extraction.
AUTOTUNE_STATE = {
    "idle": 0, "init": 1,
    "roll": 2, "roll_pause": 3,
    "pitch": 4, "pitch_pause": 5,
    "yaw": 6, "yaw_pause": 7,
    "verification": 8, "apply": 9, "test": 10,
    "fail": 11, "complete": 12, "wait_for_disarm": 13,
}
_AUTOTUNE_AXIS_STATE = {"roll": 2, "pitch": 4, "yaw": 6}


@dataclass
class AutotuneArxModel:
    """Plant recovered from a converged autotune ARX(2,2,1) coefficient vector.

    Discrete transfer, matching the identification used both onboard and in
    autotune_offline.py (``y[k] = -a1 y[k-1] - a2 y[k-2] + b0 u[k-1] + b1 u[k-2]
    + b2 u[k-3]``):

        G(z) = (b0 z⁻¹ + b1 z⁻² + b2 z⁻³) / (1 + a1 z⁻¹ + a2 z⁻²)

    so ``num = [b0, b1, b2]`` (one-sample input delay), ``den = [1, a1, a2]``.
    A continuous-equivalent 2nd-order (wn, ζ) is derived from the denominator
    poles (s = ln z / dt) so the model can be laid beside the analyzer's
    2nd-order+delay fit; it is only meaningful when ``model_dt_s`` is known.
    """
    num: List[float]                 # [b0, b1, b2]
    den: List[float]                 # [1, a1, a2]
    model_dt_s: Optional[float]      # discrete sample time (s), None if unknown
    dc_gain: Optional[float] = None  # G(1) = Σb / Σden
    wn_hz: Optional[float] = None    # continuous-equivalent natural freq (Hz)
    zeta: Optional[float] = None     # continuous-equivalent damping ratio


@dataclass
class AutotuneAxisIdent:
    """Per-axis converged identification pulled from the status stream."""
    axis: str
    coeff: Optional[List[float]] = None       # [a1, a2, b0, b1, b2]
    coeff_var: Optional[List[float]] = None   # [5] variance of the RLS estimate
    fitness: Optional[float] = None           # PX4 model fitness at convergence
    model_dt_s: Optional[float] = None
    n_samples: int = 0                        # samples spent in this axis's ID state
    model: Optional[AutotuneArxModel] = None


@dataclass
class AutotuneStatusSeries:
    """Raw autotune_attitude_control_status time series + per-axis convergence.

    Every array shares the PX4 clock, stored as seconds from the stream's own t0.
    ``axes`` maps roll/pitch/yaw → converged identification (present only for the
    axes the state machine actually reached).
    """
    t: np.ndarray                              # (N,) seconds from t0
    state: Optional[np.ndarray] = None         # (N,) int state id
    coeff: Optional[np.ndarray] = None         # (N,5) [a1 a2 b0 b1 b2]
    coeff_var: Optional[np.ndarray] = None     # (N,5)
    fitness: Optional[np.ndarray] = None       # (N,)
    rate_sp: Optional[np.ndarray] = None       # (N,3) injected rate setpoint
    dt_model: Optional[np.ndarray] = None      # (N,) model sample time (s)
    axes: Dict[str, "AutotuneAxisIdent"] = field(default_factory=dict)
    has_coeff: bool = False
    has_fitness: bool = False


def reconstruct_arx_plant(
    coeff: List[float], model_dt: Optional[float]
) -> Optional[AutotuneArxModel]:
    """Rebuild the plant model from a converged ARX(2,2,1) coeff vector.

    ``coeff = [a1, a2, b0, b1, b2]``.  Returns None on a non-finite or empty
    vector.  The continuous-equivalent (wn, ζ) is filled only when ``model_dt``
    is a positive number, since the z→s pole map needs the sample time.
    """
    if coeff is None or len(coeff) < 5:
        return None
    c = [float(v) for v in coeff[:5]]
    if not all(math.isfinite(v) for v in c):
        return None
    a1, a2, b0, b1, b2 = c
    num = [b0, b1, b2]
    den = [1.0, a1, a2]

    dc_gain = None
    den_sum = 1.0 + a1 + a2
    if abs(den_sum) > 1e-9:
        dc_gain = (b0 + b1 + b2) / den_sum

    wn_hz = zeta = None
    if model_dt is not None and model_dt > 0:
        try:
            roots = np.roots([1.0, a1, a2]).astype(complex)
            if len(roots) == 2 and np.all(np.abs(roots) > 1e-12):
                s = np.log(roots) / float(model_dt)     # z → s poles
                prod = float(np.real(s[0] * s[1]))
                if prod > 0:
                    wn = math.sqrt(prod)
                    z = -float(np.real(s[0] + s[1])) / (2.0 * wn)
                    wn_hz = wn / (2.0 * math.pi)
                    zeta = z
        except (np.linalg.LinAlgError, ValueError, FloatingPointError):
            pass

    return AutotuneArxModel(
        num=[round(v, 6) for v in num],
        den=[round(v, 6) for v in den],
        model_dt_s=round(float(model_dt), 6) if model_dt else None,
        dc_gain=round(dc_gain, 6) if dc_gain is not None else None,
        wn_hz=round(wn_hz, 3) if wn_hz is not None else None,
        zeta=round(zeta, 4) if zeta is not None else None,
    )


def parse_autotune_series(datasets: list) -> Optional["AutotuneStatusSeries"]:
    """Extract the PX4 native-autotune status stream from ULog datasets.

    Returns None if the log has no ``autotune_attitude_control_status`` topic.
    Field names are resolved through candidate lists (firmware versions vary), so
    hand-built fake datasets in the tests work without a real ``.ulg``.  For each
    of roll/pitch/yaw the converged coefficients are taken from the LAST sample
    recorded while the state machine sat in that axis's identification state, and
    a plant model is reconstructed from them.
    """
    at_ds = _find_topic(datasets, _TOPIC_CANDIDATES["autotune_status"])
    if at_ds is None:
        return None

    data = at_ds.data
    t_us = _ds_ts_us(at_ds)
    if len(t_us) == 0:
        return None
    t0 = int(t_us[0])
    t_s = (t_us.astype(np.float64) - t0) / 1e6

    coeff = _vec_field(data, "coeff", 5)
    coeff_var = _vec_field(data, "coeff_var", 5)
    fitness = _resolve_field(data, ["fitness"])
    state = _resolve_field(data, ["state"])
    # Injected rate setpoint: named rate_sp[0..2] on real logs; fall back to a
    # scalar broadcast (some firmwares log a single u/rate_sp channel).
    rate_sp = _vec_field(data, "rate_sp", 3)
    if rate_sp is None:
        rsp1 = _resolve_field(data, ["rate_sp", "u", "u_filt"])
        if rsp1 is not None:
            rate_sp = np.column_stack([rsp1, rsp1, rsp1])
    # Model sample time (per-sample) — seconds. Onboard field is dt_model.
    dt_model = _resolve_field(data, ["dt_model", "model_dt"])

    series = AutotuneStatusSeries(
        t=t_s,
        state=None if state is None else np.asarray(state, dtype=float),
        coeff=coeff,
        coeff_var=coeff_var,
        fitness=None if fitness is None else np.asarray(fitness, dtype=float),
        rate_sp=rate_sp,
        dt_model=None if dt_model is None else np.asarray(dt_model, dtype=float),
        has_coeff=coeff is not None,
        has_fitness=fitness is not None,
    )

    # Per-axis converged identification: last sample inside each axis's ID state.
    if coeff is not None and state is not None:
        st = np.asarray(state, dtype=float)
        for axis, axis_state in _AUTOTUNE_AXIS_STATE.items():
            idx = np.where(st == axis_state)[0]
            if len(idx) == 0:
                continue
            last = int(idx[-1])
            c_last = [float(v) for v in coeff[last]]
            md = None
            if dt_model is not None and math.isfinite(float(dt_model[last])) \
                    and float(dt_model[last]) > 0:
                md = float(dt_model[last])
            ident = AutotuneAxisIdent(
                axis=axis,
                coeff=[round(v, 6) for v in c_last],
                coeff_var=([round(float(v), 6) for v in coeff_var[last]]
                           if coeff_var is not None else None),
                fitness=(round(float(fitness[last]), 4)
                         if fitness is not None else None),
                model_dt_s=round(md, 6) if md else None,
                n_samples=int(len(idx)),
                model=reconstruct_arx_plant(c_last, md),
            )
            series.axes[axis] = ident

    return series


# ── Convenience function for web_server ──────────────────────────────────────

def parse_ulog(path_or_bytes) -> "UlogParseResult":
    """Top-level entry point used by the web server."""
    return UlogAdapter(path_or_bytes).parse()


def is_available() -> bool:
    return _PYULOG_AVAILABLE
