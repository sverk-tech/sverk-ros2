"""Thread-safe telemetry buffer shared between the ROS node and FastAPI."""
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Any, Dict

import numpy as np


@dataclass
class TopicStats:
    """Measured delivery statistics for one DDS topic during a recording.

    All figures are measured over the *window of retained arrivals* (the
    per-topic arrival deque is capped at ``TelemetryNode._TS_MAXLEN``), not the
    full recording — see ``compute_stats_from_arrivals``.
    """
    topic: str
    msg_count: int = 0
    achieved_rate_hz: float = 0.0   # msgs/s over the retained-arrival window
    jitter_ms: float = 0.0          # MAD of inter-arrival dt (ms) as timestamped
                                    # on the companion — includes host scheduler
                                    # jitter, not only link jitter
    drop_pct: float = 0.0           # fraction of intervals with gap > 3 × median_dt
    f_nyquist_hz: float = 0.0       # achieved_rate / 2 — max trustworthy frequency
    duration_s: float = 0.0         # span of the retained-arrival window (s)
    total_duration_s: float = 0.0   # full recording wall-clock duration (s)


def compute_stats_from_arrivals(
    ts: List[float],
    topic: str,
    total_duration_s: float = 0.0,
) -> TopicStats:
    """Compute delivery statistics for one topic from its arrival timestamps.

    ``ts`` is the list of (possibly deque-truncated) wall-clock arrival times.
    Rate/jitter/drop are measured over the *actual* window of retained arrivals
    (``ts[-1] - ts[0]``), NOT over the full recording: the arrival deque is
    capped at ``_TS_MAXLEN``, so a long high-rate recording saturates the
    numerator while the full duration keeps growing — dividing ``n`` by the full
    duration then badly under-reports the true rate (e.g. a 5-min 250 Hz topic
    reads ~66 Hz). Windowing by the retained span fixes this.
    """
    n = len(ts)
    if n < 2:
        return TopicStats(topic=topic, msg_count=n,
                          total_duration_s=round(total_duration_s, 1))

    span = ts[-1] - ts[0]
    rate = (n - 1) / span if span > 0 else 0.0
    dts_ms = np.diff(ts) * 1000.0
    dt_med = float(np.median(dts_ms))
    jitter = float(np.median(np.abs(dts_ms - dt_med)))
    drop_pct = float(np.sum(dts_ms > 3.0 * dt_med) / len(dts_ms) * 100.0)

    return TopicStats(
        topic=topic,
        msg_count=n,
        achieved_rate_hz=round(rate, 1),
        jitter_ms=round(jitter, 2),
        drop_pct=round(drop_pct, 1),
        f_nyquist_hz=round(rate / 2.0, 1),
        duration_s=round(span, 1),
        total_duration_s=round(total_duration_s, 1),
    )


@dataclass
class TelemetryFrame:
    # ── timing ──────────────────────────────────────────────────────────────
    t: float = 0.0           # PX4 time in seconds from recording start (primary)
    wall_t: float = 0.0      # wall-clock seconds (debug / fallback)
    px4_t: int = 0           # PX4 timestamp µs since boot

    nav_state: Optional[int] = None

    # ── rate measurements (rad/s) ────────────────────────────────────────────
    roll_rate:  Optional[float] = None
    pitch_rate: Optional[float] = None
    yaw_rate:   Optional[float] = None

    # ── rate setpoints (rad/s)  [NEEDS dds_topics.yaml] ─────────────────────
    roll_rate_sp:  Optional[float] = None
    pitch_rate_sp: Optional[float] = None
    yaw_rate_sp:   Optional[float] = None

    # ── attitude (rad, NED) ──────────────────────────────────────────────────
    roll:  Optional[float] = None
    pitch: Optional[float] = None
    yaw:   Optional[float] = None

    # ── attitude setpoints (rad)  [NEEDS dds_topics.yaml] ───────────────────
    roll_sp:  Optional[float] = None
    pitch_sp: Optional[float] = None
    yaw_sp:   Optional[float] = None

    # ── rate-controller torque output (normalized)  — VehicleTorqueSetpoint ─
    torque_roll:  Optional[float] = None
    torque_pitch: Optional[float] = None
    torque_yaw:   Optional[float] = None

    # ── integrators — RateCtrlStatus ─────────────────────────────────────────
    int_roll:  Optional[float] = None
    int_pitch: Optional[float] = None
    int_yaw:   Optional[float] = None

    # ── motor commands [0-1]  — ActuatorMotors ───────────────────────────────
    m0: Optional[float] = None
    m1: Optional[float] = None
    m2: Optional[float] = None
    m3: Optional[float] = None

    # ── raw actuator outputs (µs PWM) — ActuatorOutputs ─────────────────────
    ao0: Optional[float] = None
    ao1: Optional[float] = None
    ao2: Optional[float] = None
    ao3: Optional[float] = None

    # ── VehicleControlMode flags ─────────────────────────────────────────────
    armed:                  Optional[bool] = None
    ctrl_attitude_enabled:  Optional[bool] = None
    ctrl_rates_enabled:     Optional[bool] = None
    ctrl_position_enabled:  Optional[bool] = None

    # ── battery ──────────────────────────────────────────────────────────────
    v_bat:         Optional[float] = None
    bat_remaining: Optional[float] = None

    # ── local position (m, NED) ──────────────────────────────────────────────
    px: Optional[float] = None
    py: Optional[float] = None
    pz: Optional[float] = None

    # ── position estimate quality (indoor/OF confounder detection) ───────────
    eph:         Optional[float] = None   # horizontal position std (m)
    epv:         Optional[float] = None   # vertical position std (m)
    pos_xy_valid: Optional[bool] = None

    # ── excitation reference  [NEEDS ~/excitation/reference topic] ─
    # The exact injected r(t) from the excitation node, on the SAME clock as the
    # outgoing setpoint, enabling unbiased reference-based ID (Ĝ=Φ_ry/Φ_ru).
    exc_ref:     Optional[float] = None   # r(t): rad/s (rate) or rad (attitude)
    exc_baseline: Optional[float] = None  # axis baseline the ref was overlaid on
    exc_axis:    Optional[str] = None     # "roll" | "pitch" | "yaw"
    exc_loop:    Optional[int] = None     # 0 attitude, 1 rate
    exc_phase:   Optional[str] = None     # settle_in|inject|settle_out|revert
    exc_run_tag: Optional[str] = None     # correlates with RunExcitation.result

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in asdict(self).items()}


class RecordingBuffer:
    """Central shared-memory store between ROS node (producer) and FastAPI (consumer)."""

    def __init__(self, maxlen: int = 200_000):
        self._lock = threading.Lock()
        self._frames: deque = deque(maxlen=maxlen)
        self._live: deque = deque(maxlen=300)   # legacy ring — see pop_live() (deprecated)
        # Per-client fan-out: each connected WebSocket owns its own ring, so two
        # open tabs no longer steal frames from each other (the old destructive
        # pop_live() emptied a single shared ring — first reader won the frame).
        self._live_consumers: List[deque] = []
        self._recording = False
        self._run_name = ""
        self._t_start: float = 0.0
        self.topic_status: Dict[str, str] = {}
        self.topic_stats: Dict[str, TopicStats] = {}
        self._node_ref = None       # set by TelemetryNode after creation
        self._duration_s: float = 0.0

    # ------------------------------------------------------------------ control

    def start_recording(self, run_name: str = "") -> str:
        with self._lock:
            self._frames.clear()
            self._recording = True
            self._run_name = run_name or f"run_{int(time.time())}"
            self._t_start = time.time()
            name = self._run_name
        # Reset per-topic arrival deques and PX4 time epoch so this recording's
        # stats and t-axis don't inherit arrivals from a previous one. Done
        # outside the buffer lock to keep buffer-lock → node-lock ordering.
        node = self._node_ref
        if node is not None and hasattr(node, "reset_stats"):
            node.reset_stats()
        return name

    def stop_recording(self) -> List[TelemetryFrame]:
        with self._lock:
            self._recording = False
            self._duration_s = max(0.0, time.time() - self._t_start)
            return list(self._frames)

    # ------------------------------------------------------------------ write

    def push(self, frame: TelemetryFrame):
        with self._lock:
            self._live.append(frame)              # legacy ring (pop_live compat)
            for dq in self._live_consumers:       # fan-out to every WS client
                dq.append(frame)
            if self._recording:
                self._frames.append(frame)

    # -------------------------------------------------------- live fan-out

    def attach_live_consumer(self, maxlen: int = 300) -> deque:
        """Register a fresh per-client live ring and return it.

        Each WebSocket handler attaches once on connect and detaches in its
        ``finally``. ``push()`` appends every frame to all registered rings, so
        clients no longer compete for a single shared ring.
        """
        dq: deque = deque(maxlen=maxlen)
        with self._lock:
            self._live_consumers.append(dq)
        return dq

    def detach_live_consumer(self, dq: deque) -> None:
        """Unregister a per-client live ring (idempotent)."""
        with self._lock:
            try:
                self._live_consumers.remove(dq)
            except ValueError:
                pass

    def drain_live_consumer(self, dq: deque) -> List[TelemetryFrame]:
        """Atomically return and clear a per-client ring's pending frames."""
        with self._lock:
            out = list(dq)
            dq.clear()
            return out

    # ------------------------------------------------------------------ read

    def pop_live(self) -> List[TelemetryFrame]:
        """Deprecated: destructive read of the shared legacy ring.

        Kept only for backward compatibility. New code must use
        ``attach_live_consumer()`` / ``drain_live_consumer()`` — pop_live()
        steals frames when more than one reader is active.
        """
        with self._lock:
            out = list(self._live)
            self._live.clear()
            return out

    def snapshot(self) -> List[TelemetryFrame]:
        """Return copy of recorded frames (non-destructive)."""
        with self._lock:
            return list(self._frames)

    # ------------------------------------------------------------------ props

    @property
    def recording(self) -> bool:
        return self._recording

    @property
    def run_name(self) -> str:
        return self._run_name

    @property
    def frame_count(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def elapsed(self) -> float:
        if not self._recording:
            return 0.0
        return time.time() - self._t_start
