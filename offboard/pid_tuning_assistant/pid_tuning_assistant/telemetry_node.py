"""
ROS 2 Humble telemetry node — subscribes to PX4 uXRCE-DDS topics.

READ-ONLY: does not publish, arm, or modify any drone parameters.

Key design choices
------------------
* Frame timestamps use PX4 ``timestamp`` (µs since boot), not wall-clock.
  Wall-clock is stored separately for debug only.
* Frames are pushed on every VehicleAngularVelocity message (the highest-
  frequency rate-loop signal).  Other callbacks only update the shared _cur
  state.  This gives true PX4-timestamp-driven frames rather than a fixed
  50 Hz wall-clock timer.
* Per-topic arrival timestamps are tracked in ``_topic_ts`` deques so that
  achieved rate, jitter, and drop% can be measured at recording stop.
* Topics that require dds_topics.yaml additions are marked [NEEDS DDS].
  The node degrades gracefully (warning, not crash) when topics are absent.
"""
import copy
import math
import time
import threading
import importlib
import logging
from collections import deque
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy,
)

from .data_buffer import (
    RecordingBuffer, TelemetryFrame, TopicStats, compute_stats_from_arrivals,
)
from .vio_live import VioLiveMonitor, VioLiveState

log = logging.getLogger(__name__)

# ── nav_state → human name (PX4 v1.15) ──────────────────────────────────────
NAV_STATE_MAP: dict = {
    0:  "MANUAL",
    1:  "ALTCTL",
    2:  "POSCTL",
    3:  "AUTO_MISSION",
    4:  "AUTO_LOITER",
    5:  "AUTO_RTL",
    6:  "AUTO_RCRECOVER",
    7:  "AUTO_RTGS",
    8:  "AUTO_LANDENGFAIL",
    9:  "AUTO_LANDGPSFAIL",
    10: "ACRO",
    12: "DESCEND",
    13: "TERMINATION",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
    19: "AUTO_FOLLOW_TARGET",
    20: "AUTO_PRECLAND",
    21: "ORBIT",
    22: "AUTO_VTOL_TAKEOFF",
}


def nav_state_name(s: int) -> str:
    return NAV_STATE_MAP.get(s, f"STATE_{s}")


QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _import_msg(class_name: str):
    try:
        mod = importlib.import_module("px4_msgs.msg")
        return getattr(mod, class_name)
    except Exception:
        return None


def _quat_to_euler(q_wxyz) -> tuple:
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
    roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class TelemetryNode(Node):
    """
    Aggregates PX4 telemetry from multiple DDS topics into TelemetryFrame objects.

    Push strategy
    -------------
    VehicleAngularVelocity is the primary trigger: every callback from this
    topic produces one frame snapshot.  Other callbacks only update _cur in
    place.  This means frame timestamps are derived from the PX4 clock
    embedded in VehicleAngularVelocity.timestamp (µs since boot).
    """

    TOPIC_MAP = {
        # class_name                   topic path                           callback
        "VehicleAngularVelocity":  ("/fmu/out/vehicle_angular_velocity",   "_cb_ang_vel"),
        "VehicleAttitude":         ("/fmu/out/vehicle_attitude",           "_cb_attitude"),
        "VehicleStatus":           ("/fmu/out/vehicle_status",             "_cb_status"),
        "BatteryStatus":           ("/fmu/out/battery_status",             "_cb_battery"),
        "VehicleLocalPosition":    ("/fmu/out/vehicle_local_position",     "_cb_local_pos"),
        "ActuatorMotors":          ("/fmu/out/actuator_motors",            "_cb_motors"),
        "ActuatorOutputs":         ("/fmu/out/actuator_outputs",           "_cb_actuator_outputs"),
        "VehicleControlMode":      ("/fmu/out/vehicle_control_mode",       "_cb_control_mode"),
        # --- NEEDS dds_topics.yaml additions ---
        "VehicleRatesSetpoint":    ("/fmu/out/vehicle_rates_setpoint",     "_cb_rates_sp"),
        "VehicleAttitudeSetpoint": ("/fmu/out/vehicle_attitude_setpoint",  "_cb_att_sp"),
        "RateCtrlStatus":          ("/fmu/out/rate_ctrl_status",           "_cb_rate_ctrl"),
        "VehicleTorqueSetpoint":   ("/fmu/out/vehicle_torque_setpoint",    "_cb_torque"),
    }

    DDS_REQUIRED = {
        "VehicleRatesSetpoint", "VehicleAttitudeSetpoint",
        "RateCtrlStatus", "VehicleTorqueSetpoint",
    }

    # ── VIO / EV live-monitor topics ──────────────────────────────
    # OPTIONAL group, deliberately NOT in DDS_REQUIRED: absence must not crash the
    # node, only disable the live VIO panel.  These feed the separate
    # VioLiveMonitor (not the rate-loop TelemetryFrame), which has its own
    # semantics and cadence.  All are the VehicleOdometry msg type except the
    # flags topic, so the class name is stored explicitly (the map *label* — used
    # for topic_status — differs from the ROS msg class for the two EV inputs).
    #   label:                    (msg_class,              topic,                                source)
    VIO_TOPIC_MAP = {
        "EstimatorStatusFlags":   ("EstimatorStatusFlags", "/fmu/out/estimator_status_flags",   None),
        "VehicleOdometry":        ("VehicleOdometry",      "/fmu/out/vehicle_odometry",          None),
        "VehicleVisualOdometry":  ("VehicleOdometry",      "/fmu/in/vehicle_visual_odometry",    "visual"),
        "VehicleMocapOdometry":   ("VehicleOdometry",      "/fmu/in/vehicle_mocap_odometry",     "mocap"),
    }

    # Maximum per-topic arrival timestamps to keep in memory
    _TS_MAXLEN = 20_000

    def __init__(self, buffer: RecordingBuffer, vio_expected_rate_hz: float = 30.0):
        super().__init__("pid_tuning_assistant")
        self.buf = buffer
        self._lock = threading.Lock()
        self._cur = TelemetryFrame()

        # Live VIO/EV watchdog — fed by the optional VIO_TOPIC_MAP
        # callbacks, read as a non-destructive snapshot by the web layer.
        self.vio_expected_rate_hz = float(vio_expected_rate_hz)
        self._vio = VioLiveMonitor()

        # PX4 time reference: first received timestamp establishes epoch
        self._t0_px4_us: int = 0
        self._t0_wall: float = 0.0

        # Per-topic arrival timestamps for rate/jitter measurement
        self._topic_ts: Dict[str, deque] = {
            k: deque(maxlen=self._TS_MAXLEN) for k in self.TOPIC_MAP
        }
        # Track recording start/stop wall-clock for TopicStats duration
        self._rec_start_wall: float = 0.0

        buffer._node_ref = self
        self._setup_subscriptions()
        self._setup_vio_subscriptions()
        self._setup_excitation_subscription()
        self.get_logger().info("PID Tuning Assistant node ready (READ-ONLY)")

    # ── subscription setup ────────────────────────────────────────────────────

    def _setup_subscriptions(self):
        for cls_name, (topic, cb_name) in self.TOPIC_MAP.items():
            MsgCls = _import_msg(cls_name)
            cb = getattr(self, cb_name, None)
            if MsgCls is None or cb is None:
                status = "import_error"
                if cls_name in self.DDS_REQUIRED:
                    self.get_logger().warn(
                        f"[NEEDS DDS] {cls_name} unavailable. "
                        f"Add {topic} to dds_topics.yaml and rebuild firmware."
                    )
                else:
                    self.get_logger().warn(f"{cls_name} import failed — skipping.")
            else:
                try:
                    self.create_subscription(MsgCls, topic, cb, QOS_SENSOR)
                    status = "subscribed"
                    self.get_logger().info(f"  OK {topic}")
                except Exception as e:
                    status = f"error:{e}"
                    self.get_logger().error(f"  ERR {topic}: {e}")
            self.buf.topic_status[cls_name] = status

    def _setup_vio_subscriptions(self):
        """Subscribe to the optional VIO/EV live-monitor topics.

        Graceful degrade: a missing message class or an absent topic (e.g. the
        mocap input, which is often not in dds_topics.yaml) only warns and
        leaves that part of the live panel showing "unavailable" — it never
        raises.  All readers are BEST_EFFORT (QOS_SENSOR): required so a
        best-effort writer on /fmu/in/vehicle_visual_odometry is actually read
        (a reliable reader would drop it).
        """
        for label, (cls_name, topic, source) in self.VIO_TOPIC_MAP.items():
            MsgCls = _import_msg(cls_name)
            if MsgCls is None:
                self.buf.topic_status[label] = "import_error"
                self.get_logger().warn(
                    f"(VIO) {label}: {cls_name} unavailable — live VIO panel "
                    f"degraded for {topic}.")
                continue
            if source is None:
                cb = (self._cb_estimator_flags if cls_name == "EstimatorStatusFlags"
                      else self._cb_vehicle_odom)
            else:
                cb = (lambda m, s=source: self._cb_ev_input(m, s))
            try:
                self.create_subscription(MsgCls, topic, cb, QOS_SENSOR)
                self.buf.topic_status[label] = "subscribed"
                self.get_logger().info(f"  OK (VIO) {topic}")
            except Exception as e:
                self.buf.topic_status[label] = f"error:{e}"
                self.get_logger().error(f"  ERR (VIO) {topic}: {e}")

    def _setup_excitation_subscription(self):
        """Subscribe to the excitation node's reference topic.

        Optional and graceful, exactly like the rate-loop DDS topics: if
        offboard_interfaces or the topic is absent (excitation node not running or
        not built), the reference-based unbiased-ID path is simply unavailable and
        the analyzer falls back to the setpoint-based (biased) FRF. The publisher
        is RELIABLE, so we read RELIABLE to avoid dropping ID samples.
        """
        topic = "/offboard_control/excitation/reference"
        try:
            mod = importlib.import_module("offboard_interfaces.msg")
            MsgCls = getattr(mod, "ExcitationSample", None)
        except Exception:
            MsgCls = None
        if MsgCls is None:
            self.buf.topic_status["ExcitationSample"] = "import_error"
            self.get_logger().info(
                "(excitation) ExcitationSample unavailable — reference-based ID "
                "disabled; setpoint-based FRF still works.")
            return
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=200,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        try:
            self.create_subscription(MsgCls, topic, self._cb_excitation, qos)
            self.buf.topic_status["ExcitationSample"] = "subscribed"
            self.get_logger().info(f"  OK (excitation) {topic}")
        except Exception as e:
            self.buf.topic_status["ExcitationSample"] = f"error:{e}"
            self.get_logger().error(f"  ERR (excitation) {topic}: {e}")

    def _cb_excitation(self, msg):
        """Forward-fill the injected reference r(t) into the current frame.

        Published every excitation phase (ref==0 outside inject), so exc_phase in
        the frame always tracks the true phase — the analyzer masks r to
        exc_phase=='inject' per axis when forming the unbiased FRF.
        """
        with self._lock:
            self._cur.exc_ref      = float(msg.ref)
            self._cur.exc_baseline = float(msg.baseline)
            self._cur.exc_axis     = str(msg.axis)
            self._cur.exc_loop     = int(msg.loop)
            self._cur.exc_phase    = str(msg.phase)
            self._cur.exc_run_tag  = str(msg.run_tag)

    def vio_snapshot(self, expected_rate_hz: float = None) -> VioLiveState:
        """Non-destructive live VIO/EV snapshot for the web layer."""
        rate = self.vio_expected_rate_hz if expected_rate_hz is None else float(expected_rate_hz)
        with self._lock:
            in_air = bool(self._cur.armed)
        return self._vio.snapshot(now=time.time(), expected_rate_hz=rate, in_air=in_air)

    # ── internal helpers ──────────────────────────────────────────────────────

    def _record_arrival(self, topic_key: str):
        """Track wall-clock arrival time for rate estimation."""
        self._topic_ts[topic_key].append(time.time())

    def _push_frame(self):
        """Snapshot _cur and push to buffer (called from _cb_ang_vel)."""
        with self._lock:
            frame = copy.copy(self._cur)
        self.buf.push(frame)

    def reset_stats(self):
        """Clear all per-topic arrival deques and the PX4 time epoch.

        Called at the start of each recording (via RecordingBuffer._node_ref)
        so a new recording's statistics and t-axis don't include arrivals from
        a previous one. The epoch sentinel stays 0 (NOT None) to remain
        consistent with the ``== 0`` guards in _cb_ang_vel and _px4_t_to_sec —
        None would raise TypeError there and crash the callback.
        """
        with self._lock:
            for dq in self._topic_ts.values():
                dq.clear()
            self._t0_px4_us = 0
            self._t0_wall = 0.0

    def _px4_t_to_sec(self, px4_us: int) -> float:
        """Convert PX4 µs timestamp to seconds from recording epoch."""
        if self._t0_px4_us == 0:
            return 0.0
        return (px4_us - self._t0_px4_us) / 1e6

    # ── message callbacks ─────────────────────────────────────────────────────

    def _cb_ang_vel(self, msg):
        """Primary trigger — every message produces one pushed frame."""
        now_wall = time.time()
        self._record_arrival("VehicleAngularVelocity")
        px4_us = int(msg.timestamp)

        with self._lock:
            if self._t0_px4_us == 0 and px4_us > 0:
                self._t0_px4_us = px4_us
                self._t0_wall = now_wall

            self._cur.px4_t     = px4_us
            self._cur.t         = self._px4_t_to_sec(px4_us)
            self._cur.wall_t    = now_wall
            self._cur.roll_rate  = float(msg.xyz[0])
            self._cur.pitch_rate = float(msg.xyz[1])
            self._cur.yaw_rate   = float(msg.xyz[2])

        self.buf.topic_status["VehicleAngularVelocity"] = "active"
        self._push_frame()

    def _cb_rates_sp(self, msg):
        self._record_arrival("VehicleRatesSetpoint")
        with self._lock:
            self._cur.roll_rate_sp  = float(msg.roll)
            self._cur.pitch_rate_sp = float(msg.pitch)
            self._cur.yaw_rate_sp   = float(msg.yaw)
        self.buf.topic_status["VehicleRatesSetpoint"] = "active"

    def _cb_attitude(self, msg):
        self._record_arrival("VehicleAttitude")
        r, p, y = _quat_to_euler(msg.q)
        with self._lock:
            self._cur.roll  = r
            self._cur.pitch = p
            self._cur.yaw   = y
        self.buf.topic_status["VehicleAttitude"] = "active"

    def _cb_att_sp(self, msg):
        self._record_arrival("VehicleAttitudeSetpoint")
        if hasattr(msg, "q_d") and msg.q_d is not None:
            r, p, y = _quat_to_euler(msg.q_d)
            with self._lock:
                self._cur.roll_sp  = r
                self._cur.pitch_sp = p
                self._cur.yaw_sp   = y
            self.buf.topic_status["VehicleAttitudeSetpoint"] = "active"

    def _cb_rate_ctrl(self, msg):
        self._record_arrival("RateCtrlStatus")
        with self._lock:
            if hasattr(msg, "rollspeed_integ"):
                self._cur.int_roll  = float(msg.rollspeed_integ)
                self._cur.int_pitch = float(msg.pitchspeed_integ)
                self._cur.int_yaw   = float(msg.yawspeed_integ)
        self.buf.topic_status["RateCtrlStatus"] = "active"

    def _cb_torque(self, msg):
        self._record_arrival("VehicleTorqueSetpoint")
        if hasattr(msg, "xyz") and len(msg.xyz) >= 3:
            with self._lock:
                self._cur.torque_roll  = float(msg.xyz[0])
                self._cur.torque_pitch = float(msg.xyz[1])
                self._cur.torque_yaw   = float(msg.xyz[2])
            self.buf.topic_status["VehicleTorqueSetpoint"] = "active"

    def _cb_motors(self, msg):
        self._record_arrival("ActuatorMotors")
        if hasattr(msg, "control") and len(msg.control) >= 4:
            with self._lock:
                self._cur.m0 = float(msg.control[0])
                self._cur.m1 = float(msg.control[1])
                self._cur.m2 = float(msg.control[2])
                self._cur.m3 = float(msg.control[3])
            self.buf.topic_status["ActuatorMotors"] = "active"

    def _cb_status(self, msg):
        self._record_arrival("VehicleStatus")
        with self._lock:
            self._cur.nav_state = int(msg.nav_state)
        self.buf.topic_status["VehicleStatus"] = "active"

    def _cb_local_pos(self, msg):
        self._record_arrival("VehicleLocalPosition")
        with self._lock:
            self._cur.px = float(msg.x)
            self._cur.py = float(msg.y)
            self._cur.pz = float(msg.z)
            self._cur.eph = float(getattr(msg, "eph", 0.0) or 0.0)
            self._cur.epv = float(getattr(msg, "epv", 0.0) or 0.0)
            self._cur.pos_xy_valid = bool(getattr(msg, "xy_valid", False))
        self.buf.topic_status["VehicleLocalPosition"] = "active"

    def _cb_actuator_outputs(self, msg):
        self._record_arrival("ActuatorOutputs")
        if hasattr(msg, "output") and len(msg.output) >= 4:
            with self._lock:
                self._cur.ao0 = float(msg.output[0])
                self._cur.ao1 = float(msg.output[1])
                self._cur.ao2 = float(msg.output[2])
                self._cur.ao3 = float(msg.output[3])
        self.buf.topic_status["ActuatorOutputs"] = "active"

    def _cb_control_mode(self, msg):
        self._record_arrival("VehicleControlMode")
        with self._lock:
            self._cur.armed                 = bool(getattr(msg, "flag_armed", False))
            self._cur.ctrl_attitude_enabled = bool(getattr(msg, "flag_control_attitude_enabled", False))
            self._cur.ctrl_rates_enabled    = bool(getattr(msg, "flag_control_rates_enabled", False))
            self._cur.ctrl_position_enabled = bool(getattr(msg, "flag_control_position_enabled", False))
        self.buf.topic_status["VehicleControlMode"] = "active"

    def _cb_battery(self, msg):
        self._record_arrival("BatteryStatus")
        with self._lock:
            self._cur.v_bat        = float(msg.voltage_v)
            self._cur.bat_remaining = float(msg.remaining)
        self.buf.topic_status["BatteryStatus"] = "active"

    # ── VIO / EV live-monitor callbacks ───────────────────────────
    # These feed VioLiveMonitor only — never _cur / the rate-loop frames.

    def _cb_estimator_flags(self, msg):
        self.buf.topic_status["EstimatorStatusFlags"] = "active"
        self._vio.update_flags(
            cs_ev_pos=bool(getattr(msg, "cs_ev_pos", False)),
            cs_ev_yaw=bool(getattr(msg, "cs_ev_yaw", False)),
            cs_ev_hgt=bool(getattr(msg, "cs_ev_hgt", False)),
            cs_ev_vel=bool(getattr(msg, "cs_ev_vel", False)),
            cs_inertial_dead_reckoning=bool(getattr(msg, "cs_inertial_dead_reckoning", False)),
            reject_hor_pos=bool(getattr(msg, "reject_hor_pos", False)),
            reject_ver_pos=bool(getattr(msg, "reject_ver_pos", False)),
            reject_yaw=bool(getattr(msg, "reject_yaw", False)),
        )

    def _cb_vehicle_odom(self, msg):
        self.buf.topic_status["VehicleOdometry"] = "active"
        pos = getattr(msg, "position", None)
        if pos is not None and len(pos) >= 3:
            self._vio.update_ekf_output(position=[float(pos[0]), float(pos[1]), float(pos[2])])

    def _cb_ev_input(self, msg, source: str):
        now = time.time()
        label = "VehicleMocapOdometry" if source == "mocap" else "VehicleVisualOdometry"
        self.buf.topic_status[label] = "active"
        pos = getattr(msg, "position", None)
        q = getattr(msg, "q", None)
        self._vio.update_ev_input(
            now=now, source=source,
            timestamp_sample_us=int(getattr(msg, "timestamp_sample", 0) or 0),
            reset_counter=int(getattr(msg, "reset_counter", 0) or 0),
            quality=int(getattr(msg, "quality", 0) or 0),
            position=[float(x) for x in pos[:3]] if pos is not None else None,
            q=[float(x) for x in q[:4]] if q is not None else None,
        )

    # ── per-topic statistics ──────────────────────────────────────────────────

    def compute_topic_stats(self, duration_s: float) -> Dict[str, TopicStats]:
        """
        Call after stop_recording().  Computes achieved rate, jitter, and
        drop% for every subscribed topic from the recorded arrival timestamps.

        Rate/jitter/drop are measured over the retained-arrival window (see
        ``compute_stats_from_arrivals``); ``duration_s`` is the full recording
        duration, kept only in ``TopicStats.total_duration_s``.
        """
        stats: Dict[str, TopicStats] = {}
        for topic_key, ts_deque in self._topic_ts.items():
            stats[topic_key] = compute_stats_from_arrivals(
                list(ts_deque), topic_key, total_duration_s=duration_s)
        return stats


# ── thread entry point ────────────────────────────────────────────────────────

def spin_node(buffer: RecordingBuffer, stop_event: threading.Event,
              vio_expected_rate_hz: float = 30.0):
    """
    Run TelemetryNode in a background thread.
    Called from main.py; exits cleanly when stop_event is set.
    """
    rclpy.init()
    node = TelemetryNode(buffer, vio_expected_rate_hz=vio_expected_rate_hz)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.05)
    except Exception as e:
        log.error(f"ROS node error: {e}")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        try:
            executor.shutdown(timeout_sec=1)
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    log.info("ROS node stopped.")
