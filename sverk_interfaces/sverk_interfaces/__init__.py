import atexit
import json
import math
import threading
import time
from collections import namedtuple
from typing import Any, Dict, List, Optional, Type

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from offboard_interfaces.action import (
    DownloadLog,
    NavigateTo,
    ReceiveTelemetry,
    RunExcitation,
)
from offboard_interfaces.msg import OffboardStatus, Telemetry, TrajectoryCorrection
from offboard_interfaces.srv import (
    BackupParams,
    EmergencyStop,
    Flip,
    GetParam,
    GetTelemetry,
    Navigate,
    ResetParam,
    ReturnToLaunch,
    SetParam,
    SetAltitude,
    SetAttitude,
    SetPosition,
    SetRates,
    SetTrajectory,
    SetVelocity,
    SetYaw,
    SetYawRate,
    StopExcitation,
)
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import SetBool, Trigger

try:
    from led_interfaces.msg import LEDState, LEDStateArray
    from led_interfaces.srv import SetLEDEffect, SetLEDs
    _LED_AVAILABLE = True
except ImportError:
    _LED_AVAILABLE = False
    LEDState = None  # type: ignore[misc, assignment]
    LEDStateArray = None  # type: ignore[misc, assignment]
    SetLEDEffect = None  # type: ignore[misc, assignment]
    SetLEDs = None  # type: ignore[misc, assignment]

try:
    from camera_calibration.srv import (
        GetState,
        ListCaptures,
        RemoveCapture,
        SetAutoCapture,
        SetCalibrationConfig,
    )
    _CALIB_AVAILABLE = True
except ImportError:
    _CALIB_AVAILABLE = False
    GetState = None  # type: ignore[misc, assignment]
    ListCaptures = None  # type: ignore[misc, assignment]
    RemoveCapture = None  # type: ignore[misc, assignment]
    SetAutoCapture = None  # type: ignore[misc, assignment]
    SetCalibrationConfig = None  # type: ignore[misc, assignment]

try:
    import gpiod  # noqa: used by GpioAPI
    _GPIOD_AVAILABLE = True
except ImportError:
    _GPIOD_AVAILABLE = False
    gpiod = None  # type: ignore[misc, assignment]


# Один распознанный QR/штрих-код от drone.image.detect_qr().
#   data    — раскодированный текст (str)
#   type    — тип кода ("QRCODE", "CODE128", ...)
#   rect    — ограничивающий прямоугольник (x, y, w, h) в пикселях
#   polygon — вершины контура [(x, y), ...]
#   center  — центр кода (x, y) в пикселях
QRCode = namedtuple("QRCode", ["data", "type", "rect", "polygon", "center"])


_instances_lock = threading.Lock()
_instances: "set[DroneInterfaces]" = set()  # type: ignore[name-defined]
_rclpy_owned_by_library = False


def _normalize_ns(prefix: str) -> str:
    if not prefix:
        return ""
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    return prefix.rstrip("/")


def _service_name(prefix: str, name: str) -> str:
    prefix = _normalize_ns(prefix)
    if prefix:
        return f"{prefix}/{name.lstrip('/')}"
    # Global service name so it does not depend on client namespace
    return "/" + name.lstrip("/")


def _topic_name(prefix: str, name: str) -> str:
    prefix = _normalize_ns(prefix)
    if prefix:
        return f"{prefix}/{name.lstrip('/')}"
    return "/" + name.lstrip("/")


def _ensure_rclpy_init() -> None:
    global _rclpy_owned_by_library
    if not rclpy.ok():
        rclpy.init()
        _rclpy_owned_by_library = True


def _maybe_rclpy_shutdown() -> None:
    """Called when no DroneInterfaces remain and this library started rclpy."""
    global _rclpy_owned_by_library
    if _rclpy_owned_by_library and rclpy.ok():
        try:
            rclpy.shutdown()
        finally:
            _rclpy_owned_by_library = False


def _call_service(
    node: Node,
    client: rclpy.client.Client,
    request: Any,
    timeout: Optional[float],
) -> Any:
    if not client.wait_for_service(timeout_sec=timeout or 5.0):
        raise RuntimeError(f"Service {client.srv_name} is not available")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f"Service {client.srv_name} did not respond within {timeout} s")
    result = future.result()
    if result is None:
        raise RuntimeError(f"Service {client.srv_name} returned an empty result")
    return result


_SAME_AS_TIMEOUT = object()  # sentinel: result wait defaults to the goal-accept timeout


def _call_action(
    node: Node,
    client: ActionClient,
    action_name: str,
    goal: Any,
    timeout: Optional[float],
    feedback_cb: Optional[Any] = None,
    result_timeout: Any = _SAME_AS_TIMEOUT,
) -> Any:
    """Send an action goal and block until the result.

    ``feedback_cb(feedback)`` (the unwrapped ``.feedback`` part) fires for every
    feedback message while waiting. ``result_timeout`` bounds only the result
    wait — pass None to wait as long as the server needs (recommended for
    actions that enforce their own deadline, e.g. NavigateTo's goal timeout).
    """
    if result_timeout is _SAME_AS_TIMEOUT:
        result_timeout = timeout
    if not client.wait_for_server(timeout_sec=timeout or 5.0):
        raise RuntimeError(f"Action {action_name} is not available")

    wrapped_cb = None
    if feedback_cb is not None:
        def wrapped_cb(msg: Any) -> None:  # noqa: rclpy passes the wrapper message
            try:
                feedback_cb(msg.feedback)
            except Exception:
                pass

    send_goal_future = client.send_goal_async(goal, feedback_callback=wrapped_cb)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=timeout)
    if not send_goal_future.done():
        raise TimeoutError(f"Action {action_name} did not accept the goal within {timeout} s")
    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError(f"Action {action_name} rejected the goal")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=result_timeout)
    if not result_future.done():
        raise TimeoutError(f"Action {action_name} did not finish within {result_timeout} s")
    wrapped = result_future.result()
    if wrapped is None:
        raise RuntimeError(f"Action {action_name} returned an empty result")
    return wrapped.result


class OffboardControlAPI:
    """High-level access to offboard_control services via drone.control.*"""

    def __init__(self, node: Node, namespace: str = "") -> None:
        self._node = node
        self._ns = namespace

        self.default_frame_id: str = "map"
        self.default_yaw: float = 0.0
        self.default_speed: float = 0.5
        self.default_auto_arm: bool = False
        self.default_timeout: float = 60.0
        self.default_tolerance: float = 0.2
        self.default_check_interval: float = 0.2

        self.default_flip_axis: str = 'roll'
        self.default_flip_vz: float = 2.0
        self.default_flip_climb_duration: float = 0.5
        self.default_flip_rate: float = 16.0
        self.default_flip_target_angle: float = 6.0
        self.default_flip_thrust: float = 0.1

        # offboard_control service clients
        self._navigate = node.create_client(Navigate, _service_name(self._ns, "navigate"))
        self._land = node.create_client(Trigger, _service_name(self._ns, "land"))
        self._get_telemetry = node.create_client(
            GetTelemetry, _service_name(self._ns, "get_telemetry")
        )
        self._set_altitude = node.create_client(
            SetAltitude, _service_name(self._ns, "set_altitude")
        )
        self._set_yaw = node.create_client(SetYaw, _service_name(self._ns, "set_yaw"))
        self._set_yaw_rate = node.create_client(
            SetYawRate, _service_name(self._ns, "set_yaw_rate")
        )
        self._set_position = node.create_client(
            SetPosition, _service_name(self._ns, "set_position")
        )
        self._set_trajectory = node.create_client(
            SetTrajectory, _service_name(self._ns, "set_trajectory")
        )
        self._set_velocity = node.create_client(
            SetVelocity, _service_name(self._ns, "set_velocity")
        )
        self._set_attitude = node.create_client(
            SetAttitude, _service_name(self._ns, "set_attitude")
        )
        self._set_rates = node.create_client(SetRates, _service_name(self._ns, "set_rates"))
        self._flip = node.create_client(Flip, _service_name(self._ns, "flip"))

        # Hardening-block interfaces (offboard_control prompts 91-104)
        self._emergency_stop = node.create_client(
            EmergencyStop, _service_name(self._ns, "emergency_stop")
        )
        self._return_to_launch = node.create_client(
            ReturnToLaunch, _service_name(self._ns, "return_to_launch")
        )
        self._stop_excitation = node.create_client(
            StopExcitation, _service_name(self._ns, "stop_excitation")
        )
        self._navigate_to_ac = ActionClient(
            node, NavigateTo, _service_name(self._ns, "navigate_to")
        )
        self._receive_telemetry_ac = ActionClient(
            node, ReceiveTelemetry, _service_name(self._ns, "receive_telemetry")
        )
        self._run_excitation_ac = ActionClient(
            node, RunExcitation, _service_name(self._ns, "run_excitation")
        )
        # offboard_control's own node-relative interfaces (~/telemetry, ~/status,
        # parameters) resolve under its node name.
        self._status_topic = _topic_name(self._ns, "offboard_control/status")
        self._telemetry_topic = _topic_name(self._ns, "offboard_control/telemetry")
        self._offboard_set_params = node.create_client(
            SetParameters, _service_name(self._ns, "offboard_control/set_parameters")
        )
        # High-rate feed-forward correction stream layered on set_trajectory().
        # Best-effort to match the node's subscription (newest-wins, no retransmit).
        self._trajectory_correction_pub = node.create_publisher(
            TrajectoryCorrection,
            _topic_name(self._ns, "offboard_control/trajectory_correction"),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def configure_defaults(
        self,
        *,
        frame_id: Optional[str] = None,
        yaw: Optional[float] = None,
        speed: Optional[float] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        check_interval: Optional[float] = None,
    ) -> None:
        """Change defaults for navigate / navigate_wait.

        Example:
            drone.control.configure_defaults(
                frame_id="body",
                speed=0.4,
                tolerance=0.3,
            )
        """
        if frame_id is not None:
            self.default_frame_id = frame_id
        if yaw is not None:
            self.default_yaw = float(yaw)
        if speed is not None:
            self.default_speed = float(speed)
        if auto_arm is not None:
            self.default_auto_arm = bool(auto_arm)
        if timeout is not None:
            self.default_timeout = float(timeout)
        if tolerance is not None:
            self.default_tolerance = float(tolerance)
        if check_interval is not None:
            self.default_check_interval = float(check_interval)

    # --- Navigation and flight modes ---

    def navigate(
        self,
        *,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        speed: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> Navigate.Response:
        """Fly to a waypoint (see offboard_control Readme: navigate service)."""
        # Fill in defaults when arguments are omitted.
        if frame_id is None:
            frame_id = self.default_frame_id
        if yaw is None:
            yaw = self.default_yaw
        if speed is None:
            speed = self.default_speed
        if auto_arm is None:
            auto_arm = self.default_auto_arm
        if timeout is None:
            timeout = self.default_timeout

        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        req.frame_id = frame_id
        req.auto_arm = bool(auto_arm)
        return _call_service(self._node, self._navigate, req, timeout)

    def navigate_to(
        self,
        *,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        speed: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: float = 0.0,
        tolerance: float = 0.0,
        feedback: Optional[Any] = None,
        client_timeout: Optional[float] = None,
    ) -> "NavigateTo.Result":
        """Fly to a waypoint via the /navigate_to action and wait for arrival.

        The arrival is tracked ONBOARD by offboard_control — no Python-side
        telemetry polling. The action always finishes, even when the target is
        physically unreachable:

          - arrival (fine 0.08 m tolerance, or `tolerance` if > 0)
            → result.success = True;
          - `timeout` seconds passed → the node parks the drone to hold and
            returns success=False ("Timeout ..."). timeout=0.0 (default) = auto
            limit (3x nominal flight time + 10 s); timeout<0 = no limit;
          - cancel / superseded by another command → success=False.

        `feedback(fb)` (optional) fires on every action feedback:
        fb.distance_remaining (m), fb.eta (s), fb.phase (arming|moving|arrived).

        `client_timeout` bounds how long THIS call blocks. Default None waits
        for the action result — recommended, since the node enforces its own
        `timeout` and always answers.

        Returns the NavigateTo result (.success, .message,
        .final_distance_error).
        """
        if frame_id is None:
            frame_id = self.default_frame_id
        if yaw is None:
            yaw = self.default_yaw
        if speed is None:
            speed = self.default_speed
        if auto_arm is None:
            auto_arm = self.default_auto_arm

        goal = NavigateTo.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.yaw = float(yaw)
        goal.speed = float(speed)
        goal.frame_id = frame_id
        goal.auto_arm = bool(auto_arm)
        goal.timeout = float(timeout)
        goal.tolerance = float(tolerance)
        return _call_action(
            self._node,
            self._navigate_to_ac,
            _service_name(self._ns, "navigate_to"),
            goal,
            self.default_timeout,
            feedback_cb=feedback,
            result_timeout=client_timeout,
        )

    def navigate_wait(
        self,
        *,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        speed: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
        tolerance: Optional[float] = None,
        check_interval: Optional[float] = None,
    ) -> "NavigateTo.Result":
        """Fly to a waypoint and wait until the goal is reached.

        Implemented via the /navigate_to action: offboard_control tracks the
        arrival onboard (no telemetry polling from Python). The historical
        contract is kept:

          - returns an object with .success/.message on arrival
            (plus .final_distance_error);
          - raises TimeoutError when the drone could not reach the target in
            `timeout` seconds — the node parks it to hold first;
          - raises RuntimeError when the flight fails to start.

        `check_interval` is accepted for backward compatibility and ignored
        (the node reports arrival itself).
        """
        del check_interval  # legacy polling knob — arrival is tracked onboard
        eff_timeout = timeout if timeout is not None else self.default_timeout
        eff_tolerance = tolerance if tolerance is not None else self.default_tolerance

        result = self.navigate_to(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            speed=speed,
            frame_id=frame_id,
            auto_arm=auto_arm,
            timeout=eff_timeout,
            tolerance=eff_tolerance,
            # The node aborts the goal itself at eff_timeout; the margin only
            # covers transport/arming latency.
            client_timeout=eff_timeout + 30.0,
        )
        if result.success:
            return result
        if "Timeout" in result.message:
            raise TimeoutError(f"navigate_wait: {result.message}")
        raise RuntimeError(f"navigate_wait: navigate failed: {result.message}")

    def land(self, timeout: Optional[float] = None) -> Trigger.Response:
        """Land (std_srvs/Trigger)."""
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = Trigger.Request()
        return _call_service(self._node, self._land, req, eff_timeout)

    # --- Telemetry ---

    def get_telemetry(
        self, frame_id: Optional[str] = None, timeout: Optional[float] = None
    ) -> GetTelemetry.Response:
        """Request telemetry snapshot."""
        eff_frame = self.default_frame_id if not frame_id else frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = GetTelemetry.Request()
        req.frame_id = eff_frame
        return _call_service(self._node, self._get_telemetry, req, eff_timeout)

    # --- Altitude and heading ---

    def set_altitude(
        self, z: float, frame_id: Optional[str] = None, timeout: Optional[float] = None
    ) -> SetAltitude.Response:
        eff_frame = frame_id or self.default_frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetAltitude.Request()
        req.z = float(z)
        req.frame_id = eff_frame
        return _call_service(self._node, self._set_altitude, req, eff_timeout)

    def set_yaw(
        self,
        yaw: Optional[float] = None,
        frame_id: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> SetYaw.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetYaw.Request()
        req.yaw = eff_yaw
        req.frame_id = eff_frame
        return _call_service(self._node, self._set_yaw, req, eff_timeout)

    def clear_yaw_override(self, timeout: Optional[float] = None) -> SetYaw.Response:
        """Clear yaw / yaw_rate override (yaw=NaN)."""
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetYaw.Request()
        req.yaw = float("nan")
        req.frame_id = "body"
        return _call_service(self._node, self._set_yaw, req, eff_timeout)

    def set_yaw_rate(
        self, yaw_rate: float, timeout: Optional[float] = None
    ) -> SetYawRate.Response:
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetYawRate.Request()
        req.yaw_rate = float(yaw_rate)
        return _call_service(self._node, self._set_yaw_rate, req, eff_timeout)

    # --- Position and velocity ---

    def set_position(
        self,
        *,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetPosition.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetPosition.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = eff_yaw
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_position, req, eff_timeout)

    def set_velocity(
        self,
        *,
        vx: float,
        vy: float,
        vz: float,
        yaw: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetVelocity.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetVelocity.Request()
        req.vx = float(vx)
        req.vy = float(vy)
        req.vz = float(vz)
        req.yaw = eff_yaw
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_velocity, req, eff_timeout)

    def set_trajectory(
        self,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        ax: Optional[float] = None,
        ay: Optional[float] = None,
        az: Optional[float] = None,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetTrajectory.Response:
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetTrajectory.Request()
        req.x = math.nan if x is None else float(x)
        req.y = math.nan if y is None else float(y)
        req.z = math.nan if z is None else float(z)
        req.vx = math.nan if vx is None else float(vx)
        req.vy = math.nan if vy is None else float(vy)
        req.vz = math.nan if vz is None else float(vz)
        req.ax = math.nan if ax is None else float(ax)
        req.ay = math.nan if ay is None else float(ay)
        req.az = math.nan if az is None else float(az)
        req.yaw = math.nan if yaw is None else float(yaw)
        req.yaw_rate = math.nan if yaw_rate is None else float(yaw_rate)
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_trajectory, req, eff_timeout)

    def publish_trajectory_correction(
        self,
        *,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        ax: Optional[float] = None,
        ay: Optional[float] = None,
        az: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        frame_id: Optional[str] = None,
    ) -> None:
        """Publish ONE high-rate feed-forward correction on top of the base
        target previously set via :meth:`set_trajectory`.

        Fire-and-forget over a best-effort topic (no response) — call it
        repeatedly from an avoidance loop (e.g. 50-200 Hz). Each group
        (velocity vx/vy/vz, acceleration ax/ay/az) must be fully specified or
        left unset; an unset field defaults to NaN, which disables that group
        and falls back to the base ``set_trajectory`` value. If the stream goes
        silent for longer than the node's ``correction_timeout`` (default 0.5 s)
        the node drops the correction and holds the base target.

        ``frame_id`` empty (the default) means the base trajectory frame; pass
        e.g. ``"base_link"`` to push a correction in the drone body frame.
        """
        msg = TrajectoryCorrection()
        msg.stamp = self._node.get_clock().now().nanoseconds // 1000
        msg.frame_id = "" if frame_id is None else str(frame_id)
        msg.vx = math.nan if vx is None else float(vx)
        msg.vy = math.nan if vy is None else float(vy)
        msg.vz = math.nan if vz is None else float(vz)
        msg.ax = math.nan if ax is None else float(ax)
        msg.ay = math.nan if ay is None else float(ay)
        msg.az = math.nan if az is None else float(az)
        msg.yaw_rate = math.nan if yaw_rate is None else float(yaw_rate)
        self._trajectory_correction_pub.publish(msg)

    # --- Low-level control ---

    def set_attitude(
        self,
        *,
        roll: float,
        pitch: float,
        yaw: Optional[float] = None,
        thrust: float,
        frame_id: Optional[str] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetAttitude.Response:
        eff_yaw = self.default_yaw if yaw is None else float(yaw)
        eff_frame = frame_id or self.default_frame_id
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetAttitude.Request()
        req.roll = float(roll)
        req.pitch = float(pitch)
        req.yaw = float(eff_yaw)
        req.thrust = float(thrust)
        req.frame_id = eff_frame
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_attitude, req, eff_timeout)

    def set_rates(
        self,
        *,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        thrust: float,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> SetRates.Response:
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = SetRates.Request()
        req.roll_rate = float(roll_rate)
        req.pitch_rate = float(pitch_rate)
        req.yaw_rate = float(yaw_rate)
        req.thrust = float(thrust)
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._set_rates, req, eff_timeout)

    # --- Flip ---

    def flip(
        self,
        *,
        axis: Optional[str] = None,
        vz: Optional[float] = None,
        climb_duration: Optional[float] = None,
        rate: Optional[float] = None,
        target_angle: Optional[float] = None,
        thrust: Optional[float] = None,
        auto_arm: Optional[bool] = None,
        timeout: Optional[float] = None,
    ) -> Flip.Response:
        """Flip around roll/pitch/yaw (see Readme: flip)."""

        eff_axis = self.default_flip_axis if axis is None else str(axis)
        eff_vz = self.default_flip_vz if vz is None else float(vz)
        eff_climb_duration = self.default_flip_climb_duration if climb_duration is None else float(climb_duration)
        eff_rate = self.default_flip_rate if rate is None else float(rate)
        eff_target_angle = self.default_flip_target_angle if target_angle is None else float(target_angle)
        eff_thrust = self.default_flip_thrust if thrust is None else float(thrust)
        eff_auto_arm = self.default_auto_arm if auto_arm is None else bool(auto_arm)
        eff_timeout = timeout if timeout is not None else self.default_timeout

        req = Flip.Request()
        req.axis = eff_axis
        req.vz = eff_vz
        req.climb_duration = eff_climb_duration
        req.rate = eff_rate
        req.target_angle = eff_target_angle
        req.thrust = eff_thrust
        req.auto_arm = eff_auto_arm
        return _call_service(self._node, self._flip, req, eff_timeout)

    # --- Safety: emergency stop / return to launch ---

    def emergency_stop(
        self, land: bool = False, timeout: Optional[float] = None
    ) -> EmergencyStop.Response:
        """Immediately stop ANY active maneuver (navigate/set_*/flip/
        navigate_to/excitation) — works even while another service is waiting
        for arming.

        land=False — hold the current position; land=True — stop and land.

            drone.control.emergency_stop()            # hover in place
            drone.control.emergency_stop(land=True)   # emergency landing
        """
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = EmergencyStop.Request()
        req.land = bool(land)
        return _call_service(self._node, self._emergency_stop, req, eff_timeout)

    def return_to_launch(self, timeout: Optional[float] = None) -> ReturnToLaunch.Response:
        """Stop the active maneuver and return to the launch (home) point.

        Sends PX4 into AUTO.RTL: the drone flies to its takeoff position and
        lands there.

            drone.control.return_to_launch()
        """
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = ReturnToLaunch.Request()
        return _call_service(self._node, self._return_to_launch, req, eff_timeout)

    # --- Telemetry streaming and node diagnostics ---

    def stream_telemetry(
        self,
        callback: Any,
        *,
        rate_hz: float = 10.0,
        duration: float = 5.0,
        frame_id: Optional[str] = None,
        client_timeout: Optional[float] = None,
    ) -> "ReceiveTelemetry.Result":
        """Stream telemetry at `rate_hz` for `duration` seconds (blocking).

        `callback(t)` fires per sample with an offboard_interfaces/Telemetry
        message (same fields as get_telemetry + stamp). duration=0 streams
        until the process is interrupted — prefer a finite duration here, or
        subscribe to the /offboard_control/telemetry topic for open-ended
        streams (see set_telemetry_rate).

            def on_sample(t):
                print(f"z={t.z:.2f} vz={t.vz:.2f}")

            res = drone.control.stream_telemetry(on_sample, rate_hz=20, duration=3.0)
            print("samples:", res.samples_sent)
        """
        goal = ReceiveTelemetry.Goal()
        goal.rate_hz = float(rate_hz)
        goal.frame_id = frame_id if frame_id is not None else self.default_frame_id
        goal.duration = float(duration)
        eff_client_timeout = client_timeout
        if eff_client_timeout is None and duration > 0.0:
            eff_client_timeout = duration + 10.0
        return _call_action(
            self._node,
            self._receive_telemetry_ac,
            _service_name(self._ns, "receive_telemetry"),
            goal,
            self.default_timeout,
            feedback_cb=(lambda fb: callback(fb.telemetry)) if callback else None,
            result_timeout=eff_client_timeout,
        )

    def set_telemetry_rate(self, rate_hz: float, timeout: Optional[float] = None) -> bool:
        """Enable/adjust the /offboard_control/telemetry topic rate (Hz).

        0 switches the topic off (the node default). Returns True on success.
        Subscribe with the generic API:

            drone.control.set_telemetry_rate(50.0)
            from offboard_interfaces.msg import Telemetry
            drone.topic.subscribe(Telemetry, "/offboard_control/telemetry", cb)
        """
        param = RclParameter()
        param.name = "telemetry_rate_hz"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(rate_hz)
        req = SetParameters.Request()
        req.parameters = [param]
        eff_timeout = timeout if timeout is not None else self.default_timeout
        resp = _call_service(self._node, self._offboard_set_params, req, eff_timeout)
        return bool(resp.results and resp.results[0].successful)

    def subscribe_telemetry(self, callback: Any, depth: int = 10) -> Any:
        """Subscribe to the /offboard_control/telemetry topic (best-effort QoS,
        matching the publisher). Enable the topic first with
        set_telemetry_rate(hz). Returns the subscription handle
        (drone.node.destroy_subscription(sub) to stop).

            drone.control.set_telemetry_rate(50.0)
            sub = drone.control.subscribe_telemetry(lambda t: print(t.z))
            drone.topic.spin()   # or spin_once in your loop
        """
        qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        return self._node.create_subscription(
            Telemetry, self._telemetry_topic, callback, qos
        )

    def get_status(self, timeout: Optional[float] = None) -> Optional[OffboardStatus]:
        """One diagnostics snapshot from /offboard_control/status (~2 Hz).

        Fields: setpoint_type, busy, armed, nav_mode, offboard_active, the last
        published target, active_maneuver, last_abort_reason, telemetry_age_s,
        battery_pct. Returns None on timeout.

            s = drone.control.get_status()
            print(s.setpoint_type, s.active_maneuver, s.last_abort_reason)
        """
        received: list = []

        def cb(msg: Any) -> None:
            received.append(msg)

        # ~/status is published best-effort — a reliable subscriber would be
        # QoS-incompatible and receive nothing.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        sub = self._node.create_subscription(OffboardStatus, self._status_topic, cb, qos)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        deadline = time.monotonic() + eff_timeout
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            return received[0] if received else None
        finally:
            self._node.destroy_subscription(sub)

    # --- System-ID excitation (pid_tuning_assistant companion) ---

    def run_excitation(
        self,
        *,
        axis: str,
        signal: str,
        amplitude: float,
        confirm: str,
        loop: int = 0,
        duration: float = 0.0,
        f_min: float = 0.0,
        f_max: float = 0.0,
        base_period: float = 0.0,
        seed: int = 0,
        taper: float = 0.0,
        auto_arm: bool = False,
        hold_alt: float = 0.0,
        max_tilt_deg: float = 0.0,
        geofence_radius: float = 0.0,
        feedback: Optional[Any] = None,
        client_timeout: Optional[float] = None,
    ) -> "RunExcitation.Result":
        """Run a system-ID excitation maneuver (action /run_excitation).

        DANGEROUS — NOT read-only: shakes the flying drone with a windowed
        disturbance on one axis. Requires
        confirm="I_UNDERSTAND_WHAT_I_AM_DOING" (the node rejects the goal
        otherwise). SITL first; see offboard_control Readme for the safety
        envelope and the excite_* parameters.

        `feedback(fb)` fires with fb.phase / fb.progress / fb.current_ref /
        fb.current_rate / fb.current_tilt_deg / fb.current_drift_m.
        Returns the RunExcitation result (.success, .abort_reason, .run_tag,
        .samples_injected, ...).
        """
        goal = RunExcitation.Goal()
        goal.axis = str(axis)
        goal.signal = str(signal)
        goal.loop = int(loop)
        goal.amplitude = float(amplitude)
        goal.duration = float(duration)
        goal.f_min = float(f_min)
        goal.f_max = float(f_max)
        goal.base_period = float(base_period)
        goal.seed = int(seed)
        goal.taper = float(taper)
        goal.auto_arm = bool(auto_arm)
        goal.hold_alt = float(hold_alt)
        goal.max_tilt_deg = float(max_tilt_deg)
        goal.geofence_radius = float(geofence_radius)
        goal.confirm = str(confirm)
        return _call_action(
            self._node,
            self._run_excitation_ac,
            _service_name(self._ns, "run_excitation"),
            goal,
            self.default_timeout,
            feedback_cb=feedback,
            result_timeout=client_timeout,
        )

    def stop_excitation(
        self, emergency: bool = False, timeout: Optional[float] = None
    ) -> StopExcitation.Response:
        """Stop a running excitation: emergency=True — immediate revert to
        hold; False — graceful settle-out, then revert."""
        eff_timeout = timeout if timeout is not None else self.default_timeout
        req = StopExcitation.Request()
        req.emergency = bool(emergency)
        return _call_service(self._node, self._stop_excitation, req, eff_timeout)


class FmuCalibrationAPI:
    """fmu_control API via drone.fcu.*"""

    def __init__(self, node: Node, namespace: str = "/fmu_control") -> None:
        self._node = node
        self._ns = _normalize_ns(namespace) or "/fmu_control"

        self._arm = node.create_client(
            Trigger, _service_name(self._ns, "arm")
        )
        self._disarm = node.create_client(
            Trigger, _service_name(self._ns, "disarm")
        )
        self._force_disarm = node.create_client(
            Trigger, _service_name(self._ns, "force_disarm")
        )
        self._kill_switch = node.create_client(
            Trigger, _service_name(self._ns, "kill_switch")
        )
        self._reboot = node.create_client(
            Trigger, _service_name(self._ns, "reboot")
        )
        self._get_param = node.create_client(
            GetParam, _service_name(self._ns, "get_param")
        )
        self._set_param = node.create_client(
            SetParam, _service_name(self._ns, "set_param")
        )
        self._reset_param = node.create_client(
            ResetParam, _service_name(self._ns, "reset_param")
        )
        self._backup_params = node.create_client(
            BackupParams, _service_name(self._ns, "backup_params")
        )
        self._download_log = ActionClient(
            node, DownloadLog, _service_name(self._ns, "download_log")
        )

        self._calib_pub = node.create_publisher(
            UInt8, _topic_name(self._ns, "request_calibration"), 10
        )

    # --- Arm / disarm / kill switch / reboot ---

    def arm(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._arm, req, timeout)

    def disarm(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._disarm, req, timeout)

    def force_disarm(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._force_disarm, req, timeout)

    def kill_switch(self, timeout: float = 5.0) -> Trigger.Response:
        req = Trigger.Request()
        return _call_service(self._node, self._kill_switch, req, timeout)

    def reboot(self, timeout: float = 5.0) -> Trigger.Response:
        """Reboot the FCU (only when disarmed)."""
        req = Trigger.Request()
        return _call_service(self._node, self._reboot, req, timeout)

    # --- Parameters (over param_dds_bridge, no MAVLink) ---

    def get_param(self, name: str, timeout: float = 5.0) -> GetParam.Response:
        """Read a PX4 parameter by name. Response: success, value, param_type, message."""
        req = GetParam.Request()
        req.name = name
        return _call_service(self._node, self._get_param, req, timeout)

    def set_param(self, name: str, value: float,
                  timeout: float = 5.0) -> SetParam.Response:
        """Set a PX4 parameter by name (int or float — the FCU picks by stored type)."""
        req = SetParam.Request()
        req.name = name
        req.value = float(value)
        return _call_service(self._node, self._set_param, req, timeout)

    def reset_param(self, name: str = "",
                    timeout: float = 5.0) -> ResetParam.Response:
        """Reset a parameter to default. Empty name or '*' resets ALL parameters."""
        req = ResetParam.Request()
        req.name = name
        return _call_service(self._node, self._reset_param, req, timeout)

    # --- Backup ---

    def backup_params(self, timeout: float = 60.0) -> BackupParams.Response:
        """Reliable full parameter dump (by index, gap-filled).

        Returns the BackupParams response: .success, .total, .params
        (list of Param with .name/.param_type/.value). Slow but complete
        (~15 s for the full set).
        """
        req = BackupParams.Request()
        return _call_service(self._node, self._backup_params, req, timeout)

    def backup_to_file(self, path: str, timeout: float = 60.0) -> str:
        """Back up all PX4 parameters to a QGC-compatible ``.params`` file.

        Loadable in QGroundControl (Parameters -> Tools -> Load from file) and
        restorable via ``set_param``. Raises if the backup is incomplete.
        Returns the written path.
        """
        resp = self.backup_params(timeout=timeout)
        if not resp.success:
            raise RuntimeError(
                f"Backup incomplete: {getattr(resp, 'message', '')} "
                f"({len(resp.params)}/{resp.total})")

        # ParamBridge TYPE_* -> MAVLink param type codes used by QGC .params.
        qgc_type = {0: 6, 1: 9}  # 0=INT32->6, 1=FLOAT->9
        lines = [
            "# Onboard parameters for Vehicle 1",
            "#",
            "# Stack: PX4 Pro",
            "# Vehicle: Quadrotor",
            f"# Exported by sverk_interfaces at {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "#",
            "# Vehicle-Id Component-Id Name Value Type",
        ]
        for p in resp.params:
            if p.param_type == 0:  # INT32
                value = str(int(round(p.value)))
            else:                  # FLOAT
                value = f"{p.value:.18f}"
            lines.append(f"1\t1\t{p.name}\t{value}\t{qgc_type.get(p.param_type, 9)}")

        with open(path, "w", encoding="ascii") as f:
            f.write("\n".join(lines) + "\n")
        return path

    def restore_from_file(self, path: str, confirm: str = "",
                          timeout: float = 5.0):
        """Restore parameters from a QGC ``.params`` file via ``set_param``.

        DANGEROUS — overwrites live FMU parameters. Requires
        ``confirm="I_UNDERSTAND_WHAT_I_AM_DOING"`` or it refuses.

        Does NOT reboot the FCU. A reboot is MANDATORY for some parameters to
        take effect — call ``drone.fcu.reboot()`` yourself afterwards (only when
        disarmed).

        Returns ``(ok_count, failed)`` where ``failed`` is a list of
        ``(name, message)`` for params that did not apply (read-only, clipped,
        or unknown) — review them.
        """
        phrase = "I_UNDERSTAND_WHAT_I_AM_DOING"
        if confirm != phrase:
            raise PermissionError(
                f'restore_from_file requires confirm="{phrase}"')

        ok = 0
        failed = []
        with open(path, "r", encoding="ascii") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 5:
                    continue
                name, value = parts[2], parts[3]
                try:
                    resp = self.set_param(name, float(value), timeout=timeout)
                    if resp.success:
                        ok += 1
                    else:
                        failed.append((name, resp.message))
                except Exception as exc:  # noqa: BLE001 - collect and continue
                    failed.append((name, str(exc)))
        return ok, failed

    # --- Log download (log_download_bridge, no MAVLink/physical access) ---

    def download_log(self, path: str, timeout_s: float = 0.0,
                      client_timeout: Optional[float] = None) -> "DownloadLog.Result":
        """Download the latest .ulg from the FCU's SD card to a local file.

        The pull itself uses a windowed ack-driven retry loop on the
        fmu_control side (see /fmu_control/download_log) — measured
        throughput on the real UART/DDS link is ~20 KB/s, so a multi-MB log
        can legitimately take several minutes.

        `timeout_s` is the download deadline enforced by fmu_control itself:
        0 (default) auto-computes a budget from the reported log size; pass
        an explicit value to override (e.g. for a log you know is unusually
        large, or to force a quick failure for a small one).

        `client_timeout` is how long THIS call blocks waiting for the action
        to finish. Default None waits as long as it takes — recommended,
        since fmu_control already enforces its own bounded `timeout_s`; a
        short `client_timeout` here would just abandon a download that is
        still legitimately in progress server-side.

        Returns the DownloadLog action Result (.success, .message, .log_name,
        .file_size, .bytes_received). Check `.success` — an incomplete
        download (e.g. reached `timeout_s`) is reported there, not raised.
        """
        goal = DownloadLog.Goal()
        goal.path = path
        goal.timeout_s = float(timeout_s)
        return _call_action(
            self._node, self._download_log, f"{self._ns}/download_log",
            goal, client_timeout)

    # --- Calibration ---
    #
    # Only the "still" calibrations that complete WITHOUT user motion are exposed
    # here: gyroscope and level horizon. Interactive ones (accel/mag — need
    # rotating/placing the vehicle by [cal] prompts) and ESC (dangerous) are
    # intentionally NOT in this high-level API; drive them through fmu_control
    # directly (action /fmu_control/calibrate, which streams progress, or the
    # low-level /fmu_control/request_calibration topic).

    def request_calibration(self, mode: int) -> None:
        """Publish a raw calibration trigger (0=gyro,1=mag,2=baro,3=accel,4=level).

        Escape hatch for the interactive/ESC calibrations not wrapped here.
        """
        msg = UInt8()
        msg.data = int(mode) & 0xFF
        self._calib_pub.publish(msg)

    def calibrate_gyro(self) -> None:
        """Gyroscope — hold the vehicle still. Completes without user motion."""
        self.request_calibration(0)

    def calibrate_level(self) -> None:
        """Level horizon — place the vehicle level and hold still."""
        self.request_calibration(4)


# LED effect names per led_control docs (ReadMe.md)
LED_EFFECTS = (
    "fill",
    "blink",
    "blink_fast",
    "fade",
    "wipe",
    "flash",
    "rainbow",
    "rainbow_fill",
)


class LedAPI:
    """LED strip API (led_control) via drone.led.*.

    Services: /led_control/set_effect (effect + color), /led_control/set_leds (per-LED).
    State topic: /led_control/state (LEDStateArray).
    """

    def __init__(self, node: Node, namespace: str = "led_control") -> None:
        if not _LED_AVAILABLE:
            raise RuntimeError(
                "led_interfaces is not installed. Add the dependency to use the LED API."
            )
        self._node = node
        self._ns = _normalize_ns(namespace) if namespace else "/led_control"

        self._set_effect = node.create_client(
            SetLEDEffect, _service_name(self._ns, "set_effect")
        )
        self._set_leds = node.create_client(
            SetLEDs, _service_name(self._ns, "set_leds")
        )

        self.default_timeout: float = 5.0

    def set_effect(
        self,
        effect: str = "fill",
        r: int = 0,
        g: int = 0,
        b: int = 0,
        timeout: Optional[float] = None,
    ) -> Any:
        """Set strip effect and color (/led_control/set_effect).

        Effects: fill, blink, blink_fast, fade, wipe, flash, rainbow, rainbow_fill.
        Empty string is treated as fill.
        r, g, b are 0–255.
        """
        req = SetLEDEffect.Request()
        req.effect = effect or "fill"
        req.r = int(r) & 0xFF
        req.g = int(g) & 0xFF
        req.b = int(b) & 0xFF
        eff_timeout = timeout if timeout is not None else self.default_timeout
        return _call_service(self._node, self._set_effect, req, eff_timeout)

    def set_leds(
        self,
        leds: list,
        timeout: Optional[float] = None,
    ) -> Any:
        """Set per-LED colors (/led_control/set_leds).

        leds: list of (index, r, g, b), dicts with index/r/g/b keys,
        or led_interfaces.msg.LEDState. Indices 0 … led_count-1.
        """
        req = SetLEDs.Request()
        req.leds = self._to_led_state_list(leds)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        return _call_service(self._node, self._set_leds, req, eff_timeout)

    def _to_led_state_list(self, leds: list) -> list:
        out = []
        for item in leds:
            if isinstance(item, LEDState):
                out.append(item)
            elif isinstance(item, (tuple, list)) and len(item) >= 4:
                st = LEDState()
                st.index = int(item[0])
                st.r = int(item[1]) & 0xFF
                st.g = int(item[2]) & 0xFF
                st.b = int(item[3]) & 0xFF
                out.append(st)
            elif isinstance(item, dict):
                st = LEDState()
                st.index = int(item["index"])
                st.r = int(item.get("r", 0)) & 0xFF
                st.g = int(item.get("g", 0)) & 0xFF
                st.b = int(item.get("b", 0)) & 0xFF
                out.append(st)
            else:
                raise TypeError(
                    f"Strip element must be LEDState, (index,r,g,b), or dict: {item!r}"
                )
        return out

    def get_state(self, timeout: Optional[float] = None) -> Optional[Any]:
        """Wait for one /led_control/state message (LEDStateArray).

        Returns LEDStateArray or None on timeout.
        """
        received: list = []
        topic = _topic_name(self._ns, "state")

        def cb(msg: Any) -> None:
            received.append(msg)

        sub = self._node.create_subscription(LEDStateArray, topic, cb, 10)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        deadline = time.monotonic() + eff_timeout
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            return received[0] if received else None
        finally:
            self._node.destroy_subscription(sub)


class TopicAPI:
    """Generic topic publish / subscribe via drone.topic.*

    Publishers are cached by topic name — calling create_publisher twice
    for the same topic returns the same handle.

    Subscriptions are tracked internally; unsubscribe removes the sub
    and cleans it up from the node.
    """

    def __init__(self, node: Node, namespace: str = "") -> None:
        self._node = node
        self._ns = _normalize_ns(namespace) if namespace else ""
        self._subs: list = []
        self._pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.default_timeout: float = 5.0

    def _resolve_topic(self, name: str) -> str:
        return _topic_name(self._ns, name)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def create_publisher(
        self,
        msg_type: Type[Any],
        topic: str,
        qos: int = 10,
    ) -> rclpy.publisher.Publisher:
        """Create (or reuse) a publisher on *topic*.

        Usage:

            from std_msgs.msg import String
            pub = drone.topic.create_publisher(String, "/chatter")
            msg = String()
            msg.data = "hello"
            pub.publish(msg)
        """
        full = self._resolve_topic(topic)
        if full not in self._pubs:
            self._pubs[full] = self._node.create_publisher(msg_type, full, qos)
        return self._pubs[full]

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def subscribe(
        self,
        msg_type: Type[Any],
        topic: str,
        callback: Any,
        qos: int = 10,
    ) -> Any:
        """Subscribe to *topic* — *callback(msg)* fires on every message.

        Returns the subscription handle (needed for unsubscribe).

        Usage:

            from sensor_msgs.msg import BatteryState

            def battery_cb(msg):
                print(f"Voltage: {msg.voltage:.2f} V")

            sub = drone.topic.subscribe(
                BatteryState, "/fmu/out/battery_status", battery_cb
            )
            drone.topic.spin()  # blocks, processing callbacks
        """
        full = self._resolve_topic(topic)
        sub = self._node.create_subscription(msg_type, full, callback, qos)
        self._subs.append(sub)
        return sub

    def unsubscribe(self, sub: Any) -> None:
        """Destroy a subscription previously returned by subscribe()."""
        self._node.destroy_subscription(sub)
        if sub in self._subs:
            self._subs.remove(sub)

    # ------------------------------------------------------------------
    # One-shot read
    # ------------------------------------------------------------------

    def wait_for_message(
        self,
        msg_type: Type[Any],
        topic: str,
        timeout: Optional[float] = None,
    ) -> Optional[Any]:
        """Wait for ONE message on *topic*, return it (or None on timeout).

        Usage — grab a single camera frame:

            from sensor_msgs.msg import Image
            frame = drone.topic.wait_for_message(
                Image, "/camera_1/image_raw", timeout=2.0
            )
            if frame is not None:
                print(f"Got frame: {frame.width}x{frame.height}")
        """
        received: list = []
        full = self._resolve_topic(topic)

        def cb(msg: Any) -> None:
            received.append(msg)

        sub = self._node.create_subscription(msg_type, full, cb, 10)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        deadline = time.monotonic() + eff_timeout
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            return received[0] if received else None
        finally:
            self._node.destroy_subscription(sub)

    # ------------------------------------------------------------------
    # Spin helpers
    # ------------------------------------------------------------------

    def spin(self) -> None:
        """Block forever, processing callbacks — like rclpy.spin(node)."""
        rclpy.spin(self._node)

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        """Process one batch of pending callbacks / events."""
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)


class CameraCalibrationAPI:
    """Камера-калибровка через сервисы ноды camera_calibration — drone.image.calibration.*

    Заменяет ручные `ros2 service call` и работу с топиками. Типичный сценарий:

        cal = drone.image.calibration
        cal.start_session()
        cal.set_config(board_type="charuco", board_width=11, board_height=8,
                       square_size=0.022, marker_size=0.016, aruco_dict="DICT_4X4_50")
        cal.set_auto_capture(True, period_sec=1.0)   # води доской перед камерой
        # ... набрали кадры ...
        cal.set_auto_capture(False)
        print(cal.get_state())                       # сколько кадров, RMS error
        cal.calibrate()                              # рассчитать
        cal.save()                                   # сохранить latest_calibration.yaml

    Превью с распознанной доской нода публикует в /camera_calibration/processed_image
    (открой в веб-интерфейсе).
    """

    def __init__(self, node: Node, namespace: str = "/calibration") -> None:
        self._node = node
        ns = "/" + namespace.strip("/")
        self._ns = ns
        self._start = node.create_client(Trigger, f"{ns}/start_session")
        self._reset = node.create_client(Trigger, f"{ns}/reset_session")
        self._capture = node.create_client(Trigger, f"{ns}/capture")
        self._calibrate = node.create_client(Trigger, f"{ns}/calibrate")
        self._save = node.create_client(Trigger, f"{ns}/save")
        self._set_config = node.create_client(SetCalibrationConfig, f"{ns}/set_config")
        self._set_auto = node.create_client(SetAutoCapture, f"{ns}/set_auto_capture")
        self._get_state = node.create_client(GetState, f"{ns}/get_state")
        self._list = node.create_client(ListCaptures, f"{ns}/list_captures")
        self._remove = node.create_client(RemoveCapture, f"{ns}/remove_capture")
        self.default_timeout: float = 5.0

    def _t(self, timeout: Optional[float]) -> float:
        return timeout if timeout is not None else self.default_timeout

    # --- сессия ----------------------------------------------------
    def start_session(self, timeout: Optional[float] = None) -> Any:
        """Начать новую сессию калибровки (очищает накопленные кадры)."""
        return _call_service(self._node, self._start, Trigger.Request(), self._t(timeout))

    def reset_session(self, timeout: Optional[float] = None) -> Any:
        """Сбросить сессию и удалить накопленные кадры."""
        return _call_service(self._node, self._reset, Trigger.Request(), self._t(timeout))

    # --- конфигурация доски ----------------------------------------
    def set_config(
        self,
        board_type: str = "charuco",
        camera_model: str = "pinhole",
        board_width: int = 11,
        board_height: int = 8,
        square_size: float = 0.022,
        marker_size: float = 0.016,
        aruco_dict: str = "DICT_4X4_50",
        min_frames: int = 10,
        save_captures: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        """Задать параметры калибровочной доски.

        board_type: "chessboard" | "charuco" | "circles"
        camera_model: "pinhole" | "fisheye"
        board_width / board_height — число внутренних узлов
        square_size / marker_size — в метрах
        """
        req = SetCalibrationConfig.Request()
        req.board_type = board_type
        req.camera_model = camera_model
        req.board_width = int(board_width)
        req.board_height = int(board_height)
        req.square_size = float(square_size)
        req.marker_size = float(marker_size)
        req.aruco_dict = aruco_dict
        req.min_frames = int(min_frames)
        req.save_captures = bool(save_captures)
        return _call_service(self._node, self._set_config, req, self._t(timeout))

    # --- сбор кадров -----------------------------------------------
    def capture(self, timeout: Optional[float] = None) -> Any:
        """Захватить текущий кадр в набор для калибровки."""
        return _call_service(self._node, self._capture, Trigger.Request(), self._t(timeout))

    def set_auto_capture(
        self, enable: bool, period_sec: float = 1.0, timeout: Optional[float] = None
    ) -> Any:
        """Включить/выключить автозахват кадров каждые period_sec секунд."""
        req = SetAutoCapture.Request()
        req.enable = bool(enable)
        req.period_sec = float(period_sec)
        return _call_service(self._node, self._set_auto, req, self._t(timeout))

    def list_captures(self, timeout: Optional[float] = None) -> List[str]:
        """Список накопленных кадров."""
        resp = _call_service(self._node, self._list, ListCaptures.Request(), self._t(timeout))
        return list(resp.captures)

    def remove_capture(self, index: int, timeout: Optional[float] = None) -> Any:
        """Удалить кадр по индексу (нумерация с 0)."""
        req = RemoveCapture.Request()
        req.index = int(index)
        return _call_service(self._node, self._remove, req, self._t(timeout))

    # --- расчёт и сохранение ---------------------------------------
    def calibrate(self, timeout: Optional[float] = None) -> Any:
        """Рассчитать параметры камеры по накопленным кадрам (может занять время)."""
        return _call_service(
            self._node, self._calibrate, Trigger.Request(),
            timeout if timeout is not None else 60.0,
        )

    def save(self, timeout: Optional[float] = None) -> Any:
        """Сохранить результат калибровки (latest_calibration.yaml в output_dir)."""
        return _call_service(self._node, self._save, Trigger.Request(), self._t(timeout))

    # --- состояние -------------------------------------------------
    def get_state(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Текущее состояние калибровки как dict (число кадров, RMS error и т.д.)."""
        resp = _call_service(self._node, self._get_state, GetState.Request(), self._t(timeout))
        try:
            return json.loads(resp.state_json)
        except (ValueError, TypeError, AttributeError):
            return {}


class ImageAPI:
    """Image capture, streaming, publishing, and conversion via drone.image.*

    Capture / stream — return OpenCV frames (numpy BGR) by default:
    - take_picture(topic, timeout, raw=False) — grab one frame
    - stream(callback, topic, duration, raw=False) — process frames
    - stop_stream() — stop an ongoing stream

    Pass raw=True to get the original sensor_msgs/Image instead of OpenCV.

    Conversion helpers:
    - to_cv2(image_msg) — sensor_msgs/Image → numpy BGR (OpenCV)
    - to_ros(img, header, encoding) — numpy array → sensor_msgs/Image

    Publishing:
    - publish(image, topic) — publish a numpy array OR a sensor_msgs/Image

    QR / barcodes:
    - detect_qr(image=None) — decode QR/barcodes with pyzbar

    Camera calibration:
    - calibration — CameraCalibrationAPI (start_session, set_config, capture,
      calibrate, save, ...), or None if the camera_calibration package is absent

    Default input:  /camera_1/image_raw
    Default output: /out_detection
    """

    def __init__(
        self,
        node: Node,
        namespace: str = "",
        *,
        default_topic: str = "/camera_1/image_raw",
        default_out_topic: str = "/out_detection",
    ) -> None:
        self._node = node
        self._ns = _normalize_ns(namespace) if namespace else ""
        self._default_topic = (
            default_topic if default_topic.startswith("/")
            else _topic_name(self._ns, default_topic)
        )
        self._default_out_topic = (
            default_out_topic if default_out_topic.startswith("/")
            else _topic_name(self._ns, default_out_topic)
        )
        self._pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self._streaming = False
        self._stream_stop = threading.Event()
        self._last_header: Any = None  # header of the most recent frame (for publish)
        self.default_timeout: float = 5.0
        # drone.image.calibration — обёртка над нодой camera_calibration (или None)
        self.calibration: Optional["CameraCalibrationAPI"] = (
            CameraCalibrationAPI(node) if _CALIB_AVAILABLE else None
        )

    # ------------------------------------------------------------------
    # Conversion: ROS Image <-> OpenCV (numpy)
    # ------------------------------------------------------------------

    def to_cv2(self, image_msg: Any) -> Any:
        """sensor_msgs/Image → numpy array in OpenCV BGR layout.

        Colour frames come back as (H, W, 3) in BGR; mono as (H, W).
        The result is writable, so you can draw on it directly.
        Remembers the message header so a later publish() can reuse it.
        """
        import numpy as np  # noqa: lazy import

        enc = (image_msg.encoding or "").lower()
        channels = 3 if ("rgb" in enc or "bgr" in enc) else 1
        arr = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
            (image_msg.height, image_msg.width, channels)
        )
        if enc == "rgb8":
            arr = arr[..., ::-1]  # RGB → BGR (swap channels; OpenCV uses BGR)
        if channels == 1:
            arr = arr.reshape((image_msg.height, image_msg.width))
        self._last_header = getattr(image_msg, "header", None)
        return arr.copy()  # contiguous + writable

    def to_ros(
        self,
        img: Any,
        header: Optional[Any] = None,
        encoding: str = "bgr8",
    ) -> Any:
        """numpy array (OpenCV) → sensor_msgs/Image.

        *encoding* must match the array: "bgr8"/"rgb8" for colour (H, W, 3),
        "mono8" for grayscale (H, W).  *header* defaults to the header of the
        last frame returned by take_picture()/stream(), so timestamps are
        preserved automatically.
        """
        from sensor_msgs.msg import Image  # noqa: lazy import
        import numpy as np  # noqa: lazy import

        arr = np.ascontiguousarray(img)
        msg = Image()
        hdr = header if header is not None else self._last_header
        if hdr is not None:
            msg.header = hdr
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        channels = int(arr.shape[2]) if arr.ndim == 3 else 1
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(msg.width * channels)
        msg.data = arr.tobytes()
        return msg

    # ------------------------------------------------------------------
    # Single-frame capture
    # ------------------------------------------------------------------

    def take_picture(
        self,
        topic: Optional[str] = None,
        timeout: Optional[float] = None,
        raw: bool = False,
    ) -> Optional[Any]:
        """Grab one frame from *topic* (default /camera_1/image_raw).

        Returns an OpenCV image (numpy BGR) by default, or None on timeout.
        Pass raw=True to get the original sensor_msgs/Image.

        Usage:

            img = drone.image.take_picture()           # numpy BGR (OpenCV)
            msg = drone.image.take_picture(raw=True)    # sensor_msgs/Image
        """
        from sensor_msgs.msg import Image  # noqa: lazy import

        t = topic if topic else self._default_topic
        received: list = []

        def cb(msg: Any) -> None:
            received.append(msg)

        sub = self._node.create_subscription(Image, t, cb, 10)
        eff_timeout = timeout if timeout is not None else self.default_timeout
        deadline = time.monotonic() + eff_timeout
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            if not received:
                return None
            return received[0] if raw else self.to_cv2(received[0])
        finally:
            self._node.destroy_subscription(sub)

    # ------------------------------------------------------------------
    # Streaming
    # ------------------------------------------------------------------

    def stream(
        self,
        callback: Any,
        topic: Optional[str] = None,
        duration: Optional[float] = None,
        raw: bool = False,
    ) -> None:
        """Stream frames to *callback(frame)* on every incoming Image.

        By default *frame* is an OpenCV image (numpy BGR); pass raw=True to
        receive the original sensor_msgs/Image instead.

        *duration* (seconds) limits streaming time.  If None, streams until
        stop_stream() is called from another thread, or the program exits.

        Usage — stream for 10 seconds:

            import cv2

            def detect_red(frame):
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
                print(f"Red pixels: {cv2.countNonZero(mask)}")

            drone.image.stream(detect_red, duration=10.0)

        Usage — stream until Ctrl+C:

            def draw_crosshair(frame):
                h, w = frame.shape[:2]
                cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
                cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)

            drone.image.stream(draw_crosshair)

        Usage — stop from another condition:

            canary = {"aruco_found": False}

            def find_aruco(frame):
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                )
                if ids is not None:
                    canary["aruco_found"] = True

            # In thread-1:
            drone.image.stream(find_aruco)

            # In thread-2:
            while not canary["aruco_found"]:
                time.sleep(0.1)
            drone.image.stop_stream()
        """
        from sensor_msgs.msg import Image  # noqa: lazy import

        t = topic if topic else self._default_topic
        self._streaming = True
        self._stream_stop.clear()

        def _cb(msg: Any) -> None:
            try:
                callback(msg if raw else self.to_cv2(msg))
            except Exception:
                pass

        sub = self._node.create_subscription(Image, t, _cb, 10)
        start = time.monotonic()
        try:
            while self._streaming and not self._stream_stop.is_set():
                rclpy.spin_once(self._node, timeout_sec=0.1)
                if duration is not None and (time.monotonic() - start) >= duration:
                    break
        finally:
            self._streaming = False
            self._stream_stop.clear()
            self._node.destroy_subscription(sub)

    def stop_stream(self) -> None:
        """Signal an ongoing stream() to exit as soon as possible."""
        self._streaming = False
        self._stream_stop.set()

    # ------------------------------------------------------------------
    # QR / barcode detection (pyzbar)
    # ------------------------------------------------------------------

    def detect_qr(
        self,
        image: Any = None,
        topic: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> List["QRCode"]:
        """Найти и раскодировать QR/штрих-коды на кадре с помощью pyzbar.

        *image* может быть:
          - numpy-картинкой (OpenCV);
          - сообщением sensor_msgs/Image;
          - None — тогда кадр берётся сам через take_picture(topic, timeout).

        Возвращает список QRCode (data, type, rect, polygon, center); пустой
        список — если ничего не найдено.

            for code in drone.image.detect_qr():
                print(code.data, code.center)
        """
        try:
            from pyzbar import pyzbar  # noqa: lazy import
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "pyzbar не установлен. Установи: pip install pyzbar "
                "(и системную библиотеку: sudo apt install libzbar0)"
            ) from exc

        if image is None:
            image = self.take_picture(topic=topic, timeout=timeout)
            if image is None:
                return []

        # ROS Image → numpy; numpy оставляем как есть
        img = self.to_cv2(image) if hasattr(image, "encoding") else image

        results: List["QRCode"] = []
        for sym in pyzbar.decode(img):
            try:
                data = sym.data.decode("utf-8")
            except (UnicodeDecodeError, AttributeError):
                data = sym.data.decode("latin-1", "replace")
            r = sym.rect
            results.append(
                QRCode(
                    data=data,
                    type=sym.type,
                    rect=(r.left, r.top, r.width, r.height),
                    polygon=[(p.x, p.y) for p in sym.polygon],
                    center=(r.left + r.width // 2, r.top + r.height // 2),
                )
            )
        return results

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publish(
        self,
        image: Any,
        topic: Optional[str] = None,
        *,
        header: Optional[Any] = None,
        encoding: str = "bgr8",
    ) -> None:
        """Publish *image* to *topic* (default /out_detection).

        *image* may be a numpy array (OpenCV) — it is converted to
        sensor_msgs/Image automatically — or an existing sensor_msgs/Image.

        Usage:

            frame = drone.image.take_picture()   # numpy BGR
            # ... draw boxes on frame ...
            drone.image.publish(frame)                    # → /out_detection
            drone.image.publish(frame, topic="/my/topic")
        """
        from sensor_msgs.msg import Image  # noqa: lazy import

        msg = image if isinstance(image, Image) else self.to_ros(
            image, header=header, encoding=encoding
        )
        t = topic if topic else self._default_out_topic
        pub = self._pubs.get(t)
        if pub is None:
            pub = self._node.create_publisher(Image, t, 10)
            self._pubs[t] = pub
        pub.publish(msg)


class GpioAPI:
    """GPIO, servo, and magnet control via drone.gpio.*

    Direct GPIO (requires gpiod):
      - pin_on(pin) / pin_off(pin)   — set HIGH / LOW
      - pin_read(pin)                — read 0 or 1
      - pin_release(pin)             — free the pin

    Servo (ROS 2 servo_control):
      - servo_enable() / servo_disable()
      - servo_select_channel(N)      — switch to channel N (multi-servo)
      - servo_set_angle(degrees)     — move to target angle
      - servo_center()               — return to center

    Magnet (convenience wrappers):
      - magnet_on(pin=23) / magnet_off(pin=23)
    """

    def __init__(
        self,
        node: Node,
        *,
        chip_path: str = "/dev/gpiochip0",
        servo_enable_service: str = "/servo_control/enable",
        servo_angle_topic: str = "/servo_control/target_angle_deg",
        servo_center_service: str = "/servo_control/center",
    ) -> None:
        self._node = node
        self._chip_path = chip_path
        self._chip: Any = None
        self._lines: Dict[int, Any] = {}

        # Servo ROS 2 interfaces (always created — they connect when servo_control is up)
        self._servo_enable = node.create_client(
            SetBool, servo_enable_service
        )
        self._servo_angle = node.create_publisher(
            Float32, servo_angle_topic, 10
        )
        self._servo_center = node.create_client(
            Trigger, servo_center_service
        )

        # Multi-channel servo support: cache resources per channel index.
        # Channel 0 = default servo_control (paths provided at init time).
        self._active_channel: int = 0
        self._channel_resources: Dict[int, Dict[str, Any]] = {
            0: {
                "pub": self._servo_angle,
                "enable": self._servo_enable,
                "center": self._servo_center,
            }
        }

    # ------------------------------------------------------------------
    # Direct GPIO
    # ------------------------------------------------------------------

    def _ensure_gpiod(self) -> None:
        if not _GPIOD_AVAILABLE:
            raise RuntimeError(
                "gpiod is not installed. Run: sudo apt install python3-libgpiod"
            )

    def _get_chip(self) -> Any:
        if self._chip is None:
            self._ensure_gpiod()
            self._chip = gpiod.Chip(self._chip_path)
        return self._chip

    def _get_line(self, pin: int, direction: str = "output") -> Any:
        for stored_pin, (line, _) in self._lines.items():
            if stored_pin == pin:
                return line
        chip = self._get_chip()
        line = chip.get_line(pin)
        if direction == "output":
            line.request(
                consumer="sverk_interfaces",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0],
            )
        else:
            line.request(
                consumer="sverk_interfaces",
                type=gpiod.LINE_REQ_DIR_IN,
            )
        self._lines[pin] = (line, direction)
        return line

    def pin_on(self, pin: int) -> None:
        """Set GPIO *pin* HIGH (3.3 V).

        Usage — blink an LED on GPIO17:

            for _ in range(3):
                drone.gpio.pin_on(17)
                time.sleep(0.3)
                drone.gpio.pin_off(17)
                time.sleep(0.3)
        """
        line = self._get_line(pin, "output")
        line.set_value(1)

    def pin_off(self, pin: int) -> None:
        """Set GPIO *pin* LOW (0 V)."""
        line = self._get_line(pin, "output")
        line.set_value(0)

    def pin_read(self, pin: int) -> int:
        """Read GPIO *pin* — returns 0 (LOW) or 1 (HIGH).

        Usage — wait for button press on GPIO24:

            print("Press the button...")
            while drone.gpio.pin_read(24) == 0:
                time.sleep(0.05)
            print("Button pressed!")
        """
        line = self._get_line(pin, "input")
        return line.get_value()

    def pin_release(self, pin: int) -> None:
        """Release GPIO *pin* — always call when done."""
        if pin in self._lines:
            line, _ = self._lines.pop(pin)
            line.release()

    # ------------------------------------------------------------------
    # Servo (via ROS 2 servo_control)
    # ------------------------------------------------------------------

    def servo_enable(self, timeout: float = 5.0) -> Any:
        """Enable servo PWM output.

        Usage:

            drone.gpio.servo_enable()
            drone.gpio.servo_set_angle(90.0)
        """
        req = SetBool.Request()
        req.data = True
        return _call_service(self._node, self._servo_enable, req, timeout)

    def servo_disable(self, timeout: float = 5.0) -> Any:
        """Disable servo PWM output."""
        req = SetBool.Request()
        req.data = False
        return _call_service(self._node, self._servo_enable, req, timeout)

    def servo_select_channel(self, channel: int) -> None:
        """Switch to servo channel *channel* (0-based).

        Channel 0 maps to the default ``/servo_control`` instance (configured at
        init time).  Channels 1, 2, … map to ``/servo_control_1``,
        ``/servo_control_2``, … — each is a separate servo_control launched with
        a matching namespace.

        After calling this method all subsequent ``servo_enable``,
        ``servo_set_angle``, and ``servo_center`` calls target the selected
        channel until changed again.

        Usage — two servos on channels 0 and 1:

            drone.gpio.servo_enable()          # channel 0
            drone.gpio.servo_set_angle(90.0)

            drone.gpio.servo_select_channel(1) # switch to channel 1
            drone.gpio.servo_enable()
            drone.gpio.servo_set_angle(45.0)

            drone.gpio.servo_select_channel(0) # back to channel 0
            drone.gpio.servo_set_angle(0.0)
        """
        if channel == self._active_channel:
            return
        if channel not in self._channel_resources:
            suffix = f"_{channel}" if channel > 0 else ""
            angle_topic = f"/servo_control{suffix}/target_angle_deg"
            enable_srv = f"/servo_control{suffix}/enable"
            center_srv = f"/servo_control{suffix}/center"
            self._channel_resources[channel] = {
                "pub": self._node.create_publisher(Float32, angle_topic, 10),
                "enable": self._node.create_client(SetBool, enable_srv),
                "center": self._node.create_client(Trigger, center_srv),
            }
        res = self._channel_resources[channel]
        self._servo_angle = res["pub"]
        self._servo_enable = res["enable"]
        self._servo_center = res["center"]
        self._active_channel = channel

    def servo_set_angle(self, degrees: float) -> None:
        """Move servo to *degrees* (0–180).

        Sends a Float32 message to the servo_control topic.
        Servo must be enabled first with servo_enable().

        Usage:

            drone.gpio.servo_set_angle(45.0)   # half-way
            time.sleep(0.5)
            drone.gpio.servo_set_angle(135.0)  # other side
        """
        msg = Float32()
        msg.data = float(degrees)
        self._servo_angle.publish(msg)

    def servo_center(self, timeout: float = 5.0) -> Any:
        """Return servo to its center position (90°)."""
        req = Trigger.Request()
        return _call_service(self._node, self._servo_center, req, timeout)

    # ------------------------------------------------------------------
    # Magnet (convenience wrappers around pin_on / pin_off)
    # ------------------------------------------------------------------

    def magnet_on(self, pin: int = 23) -> None:
        """Turn electromagnet ON (pin HIGH → transistor conducts).

        Usage — grab a payload:

            drone.gpio.magnet_on(pin=23)
            time.sleep(3.0)
            drone.gpio.magnet_off(pin=23)
        """
        self.pin_on(pin)

    def magnet_off(self, pin: int = 23) -> None:
        """Turn electromagnet OFF (pin LOW)."""
        self.pin_off(pin)


class ServiceAPI:
    """Generic ROS 2 service client via drone.service.*

    Allows calling any ROS 2 service by name without pre-registering it.
    Clients are created on first use and cached by service name.

    Usage — Trigger service:

        from std_srvs.srv import Trigger
        resp = drone.service.call('/land', Trigger)

    Usage — service with fields passed as keyword arguments:

        from offboard_interfaces.srv import Navigate
        resp = drone.service.call(
            '/navigate', Navigate,
            x=0.0, y=0.0, z=1.5, yaw=0.0,
            speed=1.0, frame_id='body', auto_arm=True,
        )

    Usage — pre-built request object:

        from offboard_interfaces.srv import Navigate
        req = Navigate.Request()
        req.x = 1.0
        req.z = 1.5
        req.frame_id = 'map'
        resp = drone.service.call('/navigate', Navigate, req)

    Usage — Arduino rosserial service (Arduino is the server):

        from std_srvs.srv import SetBool
        resp = drone.service.call('/arduino/led_on', SetBool, data=True)
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._clients: Dict[str, rclpy.client.Client] = {}
        self.default_timeout: float = 5.0

    def call(
        self,
        service_name: str,
        srv_type: Type[Any],
        request: Optional[Any] = None,
        *,
        timeout: Optional[float] = None,
        **kwargs: Any,
    ) -> Any:
        """Call ROS 2 service *service_name* of type *srv_type*.

        *request* — pre-built ``srv_type.Request()`` instance.  If omitted,
        a fresh request is created and *kwargs* are applied as field values::

            resp = drone.service.call('/navigate', Navigate,
                                      x=1.0, z=1.5, frame_id='map')

        Returns the service response object.
        Raises ``RuntimeError`` if the service is unavailable.
        Raises ``TimeoutError`` if no response arrives within *timeout*.
        """
        if service_name not in self._clients:
            self._clients[service_name] = self._node.create_client(
                srv_type, service_name
            )
        client = self._clients[service_name]

        if request is None:
            request = srv_type.Request()
            for key, value in kwargs.items():
                setattr(request, key, value)

        eff_timeout = timeout if timeout is not None else self.default_timeout
        return _call_service(self._node, client, request, eff_timeout)


class GenericServiceFamily:
    """Generic service bundle for extending the API.

    Use via drone.add_family(...); see DroneInterfaces.add_family.
    """

    def __init__(
        self,
        node: Node,
        namespace: str,
        services: Dict[str, Type[Any]],
    ) -> None:
        self._node = node
        self._ns = namespace
        self._clients: Dict[str, rclpy.client.Client] = {}

        for name, srv_type in services.items():
            self._clients[name] = node.create_client(
                srv_type, _service_name(self._ns, name)
            )

    def call(
        self,
        service_name: str,
        request: Any,
        timeout: Optional[float] = None,
    ) -> Any:
        """Low-level call to an arbitrary service in this family.

        Example:

            from my_pkg.srv import MySrv
            fam = drone.add_family(
                name="my_family",
                namespace="/my_ns",
                services={"do_something": MySrv},
            )
            req = MySrv.Request()
            req.foo = 1
            resp = fam.call("do_something", req)
        """
        if service_name not in self._clients:
            raise KeyError(f"Unknown service '{service_name}' in family")
        client = self._clients[service_name]
        return _call_service(self._node, client, request, timeout)


class DroneInterfaces:
    """Main library handle: drone = sverk_interfaces.init(...).

    Attributes:
      - control: OffboardControlAPI (navigate, land, set_*, etc.)
      - fcu: FmuCalibrationAPI (disarm / force_disarm / kill_switch / calibration)
      - image: ImageAPI (take_picture, stream, stop_stream, publish)
      - gpio: GpioAPI (pin_on/off/read, servo_select_channel, servo_set_angle,
                       magnet_on/off)
      - topic: TopicAPI (create_publisher, subscribe, wait_for_message, spin)
      - service: ServiceAPI (call any ROS 2 service by name + type)
      - led: LedAPI or None (set_effect, set_leds, get_state) if led_interfaces installed
    """

    def __init__(
        self,
        node: Node,
        *,
        offboard_namespace: str = "",
        fcu_namespace: str = "/fmu_control",
        led_namespace: str = "led_control",
        topic_namespace: str = "",
        image_topic: str = "/camera_1/image_raw",
        image_out_topic: str = "/out_detection",
        gpio_chip: str = "/dev/gpiochip0",
        servo_enable: str = "/servo_control/enable",
        servo_angle_topic: str = "/servo_control/target_angle_deg",
        servo_center: str = "/servo_control/center",
    ) -> None:
        self._node = node
        self._closed = False
        self._families: Dict[str, GenericServiceFamily] = {}

        self.control = OffboardControlAPI(node, namespace=offboard_namespace)
        self.fcu = FmuCalibrationAPI(node, namespace=fcu_namespace)
        self.image = ImageAPI(
            node,
            default_topic=image_topic,
            default_out_topic=image_out_topic,
        )
        self.gpio = GpioAPI(
            node,
            chip_path=gpio_chip,
            servo_enable_service=servo_enable,
            servo_angle_topic=servo_angle_topic,
            servo_center_service=servo_center,
        )
        self.topic = TopicAPI(node, namespace=topic_namespace)
        self.service = ServiceAPI(node)
        if _LED_AVAILABLE:
            self.led = LedAPI(node, namespace=led_namespace)
        else:
            self.led = None  # type: ignore[assignment]

        with _instances_lock:
            _instances.add(self)

    @property
    def node(self) -> Node:
        """Underlying rclpy node for low-level use."""
        return self._node

    def add_family(
        self,
        *,
        name: str,
        namespace: str,
        services: Dict[str, Type[Any]],
    ) -> GenericServiceFamily:
        """Register a new service family.

        Example:

            from my_pkg.srv import DoThing

            drone = sverk_interfaces.init(Nodename="drone_control")
            my_family = drone.add_family(
                name="my",
                namespace="/my_node",
                services={"do_thing": DoThing},
            )
            req = DoThing.Request()
            req.param = 123
            resp = my_family.call("do_thing", req)
        """
        family = GenericServiceFamily(self._node, namespace, services)
        self._families[name] = family
        setattr(self, name, family)
        return family

    def close(self) -> None:
        """Explicit cleanup (node + rclpy if owned by library)."""
        if self._closed:
            return
        self._closed = True

        try:
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        finally:
            with _instances_lock:
                _instances.discard(self)
                # Shutdown rclpy when no instances remain and we started it.
                if not _instances:
                    _maybe_rclpy_shutdown()

    def __del__(self) -> None:  # best-effort cleanup
        try:
            self.close()
        except Exception:
            pass


def init(
    Nodename: Optional[str] = None,
    *,
    node_name: Optional[str] = None,
    offboard_namespace: str = "",
    fcu_namespace: str = "/fmu_control",
    led_namespace: str = "led",
    topic_namespace: str = "",
    image_topic: str = "/camera_1/image_raw",
    image_out_topic: str = "/out_detection",
    gpio_chip: str = "/dev/gpiochip0",
    servo_enable: str = "/servo_control/enable",
    servo_angle_topic: str = "/servo_control/target_angle_deg",
    servo_center: str = "/servo_control/center",
) -> DroneInterfaces:
    """Initialize the library.

    Examples:

        import sverk_interfaces

        drone = sverk_interfaces.init(Nodename="drone_control")
        drone.control.navigate(x=0.0, y=0.0, z=1.5, yaw=0.0,
                                speed=1.0, frame_id="body", auto_arm=True)
        frame = drone.image.take_picture(timeout=2.0)
        drone.gpio.pin_on(23)
        drone.fcu.force_disarm()
        drone.topic.subscribe(SomeMsg, "/some/topic", my_callback)
        if drone.led:
            drone.led.set_effect("fill", r=255, g=0, b=0)

    Parameters:
      - Nodename / node_name: name of the created rclpy node
      - offboard_namespace: namespace of offboard_control
        (default root → /navigate, /land, ...)
      - fcu_namespace: fmu_control namespace
        (default /fmu_control)
      - led_namespace: led_control node namespace (default led_control → /led_control/set_effect, /led_control/set_leds)
      - topic_namespace: default namespace prefix for drone.topic.*
        (default empty → absolute topic names)
      - image_topic: default input topic for take_picture / stream
        (default /camera_1/image_raw)
      - image_out_topic: default output topic for publish
        (default /out_detection)
      - gpio_chip: path to gpiochip device (default /dev/gpiochip0)
      - servo_enable: enable/disable service for servo_control
      - servo_angle_topic: topic to set servo angle (onboard servo)
      - servo_center: center service for servo_control
    """
    _ensure_rclpy_init()

    name = node_name or Nodename or "sverk_drone_client"
    node = rclpy.create_node(name)
    return DroneInterfaces(
        node,
        offboard_namespace=offboard_namespace,
        fcu_namespace=fcu_namespace,
        led_namespace=led_namespace,
        topic_namespace=topic_namespace,
        image_topic=image_topic,
        image_out_topic=image_out_topic,
        gpio_chip=gpio_chip,
        servo_enable=servo_enable,
        servo_angle_topic=servo_angle_topic,
        servo_center=servo_center,
    )


def _atexit_cleanup() -> None:
    # On process exit: destroy nodes and shutdown rclpy if the library started it.
    with _instances_lock:
        instances = list(_instances)
    for inst in instances:
        try:
            inst.close()
        except Exception:
            pass


atexit.register(_atexit_cleanup)


__all__ = [
    "DroneInterfaces",
    "LedAPI",
    "LED_EFFECTS",
    "OffboardControlAPI",
    "FmuCalibrationAPI",
    "ImageAPI",
    "CameraCalibrationAPI",
    "QRCode",
    "GpioAPI",
    "TopicAPI",
    "ServiceAPI",
    "GenericServiceFamily",
    "init",
]

