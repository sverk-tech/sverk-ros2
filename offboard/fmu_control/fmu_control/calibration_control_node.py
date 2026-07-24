#!/usr/bin/env python3
"""
PX4 calibration, arm/disarm, reboot and param get/set/reset — all over uXRCE-DDS.

Commands go to /fmu/in/vehicle_command (calibration/arm/disarm/kill/reboot).
Parameters go through the custom param_dds_bridge PX4 module by NAME:
  publish  ParamBridgeRequest  -> /fmu/in/param_bridge_request
  receive  ParamBridgeResponse <- /fmu/out/param_bridge_response
No MAVLink link is required — everything runs on the single DDS/UART transport.

See: https://docs.px4.io/main/en/middleware/dds_topics
     px4_msgs VehicleCommand.msg
       VEHICLE_CMD_PREFLIGHT_CALIBRATION    = 241
       VEHICLE_CMD_COMPONENT_ARM_DISARM     = 400
       VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN= 246
       VEHICLE_CMD_DO_FLIGHTTERMINATION     = 185
"""
import collections
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleCommand, ParamBridgeRequest, ParamBridgeResponse,
    VehicleStatus, LogMessage, LogDownloadRequest, LogDownloadResponse)
from std_srvs.srv import Trigger
from std_msgs.msg import UInt8
from offboard_interfaces.srv import (
    GetParam, SetParam, ResetParam, BackupParams, MotorTest)
from offboard_interfaces.msg import Param
from offboard_interfaces.action import Calibrate, DownloadLog


CALIBRATION_GYRO = 0
CALIBRATION_MAG = 1
CALIBRATION_BARO = 2
CALIBRATION_ACCEL = 3
CALIBRATION_LEVEL = 4
CALIBRATION_ESC = 5

CALIBRATION_NAMES = {
    CALIBRATION_GYRO: 'gyro',
    CALIBRATION_MAG: 'mag',
    CALIBRATION_BARO: 'baro',
    CALIBRATION_ACCEL: 'accel',
    CALIBRATION_LEVEL: 'level',
    CALIBRATION_ESC: 'esc',
}

CALIBRATION_PARAM_MAP = {
    CALIBRATION_GYRO: (1, 1.0),
    CALIBRATION_MAG: (2, 1.0),
    CALIBRATION_BARO: (3, 1.0),
    CALIBRATION_ACCEL: (5, 1.0),
    CALIBRATION_LEVEL: (5, 2.0),
    CALIBRATION_ESC: (7, 1.0),
}

MAV_MODE_FLAG_FORCE = 21196

NAME_LEN = 17  # ParamBridge char[17]
PARAM_TIMEOUT = 3.0
PARAM_RETRIES = 3

PARAM_TYPE_NAMES = {
    ParamBridgeResponse.TYPE_INT32: 'INT32',
    ParamBridgeResponse.TYPE_FLOAT: 'FLOAT',
    ParamBridgeResponse.TYPE_UNKNOWN: 'UNKNOWN',
}

# --- Log download (log_download_bridge) ---
# Ack-driven windowed pull, same algorithm validated in
# offboard/fmu_control/test/log_download_receiver.py: at most LOG_DL_WINDOW
# requests in flight, a freed slot is refilled immediately, an unanswered
# chunk is retried after LOG_DL_CHUNK_TIMEOUT. This keeps the send rate
# self-limited to what the best-effort UART/DDS link can actually drain,
# instead of flooding it (flooding was shown to collapse the bridge).
#
# A sweep on real hardware (16/20/24/32/48, 300s each) found throughput
# plateaus at ~26.8 KB/s starting at window=16 and does NOT improve with a
# bigger window (the real limit is UART/DDS processing overhead, not
# request concurrency) — larger windows only add MicroXRCEAgent CPU
# (+0.2%/step, 4.6%->5.5% at window=48) with zero speed benefit.
#
# LOG_DL_WINDOW=12 (below that plateau) trades a bit of that throughput for
# extra margin: a real download at window=16 was followed by the FMU's
# incoming DDS path going completely dead (uxrce_dds_client rx=0 B/s, one
# client-loop stall of ~28s in its own stats) after back-to-back downloads
# with the rest of the perception stack (camera/aruco) also running —
# needed an FMU reboot to recover. The likely trigger was repeated
# back-to-back transfers plus concurrent DDS load rather than window size
# alone (see the soak-test finding: repeated downloads without a cooldown
# degrade the bridge even at a fixed window), but a smaller window lowers
# peak concurrency for cheap insurance until that congestion-recovery gap
# is actually fixed firmware-side.
LOG_DL_CHUNK = 240              # LogDownloadResponse.data length
LOG_DL_WINDOW = 12
LOG_DL_CHUNK_TIMEOUT = 0.3      # s, before a chunk is re-requested
LOG_DL_INFO_RETRIES = 15
LOG_DL_INFO_TIMEOUT = 5.0       # s, total budget for OP_INFO retries
LOG_DL_TICK = 0.005             # s, poll period for the window/timeout loop

# Overall download deadline, auto-computed from the reported file size when
# the goal's timeout_s is 0 (see _dl_execute). Measured throughput on the
# real UART/DDS link is ~20 KB/s; assume a conservative floor so normal-sized
# logs (a few MB) don't time out mid-transfer, but still cap the worst case
# so a stuck/slow download can't hang the action forever.
LOG_DL_MIN_TIME = 60.0                  # s, floor for tiny logs
LOG_DL_THROUGHPUT_FLOOR = 10.0 * 1024   # bytes/s, conservative (measured ~20 KB/s)
LOG_DL_TIME_MARGIN = 2.0                # safety factor over the floor throughput
LOG_DL_MAX_TIME_CAP = 1800.0            # s, hard cap for auto-computed deadlines


def name_to_field(name: str):
    """Python str -> PX4 char[17] (list of char codes, null-padded)."""
    raw = name.encode('ascii')[:NAME_LEN - 1]
    return list(raw) + [0] * (NAME_LEN - len(raw))


def field_to_name(field) -> str:
    b = bytes(int(c) & 0xFF for c in field)
    return b.split(b'\x00', 1)[0].decode('ascii', 'replace')


# --- Calibration over Action -------------------------------------------------

ESC_CONFIRM_PHRASE = 'I_UNDERSTAND_WHAT_I_AM_DOING'

# Action sensor id -> (name, PREFLIGHT_CALIBRATION param slot, value, timeout s, steps)
CAL_SENSORS = {
    Calibrate.Goal.SENSOR_GYRO:  ('gyro',  1, 1.0,  30.0, 1),
    Calibrate.Goal.SENSOR_MAG:   ('mag',   2, 1.0, 180.0, 1),
    Calibrate.Goal.SENSOR_BARO:  ('baro',  3, 1.0,  30.0, 1),
    Calibrate.Goal.SENSOR_ACCEL: ('accel', 5, 1.0, 180.0, 6),
    Calibrate.Goal.SENSOR_LEVEL: ('level', 5, 2.0,  40.0, 1),
    Calibrate.Goal.SENSOR_ESC:   ('esc',   7, 1.0,  90.0, 1),
}

_CAL_SIDES = ('down', 'up', 'left', 'right', 'front', 'back')


def parse_cal_text(text: str):
    """Parse a PX4 [cal] log line into a structured event.

    Returns dict {type, progress, orientation, instruction, done, success}
    or None if the line is not a [cal] message.
    """
    if '[cal]' not in text.lower():
        return None

    low = text.lower()
    # Strip the "[module] [cal] " prefix for a clean UI instruction.
    clean = re.sub(r'.*\[cal\]\s*', '', text, flags=re.IGNORECASE).strip()
    ev = {'type': 'info', 'progress': None, 'orientation': 'none',
          'instruction': clean, 'side': None, 'done': False, 'success': True}

    m = re.search(r'progress\s*<?(\d+)>?', low)
    if m:
        ev['type'] = 'progress'
        ev['progress'] = min(1.0, int(m.group(1)) / 100.0)
        return ev

    if re.search(r'calibration (done|complete|success)', low):
        ev['type'] = 'done'
        ev['done'] = True
        ev['progress'] = 1.0
        ev['instruction'] = 'Calibration complete'
        return ev

    if re.search(r'calibration (failed|error|denied|aborted)', low):
        ev['type'] = 'failed'
        ev['done'] = True
        ev['success'] = False
        ev['instruction'] = 'Calibration failed'
        return ev

    for side in _CAL_SIDES:
        if f'measuring {side} side' in low or f'{side} orientation detected' in low:
            ev['type'] = 'orientation'
            ev['orientation'] = side
            ev['side'] = side
            ev['instruction'] = f'Hold still: {side} side'
            return ev
        if f'{side} side done' in low:
            ev['type'] = 'side_done'
            ev['side'] = side
            ev['instruction'] = f'{side} side done — rotate to a different side'
            return ev

    return ev


class CalibrationControlNode(Node):
    def __init__(self):
        super().__init__('fmu_control')

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        resp_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Separate callback groups so blocking param calls never stall the
        # safety services (arm/disarm/kill).
        self._safety_cbg = MutuallyExclusiveCallbackGroup()
        self._param_cbg = MutuallyExclusiveCallbackGroup()
        self._resp_cbg = MutuallyExclusiveCallbackGroup()

        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', cmd_qos)

        # --- Safety / command services ---
        self._arm_srv = self.create_service(
            Trigger, '/fmu_control/arm', self._handle_arm,
            callback_group=self._safety_cbg)
        self._disarm_srv = self.create_service(
            Trigger, '/fmu_control/disarm', self._handle_disarm,
            callback_group=self._safety_cbg)
        self._force_disarm_srv = self.create_service(
            Trigger, '/fmu_control/force_disarm', self._handle_force_disarm,
            callback_group=self._safety_cbg)
        self._kill_switch_srv = self.create_service(
            Trigger, '/fmu_control/kill_switch', self._handle_kill_switch,
            callback_group=self._safety_cbg)
        self._reboot_srv = self.create_service(
            Trigger, '/fmu_control/reboot', self._handle_reboot,
            callback_group=self._safety_cbg)

        # --- Parameter services (over param_dds_bridge) ---
        self._get_param_srv = self.create_service(
            GetParam, '/fmu_control/get_param', self._handle_get_param,
            callback_group=self._param_cbg)
        self._set_param_srv = self.create_service(
            SetParam, '/fmu_control/set_param', self._handle_set_param,
            callback_group=self._param_cbg)
        self._reset_param_srv = self.create_service(
            ResetParam, '/fmu_control/reset_param', self._handle_reset_param,
            callback_group=self._param_cbg)

        self._param_req_pub = self.create_publisher(
            ParamBridgeRequest, '/fmu/in/param_bridge_request', cmd_qos)
        self._param_resp_sub = self.create_subscription(
            ParamBridgeResponse, '/fmu/out/param_bridge_response',
            self._on_param_response, resp_qos, callback_group=self._resp_cbg)

        # Request/response correlation (param services are serialized, so a
        # single pending slot is enough).
        self._resp_lock = threading.Lock()
        self._pending_name = None
        self._pending_index = None
        self._last_resp = None
        self._resp_event = threading.Event()

        self._calibrate_sub = self.create_subscription(
            UInt8, '/fmu_control/request_calibration',
            self._handle_calibration_request, 10,
            callback_group=self._safety_cbg)

        # --- Calibration Action (with live [cal] feedback) ---
        self._cal_cbg = ReentrantCallbackGroup()
        self._cal_lock = threading.Lock()
        self._cal_events = []          # buffered parsed [cal] events
        self._arming_state = 0         # from vehicle_status (1=disarmed, 2=armed)

        self.create_subscription(
            LogMessage, '/fmu/out/log_message',
            self._on_log_message, resp_qos, callback_group=self._cal_cbg)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._on_vehicle_status, resp_qos, callback_group=self._cal_cbg)

        self._calibrate_action = ActionServer(
            self, Calibrate, '/fmu_control/calibrate',
            execute_callback=self._cal_execute,
            goal_callback=self._cal_goal,
            cancel_callback=self._cal_cancel,
            callback_group=self._cal_cbg)

        # --- Log download Action (log_download_bridge, windowed pull) ---
        self._dl_cbg = ReentrantCallbackGroup()
        self._dl_lock = threading.Lock()
        self._dl_info = None
        self._dl_info_event = threading.Event()
        self._dl_buf = None
        self._dl_got = None
        self._dl_n_got = 0

        self._dl_req_pub = self.create_publisher(
            LogDownloadRequest, '/fmu/in/log_download_request', cmd_qos)
        self.create_subscription(
            LogDownloadResponse, '/fmu/out/log_download_response',
            self._on_log_download_response, resp_qos,
            callback_group=self._dl_cbg)

        self._download_log_action = ActionServer(
            self, DownloadLog, '/fmu_control/download_log',
            execute_callback=self._dl_execute,
            goal_callback=self._dl_goal,
            cancel_callback=self._dl_cancel,
            callback_group=self._dl_cbg)

        # --- Backup (GET_INDEX) + motor test services ---
        self._backup_srv = self.create_service(
            BackupParams, '/fmu_control/backup_params',
            self._handle_backup_params, callback_group=self._param_cbg)
        self._motor_test_srv = self.create_service(
            MotorTest, '/fmu_control/motor_test',
            self._handle_motor_test, callback_group=self._safety_cbg)
        self._stop_motors_srv = self.create_service(
            Trigger, '/fmu_control/stop_all_motors',
            self._handle_stop_motors, callback_group=self._safety_cbg)

        self.get_logger().info(
            'fmu_control: arm/disarm/force_disarm/kill_switch/reboot, '
            'get/set/reset_param, backup_params, motor_test/stop_all_motors, '
            'action /fmu_control/calibrate (0=gyro,1=mag,2=baro,3=accel,'
            '4=level,5=esc; ESC needs confirm phrase), '
            'action /fmu_control/download_log (path -> latest .ulg from SD)'
        )

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_vehicle_command(self, command: int, **params):
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_pub.publish(msg)

    def _handle_arm(self, _request, response):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        response.success = True
        response.message = 'Arm command sent'
        return response

    def _handle_disarm(self, _request, response):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        response.success = True
        response.message = 'Disarm command sent'
        return response

    def _handle_force_disarm(self, _request, response):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0, param2=float(MAV_MODE_FLAG_FORCE))
        self.get_logger().info('Force disarm command sent')
        response.success = True
        response.message = 'Force disarm command sent'
        return response

    def _handle_kill_switch(self, _request, response):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, param1=1.0)
        self.get_logger().info('Kill switch (flight termination) sent')
        response.success = True
        response.message = 'Kill switch sent'
        return response

    def _handle_reboot(self, _request, response):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1.0)
        self.get_logger().info('Reboot command sent')
        response.success = True
        response.message = 'Reboot command sent'
        return response

    # ------------------------------------------------------------------
    # Parameters via param_dds_bridge
    # ------------------------------------------------------------------

    def _on_param_response(self, msg: ParamBridgeResponse):
        with self._resp_lock:
            match = False
            if (self._pending_name is not None
                    and field_to_name(msg.name) == self._pending_name):
                match = True
            elif (self._pending_index is not None
                    and int(msg.index) == self._pending_index):
                match = True
            if match:
                self._last_resp = msg
                self._resp_event.set()

    def _param_get_index(self, index):
        """Reliable read of a parameter by index (for backup), with retries."""
        req = ParamBridgeRequest()
        req.operation = ParamBridgeRequest.OPERATION_GET_INDEX
        req.name = name_to_field('')
        req.index = int(index)

        with self._resp_lock:
            self._pending_index = int(index)
            self._pending_name = None
            self._last_resp = None
            self._resp_event.clear()
        try:
            for _ in range(PARAM_RETRIES):
                req.timestamp = self._timestamp_us()
                self._param_req_pub.publish(req)
                if self._resp_event.wait(PARAM_TIMEOUT / PARAM_RETRIES):
                    break
        finally:
            with self._resp_lock:
                resp = self._last_resp
                self._pending_index = None
        return resp

    def _param_request(self, operation, name='', value=None):
        """Publish a ParamBridgeRequest and wait for the matching response.

        BEST_EFFORT transport may drop a packet, so retry a few times.
        Returns the ParamBridgeResponse or None on timeout.
        """
        req = ParamBridgeRequest()
        req.operation = operation
        req.name = name_to_field(name)
        if value is not None:
            # The bridge picks int/float by the parameter's stored type,
            # so fill both.
            try:
                req.int_value = int(round(float(value)))
            except (TypeError, ValueError):
                req.int_value = 0
            req.float_value = float(value)

        with self._resp_lock:
            self._pending_name = name
            self._last_resp = None
            self._resp_event.clear()

        try:
            for _ in range(PARAM_RETRIES):
                req.timestamp = self._timestamp_us()
                self._param_req_pub.publish(req)
                if self._resp_event.wait(PARAM_TIMEOUT / PARAM_RETRIES):
                    break
        finally:
            with self._resp_lock:
                resp = self._last_resp
                self._pending_name = None
        return resp

    def _handle_get_param(self, request, response):
        resp = self._param_request(ParamBridgeRequest.OPERATION_GET, request.name)
        if resp is None:
            response.success = False
            response.message = f'Timeout reading {request.name} (param_dds_bridge?)'
            return response
        if not resp.found:
            response.success = False
            response.message = f'Parameter {request.name} not found'
            return response

        is_int = (resp.param_type == ParamBridgeResponse.TYPE_INT32)
        value = resp.int_value if is_int else resp.float_value
        response.success = bool(resp.success)
        response.value = float(value)
        response.param_type = int(resp.param_type)
        tname = PARAM_TYPE_NAMES.get(resp.param_type, str(resp.param_type))
        response.message = f'{request.name} = {value} ({tname})'
        self.get_logger().info(f'GetParam: {response.message}')
        return response

    def _handle_set_param(self, request, response):
        resp = self._param_request(
            ParamBridgeRequest.OPERATION_SET, request.name, request.value)
        if resp is None:
            response.success = False
            response.message = f'Timeout setting {request.name} (param_dds_bridge?)'
            return response
        if not resp.found:
            response.success = False
            response.message = f'Parameter {request.name} not found'
            return response

        is_int = (resp.param_type == ParamBridgeResponse.TYPE_INT32)
        readback = resp.int_value if is_int else resp.float_value
        if resp.success and abs(float(readback) - float(request.value)) < 1e-3:
            response.success = True
            response.message = f'{request.name} set to {readback}'
            self.get_logger().info(f'SetParam: {response.message}')
        else:
            # PX4 may clip to a valid range — report the actual stored value.
            response.success = False
            response.message = (
                f'{request.name} readback mismatch: '
                f'expected {request.value}, got {readback}')
            self.get_logger().warn(f'SetParam: {response.message}')
        return response

    def _handle_reset_param(self, request, response):
        name = request.name or ''
        if name in ('', '*'):
            resp = self._param_request(ParamBridgeRequest.OPERATION_RESET_ALL)
            label = 'all parameters'
        else:
            resp = self._param_request(ParamBridgeRequest.OPERATION_RESET, name)
            label = name

        if resp is None:
            response.success = False
            response.message = f'Timeout resetting {label} (param_dds_bridge?)'
            return response

        response.success = bool(resp.success)
        response.message = (f'{label} reset'
                            if resp.success else f'Reset failed for {label}')
        self.get_logger().info(f'ResetParam: {response.message}')
        return response

    def _handle_calibration_request(self, msg: UInt8):
        cal_type = msg.data
        mapping = CALIBRATION_PARAM_MAP.get(cal_type)
        if mapping is None:
            self.get_logger().warn(
                f'Unknown calibration type {cal_type}, use 0..5 '
                '(0=gyro,1=mag,2=baro,3=accel,4=level,5=esc)'
            )
            return
        param_index, value = mapping
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION,
            **{f'param{param_index}': value},
        )
        self.get_logger().info(
            f'Calibration command sent: {CALIBRATION_NAMES[cal_type]} '
            f'(param{param_index}={value})'
        )

    # ------------------------------------------------------------------
    # Calibration Action (live [cal] feedback from /fmu/out/log_message)
    # ------------------------------------------------------------------

    def _on_vehicle_status(self, msg: VehicleStatus):
        self._arming_state = int(msg.arming_state)

    def _on_log_message(self, msg: LogMessage):
        # LogMessage.text is char[127] -> in rclpy a byte array; decode to str.
        try:
            text = bytes(int(c) & 0xFF for c in msg.text).split(b'\x00', 1)[0] \
                .decode('ascii', 'replace')
        except TypeError:
            text = str(msg.text)
        ev = parse_cal_text(text)
        if ev is not None:
            with self._cal_lock:
                self._cal_events.append(ev)

    def _cal_goal(self, goal_request):
        sensor = goal_request.sensor
        info = CAL_SENSORS.get(sensor)
        if info is None:
            self.get_logger().warn(f'Calibrate: unknown sensor {sensor}')
            return GoalResponse.REJECT
        if sensor == Calibrate.Goal.SENSOR_ESC \
                and goal_request.confirm != ESC_CONFIRM_PHRASE:
            self.get_logger().warn(
                'Calibrate: ESC rejected — confirm phrase required '
                f'(confirm="{ESC_CONFIRM_PHRASE}")')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cal_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _cal_execute(self, goal_handle):
        sensor = goal_handle.request.sensor
        name, slot, value, timeout, total_steps = CAL_SENSORS[sensor]
        result = Calibrate.Result()

        with self._cal_lock:
            self._cal_events.clear()

        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION,
            **{f'param{slot}': value})
        self.get_logger().info(f'Calibrate: started {name}')

        progress = 0.0
        orientation = 'none'
        done_sides = set()          # dedupe: PX4 emits "side done" twice
        deadline = time.time() + timeout

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                # PX4 cancels calibration on an all-zero PREFLIGHT_CALIBRATION.
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION)
                goal_handle.canceled()
                result.success = False
                result.message = f'{name} calibration cancelled'
                self.get_logger().info(result.message)
                return result

            with self._cal_lock:
                events = self._cal_events
                self._cal_events = []

            for ev in events:
                if ev['progress'] is not None:
                    progress = ev['progress']
                if ev['orientation'] != 'none':
                    orientation = ev['orientation']
                if ev['type'] == 'side_done' and ev.get('side'):
                    done_sides.add(ev['side'])

                fb = Calibrate.Feedback()
                fb.progress = float(progress)
                fb.instruction = ev['instruction']
                fb.orientation = orientation
                fb.step = min(len(done_sides), total_steps)
                fb.total_steps = total_steps
                goal_handle.publish_feedback(fb)

                if ev['done']:
                    if ev['success']:
                        goal_handle.succeed()
                        result.success = True
                        result.message = f'{name} calibration complete'
                    else:
                        goal_handle.abort()
                        result.success = False
                        result.message = f'{name} calibration failed'
                    self.get_logger().info(f'Calibrate: {result.message}')
                    return result

            time.sleep(0.05)

        goal_handle.abort()
        result.success = False
        result.message = f'{name} calibration timed out after {timeout:.0f}s'
        self.get_logger().warn(f'Calibrate: {result.message}')
        return result

    # ------------------------------------------------------------------
    # Backup all parameters (reliable, by index)
    # ------------------------------------------------------------------

    def _handle_backup_params(self, _request, response):
        first = self._param_get_index(0)
        if first is None:
            response.success = False
            response.message = 'No response for index 0 (param_dds_bridge?)'
            return response

        total = int(first.total)
        response.total = total
        got = {}

        def store(resp):
            p = Param()
            p.name = field_to_name(resp.name)
            p.param_type = int(resp.param_type)
            p.value = float(resp.int_value
                            if resp.param_type == ParamBridgeResponse.TYPE_INT32
                            else resp.float_value)
            got[int(resp.index)] = p

        store(first)
        for i in range(1, total):
            r = self._param_get_index(i)
            if r is not None:
                store(r)

        # Second pass: fill any gaps left by dropped packets.
        missing = [i for i in range(total) if i not in got]
        for i in missing:
            r = self._param_get_index(i)
            if r is not None:
                store(r)

        response.params = [got[i] for i in sorted(got)]
        response.success = (len(got) == total)
        response.message = (
            f'backed up {len(got)}/{total} parameters'
            + ('' if response.success else f', still missing {total - len(got)}'))
        self.get_logger().info(f'BackupParams: {response.message}')
        return response

    # ------------------------------------------------------------------
    # Log download Action (log_download_bridge: reliable windowed pull of
    # the latest .ulg from the SD card, no MAVLink/physical access needed)
    # ------------------------------------------------------------------

    def _on_log_download_response(self, msg: LogDownloadResponse):
        if msg.operation == LogDownloadRequest.OP_INFO:
            self._dl_info = msg
            self._dl_info_event.set()
            return
        if msg.operation != LogDownloadRequest.OP_READ:
            return
        with self._dl_lock:
            if self._dl_got is None or msg.result != LogDownloadResponse.RESULT_OK:
                return
            idx = int(msg.offset) // LOG_DL_CHUNK
            n = int(msg.length)
            if 0 <= idx < len(self._dl_got) and not self._dl_got[idx] and n > 0:
                data = bytes(int(b) & 0xFF for b in msg.data[:n])
                self._dl_buf[msg.offset:msg.offset + len(data)] = data
                self._dl_got[idx] = True
                self._dl_n_got += 1

    def _dl_fire_info(self):
        req = LogDownloadRequest()
        req.timestamp = self._timestamp_us()
        req.operation = LogDownloadRequest.OP_INFO
        self._dl_req_pub.publish(req)

    def _dl_fire_read(self, offset, length):
        req = LogDownloadRequest()
        req.timestamp = self._timestamp_us()
        req.operation = LogDownloadRequest.OP_READ
        req.offset = int(offset)
        req.length = int(length)
        self._dl_req_pub.publish(req)

    def _dl_goal(self, goal_request):
        if not goal_request.path:
            self.get_logger().warn('DownloadLog: empty path rejected')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _dl_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _dl_execute(self, goal_handle):
        path = goal_handle.request.path
        result = DownloadLog.Result()

        # --- OP_INFO handshake: which log, how big ---
        self._dl_info = None
        self._dl_info_event.clear()
        for _ in range(LOG_DL_INFO_RETRIES):
            self._dl_fire_info()
            if self._dl_info_event.wait(LOG_DL_INFO_TIMEOUT / LOG_DL_INFO_RETRIES):
                break
        info = self._dl_info

        if info is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Timeout waiting for log_download_bridge INFO (module running?)'
            self.get_logger().warn(f'DownloadLog: {result.message}')
            return result
        if info.result == LogDownloadResponse.RESULT_NO_FILE:
            goal_handle.abort()
            result.success = False
            result.message = 'No .ulg found on the SD card'
            self.get_logger().warn(f'DownloadLog: {result.message}')
            return result
        if info.result != LogDownloadResponse.RESULT_OK:
            goal_handle.abort()
            result.success = False
            result.message = f'INFO error: result={info.result}'
            self.get_logger().warn(f'DownloadLog: {result.message}')
            return result

        name = bytes(int(c) & 0xFF for c in info.name).split(b'\x00', 1)[0] \
            .decode('ascii', 'replace')
        size = int(info.file_size)
        nchunks = (size + LOG_DL_CHUNK - 1) // LOG_DL_CHUNK
        result.log_name = name
        result.file_size = size

        requested_timeout = float(goal_handle.request.timeout_s)
        if requested_timeout > 0.0:
            budget = requested_timeout
        else:
            budget = min(
                LOG_DL_MAX_TIME_CAP,
                max(LOG_DL_MIN_TIME,
                    size / LOG_DL_THROUGHPUT_FLOOR * LOG_DL_TIME_MARGIN))
        self.get_logger().info(
            f'DownloadLog: {name} ({size} bytes, {nchunks} chunks) -> {path} '
            f'(budget {budget:.0f}s)')

        with self._dl_lock:
            self._dl_buf = bytearray(size)
            self._dl_got = [False] * nchunks
            self._dl_n_got = 0

        # --- ack-driven windowed pull ---
        queue = collections.deque(range(nchunks))
        inflight = {}
        deadline = time.time() + budget
        last_feedback = 0.0

        while True:
            with self._dl_lock:
                done = self._dl_n_got
            if done >= nchunks:
                break
            if time.time() >= deadline:
                break

            if goal_handle.is_cancel_requested:
                with self._dl_lock:
                    done = self._dl_n_got
                goal_handle.canceled()
                result.success = False
                result.bytes_received = done * LOG_DL_CHUNK
                result.message = f'{name} download cancelled ({done}/{nchunks} chunks)'
                self.get_logger().info(f'DownloadLog: {result.message}')
                return result

            now = time.time()
            with self._dl_lock:
                got = self._dl_got
                while len(inflight) < LOG_DL_WINDOW and queue:
                    idx = queue.popleft()
                    if got[idx]:
                        continue
                    off = idx * LOG_DL_CHUNK
                    self._dl_fire_read(off, min(LOG_DL_CHUNK, size - off))
                    inflight[idx] = now

            for idx in list(inflight.keys()):
                with self._dl_lock:
                    got_it = self._dl_got[idx]
                if got_it:
                    del inflight[idx]
                elif now - inflight[idx] > LOG_DL_CHUNK_TIMEOUT:
                    del inflight[idx]
                    queue.append(idx)

            if now - last_feedback > 0.5:
                with self._dl_lock:
                    done = self._dl_n_got
                fb = DownloadLog.Feedback()
                fb.progress = done / max(nchunks, 1)
                fb.bytes_received = done * LOG_DL_CHUNK
                fb.file_size = size
                fb.chunks_received = done
                fb.chunks_total = nchunks
                goal_handle.publish_feedback(fb)
                last_feedback = now

            time.sleep(LOG_DL_TICK)

        with self._dl_lock:
            done = self._dl_n_got
            data = bytes(self._dl_buf)

        complete = (done >= nchunks)
        result.bytes_received = size if complete else done * LOG_DL_CHUNK

        if not complete:
            goal_handle.abort()
            result.success = False
            result.message = (
                f'{name} incomplete: {done}/{nchunks} chunks missing '
                f'{nchunks - done} after {budget:.0f}s')
            self.get_logger().warn(f'DownloadLog: {result.message}')
            return result

        try:
            with open(path, 'wb') as f:
                f.write(data)
        except OSError as exc:
            goal_handle.abort()
            result.success = False
            result.message = f'Failed to write {path}: {exc}'
            self.get_logger().error(f'DownloadLog: {result.message}')
            return result

        goal_handle.succeed()
        result.success = True
        result.message = f'{name} downloaded fully ({size} bytes) -> {path}'
        self.get_logger().info(f'DownloadLog: {result.message}')
        return result

    # ------------------------------------------------------------------
    # Motor test (props off!) — VEHICLE_CMD_DO_MOTOR_TEST
    # ------------------------------------------------------------------

    def _handle_motor_test(self, request, response):
        if self._arming_state == 2:  # ARMED
            response.success = False
            response.message = 'Refused: vehicle is ARMED'
            return response
        throttle = max(0.0, min(1.0, float(request.throttle)))
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
            param1=float(request.motor_id),   # motor number (1-based)
            param2=0.0,                        # throttle type: 0 = percent
            param3=throttle * 100.0,           # throttle value
            param4=float(request.timeout_s),   # timeout (s)
            param5=0.0)                        # motor count (0 = single)
        response.success = True
        response.message = (f'motor {request.motor_id} @ {throttle*100:.0f}% '
                            f'for {request.timeout_s:.1f}s')
        self.get_logger().info(f'MotorTest: {response.message}')
        return response

    def _handle_stop_motors(self, _request, response):
        for m in range(1, 9):
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                param1=float(m), param2=0.0, param3=0.0, param4=0.0, param5=0.0)
        response.success = True
        response.message = 'stop sent to motors 1..8'
        self.get_logger().info('MotorTest: stop all')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
