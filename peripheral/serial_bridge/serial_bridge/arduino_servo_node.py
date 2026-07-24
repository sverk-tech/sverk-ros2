#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from time import sleep
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from .transport import SerialConnectionError
from .transport import SerialTransport
from .transport import SerialTransportConfig


CMD_CENTER = 0xFD
CMD_ENABLE = 0xFE
CMD_DISABLE = 0xFF


@dataclass(slots=True)
class ServoStatus:
    enabled: bool = True
    angle_deg: float = 0.0
    connected: bool = False


@dataclass(slots=True)
class PendingCommand:
    payload: bytes
    description: str


class ArduinoServoNode(Node):
    """Bridge ROS 2 servo commands to a compact single-byte Arduino protocol."""

    def __init__(self) -> None:
        super().__init__("serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("poll_rate_hz", 20.0)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("center_angle_deg", 90.0)
        self.declare_parameter("startup_delay_sec", 2.5)
        self.declare_parameter("use_feedback", False)
        self.declare_parameter("command_angle_deg", 0.0)
        self.declare_parameter("command_repeats", 3)
        self.declare_parameter("command_repeat_delay_sec", 0.05)

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        self._min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self._max_angle_deg = float(self.get_parameter("max_angle_deg").value)
        poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)
        reconnect_interval_sec = float(self.get_parameter("reconnect_interval_sec").value)
        self._center_angle_deg = float(self.get_parameter("center_angle_deg").value)
        self._startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self._use_feedback = bool(self.get_parameter("use_feedback").value)
        self._command_repeats = int(self.get_parameter("command_repeats").value)
        self._command_repeat_delay_sec = float(
            self.get_parameter("command_repeat_delay_sec").value
        )

        if self._min_angle_deg < 0.0 or self._max_angle_deg > 180.0:
            raise ValueError("Angle range must stay within 0..180 degrees")
        if self._min_angle_deg >= self._max_angle_deg:
            raise ValueError("min_angle_deg must be smaller than max_angle_deg")
        if poll_rate_hz <= 0.0:
            raise ValueError("poll_rate_hz must be positive")
        if self._startup_delay_sec < 0.0:
            raise ValueError("startup_delay_sec must not be negative")
        if self._command_repeats <= 0:
            raise ValueError("command_repeats must be positive")
        if self._command_repeat_delay_sec < 0.0:
            raise ValueError("command_repeat_delay_sec must not be negative")

        if self._use_feedback:
            self.get_logger().warning(
                "The demo Arduino servo protocol is write-only; forcing use_feedback=false."
            )
            self._use_feedback = False

        self._transport = SerialTransport(
            SerialTransportConfig(
                port=port,
                baudrate=baudrate,
                reconnect_interval=reconnect_interval_sec,
            )
        )
        self._status = ServoStatus()
        self._last_connection_state = False
        self._connected_since = 0.0
        self._deferred_command: Optional[PendingCommand] = None

        self._angle_pub = self.create_publisher(Float32, "~/current_angle_deg", 10)
        self._enabled_pub = self.create_publisher(Bool, "~/enabled", 10)
        self._connected_pub = self.create_publisher(Bool, "~/connected", 10)

        self.create_subscription(Float32, "~/target_angle_deg", self._handle_target_angle, 10)

        self.create_service(SetBool, "~/enable", self._handle_enable)
        self.create_service(Trigger, "~/center", self._handle_center)
        self.create_service(Trigger, "~/refresh_status", self._handle_refresh_status)
        self.add_on_set_parameters_callback(self._handle_set_parameters)

        self.create_timer(1.0 / poll_rate_hz, self._poll_serial)

        self.get_logger().info(
            "Arduino servo bridge ready on %s @ %d baud using compact byte protocol"
            % (port, baudrate)
        )

    def _clamp_angle(self, angle_deg: float) -> float:
        return max(self._min_angle_deg, min(self._max_angle_deg, angle_deg))

    def _publish_status(self) -> None:
        angle_msg = Float32()
        angle_msg.data = float(self._status.angle_deg)
        self._angle_pub.publish(angle_msg)

        enabled_msg = Bool()
        enabled_msg.data = self._status.enabled
        self._enabled_pub.publish(enabled_msg)

        connected_msg = Bool()
        connected_msg.data = self._status.connected
        self._connected_pub.publish(connected_msg)

    def _update_connection_state(self, connected: bool) -> None:
        if connected and not self._status.connected:
            self._connected_since = monotonic()

        if not connected and self._status.connected:
            self._connected_since = 0.0

        self._status.connected = connected
        if connected != self._last_connection_state:
            if connected:
                self.get_logger().info("Connected to Arduino servo controller.")
            else:
                self.get_logger().warning("Disconnected from Arduino servo controller.")
            self._last_connection_state = connected
        self._publish_status()

    def _is_startup_delay_complete(self) -> bool:
        if not self._status.connected:
            return False
        return (monotonic() - self._connected_since) >= self._startup_delay_sec

    def _write_payload_now(self, payload: bytes, description: str) -> bool:
        try:
            for repeat_index in range(self._command_repeats):
                self._transport.write_bytes(payload)
                if repeat_index < (self._command_repeats - 1) and self._command_repeat_delay_sec > 0.0:
                    sleep(self._command_repeat_delay_sec)
        except SerialConnectionError as exc:
            self._update_connection_state(False)
            self.get_logger().warning(f"Failed to send {description}: {exc}")
            return False

        self._update_connection_state(True)
        return True

    def _send_payload(self, payload: bytes, description: str) -> bool:
        pending = PendingCommand(payload=payload, description=description)

        if not self._status.connected:
            self._deferred_command = pending
            connected = self._transport.connect()
            self._update_connection_state(connected)
            if not connected:
                self.get_logger().warning(
                    f"Deferred {description} until Arduino serial port becomes available."
                )
                return False

        if not self._is_startup_delay_complete():
            self._deferred_command = pending
            self.get_logger().info(
                f"Deferred {description} until controller startup delay completes."
            )
            return True

        self._deferred_command = None
        return self._write_payload_now(payload, description)

    def _poll_serial(self) -> None:
        connected = self._transport.connect()
        self._update_connection_state(connected)
        if not connected:
            return

        self._transport.read_available_bytes()

        if self._deferred_command and self._is_startup_delay_complete():
            deferred_command = self._deferred_command
            self._deferred_command = None
            self.get_logger().info(f"Sending deferred {deferred_command.description}.")
            self._write_payload_now(
                deferred_command.payload,
                deferred_command.description,
            )

    def _request_angle(self, angle_deg: float, source: str) -> None:
        angle_deg = round(self._clamp_angle(float(angle_deg)))
        payload = bytes([int(angle_deg)])
        description = f"angle {int(angle_deg)} deg"
        if self._send_payload(payload, description):
            self._status.angle_deg = float(angle_deg)
            self._publish_status()
            self.get_logger().info(
                f"Requested servo angle {angle_deg:.0f} deg via {source}"
            )

    def _handle_target_angle(self, msg: Float32) -> None:
        self._request_angle(float(msg.data), source="topic")

    def _handle_set_parameters(self, params) -> SetParametersResult:
        for param in params:
            if param.name != "command_angle_deg":
                continue

            try:
                angle_deg = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason="command_angle_deg must be convertible to float",
                )

            self._request_angle(angle_deg, source="parameter")

        return SetParametersResult(successful=True)

    def _handle_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        payload = bytes([CMD_ENABLE if request.data else CMD_DISABLE])
        description = "enable" if request.data else "disable"
        response.success = self._send_payload(payload, description)
        response.message = "command sent" if response.success else "arduino unavailable"
        if response.success:
            self._status.enabled = bool(request.data)
            self._publish_status()
        return response

    def _handle_center(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = self._send_payload(bytes([CMD_CENTER]), "center")
        response.message = "command sent" if response.success else "arduino unavailable"
        if response.success:
            self._status.angle_deg = float(self._center_angle_deg)
            self._publish_status()
        return response

    def _handle_refresh_status(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        response.message = "write-only protocol; returning cached state"
        self._publish_status()
        return response

    def destroy_node(self) -> bool:
        self._transport.disconnect()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ArduinoServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
