#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, UInt16
from std_srvs.srv import SetBool, Trigger


class ServoNode(Node):
    """Drive one PWM servo via Raspberry Pi hardware PWM."""

    def __init__(self) -> None:
        super().__init__("servo_control")

        self.declare_parameter("pwm_chip", 0)
        self.declare_parameter("pwm_channel", 0)
        self.declare_parameter("frequency_hz", 50)
        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("center_angle_deg", 90.0)
        self.declare_parameter("startup_angle_deg", 90.0)
        self.declare_parameter("min_pulse_width_us", 500.0)
        self.declare_parameter("max_pulse_width_us", 2500.0)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("status_publish_rate_hz", 2.0)

        self._pwm_chip = self.get_parameter("pwm_chip").value
        self._pwm_channel = self.get_parameter("pwm_channel").value
        self._frequency_hz = int(self.get_parameter("frequency_hz").value)
        self._min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self._max_angle_deg = float(self.get_parameter("max_angle_deg").value)
        self._center_angle_deg = float(self.get_parameter("center_angle_deg").value)
        self._startup_angle_deg = float(self.get_parameter("startup_angle_deg").value)
        self._min_pulse_width_us = float(self.get_parameter("min_pulse_width_us").value)
        self._max_pulse_width_us = float(self.get_parameter("max_pulse_width_us").value)
        self._start_enabled = bool(self.get_parameter("start_enabled").value)
        status_publish_rate_hz = float(self.get_parameter("status_publish_rate_hz").value)

        if self._min_angle_deg >= self._max_angle_deg:
            raise ValueError("min_angle_deg must be smaller than max_angle_deg")
        if self._min_pulse_width_us >= self._max_pulse_width_us:
            raise ValueError("min_pulse_width_us must be smaller than max_pulse_width_us")
        if self._frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")

        try:
            from rpi_hardware_pwm import HardwarePWM
        except ImportError as exc:
            raise RuntimeError(
                "rpi_hardware_pwm is not installed. Rebuild the container or host "
                "environment with the PWM dependency."
            ) from exc

        self._pwm = HardwarePWM(
            pwm_channel=self._pwm_channel,
            hz=self._frequency_hz,
            chip=self._pwm_chip,
        )
        self._enabled = False
        self._current_angle_deg: Optional[float] = None
        self._current_pulse_width_us: Optional[float] = None

        self._angle_pub = self.create_publisher(Float32, "~/current_angle_deg", 10)
        self._enabled_pub = self.create_publisher(Bool, "~/enabled", 10)

        self.create_subscription(Float32, "~/target_angle_deg", self._target_angle_callback, 10)
        self.create_subscription(UInt16, "~/target_pulse_width_us", self._target_pulse_callback, 10)

        self.create_service(SetBool, "~/enable", self._handle_enable)
        self.create_service(Trigger, "~/center", self._handle_center)

        if status_publish_rate_hz > 0.0:
            self.create_timer(1.0 / status_publish_rate_hz, self._publish_status)

        if self._start_enabled:
            self._set_enabled(True)
            self._set_angle(self._startup_angle_deg)
        else:
            self.get_logger().info("Servo node started disabled.")

        self.get_logger().info(
            "Servo node ready: chip=%d channel=%d freq=%dHz angle=[%.1f, %.1f] pulse=[%.1f, %.1f]us"
            % (
                self._pwm_chip,
                self._pwm_channel,
                self._frequency_hz,
                self._min_angle_deg,
                self._max_angle_deg,
                self._min_pulse_width_us,
                self._max_pulse_width_us,
            )
        )

    def _clamp_angle(self, angle_deg: float) -> float:
        return max(self._min_angle_deg, min(self._max_angle_deg, angle_deg))

    def _pulse_width_to_duty_cycle(self, pulse_width_us: float) -> float:
        period_us = 1_000_000.0 / float(self._frequency_hz)
        return max(0.0, min(100.0, (pulse_width_us / period_us) * 100.0))

    def _angle_to_pulse_width_us(self, angle_deg: float) -> float:
        clamped_angle = self._clamp_angle(angle_deg)
        angle_span = self._max_angle_deg - self._min_angle_deg
        pulse_span = self._max_pulse_width_us - self._min_pulse_width_us
        return self._min_pulse_width_us + (
            (clamped_angle - self._min_angle_deg) / angle_span
        ) * pulse_span

    def _ensure_enabled(self) -> None:
        if not self._enabled:
            self._set_enabled(True)

    def _set_enabled(self, enabled: bool) -> None:
        if enabled and not self._enabled:
            initial_duty = self._pulse_width_to_duty_cycle(
                self._current_pulse_width_us or self._angle_to_pulse_width_us(self._startup_angle_deg)
            )
            self._pwm.start(initial_duty)
            self._enabled = True
            self.get_logger().info("PWM output enabled.")
            return

        if not enabled and self._enabled:
            self._pwm.stop()
            self._enabled = False
            self.get_logger().info("PWM output disabled.")

    def _apply_pulse_width(self, pulse_width_us: float, angle_deg: Optional[float]) -> None:
        self._ensure_enabled()
        clamped_pulse = max(
            self._min_pulse_width_us, min(self._max_pulse_width_us, pulse_width_us)
        )
        duty_cycle = self._pulse_width_to_duty_cycle(clamped_pulse)
        self._pwm.change_duty_cycle(duty_cycle)
        self._current_pulse_width_us = clamped_pulse
        self._current_angle_deg = angle_deg

    def _set_angle(self, angle_deg: float) -> float:
        clamped_angle = self._clamp_angle(angle_deg)
        pulse_width_us = self._angle_to_pulse_width_us(clamped_angle)
        self._apply_pulse_width(pulse_width_us, clamped_angle)
        return clamped_angle

    def _target_angle_callback(self, msg: Float32) -> None:
        angle_deg = self._set_angle(float(msg.data))
        self.get_logger().info("Set servo angle to %.1f deg" % angle_deg)

    def _target_pulse_callback(self, msg: UInt16) -> None:
        pulse_width_us = float(msg.data)
        self._apply_pulse_width(pulse_width_us, angle_deg=None)
        self.get_logger().info("Set servo pulse width to %.1f us" % self._current_pulse_width_us)

    def _handle_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._set_enabled(request.data)
        response.success = True
        response.message = "enabled" if self._enabled else "disabled"
        return response

    def _handle_center(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        angle_deg = self._set_angle(self._center_angle_deg)
        response.success = True
        response.message = f"centered at {angle_deg:.1f} deg"
        return response

    def _publish_status(self) -> None:
        angle_msg = Float32()
        angle_msg.data = float(self._current_angle_deg or 0.0)
        self._angle_pub.publish(angle_msg)

        enabled_msg = Bool()
        enabled_msg.data = self._enabled
        self._enabled_pub.publish(enabled_msg)

    def destroy_node(self) -> bool:
        try:
            self._set_enabled(False)
        except Exception as exc:  # pragma: no cover - cleanup path
            self.get_logger().warning(f"Failed to stop PWM cleanly: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
