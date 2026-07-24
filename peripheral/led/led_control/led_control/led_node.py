#!/usr/bin/env python3
"""
ROS2 LED node for WS2812 strip.
- Active backend path: ws2812_spidev, fallback to rpi5_ws2812.
- MRAA/UART runtime branches are intentionally commented out for development.
- Service /led_control/set_effect (effect, r, g, b)
- Service /led_control/set_leds (low-level per-LED)
- Topic /led_control/state (current strip state)
- Event visualization from PX4: /fmu/out/vehicle_status, /fmu/out/battery_status, /rosout (config in led_params.yaml)
  See: https://docs.px4.io/main/en/ros2/user_guide, https://docs.px4.io/main/en/middleware/dds_topics
"""

import time
import yaml
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from led_control.ws2812_spidev import Color, WS2812SpiDriver
except ImportError:
    from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver  # type: ignore[no-redef]
### from led_control.ws2812_mraa import Color, WS2812SpiDriver

from led_interfaces.msg import LEDState, LEDStateArray
from led_interfaces.srv import SetLEDEffect, SetLEDs

# Optional: PX4 and rosout for event visualization
try:
    from px4_msgs.msg import VehicleStatus, BatteryStatus
    _PX4_MSGS_AVAILABLE = True
except ImportError:
    _PX4_MSGS_AVAILABLE = False

try:
    from rcl_interfaces.msg import Log
    _RCL_INTERFACES_AVAILABLE = True
except ImportError:
    _RCL_INTERFACES_AVAILABLE = False


# Effect names
EFFECT_FILL = "fill"
EFFECT_BLINK = "blink"
EFFECT_BLINK_FAST = "blink_fast"
EFFECT_FADE = "fade"
EFFECT_WIPE = "wipe"
EFFECT_FLASH = "flash"
EFFECT_RAINBOW = "rainbow"
EFFECT_RAINBOW_FILL = "rainbow_fill"

EFFECTS = (
    EFFECT_FILL,
    EFFECT_BLINK,
    EFFECT_BLINK_FAST,
    EFFECT_FADE,
    EFFECT_WIPE,
    EFFECT_FLASH,
    EFFECT_RAINBOW,
    EFFECT_RAINBOW_FILL,
)

# PX4 VehicleStatus constants (from VehicleStatus.msg)
# https://docs.px4.io/main/en/msg_docs/VehicleStatus.html
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2
# nav_state (Currently active mode)
NAVIGATION_STATE_MANUAL = 0
NAVIGATION_STATE_ALTCTL = 1
NAVIGATION_STATE_POSCTL = 2
NAVIGATION_STATE_AUTO_MISSION = 3
NAVIGATION_STATE_AUTO_LOITER = 4
NAVIGATION_STATE_AUTO_RTL = 5
NAVIGATION_STATE_POSITION_SLOW = 6
NAVIGATION_STATE_ALTITUDE_CRUISE = 8
NAVIGATION_STATE_ACRO = 10
NAVIGATION_STATE_DESCEND = 12
NAVIGATION_STATE_TERMINATION = 13
NAVIGATION_STATE_OFFBOARD = 14
NAVIGATION_STATE_STAB = 15
NAVIGATION_STATE_AUTO_TAKEOFF = 17
NAVIGATION_STATE_AUTO_LAND = 18
NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
NAVIGATION_STATE_AUTO_PRECLAND = 20
NAVIGATION_STATE_ORBIT = 21
NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22

# Map PX4 nav_state -> event name (see VehicleStatus.msg)
NAV_STATE_TO_EVENT = {
    NAVIGATION_STATE_MANUAL: "stabilized",
    NAVIGATION_STATE_ALTCTL: "altctl",
    NAVIGATION_STATE_POSCTL: "posctl",
    NAVIGATION_STATE_AUTO_MISSION: "mission",
    NAVIGATION_STATE_AUTO_LOITER: "mission",
    NAVIGATION_STATE_AUTO_RTL: "rtl",
    NAVIGATION_STATE_POSITION_SLOW: "position_slow",
    NAVIGATION_STATE_ALTITUDE_CRUISE: "altitude_cruise",
    NAVIGATION_STATE_ACRO: "acro",
    NAVIGATION_STATE_DESCEND: "descend",
    NAVIGATION_STATE_TERMINATION: "termination",
    NAVIGATION_STATE_OFFBOARD: "offboard",
    NAVIGATION_STATE_STAB: "stabilized",
    NAVIGATION_STATE_AUTO_TAKEOFF: "takeoff",
    NAVIGATION_STATE_AUTO_LAND: "land",
    NAVIGATION_STATE_AUTO_FOLLOW_TARGET: "follow_target",
    NAVIGATION_STATE_AUTO_PRECLAND: "precland",
    NAVIGATION_STATE_ORBIT: "orbit",
    NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: "vtol_takeoff",
}

# ROS 2 Log level (rcl_interfaces/msg/Log)
LOG_LEVEL_ERROR = 40


def hsv_to_rgb(h: float, s: float, v: float) -> tuple[int, int, int]:
    """h in [0,1), s,v in [0,1]. Returns (r,g,b) 0-255."""
    import colorsys
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


def _parse_events_config(events_yaml: str) -> dict[str, dict[str, Any]]:
    """Parse events YAML string into { event_name: { effect?, r, g, b } }.
    Entries with effect: none are excluded — such events are disabled and not applied."""
    if not events_yaml or not events_yaml.strip():
        return {}
    try:
        data = yaml.safe_load(events_yaml)
        if not isinstance(data, dict):
            return {}
        # Skip events with effect: none — nothing is assigned for them
        return {
            k: v for k, v in data.items()
            if isinstance(v, dict) and (v.get("effect") or "").strip().lower() != "none"
        }
    except yaml.YAMLError:
        return {}


class LEDNode(Node):
    def __init__(self):
        super().__init__("led_node")

        self.declare_parameter("led_count", 58)
        self.declare_parameter("led_transport", "auto")
        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_device", 0)
        self.declare_parameter("uart_device", "/dev/ttyS2")
        self.declare_parameter("uart_baudrate", 2500000)
        self.declare_parameter("uart_bit0_symbol", 192)
        self.declare_parameter("uart_bit1_symbol", 248)
        self.declare_parameter("uart_reset_us", 80)
        self.declare_parameter("brightness", 100.0)
        self.declare_parameter("brightness_low_battery", 1.0)
        self.declare_parameter("state_publish_rate", 10.0)
        self.declare_parameter("animation_rate", 30.0)
        self.declare_parameter("led_notify", True)
        self.declare_parameter("fmu_out_prefix", "/fmu/out")
        self.declare_parameter("vehicle_status_timeout", 2.0)
        self.declare_parameter("battery_min_voltage_per_cell", 3.5)
        self.declare_parameter("battery_min_voltage", 6.0)
        self.declare_parameter("events", "")

        led_count = self.get_parameter("led_count").get_parameter_value().integer_value
        led_transport = self.get_parameter("led_transport").get_parameter_value().string_value.strip().lower()
        spi_bus = self.get_parameter("spi_bus").get_parameter_value().integer_value
        spi_device = self.get_parameter("spi_device").get_parameter_value().integer_value
        uart_device = self.get_parameter("uart_device").get_parameter_value().string_value
        uart_baudrate = self.get_parameter("uart_baudrate").get_parameter_value().integer_value
        uart_bit0_symbol = self.get_parameter("uart_bit0_symbol").get_parameter_value().integer_value
        uart_bit1_symbol = self.get_parameter("uart_bit1_symbol").get_parameter_value().integer_value
        uart_reset_us = self.get_parameter("uart_reset_us").get_parameter_value().integer_value

        self.get_logger().info(
            "Initializing strip: led_count=%d, transport=%s, spi_bus=%d, spi_device=%d, "
            "uart_device=%s, uart_baudrate=%d"
            % (
                led_count,
                led_transport,
                spi_bus,
                spi_device,
                uart_device,
                uart_baudrate,
            )
        )
        if led_transport == "uart":
            self.get_logger().warn(
                "led_transport='uart' is temporarily disabled in runtime. Falling back to SPI path."
            )
        try:
            if led_transport in ("auto", "spi", "uart"):
                driver = WS2812SpiDriver(
                    spi_bus=spi_bus, spi_device=spi_device, led_count=led_count
                )
            else:
                raise ValueError(
                    "Unsupported led_transport='%s'. Use: auto|spi|uart"
                    % led_transport
                )
            self._strip = driver.get_strip()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize LED strip backend: {e}")
            raise

        brightness_low_batt_pct = max(
            0.0,
            min(
                100.0,
                self.get_parameter("brightness_low_battery")
                .get_parameter_value()
                .double_value,
            ),
        )
        brightness_pct = max(
            0.0,
            min(100.0, self.get_parameter("brightness").get_parameter_value().double_value),
        )
        # Start at reduced brightness (low_battery) until valid battery data arrives.
        self._brightness_normal = brightness_pct
        self._brightness_low_battery = brightness_low_batt_pct
        # Driver already uses low-battery brightness; keep flag True so the first
        # valid battery packet (voltage_v >= battery_min_voltage) can restore "brightness".
        self._brightness_low_active = True
        self._strip.set_brightness(self._brightness_low_battery / 100.0)

        self._n = led_count
        self._effect: str = EFFECT_FILL
        self._effect_rgb: tuple[int, int, int] = (255, 255, 255)
        self._previous_effect: str = EFFECT_FILL
        self._previous_rgb: tuple[int, int, int] = (255, 255, 255)
        self._manual_leds: dict[int, tuple[int, int, int]] = {}
        self._effect_start_time = time.monotonic()

        self._phase = 0
        self._wipe_pos = 0
        self._rainbow_hue = 0.0
        self._flash_phase = 0
        self._fade_start_pixels: Optional[list[tuple[int, int, int]]] = None
        self._last_pixels: list[tuple[int, int, int]] = [(0, 0, 0)] * self._n

        # Event visualization (PX4 topics)
        events_str = self.get_parameter("events").get_parameter_value().string_value
        self._events_config: dict[str, dict[str, Any]] = _parse_events_config(events_str)
        self._led_notify = self.get_parameter("led_notify").get_parameter_value().bool_value
        self._vehicle_status_timeout = self.get_parameter("vehicle_status_timeout").get_parameter_value().double_value
        self._battery_min_voltage_per_cell = self.get_parameter("battery_min_voltage_per_cell").get_parameter_value().double_value
        self._battery_min_voltage = self.get_parameter("battery_min_voltage").get_parameter_value().double_value
        self._fmu_out_prefix = self.get_parameter("fmu_out_prefix").get_parameter_value().string_value

        self._last_vehicle_status_time: Optional[float] = None
        self._connected = False
        self._last_nav_event: Optional[str] = None
        self._last_arming_event: Optional[str] = None
        self._error_active_until: float = 0.0
        self._low_battery_active: bool = False
        self._logged_no_vehicle_status: bool = False
        self._battery_seen: bool = False

        self._state_pub = self.create_publisher(LEDStateArray, "/led_control/state", 10)
        self._set_effect_srv = self.create_service(
            SetLEDEffect, "/led_control/set_effect", self._cb_set_effect
        )
        self._set_leds_srv = self.create_service(SetLEDs, "/led_control/set_leds", self._cb_set_leds)

        state_rate = self.get_parameter("state_publish_rate").get_parameter_value().double_value
        anim_rate = self.get_parameter("animation_rate").get_parameter_value().double_value
        self._state_timer = self.create_timer(1.0 / state_rate, self._publish_state)
        self._anim_timer = self.create_timer(1.0 / anim_rate, self._animation_tick)

        if self._led_notify and self._events_config:
            if "startup" in self._events_config:
                self._apply_event("startup")
            if _PX4_MSGS_AVAILABLE:
                self._setup_px4_subscriptions()
            else:
                self.get_logger().warn(
                    "led_notify=true but px4_msgs not found: event visualization from PX4 disabled. "
                    "Add px4_msgs to your workspace (see PX4 ROS2 User Guide)."
                )
            self._watchdog_timer = self.create_timer(1.0, self._watchdog_cb)
            if _RCL_INTERFACES_AVAILABLE:
                self._rosout_sub = self.create_subscription(
                    Log, "/rosout", self._cb_rosout, 10
                )
        else:
            self._watchdog_timer = None

        self.get_logger().info(
            "LED node started (event_notify=%s)" % self._led_notify
        )

    def _setup_px4_subscriptions(self) -> None:
        prefix = self._fmu_out_prefix.rstrip("/")
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f"{prefix}/vehicle_status",
            self._cb_vehicle_status,
            qos_px4,
        )
        self._battery_status_sub = self.create_subscription(
            BatteryStatus,
            f"{prefix}/battery_status",
            self._cb_battery_status,
            qos_px4,
        )

    def _apply_event(self, event_name: str) -> None:
        """Apply effect/color from events config (FC, errors, battery)."""
        cfg = self._events_config.get(event_name)
        if not cfg:
            self.get_logger().info("[LED/event] event '%s' not in events config" % event_name)
            return
        effect = cfg.get("effect", EFFECT_FILL)
        if effect not in EFFECTS:
            effect = EFFECT_FILL
        r = int(cfg.get("r", 0))
        g = int(cfg.get("g", 0))
        b = int(cfg.get("b", 0))
        self._previous_effect = self._effect
        self._previous_rgb = self._effect_rgb
        self._effect = effect
        self._effect_rgb = (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)))
        self._effect_start_time = time.monotonic()
        self._phase = 0
        self._wipe_pos = 0
        self._flash_phase = 0
        self._manual_leds.clear()
        self._fade_start_pixels = None
        self.get_logger().info(
            "[LED/event] event=%s -> effect=%s rgb=(%d,%d,%d)"
            % (event_name, effect, self._effect_rgb[0], self._effect_rgb[1], self._effect_rgb[2])
        )

    def _cb_vehicle_status(self, msg: "VehicleStatus") -> None:
        now = time.monotonic()
        self._last_vehicle_status_time = now

        if not self._connected:
            self._connected = True
            self.get_logger().info("[LED/vehicle_status] first message — connected event")
            self._apply_event("connected")

        # Arming
        if msg.arming_state == ARMING_STATE_ARMED:
            if self._last_arming_event != "armed":
                self._last_arming_event = "armed"
                self.get_logger().info("[LED/vehicle_status] arming_state=ARMED -> armed event")
                self._apply_event("armed")
        else:
            if self._last_arming_event != "disarmed":
                self._last_arming_event = "disarmed"
                self.get_logger().info("[LED/vehicle_status] arming_state=DISARMED -> disarmed event")
                self._apply_event("disarmed")

        # Flight mode (nav_state)
        nav_event = NAV_STATE_TO_EVENT.get(msg.nav_state)
        if nav_event and nav_event != self._last_nav_event:
            self.get_logger().info(
                "[LED/vehicle_status] nav_state=%s -> flight mode event: %s"
                % (msg.nav_state, nav_event)
            )
            self._last_nav_event = nav_event
            self._apply_event(nav_event)

    def _cb_battery_status(self, msg: "BatteryStatus") -> None:
        # BatteryStatus (PX4): voltage_v [V] (Invalid: 0), cell_count (Invalid: 0)
        # Low indication uses per-cell voltage: voltage_v / cell_count < battery_min_voltage_per_cell
        # Invalid voltage_v or cell_count: do not apply low_battery event
        voltage_v = getattr(msg, "voltage_v", 0.0) or 0.0
        cell_count = int(getattr(msg, "cell_count", 0) or 0)
        if voltage_v <= 0 or cell_count <= 0:
            # No or invalid battery data: keep low-battery brightness.
            self._low_battery_active = False
            if not self._brightness_low_active:
                self._brightness_low_active = True
                self.get_logger().info(
                    "[LED/battery] battery_status invalid/absent -> force low brightness %.1f%%"
                    % self._brightness_low_battery
                )
                self._strip.set_brightness(self._brightness_low_battery / 100.0)
            return

        # First valid packet means battery is present
        if not self._battery_seen:
            self._battery_seen = True
        voltage_per_cell = voltage_v / cell_count
        is_low = voltage_per_cell < self._battery_min_voltage_per_cell
        if is_low:
            if not self._low_battery_active:
                self._low_battery_active = True
                self.get_logger().info(
                    "[LED/battery] per-cell voltage %.2f V < %.2f V -> low_battery event"
                    % (voltage_per_cell, self._battery_min_voltage_per_cell)
                )
                self._apply_event("low_battery")
        else:
            self._low_battery_active = False

        # Also dim/restore brightness from pack voltage
        if voltage_v < self._battery_min_voltage:
            if not self._brightness_low_active:
                self._brightness_low_active = True
                self.get_logger().info(
                    "[LED/battery] voltage_v=%.2f V < %.2f V -> dim to %.1f%%"
                    % (voltage_v, self._battery_min_voltage, self._brightness_low_battery)
                )
                self._strip.set_brightness(self._brightness_low_battery / 100.0)
        else:
            if self._brightness_low_active:
                self._brightness_low_active = False
                self.get_logger().info(
                    "[LED/battery] voltage_v=%.2f V >= %.2f V -> restore brightness to %.1f%%"
                    % (voltage_v, self._battery_min_voltage, self._brightness_normal)
                )
                self._strip.set_brightness(self._brightness_normal / 100.0)

    def _cb_rosout(self, msg: Log) -> None:
        if msg.level == LOG_LEVEL_ERROR:
            self._error_active_until = time.monotonic() + 5.0
            self.get_logger().info("[LED/rosout] error on /rosout -> error event (5 s)")
            self._apply_event("error")

    def _watchdog_cb(self) -> None:
        now = time.monotonic()
        if self._led_notify and not self._logged_no_vehicle_status and self._last_vehicle_status_time is None:
            if now > 3.0:
                self._logged_no_vehicle_status = True
                self.get_logger().warn(
                    "[LED/watchdog] no vehicle_status for 3 s. "
                    "Check topic %s/vehicle_status and PX4 publishing to DDS/ROS2"
                    % self._fmu_out_prefix
                )
        if not self._led_notify or not self._connected:
            return
        if self._last_vehicle_status_time is not None:
            if now - self._last_vehicle_status_time > self._vehicle_status_timeout:
                self.get_logger().warn(
                    "[LED/watchdog] vehicle_status timeout (%.1f s) — disconnected event"
                    % self._vehicle_status_timeout
                )
                self._connected = False
                self._last_vehicle_status_time = None
                self._last_nav_event = None
                self._last_arming_event = None
                self._apply_event("disconnected")

    def _cb_set_effect(self, request: SetLEDEffect.Request, response: SetLEDEffect.Response):
        effect = request.effect.strip().lower() if request.effect else EFFECT_FILL
        if effect not in EFFECTS:
            effect = EFFECT_FILL
        r = max(0, min(255, request.r))
        g = max(0, min(255, request.g))
        b = max(0, min(255, request.b))
        self._previous_effect = self._effect
        self._previous_rgb = self._effect_rgb
        self._effect = effect
        self._effect_rgb = (r, g, b)
        self._effect_start_time = time.monotonic()
        self._phase = 0
        self._wipe_pos = 0
        self._flash_phase = 0
        self._manual_leds.clear()
        self._fade_start_pixels = None
        self.get_logger().info("[LED/set_effect] service: effect=%s rgb=(%d,%d,%d)" % (effect, r, g, b))
        response.success = True
        return response

    def _cb_set_leds(self, request: SetLEDs.Request, response: SetLEDs.Response):
        for led in request.leds:
            if 0 <= led.index < self._n:
                self._manual_leds[led.index] = (
                    max(0, min(255, led.r)),
                    max(0, min(255, led.g)),
                    max(0, min(255, led.b)),
                )
        self.get_logger().info("[LED/set_leds] service: set %d LEDs" % len(request.leds))
        response.success = True
        return response

    def _publish_state(self):
        msg = LEDStateArray()
        for i in range(self._n):
            pr, pg, pb = self._last_pixels[i]
            msg.leds.append(LEDState(index=i, r=pr, g=pg, b=pb))
        self._state_pub.publish(msg)

    def _apply_manual_leds(self):
        for idx, (r, g, b) in self._manual_leds.items():
            self._strip.set_pixel_color(idx, Color(r, g, b))

    def _animation_tick(self):
        if self._manual_leds:
            self._apply_manual_leds()
            self._strip.show()
            for i in range(self._n):
                p = self._strip._pixels[i]
                self._last_pixels[i] = (p.r, p.g, p.b)
            return

        t = time.monotonic() - self._effect_start_time
        effect = self._effect
        r, g, b = self._effect_rgb

        if effect == EFFECT_FILL:
            self._strip.set_all_pixels(Color(r, g, b))
            self._strip.show()
            self._last_pixels = [(r, g, b)] * self._n

        elif effect == EFFECT_BLINK:
            on = (int(t * 2) % 2) == 0
            c = Color(r, g, b) if on else Color(0, 0, 0)
            self._strip.set_all_pixels(c)
            self._strip.show()
            self._last_pixels = [(r, g, b)] * self._n if on else [(0, 0, 0)] * self._n

        elif effect == EFFECT_BLINK_FAST:
            on = (int(t * 6) % 2) == 0
            c = Color(r, g, b) if on else Color(0, 0, 0)
            self._strip.set_all_pixels(c)
            self._strip.show()
            self._last_pixels = [(r, g, b)] * self._n if on else [(0, 0, 0)] * self._n

        elif effect == EFFECT_FADE:
            duration = 1.0
            if self._fade_start_pixels is None:
                self._fade_start_pixels = list(self._last_pixels)
            k = min(1.0, t / duration)
            for i in range(self._n):
                sr, sg, sb = self._fade_start_pixels[i]
                nr = int(sr + (r - sr) * k)
                ng = int(sg + (g - sg) * k)
                nb = int(sb + (b - sb) * k)
                self._strip.set_pixel_color(i, Color(nr, ng, nb))
            self._strip.show()
            for i in range(self._n):
                p = self._strip._pixels[i]
                self._last_pixels[i] = (p.r, p.g, p.b)
            if k >= 1.0:
                self._fade_start_pixels = None

        elif effect == EFFECT_WIPE:
            pos = int((t * 15) * self._n) % (self._n + 1)
            for i in range(self._n):
                c = Color(r, g, b) if i < pos else Color(0, 0, 0)
                self._strip.set_pixel_color(i, c)
            self._strip.show()
            for i in range(self._n):
                p = self._strip._pixels[i]
                self._last_pixels[i] = (p.r, p.g, p.b)

        elif effect == EFFECT_FLASH:
            if t < 0.1:
                self._strip.set_all_pixels(Color(r, g, b))
            elif t < 0.2:
                self._strip.clear()
            elif t < 0.3:
                self._strip.set_all_pixels(Color(r, g, b))
            elif t < 0.4:
                self._strip.clear()
            else:
                self._effect = self._previous_effect
                self._effect_rgb = self._previous_rgb
                self._effect_start_time = time.monotonic()
                self._strip.set_all_pixels(Color(*self._previous_rgb))
            self._strip.show()
            for i in range(self._n):
                p = self._strip._pixels[i]
                self._last_pixels[i] = (p.r, p.g, p.b)

        elif effect == EFFECT_RAINBOW:
            self._rainbow_hue += 0.005
            for i in range(self._n):
                h = (self._rainbow_hue + i / self._n) % 1.0
                nr, ng, nb = hsv_to_rgb(h, 1.0, 1.0)
                self._strip.set_pixel_color(i, Color(nr, ng, nb))
            self._strip.show()
            for i in range(self._n):
                p = self._strip._pixels[i]
                self._last_pixels[i] = (p.r, p.g, p.b)

        elif effect == EFFECT_RAINBOW_FILL:
            self._rainbow_hue += 0.01
            nr, ng, nb = hsv_to_rgb(self._rainbow_hue, 1.0, 1.0)
            self._strip.set_all_pixels(Color(nr, ng, nb))
            self._strip.show()
            self._last_pixels = [(nr, ng, nb)] * self._n

        else:
            self._strip.set_all_pixels(Color(r, g, b))
            self._strip.show()
            self._last_pixels = [(r, g, b)] * self._n


def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
