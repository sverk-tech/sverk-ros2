"""
WS2812/WS2812B driver over UART (/dev/ttyS*), experimental backend.

This backend encodes each WS2812 data bit into one UART payload byte:
  bit 1 -> uart_bit1_symbol (default 0xF8)
  bit 0 -> uart_bit0_symbol (default 0xC0)

Order on wire: GRB, same as ws2812_spidev/ws2812_mraa backends.

Notes:
- Requires Python package `pyserial`.
- For stable WS2812 timing, high UART baudrates are typically required.
- The exact symbols/baudrate can be tuned from ROS params.
"""

from __future__ import annotations

import time

import serial

_DEFAULT_RESET_US = 80

_BYTE_TO_WS: list[bytes] = [b""] * 256


def _rebuild_lut(bit0_symbol: int, bit1_symbol: int) -> None:
    bit0 = int(bit0_symbol) & 0xFF
    bit1 = int(bit1_symbol) & 0xFF
    for val in range(256):
        _BYTE_TO_WS[val] = bytes(
            bit1 if (val >> bit) & 1 else bit0
            for bit in range(7, -1, -1)
        )


class Color:
    __slots__ = ("r", "g", "b")

    def __init__(self, r: int = 0, g: int = 0, b: int = 0) -> None:
        self.r = int(r)
        self.g = int(g)
        self.b = int(b)

    def __repr__(self) -> str:  # pragma: no cover
        return f"Color({self.r}, {self.g}, {self.b})"


class _WS2812Strip:
    def __init__(self, ser: serial.Serial, led_count: int, reset_us: int) -> None:
        self._ser = ser
        self._n = int(led_count)
        self._brightness: float = 1.0
        self._reset_us = max(50, int(reset_us))
        self._pixels: list[Color] = [Color() for _ in range(self._n)]

    def set_brightness(self, value: float) -> None:
        self._brightness = max(0.0, min(1.0, float(value)))

    def set_pixel_color(self, index: int, color: Color) -> None:
        if 0 <= index < self._n:
            self._pixels[index] = color

    def set_all_pixels(self, color: Color) -> None:
        c = Color(color.r, color.g, color.b)
        for i in range(self._n):
            self._pixels[i] = c

    def clear(self) -> None:
        self.set_all_pixels(Color(0, 0, 0))
        self.show()

    def show(self) -> None:
        b = self._brightness
        buf = bytearray(self._n * 24)
        idx = 0
        for px in self._pixels:
            r = min(255, int(px.r * b))
            g = min(255, int(px.g * b))
            blue = min(255, int(px.b * b))
            for encoded in (_BYTE_TO_WS[g], _BYTE_TO_WS[r], _BYTE_TO_WS[blue]):
                buf[idx : idx + 8] = encoded
                idx += 8

        self._ser.write(buf)
        self._ser.flush()
        # WS2812 latch/reset low pulse (>50 us). UART TX idles HIGH, so we
        # insert a quiet gap between frames as a practical reset approximation.
        time.sleep(self._reset_us / 1_000_000.0)


class WS2812UartDriver:
    """
    Open UART backend and expose strip object compatible with SPI backends.
    """

    def __init__(
        self,
        uart_device: str = "/dev/ttyS2",
        uart_baudrate: int = 2_500_000,
        led_count: int = 1,
        uart_bit0_symbol: int = 0xC0,
        uart_bit1_symbol: int = 0xF8,
        uart_reset_us: int = _DEFAULT_RESET_US,
    ) -> None:
        _rebuild_lut(uart_bit0_symbol, uart_bit1_symbol)
        self._ser = serial.Serial(
            port=uart_device,
            baudrate=int(uart_baudrate),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        )
        self._strip = _WS2812Strip(self._ser, led_count=led_count, reset_us=uart_reset_us)

    def get_strip(self) -> _WS2812Strip:
        return self._strip

    def close(self) -> None:
        self._ser.close()
