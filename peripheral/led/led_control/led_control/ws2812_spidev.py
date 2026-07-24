"""
WS2812/WS2812B driver over Linux spidev.

Implementation is aligned with the known-working local `spidev.py` approach:
- 1 WS2812 bit -> 1 SPI byte
- bit 0 symbol: 0x80
- bit 1 symbol: 0xF8
- byte order on wire: GRB
"""

from __future__ import annotations

import time

import spidev

_SPI_SPEED_HZ = 2_400_000

_BIT0 = 0x80
_BIT1 = 0xF8

_BYTE_TO_SPI: list[bytes] = [b""] * 256
for _v in range(256):
    _BYTE_TO_SPI[_v] = bytes(
        _BIT1 if (_v >> _b) & 1 else _BIT0 for _b in range(7, -1, -1)
    )

_RESET_TIME_S = 80e-6


class Color:
    """Single LED color (R, G, B in the 0-255 range)."""

    __slots__ = ("r", "g", "b")

    def __init__(self, r: int = 0, g: int = 0, b: int = 0) -> None:
        self.r = int(r)
        self.g = int(g)
        self.b = int(b)

    def __repr__(self) -> str:  # pragma: no cover
        return f"Color({self.r}, {self.g}, {self.b})"


class _WS2812Strip:
    """Internal strip object created by WS2812SpiDriver.get_strip()."""

    def __init__(self, spi: spidev.SpiDev, led_count: int, chunk_size: int, speed_hz: int) -> None:
        self._spi = spi
        self._n = led_count
        self._chunk = chunk_size
        self._speed_hz = speed_hz
        self._brightness: float = 1.0
        self._pixels: list[Color] = [Color() for _ in range(led_count)]

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
        """Send frame buffer to strip over SPI."""
        b = self._brightness
        buf = bytearray(self._n * 24)
        idx = 0
        for px in self._pixels:
            r = min(255, int(px.r * b))
            g = min(255, int(px.g * b))
            blue = min(255, int(px.b * b))
            for encoded in (_BYTE_TO_SPI[g], _BYTE_TO_SPI[r], _BYTE_TO_SPI[blue]):
                buf[idx : idx + 8] = encoded
                idx += 8
        self._write(buf)
        # Latch/reset window for WS2812.
        time.sleep(_RESET_TIME_S)

    def _write(self, buf: bytearray) -> None:
        """Write in chunks respecting spidev transaction size limits."""
        mv = memoryview(buf)
        offset = 0
        length = len(buf)
        while offset < length:
            end = min(offset + self._chunk, length)
            chunk = list(mv[offset:end])
            self._spi.xfer2(chunk, self._speed_hz)
            offset = end


class WS2812SpiDriver:
    """
    Opens /dev/spidev<spi_bus>.<spi_device> and creates a strip object.

    Parameters:
        spi_bus - X in /dev/spidevX.Y (check with ls /dev/spidev* on the target system).
        spi_device - Y (usually 0).
        led_count - number of LEDs.
        spi_chunk_size — max bytes per one transfer (default 4096).
        spi_speed_hz — SPI clock for WS2812 stream (default 2_400_000).
    """

    def __init__(
        self,
        spi_bus: int = 0,
        spi_device: int = 0,
        led_count: int = 1,
        spi_chunk_size: int = 4096,
        spi_speed_hz: int = _SPI_SPEED_HZ,
    ) -> None:
        self._dev = spidev.SpiDev()
        self._dev.open(spi_bus, spi_device)
        self._dev.max_speed_hz = int(spi_speed_hz)
        self._dev.mode = 0b00
        self._dev.bits_per_word = 8
        self._dev.lsbfirst = False

        self._strip = _WS2812Strip(self._dev, led_count, spi_chunk_size, int(spi_speed_hz))

    def get_strip(self) -> _WS2812Strip:
        return self._strip

    def close(self) -> None:
        self._dev.close()
