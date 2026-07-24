"""
WS2812/WS2812B driver via Eclipse MRAA (SPI).

Uses the same public API as ws2812_spidev: Color, WS2812SpiDriver.get_strip().

Requires Python module `mraa` built for the same Python version as ROS.
On Bookworm, it is usually more reliable to build packages from
https://github.com/radxa-pkg/mraa via `dpkg-buildpackage` (see ReadMe.md);
errors like `uint8Array`/`_mraa` are typically caused by mismatch between
`mraa.py` and `_mraa.so` after mixed installs in `/usr/local`.

The SPI bus logical index (`spi_bus`) follows Radxa MRAA examples (`SPI_BUS 0` in C).
In MRAA, `spi_device` is usually not configured separately (CS selection depends on
the board description in MRAA); for spidev0.1 configure the overlay and, if needed,
use another bus index in `led_params.yaml`.

Documentation: https://docs.radxa.com/en/som/cm/cm5/radxa-os/app-dev/mraa

WS2812 over SPI is implemented similarly to ws2812_spidev (6.4 MHz, GRB bit encoding).
"""

from __future__ import annotations

import mraa

_SPI_SPEED_HZ = 6_400_000

_BIT1 = 0xF8
_BIT0 = 0xC0

_BYTE_TO_SPI: list[bytes] = [b""] * 256
for _v in range(256):
    _BYTE_TO_SPI[_v] = bytes(
        _BIT1 if (_v >> _b) & 1 else _BIT0 for _b in range(7, -1, -1)
    )

_RESET_LEN = 50


class Color:
    """Single LED color (R, G, B in the 0-255 range)."""

    __slots__ = ("r", "g", "b")

    def __init__(self, r: int = 0, g: int = 0, b: int = 0) -> None:
        self.r = int(r)
        self.g = int(g)
        self.b = int(b)

    def __repr__(self) -> str:  # pragma: no cover
        return f"Color({self.r}, {self.g}, {self.b})"


def _open_spi(spi_bus: int, _spi_device: int) -> mraa.Spi:
    # _spi_device: CS comes from board description in MRAA, not spidev minor.
    spi = mraa.Spi(spi_bus)
    spi.frequency(_SPI_SPEED_HZ)
    try:
        spi.mode(0)
    except Exception:
        pass
    try:
        spi.lsbmode(False)
    except Exception:
        pass
    return spi


class _WS2812Strip:
    def __init__(self, spi: mraa.Spi, led_count: int, chunk_size: int) -> None:
        self._spi = spi
        self._n = led_count
        self._chunk = chunk_size
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
        b = self._brightness
        buf = bytearray(self._n * 24 + _RESET_LEN)
        idx = 0
        for px in self._pixels:
            r = min(255, int(px.r * b))
            g = min(255, int(px.g * b))
            blue = min(255, int(px.b * b))
            for encoded in (_BYTE_TO_SPI[g], _BYTE_TO_SPI[r], _BYTE_TO_SPI[blue]):
                buf[idx : idx + 8] = encoded
                idx += 8
        self._write(buf)

    def _write(self, buf: bytearray) -> None:
        mv = memoryview(buf)
        offset = 0
        length = len(buf)
        while offset < length:
            end = min(offset + self._chunk, length)
            chunk = bytearray(mv[offset:end])
            try:
                self._spi.write(chunk)
            except (TypeError, AttributeError):
                self._spi.write(list(chunk))
            offset = end


class WS2812SpiDriver:
    """
    Opens SPI via MRAA (board-level logical spi_bus / spi_device).

    spi_chunk_size - write chunk size (default 4096).
    """

    def __init__(
        self,
        spi_bus: int = 0,
        spi_device: int = 0,
        led_count: int = 1,
        spi_chunk_size: int = 4096,
    ) -> None:
        self._spi = _open_spi(spi_bus, spi_device)
        self._strip = _WS2812Strip(self._spi, led_count, spi_chunk_size)

    def get_strip(self) -> _WS2812Strip:
        return self._strip

    def close(self) -> None:
        try:
            self._spi.close()
        except Exception:
            pass
