#!/usr/bin/env python3
"""
Standalone WS2812 UART tester (no ROS).

Examples:
  python3 ws2812_uart_standalone_test.py --port /dev/ttyS2 --count 112 --mode fill --color 255,0,0
  python3 ws2812_uart_standalone_test.py --port /dev/ttyS2 --count 112 --mode rainbow --fps 25
  python3 ws2812_uart_standalone_test.py --port /dev/ttyS2 --count 112 --mode blink --color 0,255,0 --period 0.5
"""

from __future__ import annotations

import argparse
import colorsys
import time
from dataclasses import dataclass

import serial


def parse_rgb(value: str) -> tuple[int, int, int]:
    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("color must be in R,G,B format")
    try:
        r, g, b = (int(p.strip()) for p in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("color must be integers") from exc
    if not all(0 <= x <= 255 for x in (r, g, b)):
        raise argparse.ArgumentTypeError("color channels must be in range 0..255")
    return r, g, b


def hsv_to_rgb(h: float, s: float, v: float) -> tuple[int, int, int]:
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


@dataclass
class Color:
    r: int = 0
    g: int = 0
    b: int = 0


class WS2812Uart:
    def __init__(
        self,
        port: str,
        baudrate: int,
        led_count: int,
        bit0_symbol: int = 0xC0,
        bit1_symbol: int = 0xF8,
        reset_us: int = 80,
        brightness: float = 1.0,
    ) -> None:
        self.led_count = int(led_count)
        self.reset_us = max(50, int(reset_us))
        self.brightness = max(0.0, min(1.0, float(brightness)))
        self._pixels = [Color() for _ in range(self.led_count)]
        self._lut = self._build_lut(bit0_symbol, bit1_symbol)

        self._ser = serial.Serial(
            port=port,
            baudrate=int(baudrate),
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

    @staticmethod
    def _build_lut(bit0_symbol: int, bit1_symbol: int) -> list[bytes]:
        bit0 = int(bit0_symbol) & 0xFF
        bit1 = int(bit1_symbol) & 0xFF
        lut: list[bytes] = [b""] * 256
        for val in range(256):
            lut[val] = bytes(
                bit1 if (val >> bit) & 1 else bit0
                for bit in range(7, -1, -1)
            )
        return lut

    def close(self) -> None:
        if getattr(self, "_ser", None) is not None and self._ser.is_open:
            self._ser.close()

    def set_all(self, r: int, g: int, b: int) -> None:
        c = Color(r, g, b)
        for i in range(self.led_count):
            self._pixels[i] = c

    def clear(self) -> None:
        self.set_all(0, 0, 0)
        self.show()

    def set_pixel(self, idx: int, r: int, g: int, b: int) -> None:
        if 0 <= idx < self.led_count:
            self._pixels[idx] = Color(r, g, b)

    def show(self) -> None:
        if not self._ser.is_open:
            return
        b = self.brightness
        out = bytearray(self.led_count * 24)
        pos = 0
        for px in self._pixels:
            r = min(255, int(px.r * b))
            g = min(255, int(px.g * b))
            bl = min(255, int(px.b * b))
            for enc in (self._lut[g], self._lut[r], self._lut[bl]):
                out[pos : pos + 8] = enc
                pos += 8
        self._ser.write(out)
        self._ser.flush()
        time.sleep(self.reset_us / 1_000_000.0)


def apply_test_pattern(strip: WS2812Uart) -> None:
    """
    Pattern for visual diagnostics:
    LED0=R, LED1=G, LED2=B, LED3=W, repeats.
    """
    pattern = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255))
    for i in range(strip.led_count):
        strip.set_pixel(i, *pattern[i % 4])
    strip.show()


def run_fill(strip: WS2812Uart, color: tuple[int, int, int]) -> None:
    strip.set_all(*color)
    strip.show()
    print("Fill sent. Press Ctrl+C to clear and exit.")
    while True:
        time.sleep(1.0)


def run_blink(strip: WS2812Uart, color: tuple[int, int, int], period: float) -> None:
    print("Blink mode. Press Ctrl+C to clear and exit.")
    half = max(0.02, period / 2.0)
    while True:
        strip.set_all(*color)
        strip.show()
        time.sleep(half)
        strip.set_all(0, 0, 0)
        strip.show()
        time.sleep(half)


def run_rainbow(strip: WS2812Uart, fps: float, hue_step: float) -> None:
    print("Rainbow mode. Press Ctrl+C to clear and exit.")
    dt = 1.0 / max(1.0, fps)
    hue = 0.0
    n = strip.led_count
    while True:
        for i in range(n):
            r, g, b = hsv_to_rgb(hue + i / max(1, n), 1.0, 1.0)
            strip.set_pixel(i, r, g, b)
        strip.show()
        hue = (hue + hue_step) % 1.0
        time.sleep(dt)


def run_chase(strip: WS2812Uart, color: tuple[int, int, int], fps: float) -> None:
    print("Chase mode. Press Ctrl+C to clear and exit.")
    dt = 1.0 / max(1.0, fps)
    pos = 0
    n = strip.led_count
    while True:
        strip.set_all(0, 0, 0)
        strip.set_pixel(pos % n, *color)
        strip.show()
        pos += 1
        time.sleep(dt)


def parse_int_list(value: str) -> list[int]:
    items = [x.strip() for x in value.split(",") if x.strip()]
    if not items:
        raise argparse.ArgumentTypeError("value list cannot be empty")
    out: list[int] = []
    for item in items:
        out.append(int(item, 0))
    return out


def run_sweep(args: argparse.Namespace) -> None:
    print("Sweep mode started. Ctrl+C to stop.")
    combo_idx = 0
    for baud in args.sweep_bauds:
        for bit0 in args.sweep_bit0:
            for bit1 in args.sweep_bit1:
                if bit0 == bit1:
                    continue
                combo_idx += 1
                strip = None
                try:
                    strip = WS2812Uart(
                        port=args.port,
                        baudrate=baud,
                        led_count=args.count,
                        bit0_symbol=bit0,
                        bit1_symbol=bit1,
                        reset_us=args.reset_us,
                        brightness=args.brightness,
                    )
                    print(
                        "[%04d] baud=%d bit0=0x%02X bit1=0x%02X"
                        % (combo_idx, baud, bit0, bit1)
                    )
                    apply_test_pattern(strip)
                    time.sleep(args.step_seconds)
                    strip.clear()
                except KeyboardInterrupt:
                    if strip is not None:
                        strip.clear()
                        strip.close()
                    raise
                except Exception as exc:
                    print(
                        "[%04d] skip (baud=%d bit0=0x%02X bit1=0x%02X): %s"
                        % (combo_idx, baud, bit0, bit1, exc)
                    )
                finally:
                    if strip is not None:
                        strip.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Standalone WS2812 UART tester")
    parser.add_argument("--port", default="/dev/ttyS2", help="UART device, e.g. /dev/ttyS2")
    parser.add_argument("--baud", type=int, default=2_500_000, help="UART baudrate")
    parser.add_argument("--count", type=int, default=112, help="LED count")
    parser.add_argument("--bit0", type=lambda x: int(x, 0), default=0xC0, help="UART symbol for WS bit 0 (e.g. 0xC0)")
    parser.add_argument("--bit1", type=lambda x: int(x, 0), default=0xF8, help="UART symbol for WS bit 1 (e.g. 0xF8)")
    parser.add_argument("--reset-us", type=int, default=80, help="Inter-frame pause in microseconds")
    parser.add_argument("--brightness", type=float, default=1.0, help="0.0..1.0 brightness scale")
    parser.add_argument("--mode", choices=("fill", "blink", "rainbow", "chase", "sweep"), default="fill")
    parser.add_argument("--color", type=parse_rgb, default=(255, 0, 0), help="R,G,B color for fill/blink/chase")
    parser.add_argument("--period", type=float, default=0.5, help="Blink period in seconds")
    parser.add_argument("--fps", type=float, default=25.0, help="FPS for rainbow/chase")
    parser.add_argument("--hue-step", type=float, default=0.01, help="Hue increment per frame in rainbow mode")
    parser.add_argument(
        "--sweep-bauds",
        type=parse_int_list,
        default=parse_int_list("2000000,2500000,3000000,3500000,4000000"),
        help="Comma-separated baudrates for sweep mode, e.g. 2000000,2500000,3000000",
    )
    parser.add_argument(
        "--sweep-bit0",
        type=parse_int_list,
        default=parse_int_list("0x40,0x44,0x60,0x70,0x80,0xC0"),
        help="Comma-separated bit0 symbols (decimal/hex) for sweep mode",
    )
    parser.add_argument(
        "--sweep-bit1",
        type=parse_int_list,
        default=parse_int_list("0x6C,0x78,0x7C,0xE0,0xF0,0xF8,0xFC"),
        help="Comma-separated bit1 symbols (decimal/hex) for sweep mode",
    )
    parser.add_argument(
        "--step-seconds",
        type=float,
        default=1.2,
        help="How long to show each combo in sweep mode",
    )
    args = parser.parse_args()

    if args.mode == "sweep":
        try:
            run_sweep(args)
        except KeyboardInterrupt:
            print("\nSweep stopped by user.")
        return 0

    strip = WS2812Uart(
        port=args.port,
        baudrate=args.baud,
        led_count=args.count,
        bit0_symbol=args.bit0,
        bit1_symbol=args.bit1,
        reset_us=args.reset_us,
        brightness=args.brightness,
    )
    print(
        "Opened %s @ %d, leds=%d, bit0=0x%02X bit1=0x%02X reset_us=%d brightness=%.2f"
        % (args.port, args.baud, args.count, args.bit0, args.bit1, args.reset_us, args.brightness)
    )

    try:
        if args.mode == "fill":
            run_fill(strip, args.color)
        elif args.mode == "blink":
            run_blink(strip, args.color, args.period)
        elif args.mode == "rainbow":
            run_rainbow(strip, args.fps, args.hue_step)
        elif args.mode == "chase":
            run_chase(strip, args.color, args.fps)
    except KeyboardInterrupt:
        print("\nStopping: clear strip and close UART...")
    finally:
        try:
            strip.clear()
        finally:
            strip.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
