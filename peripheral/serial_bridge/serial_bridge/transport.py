from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from time import monotonic
from time import sleep
from typing import List, Optional

import serial


class SerialConnectionError(RuntimeError):
    """Raised when serial transport is unavailable or disconnected."""


@dataclass(slots=True)
class SerialTransportConfig:

    port: str
    baudrate: int = 115200
    timeout: float = 0.05
    write_timeout: float = 0.5
    reconnect_interval: float = 1.0
    encoding: str = "utf-8"
    newline: str = "\n"


class SerialTransport:

    def __init__(self, config: SerialTransportConfig) -> None:
        self._config = config
        self._serial: Optional[serial.Serial] = None
        self._lock = Lock()
        self._rx_buffer = bytearray()
        self._last_connect_attempt = 0.0

    @property
    def config(self) -> SerialTransportConfig:
        return self._config

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self, force: bool = False) -> bool:
        with self._lock:
            if self.is_connected:
                return True

            now = monotonic()
            if not force and (now - self._last_connect_attempt) < self._config.reconnect_interval:
                return False

            self._last_connect_attempt = now

            try:
                self._serial = serial.Serial(
                    port=self._config.port,
                    baudrate=self._config.baudrate,
                    timeout=self._config.timeout,
                    write_timeout=self._config.write_timeout,
                )
            except serial.SerialException:
                self._serial = None
                return False

            self._rx_buffer.clear()
            return True

    def disconnect(self) -> None:
        with self._lock:
            self._disconnect_locked()

    def _disconnect_locked(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        self._rx_buffer.clear()

    def ensure_connection(self) -> None:
        if not self.connect():
            raise SerialConnectionError(f"Serial port {self._config.port} is not connected")

    def write_bytes(self, payload: bytes) -> None:
        with self._lock:
            if not self.is_connected and not self.connect():
                raise SerialConnectionError(f"Serial port {self._config.port} is not connected")

            assert self._serial is not None
            try:
                self._serial.write(payload)
                self._serial.flush()
            except serial.SerialException as exc:
                self._disconnect_locked()
                raise SerialConnectionError(str(exc)) from exc

    def write_line(self, line: str) -> None:
        payload = (line + self._config.newline).encode(self._config.encoding)
        self.write_bytes(payload)

    def read_available_bytes(self, max_chunk_size: int = 4096) -> bytes:
        with self._lock:
            if not self.is_connected and not self.connect():
                return b""

            assert self._serial is not None
            try:
                pending = self._serial.in_waiting
                if pending <= 0:
                    return b""
                return self._serial.read(min(pending, max_chunk_size))
            except serial.SerialException:
                self._disconnect_locked()
                return b""

    def read_available_lines(self) -> List[str]:
        chunk = self.read_available_bytes()
        if chunk:
            self._rx_buffer.extend(chunk)

        lines: List[str] = []
        newline_bytes = self._config.newline.encode(self._config.encoding)

        while True:
            newline_index = self._rx_buffer.find(newline_bytes)
            if newline_index < 0:
                break

            raw_line = self._rx_buffer[:newline_index]
            del self._rx_buffer[: newline_index + len(newline_bytes)]
            lines.append(raw_line.decode(self._config.encoding, errors="replace"))

        return lines

    def request_line(self, line: str, response_timeout: float = 1.0) -> Optional[str]:
        self.write_line(line)

        deadline = monotonic() + response_timeout
        while monotonic() < deadline:
            lines = self.read_available_lines()
            if lines:
                return lines[0]
            sleep(0.01)
        return None
