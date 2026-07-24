from __future__ import annotations

from typing import Optional

from .transport import SerialTransport


class LineProtocol:

    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    @property
    def transport(self) -> SerialTransport:
        return self._transport

    def send_command(self, command: str) -> None:
        self._transport.write_line(command)

    def read_lines(self) -> list[str]:
        return self._transport.read_available_lines()

    def request(self, command: str, response_timeout: float = 1.0) -> Optional[str]:
        return self._transport.request_line(command, response_timeout=response_timeout)
