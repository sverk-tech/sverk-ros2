"""
UWB Message Parser
"""

from typing import Optional, Tuple
from .protocol import (
    UWB_START_MARKER,
    UWB_PROTOCOL_VERSION,
    UwbHeader,
)


class UwbParser:
    """Parser for UWB protocol messages."""

    def __init__(self):
        """Initialize UWB parser."""
        self._rx_buffer = bytearray()

    def reset(self):
        """Reset receive buffer."""
        self._rx_buffer = bytearray()

    def feed_data(self, data: bytes):
        """
        Feed raw data to parser.

        Args:
            data: Raw bytes from UWB
        """
        self._rx_buffer.extend(data)

    def read_message(self) -> Optional[Tuple[int, bytes]]:
        """
        Try to read a complete message from buffer.

        Returns:
            Tuple of (msg_type, payload) or None if no complete message
        """
        while len(self._rx_buffer) >= UwbHeader.SIZE:
            # Find start marker
            if self._rx_buffer[0] != UWB_START_MARKER:
                del self._rx_buffer[0]
                continue

            # Parse header
            header = UwbHeader.parse(bytes(self._rx_buffer[:UwbHeader.SIZE]))
            if header is None:
                del self._rx_buffer[0]
                continue

            # Validate header
            if header.start_marker != UWB_START_MARKER or header.version != UWB_PROTOCOL_VERSION:
                del self._rx_buffer[0]
                continue

            # Check if we have full message
            if len(self._rx_buffer) < UwbHeader.SIZE + header.payload_length:
                return None

            # Extract payload
            payload = bytes(self._rx_buffer[UwbHeader.SIZE:UwbHeader.SIZE + header.payload_length])
            del self._rx_buffer[:UwbHeader.SIZE + header.payload_length]

            return header.msg_type, payload

        return None

    def process_data(self, data: bytes) -> list:
        """
        Process raw data and return all complete messages.

        Args:
            data: Raw bytes from UWB

        Returns:
            List of (msg_type, payload) tuples
        """
        self.feed_data(data)
        messages = []

        while True:
            msg = self.read_message()
            if msg is None:
                break
            messages.append(msg)

        return messages
