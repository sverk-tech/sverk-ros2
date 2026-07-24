"""
UWB Packet Encoder
"""

import time
from typing import Optional

from .protocol import UWB_START_MARKER, UWB_PROTOCOL_VERSION
from .protocol.header import UwbHeader


class UwbEncoder:
    """UWB protocol packet encoder."""

    @staticmethod
    def encode_packet(msg_type: int, payload: bytes, timestamp_ms: Optional[int] = None) -> bytes:
        """Encode a UWB packet with header and payload."""
        if timestamp_ms is None:
            timestamp_ms = int(time.monotonic() * 1000)
        header = UwbHeader(
            start_marker=UWB_START_MARKER,
            version=UWB_PROTOCOL_VERSION,
            msg_type=msg_type,
            payload_length=len(payload),
            timestamp=timestamp_ms
        )
        return header.encode() + payload
