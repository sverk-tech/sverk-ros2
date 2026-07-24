"""
UWB Message Header
"""

import struct
from dataclasses import dataclass
from typing import Optional


@dataclass
class UwbHeader:
    """UWB message header."""
    start_marker: int
    version: int
    msg_type: int
    payload_length: int
    timestamp: int

    # Message header struct (little-endian)
    FORMAT = '<BBBHI'
    SIZE = struct.calcsize('<BBBHI')

    def encode(self) -> bytes:
        """Encode header to bytes."""
        return struct.pack(
            self.FORMAT,
            int(self.start_marker) & 0xFF,
            int(self.version) & 0xFF,
            int(self.msg_type) & 0xFF,
            int(self.payload_length) & 0xFFFF,
            int(self.timestamp) & 0xFFFFFFFF,
        )

    @classmethod
    def parse(cls, data: bytes) -> Optional['UwbHeader']:
        """Parse header from bytes."""
        if len(data) != cls.SIZE:
            return None

        try:
            start_marker, version, msg_type, payload_length, timestamp = struct.unpack(
                cls.FORMAT, data
            )

            return cls(
                start_marker=start_marker,
                version=version,
                msg_type=msg_type,
                payload_length=payload_length,
                timestamp=timestamp
            )
        except struct.error:
            return None
