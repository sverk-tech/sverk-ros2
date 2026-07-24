"""
LUT Chunk Acknowledgment Message
"""

import struct
from dataclasses import dataclass


@dataclass
class LutChunkAck:
    """LUT chunk acknowledgment message."""
    offset: int
    status: int

    TYPE_ID = 0x41
    FORMAT = '<HB'

    def to_dict(self) -> dict:
        return {'offset': self.offset, 'status': self.status}

    @classmethod
    def from_dict(cls, data: dict) -> 'LutChunkAck':
        return cls(offset=data.get('offset', 0), status=data.get('status', 0))

    def encode(self) -> bytes:
        return struct.pack(self.FORMAT, int(self.offset) & 0xFFFF, int(self.status) & 0xFF)

    @classmethod
    def parse(cls, data: bytes) -> 'LutChunkAck':
        offset, status = struct.unpack(cls.FORMAT, data)
        return cls(offset=offset, status=status)
