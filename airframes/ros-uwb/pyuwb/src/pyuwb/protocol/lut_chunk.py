"""
LUT Chunk Message
"""

import struct
from dataclasses import dataclass
from typing import List


@dataclass
class LutChunk:
    """LUT chunk message containing offset, count and data."""
    offset: int
    count: int
    data: List[int]

    TYPE_ID = 0x40
    HEADER_FORMAT = '<HH'
    DATA_FORMAT = '<h'

    def to_dict(self) -> dict:
        return {'offset': self.offset, 'count': self.count, 'data': self.data}

    @classmethod
    def from_dict(cls, data: dict) -> 'LutChunk':
        return cls(offset=data.get('offset', 0), count=data.get('count', 0), data=data.get('data', []))

    def encode(self) -> bytes:
        payload = struct.pack(self.HEADER_FORMAT, int(self.offset) & 0xFFFF, int(self.count) & 0xFFFF)
        for value in self.data:
            payload += struct.pack(self.DATA_FORMAT, int(value))
        return payload

    @classmethod
    def parse(cls, data: bytes) -> 'LutChunk':
        if len(data) < 4:
            raise ValueError("Insufficient data for LUT chunk header")
        offset, count = struct.unpack(cls.HEADER_FORMAT, data[:4])
        elem_size = struct.calcsize(cls.DATA_FORMAT)
        if len(data) != 4 + count * elem_size:
            raise ValueError(f"Expected {count * elem_size} data bytes, got {len(data) - 4}")
        values = [struct.unpack(cls.DATA_FORMAT, data[4 + i*elem_size:4 + (i+1)*elem_size])[0] for i in range(count)]
        return cls(offset=offset, count=count, data=values)
