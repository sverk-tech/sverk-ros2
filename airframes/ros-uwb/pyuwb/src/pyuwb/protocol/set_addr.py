"""
Set Address Message
"""

import struct
from dataclasses import dataclass


@dataclass
class SetAddr:
    """Set address command containing module address and timeslot."""
    addr: int
    timeslot: int

    TYPE_ID = 0x31
    FORMAT = '<BB'  # addr, timeslot

    def to_dict(self) -> dict:
        return {'addr': self.addr, 'timeslot': self.timeslot}

    @classmethod
    def from_dict(cls, data: dict) -> 'SetAddr':
        return cls(addr=data.get('addr', 0), timeslot=data.get('timeslot', 0))

    def encode(self) -> bytes:
        return struct.pack(self.FORMAT, int(self.addr) & 0xFF, int(self.timeslot) & 0xFF)

    @classmethod
    def parse(cls, data: bytes) -> 'SetAddr':
        addr, timeslot = struct.unpack(cls.FORMAT, data)
        return cls(addr=addr, timeslot=timeslot)
