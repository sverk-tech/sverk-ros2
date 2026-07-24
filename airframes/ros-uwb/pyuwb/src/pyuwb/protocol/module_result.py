"""
Module Result Message
"""

import struct
from dataclasses import dataclass


@dataclass
class ModuleResult:
    """Module result message containing ranging and AoA data."""
    id: int
    range: float
    aoa: float

    TYPE_ID = 0x02
    FORMAT = '<Hff'  # id, range, aoa

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            'id': self.id,
            'range': self.range,
            'aoa': self.aoa,
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'ModuleResult':
        """Create from dictionary."""
        return cls(
            id=data.get('id', 0),
            range=data.get('range', 0.0),
            aoa=data.get('aoa', 0.0),
        )

    def encode(self) -> bytes:
        """Encode payload to bytes."""
        return struct.pack(
            self.FORMAT,
            int(self.id) & 0xFFFF,
            float(self.range),
            float(self.aoa),
        )

    @classmethod
    def parse(cls, data: bytes) -> 'ModuleResult':
        """Parse payload from bytes."""
        id_val, range_val, aoa_val = struct.unpack(cls.FORMAT, data)
        return cls(id=id_val, range=range_val, aoa=aoa_val)
