"""
Pong Result Message
"""

import struct
from dataclasses import dataclass


@dataclass
class PongResult:
    """Pong result message containing id, PDOA, AoA, RSSI and CIR amplitude data."""
    id: int
    pdoa: float
    aoa: float
    rssi: float
    cir_amplitude: float

    TYPE_ID = 0x33
    FORMAT = '<Hffff'  # id, pdoa, aoa, rssi, cir_amplitude

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'pdoa': self.pdoa,
            'aoa': self.aoa,
            'rssi': self.rssi,
            'cir_amplitude': self.cir_amplitude,
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'PongResult':
        return cls(
            id=data.get('id', 0),
            pdoa=data.get('pdoa', 0.0),
            aoa=data.get('aoa', 0.0),
            rssi=data.get('rssi', 0.0),
            cir_amplitude=data.get('cir_amplitude', 0.0),
        )

    def encode(self) -> bytes:
        return struct.pack(
            self.FORMAT,
            int(self.id) & 0xFFFF,
            float(self.pdoa), float(self.aoa),
            float(self.rssi), float(self.cir_amplitude),
        )

    @classmethod
    def parse(cls, data: bytes) -> 'PongResult':
        id_val, pdoa, aoa, rssi, cir = struct.unpack(cls.FORMAT, data)
        return cls(id=id_val, pdoa=pdoa, aoa=aoa, rssi=rssi, cir_amplitude=cir)
