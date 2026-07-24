"""
Ping Request Message
"""

import struct
from dataclasses import dataclass


@dataclass
class PingRequest:
    """Ping request message."""
    require_tdma: bool

    TYPE_ID = 0x32
    FORMAT = '<?'

    def to_dict(self) -> dict:
        return {'require_tdma': self.require_tdma}

    @classmethod
    def from_dict(cls, data: dict) -> 'PingRequest':
        return cls(require_tdma=bool(data.get('require_tdma', False)))

    def encode(self) -> bytes:
        return struct.pack(self.FORMAT, bool(self.require_tdma))

    @classmethod
    def parse(cls, data: bytes) -> 'PingRequest':
        (require_tdma,) = struct.unpack(cls.FORMAT, data)
        return cls(require_tdma=bool(require_tdma))
