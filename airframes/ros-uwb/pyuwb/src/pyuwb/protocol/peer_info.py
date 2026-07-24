"""
Peer Info Message
"""

import struct
from dataclasses import dataclass


@dataclass
class PeerInfo:
    """Peer info message containing id and user data."""
    id: int
    user_data: bytes

    TYPE_ID = 0x03
    FORMAT = '<H64s'  # id, user_data[64]

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            'id': self.id,
            'user_data': list(self.user_data),
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'PeerInfo':
        """Create from dictionary."""
        return cls(
            id=data.get('id', 0),
            user_data=bytes(data.get('user_data', [0] * 64))[:64].ljust(64, b'\x00'),
        )

    def encode(self) -> bytes:
        """Encode payload to bytes."""
        return struct.pack(
            self.FORMAT,
            int(self.id) & 0xFFFF,
            bytes(self.user_data)[:64].ljust(64, b'\x00'),
        )

    @classmethod
    def parse(cls, data: bytes) -> 'PeerInfo':
        """Parse payload from bytes."""
        id_val, user_data = struct.unpack(cls.FORMAT, data)
        return cls(id=id_val, user_data=user_data)
