"""
Set User Data Message
"""

import struct
from dataclasses import dataclass


@dataclass
class SetUserData:
    """Set user data message containing 64 bytes of arbitrary data."""
    user_data: bytes

    TYPE_ID = 0x01
    FORMAT = '<64s'  # user_data[64]

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            'user_data': list(self.user_data),
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'SetUserData':
        """Create from dictionary."""
        return cls(
            user_data=bytes(data.get('user_data', [0] * 64))[:64].ljust(64, b'\x00'),
        )

    def encode(self) -> bytes:
        """Encode payload to bytes."""
        return struct.pack(
            self.FORMAT,
            bytes(self.user_data)[:64].ljust(64, b'\x00'),
        )

    @classmethod
    def parse(cls, data: bytes) -> 'SetUserData':
        """Parse payload from bytes."""
        (user_data,) = struct.unpack(cls.FORMAT, data)
        return cls(user_data=user_data)
