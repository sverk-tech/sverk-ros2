"""Tests for UwbEncoder."""

import pytest
from pyuwb.encoder import UwbEncoder
from pyuwb.protocol import (
    UWB_START_MARKER,
    UWB_PROTOCOL_VERSION,
    ModuleResult,
    SetUserData,
)
from pyuwb.protocol.header import UwbHeader


def test_encoder_encode_packet():
    """Test packet encoding."""
    msg_type = 0x02
    payload = b'\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    timestamp_ms = 1000
    packet = UwbEncoder.encode_packet(msg_type, payload, timestamp_ms=timestamp_ms)
    
    # Check header
    header = UwbHeader.parse(packet[:UwbHeader.SIZE])
    assert header is not None
    assert header.start_marker == UWB_START_MARKER
    assert header.version == UWB_PROTOCOL_VERSION
    assert header.msg_type == msg_type  # msg_type
    assert header.payload_length == len(payload)
    assert header.timestamp == timestamp_ms

    # Check payload
    assert packet[UwbHeader.SIZE:] == payload


def test_encoder_encode_packet_auto_timestamp():
    """Test packet encoding with auto timestamp."""
    msg_type = 0x02
    payload = b'\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    import time
    timestamp_ms = time.monotonic() * 1000
    packet = UwbEncoder.encode_packet(msg_type, payload)

    # Check header
    header = UwbHeader.parse(packet[:UwbHeader.SIZE])
    assert header is not None
    assert header.start_marker == UWB_START_MARKER
    assert header.version == UWB_PROTOCOL_VERSION
    assert header.msg_type == msg_type  # msg_type
    assert header.payload_length == len(payload)
    assert abs(header.timestamp - timestamp_ms) < 100

    # Check payload
    assert packet[UwbHeader.SIZE:] == payload


def test_encoder_encode_message():
    """Test encoding a complete message."""
    msg = ModuleResult(id=42, range=1.23, aoa=45.67)
    payload = msg.encode()
    timestamp_ms = 1000
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload, timestamp_ms=timestamp_ms)

    # Check payload
    assert len(packet) == UwbHeader.SIZE + len(payload)
    assert packet[UwbHeader.SIZE:] == payload
