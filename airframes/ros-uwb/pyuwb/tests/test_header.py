"""Tests for UwbHeader."""

import pytest
from pyuwb.protocol.header import UwbHeader
from pyuwb.protocol import UWB_START_MARKER, UWB_PROTOCOL_VERSION


def test_header_encode():
    """Test header encoding."""
    header = UwbHeader(
        start_marker=UWB_START_MARKER,
        version=UWB_PROTOCOL_VERSION,
        msg_type=0x02,
        payload_length=12,
        timestamp=12345
    )
    data = header.encode()
    assert len(data) == UwbHeader.SIZE
    assert data[0] == UWB_START_MARKER
    assert data[1] == UWB_PROTOCOL_VERSION


def test_header_parse():
    """Test header parsing."""
    header = UwbHeader(
        start_marker=UWB_START_MARKER,
        version=UWB_PROTOCOL_VERSION,
        msg_type=0x02,
        payload_length=12,
        timestamp=12345
    )
    data = header.encode()
    parsed = UwbHeader.parse(data)
    assert parsed is not None
    assert parsed.start_marker == UWB_START_MARKER
    assert parsed.version == UWB_PROTOCOL_VERSION
    assert parsed.msg_type == 0x02
    assert parsed.payload_length == 12
    assert parsed.timestamp == 12345


def test_header_parse_invalid_length():
    """Test header parsing with invalid length."""
    data = b'\xAA\x01\x02\x00\x0C'  # Too short
    parsed = UwbHeader.parse(data)
    assert parsed is None


def test_header_encode_parse_roundtrip():
    """Test header encode/parse roundtrip."""
    original = UwbHeader(
        start_marker=UWB_START_MARKER,
        version=UWB_PROTOCOL_VERSION,
        msg_type=0x33,
        payload_length=64,
        timestamp=999999
    )
    data = original.encode()
    parsed = UwbHeader.parse(data)
    assert parsed.start_marker == original.start_marker
    assert parsed.version == original.version
    assert parsed.msg_type == original.msg_type
    assert parsed.payload_length == original.payload_length
    assert parsed.timestamp == original.timestamp
