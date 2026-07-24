"""Tests for UwbParser."""

import pytest
from pyuwb.parser import UwbParser
from pyuwb.encoder import UwbEncoder
from pyuwb.protocol import (
    UwbHeader,
    ModuleResult,
    PeerInfo,
    SetUserData,
    UWB_START_MARKER,
    UWB_PROTOCOL_VERSION,
)


def test_parser_feed_data():
    """Test feeding data to parser."""
    parser = UwbParser()
    data = b'\xAA\x01\x02\x00\x0C\x00\x00\x00\x10'
    parser.feed_data(data)
    assert len(parser._rx_buffer) == len(data)


def test_parser_reset():
    """Test parser reset."""
    parser = UwbParser()
    parser.feed_data(b'\xAA\x01\x02\x00\x0C\x00\x00\x00\x10')
    parser.reset()
    assert len(parser._rx_buffer) == 0


def test_parser_read_message():
    """Test reading a complete message."""
    parser = UwbParser()
    
    # Create a valid packet
    msg = ModuleResult(id=42, range=1.23, aoa=45.67)
    payload = msg.encode()
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload, timestamp_ms=1000)
    
    parser.feed_data(packet)
    msg_type, parsed_payload = parser.read_message()
    
    assert msg_type == ModuleResult.TYPE_ID
    assert parsed_payload == payload
    assert len(parser._rx_buffer) == 0


def test_parser_process_data():
    """Test processing data."""
    parser = UwbParser()
    
    # Create two messages
    msg1 = ModuleResult(id=42, range=1.23, aoa=45.67)
    msg2 = PeerInfo(id=10, user_data=bytes(range(64)))
    
    packet1 = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, msg1.encode(), timestamp_ms=1000)
    packet2 = UwbEncoder.encode_packet(PeerInfo.TYPE_ID, msg2.encode(), timestamp_ms=2000)
    
    combined = packet1 + packet2
    messages = parser.process_data(combined)
    
    assert len(messages) == 2
    assert messages[0][0] == ModuleResult.TYPE_ID
    assert messages[1][0] == PeerInfo.TYPE_ID


def test_parser_partial_message():
    """Test handling partial message."""
    parser = UwbParser()
    
    msg = ModuleResult(id=42, range=1.23, aoa=45.67)
    payload = msg.encode()
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload, timestamp_ms=1000)
    
    # Feed first half of the packet
    parser.feed_data(packet[:len(packet)//2])
    result = parser.read_message()
    assert result is None  # Not enough data yet
    
    # Feed second half of the packet
    parser.feed_data(packet[len(packet)//2:])
    result = parser.read_message()
    assert result is not None  # Should now have complete message
    
    msg_type, parsed_payload = result
    assert msg_type == ModuleResult.TYPE_ID
    assert parsed_payload == payload


def test_parser_invalid_start_marker():
    """Test handling invalid start marker."""
    parser = UwbParser()
    
    # Invalid start marker
    invalid_data = b'\xFF\x01\x02\x00\x0C\x00\x00\x00\x10'
    parser.feed_data(invalid_data)

    # Parser should have consumed the invalid byte and kept the rest
    result = parser.read_message()
    assert result is None
    assert len(parser._rx_buffer) == len(invalid_data) - 1


def test_parser_invalid_version():
    """Test handling invalid version."""
    parser = UwbParser()
    
    # Create packet with invalid version
    msg = ModuleResult(id=42, range=1.23, aoa=45.67)
    payload = msg.encode()
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload, timestamp_ms=1000)
    
    # Corrupt version byte
    packet = bytearray(packet)
    packet[1] = 0xFF  # Invalid version
    packet = bytes(packet)
    
    parser.feed_data(packet)

    # Parser should have consumed all invalid bytes
    # keeping the part which doesn't fit the header
    result = parser.read_message()
    assert result is None
    assert len(parser._rx_buffer) < UwbHeader.SIZE


def test_parser_roundtrip():
    """Test complete encode/parse roundtrip."""
    parser = UwbParser()
    
    # Encode message
    original = ModuleResult(id=100, range=5.5, aoa=90.0)
    payload = original.encode()
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload, timestamp_ms=5000)
    
    # Parse message
    messages = parser.process_data(packet)
    assert len(messages) == 1
    
    msg_type, parsed_payload = messages[0]
    assert msg_type == ModuleResult.TYPE_ID
    
    # Decode message
    parsed = ModuleResult.parse(parsed_payload)
    assert parsed.id == original.id
    assert parsed.range == original.range
    assert parsed.aoa == original.aoa
