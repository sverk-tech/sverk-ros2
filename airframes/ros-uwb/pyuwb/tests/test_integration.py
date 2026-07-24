"""Integration tests for UART communication simulation."""

import pytest
from pyuwb.encoder import UwbEncoder
from pyuwb.parser import UwbParser
from pyuwb.protocol import (
    ModuleResult,
    PeerInfo,
    SetUserData,
    SetAddr,
    PingRequest,
    PongResult,
)


class MockSerial:
    """Mock serial port for testing."""
    
    def __init__(self):
        self.write_buffer = bytearray()
        self.read_buffer = bytearray()
    
    def write(self, data):
        """Write data to buffer."""
        self.write_buffer.extend(data)
        return len(data)
    
    def read(self, size):
        """Read data from buffer."""
        data = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        return data
    
    def in_waiting(self):
        """Return number of bytes waiting."""
        return len(self.read_buffer)


def test_uart_communication_roundtrip():
    """Test complete UART communication simulation."""
    # Setup
    serial = MockSerial()
    parser = UwbParser()
    
    # Send message (encode and write)
    msg = ModuleResult(id=42, range=1.23, aoa=45.67)
    payload = msg.encode()
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload)
    serial.write(packet)
    
    # Simulate receiving (read and parse)
    serial.read_buffer = serial.write_buffer
    data = serial.read(serial.in_waiting())
    messages = parser.process_data(data)
    
    # Verify
    assert len(messages) == 1
    msg_type, parsed_payload = messages[0]
    assert msg_type == ModuleResult.TYPE_ID
    
    parsed = ModuleResult.parse(parsed_payload)
    assert parsed.id == msg.id
    assert parsed.range == pytest.approx(msg.range, rel=1e-6)
    assert parsed.aoa == pytest.approx(msg.aoa, rel=1e-6)


def test_multiple_messages_communication():
    """Test multiple messages over UART."""
    serial = MockSerial()
    parser = UwbParser()
    
    # Send multiple messages
    messages = [
        ModuleResult(id=1, range=1.0, aoa=10.0),
        PeerInfo(id=2, user_data=bytes(range(64))),
        SetUserData(user_data=bytes([1, 2, 3, 4] + [0] * 60)),
    ]
    
    for msg in messages:
        payload = msg.encode()
        packet = UwbEncoder.encode_packet(msg.TYPE_ID, payload)
        serial.write(packet)
    
    # Receive and parse
    serial.read_buffer = serial.write_buffer
    data = serial.read(serial.in_waiting())
    parsed_messages = parser.process_data(data)
    
    # Verify
    assert len(parsed_messages) == len(messages)
    
    for i, (msg_type, payload) in enumerate(parsed_messages):
        if i == 0:
            assert msg_type == ModuleResult.TYPE_ID
            parsed = ModuleResult.parse(payload)
            assert parsed.id == messages[0].id
        elif i == 1:
            assert msg_type == PeerInfo.TYPE_ID
            parsed = PeerInfo.parse(payload)
            assert parsed.id == messages[1].id
        elif i == 2:
            assert msg_type == SetUserData.TYPE_ID
            parsed = SetUserData.parse(payload)
            assert parsed.user_data == messages[2].user_data


def test_command_response_scenario():
    """Test command-response scenario."""
    serial = MockSerial()
    parser = UwbParser()
    
    # Send command
    cmd = SetAddr(addr=5, timeslot=10)
    cmd_payload = cmd.encode()
    cmd_packet = UwbEncoder.encode_packet(SetAddr.TYPE_ID, cmd_payload)
    serial.write(cmd_packet)
    
    # Simulate response
    response = PongResult(
        id=5,
        pdoa=1.5,
        aoa=30.0,
        rssi=-50.5,
        cir_amplitude=1000.0
    )
    response_payload = response.encode()
    response_packet = UwbEncoder.encode_packet(PongResult.TYPE_ID, response_payload)
    serial.write(response_packet)
    
    # Receive and parse
    serial.read_buffer = serial.write_buffer
    data = serial.read(serial.in_waiting())
    messages = parser.process_data(data)
    
    # Verify
    assert len(messages) == 2
    assert messages[0][0] == SetAddr.TYPE_ID
    assert messages[1][0] == PongResult.TYPE_ID
    
    parsed_response = PongResult.parse(messages[1][1])
    assert parsed_response.id == response.id


def test_fragmented_data():
    """Test handling fragmented data."""
    serial = MockSerial()
    parser = UwbParser()
    
    # Send message
    msg = ModuleResult(id=42, range=1.23, aoa=45.67)
    payload = msg.encode()
    packet = UwbEncoder.encode_packet(ModuleResult.TYPE_ID, payload)
    
    # Simulate fragmented reception
    chunk_size = 5
    for i in range(0, len(packet), chunk_size):
        chunk = packet[i:i+chunk_size]
        messages = parser.process_data(chunk)
        if i + chunk_size < len(packet):
            assert len(messages) == 0  # Not complete yet
        else:
            assert len(messages) == 1  # Complete message
            parsed = ModuleResult.parse(messages[0][1])
            assert parsed.id == msg.id
