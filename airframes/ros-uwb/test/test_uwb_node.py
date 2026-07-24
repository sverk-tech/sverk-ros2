#!/usr/bin/env python3
"""
Tests for ros_uwb UwbNode
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
from threading import Thread
import time

import rclpy
from rclpy.node import Node
from ros_uwb.msg import UwbResult, UwbPeerInfo
from ros_uwb.srv import SetUserData

from pyuwb import (
    UwbParser,
    UwbEncoder,
    UWB_MSG_RESULT,
    UWB_MSG_PEER_INFO,
    UWB_MSG_SET_USER_DATA,
    ModuleResult,
    PeerInfo,
    SetUserData as UwbSetUserData,
)


class MockSerial:
    """Mock serial port for testing."""
    
    def __init__(self, port, baudrate, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self._write_buffer = []
        self._read_buffer = bytearray()
        
    def reset_input_buffer(self):
        self._read_buffer = bytearray()
        
    def write(self, data):
        self._write_buffer.append(data)
        
    @property
    def in_waiting(self):
        return len(self._read_buffer)
    
    def read(self, size):
        data = bytes(self._read_buffer[:size])
        del self._read_buffer[:size]
        return data
    
    def close(self):
        self.is_open = False


@pytest.fixture
def node():
    """Fixture providing UwbNode instance."""
    rclpy.init()
    try:
        with patch('serial.Serial', return_value=MockSerial('/dev/ttyUSB0', 115200)):
            from uwb_node import UwbNode
            node = UwbNode()
            yield node
            node.destroy_node()
    finally:
        rclpy.shutdown()


class TestUwbNode:
    """Test cases for UwbNode."""
    
    def test_node_initialization(self, node):
        """Test that node initializes correctly."""
        assert node is not None
        assert node._parser is not None
        assert isinstance(node._parser, UwbParser)
    
    def test_publishers_created(self, node):
        """Test that publishers are created."""
        assert node._result_pub is not None
        assert node._peer_info_pub is not None
    
    def test_service_created(self, node):
        """Test that service is created."""
        assert node._set_user_data_srv is not None
    
    def test_module_result_message_dispatch(self, node):
        """Test dispatch of ModuleResult message."""
        # Create a valid ModuleResult payload
        result = ModuleResult(id=1, range=2.5, aoa=0.5)
        payload = result.encode()
        
        # Mock the publishers
        result_msg = None
        def capture_result(msg):
            nonlocal result_msg
            result_msg = msg
        
        node._result_pub.publish = capture_result
        
        # Dispatch the message
        node._dispatch(UWB_MSG_RESULT, payload)
        
        # Verify the message was published
        assert result_msg is not None
        assert result_msg.id == 1
        assert result_msg.range == 2.5
        assert result_msg.aoa == 0.5
    
    def test_peer_info_message_dispatch(self, node):
        """Test dispatch of PeerInfo message."""
        # Create a valid PeerInfo payload
        user_data = bytes([1, 2, 3, 4, 5])
        peer_info = PeerInfo(id=2, user_data=user_data)
        payload = peer_info.encode()
        
        # Mock the publishers
        peer_msg = None
        def capture_peer(msg):
            nonlocal peer_msg
            peer_msg = msg
        
        node._peer_info_pub.publish = capture_peer
        
        # Dispatch the message
        node._dispatch(UWB_MSG_PEER_INFO, payload)
        
        # Verify the message was published
        assert peer_msg is not None
        assert peer_msg.id == 2
        assert list(peer_msg.user_data[:5]) == [1, 2, 3, 4, 5]
    
    def test_set_user_data_service_success(self, node):
        """Test set_user_data service success."""
        # Create request
        request = SetUserData.Request()
        request.user_data = [1, 2, 3, 4, 5]
        
        # Mock serial write
        written_data = []
        def mock_write(data):
            written_data.append(data)
        
        node._serial.write = mock_write
        
        # Call service
        response = node._handle_set_user_data(request, SetUserData.Response())
        
        # Verify response
        assert response.success is True
        assert response.message == 'OK'
        
        # Verify data was written
        assert len(written_data) == 1
    
    def test_set_user_data_service_failure(self, node):
        """Test set_user_data service failure."""
        # Create invalid request
        request = SetUserData.Request()
        request.user_data = []
        
        # Mock serial write to raise exception
        def mock_write(data):
            raise Exception("Serial error")
        
        node._serial.write = mock_write
        
        # Call service
        response = node._handle_set_user_data(request, SetUserData.Response())
        
        # Verify response
        assert response.success is False
        assert "Serial error" in response.message
    
    def test_invalid_message_type_ignored(self, node):
        """Test that unknown message types are ignored."""
        # Use an invalid message type
        invalid_payload = bytes([0x00, 0x01, 0x02])
        
        # Mock publishers to ensure nothing is published
        result_published = False
        peer_published = False
        
        def capture_result(msg):
            nonlocal result_published
            result_published = True
        
        def capture_peer(msg):
            nonlocal peer_published
            peer_published = True
        
        node._result_pub.publish = capture_result
        node._peer_info_pub.publish = capture_peer
        
        # Dispatch with invalid type
        node._dispatch(0xFF, invalid_payload)
        
        # Verify nothing was published
        assert result_published is False
        assert peer_published is False
