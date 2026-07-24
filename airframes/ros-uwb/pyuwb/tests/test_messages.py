"""Tests for message encoding and parsing."""

import pytest
from pyuwb.protocol import (
    ModuleResult,
    PeerInfo,
    SetUserData,
    SetAddr,
    PingRequest,
    PongResult,
    LutChunk,
    LutChunkAck,
)


def test_module_result_encode_parse():
    """Test ModuleResult encode/parse."""
    original = ModuleResult(id=42, range=1.23, aoa=45.67)
    data = original.encode()
    parsed = ModuleResult.parse(data)
    assert parsed.id == original.id
    assert parsed.range == pytest.approx(original.range, rel=1e-6)
    assert parsed.aoa == pytest.approx(original.aoa, rel=1e-6)


def test_peer_info_encode_parse():
    """Test PeerInfo encode/parse."""
    user_data = bytes(range(64))
    original = PeerInfo(id=10, user_data=user_data)
    data = original.encode()
    parsed = PeerInfo.parse(data)
    assert parsed.id == original.id
    assert parsed.user_data == original.user_data


def test_set_user_data_encode_parse():
    """Test SetUserData encode/parse."""
    user_data = bytes([1, 2, 3, 4] + [0] * 60)
    original = SetUserData(user_data=user_data)
    data = original.encode()
    parsed = SetUserData.parse(data)
    assert parsed.user_data == original.user_data


def test_set_addr_encode_parse():
    """Test SetAddr encode/parse."""
    original = SetAddr(addr=5, timeslot=10)
    data = original.encode()
    parsed = SetAddr.parse(data)
    assert parsed.addr == original.addr
    assert parsed.timeslot == original.timeslot


def test_ping_request_encode_parse():
    """Test PingRequest encode/parse."""
    original = PingRequest(require_tdma=True)
    data = original.encode()
    parsed = PingRequest.parse(data)
    assert parsed.require_tdma == original.require_tdma


def test_pong_result_encode_parse():
    """Test PongResult encode/parse."""
    original = PongResult(
        id=100,
        pdoa=1.5,
        aoa=30.0,
        rssi=-50.5,
        cir_amplitude=1000.0
    )
    data = original.encode()
    parsed = PongResult.parse(data)
    assert parsed.id == original.id
    assert parsed.pdoa == original.pdoa
    assert parsed.aoa == original.aoa
    assert parsed.rssi == original.rssi
    assert parsed.cir_amplitude == original.cir_amplitude


def test_lut_chunk_encode_parse():
    """Test LutChunk encode/parse."""
    original = LutChunk(offset=0, count=4, data=[100, 200, -50, -100])
    data = original.encode()
    parsed = LutChunk.parse(data)
    assert parsed.offset == original.offset
    assert parsed.count == original.count
    assert parsed.data == original.data


def test_lut_chunk_ack_encode_parse():
    """Test LutChunkAck encode/parse."""
    original = LutChunkAck(offset=0, status=0)
    data = original.encode()
    parsed = LutChunkAck.parse(data)
    assert parsed.offset == original.offset
    assert parsed.status == original.status


def test_module_result_to_dict_from_dict():
    """Test ModuleResult to_dict/from_dict."""
    original = ModuleResult(id=42, range=1.23, aoa=45.67)
    d = original.to_dict()
    restored = ModuleResult.from_dict(d)
    assert restored.id == original.id
    assert restored.range == original.range
    assert restored.aoa == original.aoa
