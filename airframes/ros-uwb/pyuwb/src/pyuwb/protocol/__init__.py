"""
UWB Protocol Constants and Definitions
"""

# Protocol constants
UWB_START_MARKER = 0xAA
UWB_PROTOCOL_VERSION = 1


# Import all message types
from .header import UwbHeader
from .module_result import ModuleResult
from .peer_info import PeerInfo
from .set_user_data import SetUserData
from .set_addr import SetAddr
from .ping_request import PingRequest
from .pong_result import PongResult
from .lut_chunk import LutChunk
from .lut_chunk_ack import LutChunkAck

# Message type constants (for backward compatibility)
UWB_MSG_SET_USER_DATA = SetUserData.TYPE_ID
UWB_MSG_RESULT = ModuleResult.TYPE_ID
UWB_MSG_PEER_INFO = PeerInfo.TYPE_ID
UWB_MSG_SET_ADDR = SetAddr.TYPE_ID
UWB_MSG_PING_REQUEST = PingRequest.TYPE_ID
UWB_MSG_PONG_RESULT = PongResult.TYPE_ID
UWB_MSG_LUT_CHUNK = LutChunk.TYPE_ID
UWB_MSG_LUT_ACK = LutChunkAck.TYPE_ID

__all__ = [
    'UwbHeader',
    'UWB_START_MARKER',
    'UWB_PROTOCOL_VERSION',
    'UWB_MSG_RESULT',
    'UWB_MSG_PEER_INFO',
    'UWB_MSG_SET_USER_DATA',
    'UWB_MSG_SET_ADDR',
    'UWB_MSG_PING_REQUEST',
    'UWB_MSG_PONG_RESULT',
    'UWB_MSG_LUT_CHUNK',
    'UWB_MSG_LUT_ACK',
    'ModuleResult',
    'PeerInfo',
    'SetUserData',
    'SetAddr',
    'PingRequest',
    'PongResult',
    'LutChunk',
    'LutChunkAck',
]
