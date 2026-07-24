#!/usr/bin/env python3
"""
UWB ROS2 driver node.

Subscribes to a serial UART device running the UWB protocol and:
  - Publishes UWB_MSG_RESULT   -> /uwb/result    (ros_uwb/UwbResult)
  - Publishes UWB_MSG_PEER_INFO -> /uwb/peer_info (ros_uwb/UwbPeerInfo)
  - Serves   UWB_MSG_SET_USER_DATA via /uwb/set_user_data (ros_uwb/SetUserData)

Parameters:
  port     (string, default '/dev/ttyAMA0') - serial port
  baudrate (int,    default 500000)         - baud rate
"""

import threading

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
from ros_uwb.msg import UwbPeerInfo, UwbResult
from ros_uwb.srv import SetUserData


class UwbNode(Node):

    def __init__(self):
        super().__init__('uwb_node')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 500000)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._result_pub = self.create_publisher(UwbResult, 'uwb/result', qos)
        self._peer_info_pub = self.create_publisher(UwbPeerInfo, 'uwb/peer_info', qos)

        self._set_user_data_srv = self.create_service(
            SetUserData, 'uwb/set_user_data', self._handle_set_user_data
        )

        self._parser = UwbParser()
        self._serial_lock = threading.Lock()

        try:
            self._serial = serial.Serial(port, baudrate=baudrate, timeout=0.1)
            self._serial.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open serial port {port}: {e}')
            raise

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(f'UWB node started on {port} @ {baudrate} baud')

    def _read_loop(self):
        while self._running:
            try:
                waiting = self._serial.in_waiting
                if waiting > 0:
                    data = self._serial.read(waiting)
                    for msg_type, payload in self._parser.process_data(data):
                        self._dispatch(msg_type, payload)
            except serial.SerialException as e:
                if self._running:
                    self.get_logger().error(f'Serial read error: {e}')

    def _dispatch(self, msg_type: int, payload: bytes):
        now = self.get_clock().now().to_msg()

        if msg_type == UWB_MSG_RESULT:
            try:
                parsed = ModuleResult.parse(payload)
            except Exception as e:
                self.get_logger().warn(f'Failed to parse UwbResult: {e}')
                return
            msg = UwbResult()
            msg.header.stamp = now
            msg.id = parsed.id
            msg.range = parsed.range
            msg.aoa = parsed.aoa
            self._result_pub.publish(msg)

        elif msg_type == UWB_MSG_PEER_INFO:
            try:
                parsed = PeerInfo.parse(payload)
            except Exception as e:
                self.get_logger().warn(f'Failed to parse UwbPeerInfo: {e}')
                return
            msg = UwbPeerInfo()
            msg.header.stamp = now
            msg.id = parsed.id
            msg.user_data = list(parsed.user_data[:64])
            self._peer_info_pub.publish(msg)

    def _handle_set_user_data(
        self, request: SetUserData.Request, response: SetUserData.Response
    ) -> SetUserData.Response:
        try:
            payload = UwbSetUserData(user_data=bytes(request.user_data)).encode()
            packet = UwbEncoder.encode_packet(UWB_MSG_SET_USER_DATA, payload)
            with self._serial_lock:
                self._serial.write(packet)
            response.success = True
            response.message = 'OK'
        except Exception as e:
            self.get_logger().error(f'set_user_data failed: {e}')
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
