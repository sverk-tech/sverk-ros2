#!/usr/bin/env python3
"""
Тест param_dds_bridge: чтение/запись параметров PX4 по ИМЕНИ через uXRCE-DDS.

Требует прошивку с модулем param_dds_bridge и пересобранный px4_msgs с
ParamBridgeRequest/Response.

Публикует ParamBridgeRequest в /fmu/in/param_bridge_request и ждёт
ParamBridgeResponse из /fmu/out/param_bridge_response.

Примеры:
  # прочитать параметр
  python3 param_dds_test_node.py --get MPC_XY_CRUISE
  # записать float
  python3 param_dds_test_node.py --set MPC_XY_CRUISE --value 8
  # записать int
  python3 param_dds_test_node.py --set COM_ARM_WO_GPS --value 1
  # сбросить один / все
  python3 param_dds_test_node.py --reset MPC_XY_CRUISE
  python3 param_dds_test_node.py --reset-all
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import ParamBridgeRequest, ParamBridgeResponse

NAME_LEN = 17  # char[17]

QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

TYPE_NAMES = {0: 'INT32', 1: 'FLOAT', 255: 'UNKNOWN'}


def name_to_field(name: str):
    """PX4 char[17] name -> list of 17 char codes (null-padded)."""
    raw = name.encode('ascii')[:NAME_LEN - 1]
    return list(raw) + [0] * (NAME_LEN - len(raw))


def field_to_name(field) -> str:
    b = bytes(int(c) & 0xFF for c in field)
    return b.split(b'\x00', 1)[0].decode('ascii', 'replace')


class ParamBridgeTest(Node):
    def __init__(self):
        super().__init__('param_bridge_test')
        self._resp = None
        self._pub = self.create_publisher(
            ParamBridgeRequest, '/fmu/in/param_bridge_request', QOS_PUB)
        self.create_subscription(
            ParamBridgeResponse, '/fmu/out/param_bridge_response',
            self._on_resp, QOS_SUB)

    def _on_resp(self, msg: ParamBridgeResponse):
        self._resp = msg

    def send(self, operation, name='', value=None):
        req = ParamBridgeRequest()
        req.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        req.operation = operation
        req.name = name_to_field(name)
        if value is not None:
            # Модуль сам выберет поле по реальному типу параметра —
            # заполняем оба.
            try:
                req.int_value = int(round(float(value)))
            except (TypeError, ValueError):
                req.int_value = 0
            req.float_value = float(value)
        self._pub.publish(req)


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--get', metavar='NAME')
    g.add_argument('--set', metavar='NAME')
    g.add_argument('--reset', metavar='NAME')
    g.add_argument('--reset-all', action='store_true')
    g.add_argument('--save', action='store_true')
    ap.add_argument('--value')
    ap.add_argument('--timeout', type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = ParamBridgeTest()

    # дать discovery подняться
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if args.get:
        op, name, val = ParamBridgeRequest.OPERATION_GET, args.get, None
    elif args.set:
        if args.value is None:
            ap.error('--set требует --value')
        op, name, val = ParamBridgeRequest.OPERATION_SET, args.set, args.value
    elif args.reset:
        op, name, val = ParamBridgeRequest.OPERATION_RESET, args.reset, None
    elif args.reset_all:
        op, name, val = ParamBridgeRequest.OPERATION_RESET_ALL, '', None
    else:
        op, name, val = ParamBridgeRequest.OPERATION_SAVE, '', None

    node.send(op, name, val)
    print(f'SENT op={op} name={name!r} value={val}')

    end = time.time() + args.timeout
    while time.time() < end and node._resp is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('\n===== RESPONSE =====')
    if node._resp is None:
        print('  таймаут: ответа нет. Модуль не получил запрос или не '
              'публикует (проверь param_dds_bridge status на FMU).')
        rc = 1
    else:
        r = node._resp
        tname = TYPE_NAMES.get(r.param_type, str(r.param_type))
        value = r.int_value if r.param_type == 0 else r.float_value
        print(f'  name    : {field_to_name(r.name)!r}')
        print(f'  found   : {r.found}')
        print(f'  success : {r.success}')
        print(f'  type    : {tname}')
        print(f'  value   : {value}')
        rc = 0 if (r.found and r.success) else 1

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
