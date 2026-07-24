#!/usr/bin/env python3
"""
End-to-end проверка новой прошивки param_dds_bridge + проброшенных топиков —
ВСЁ по чистому DDS, без изменений в пакете fmu_control.

Что проверяет (каждый шаг с таймаутом, ничего не висит):
  1. telemetry  — приходят ли /fmu/out/{vehicle_status,battery_status,
                  vehicle_attitude,vehicle_gps_position}
  2. param by-name  — GET/SET/RESET MPC_XY_CRUISE
  3. param GET_INDEX / backup — проход param_for_index(0..total-1) с ретраями
  4. input_rc   — живые каналы RC (подвигай стики)
  5. calibration — триггер PREFLIGHT_CALIBRATION + живой поток [cal] из
                   /fmu/out/log_message (крути/ставь дрон по подсказкам)
  6. motors     — DO_MOTOR_TEST (на питании от кабеля моторы не крутятся —
                  просто проверяем, что команда уходит)

Требует: прошивку с param_dds_bridge + log_message/input_rc в dds_topics,
и пересобранный px4_msgs (ParamBridgeRequest/Response с index/total, GET_INDEX).

Запуск (после source install/setup.bash):
  python3 fmu_dds_e2e_check.py                 # весь сценарий по порядку
  python3 fmu_dds_e2e_check.py --only telemetry param rc
  python3 fmu_dds_e2e_check.py --only backup --backup-limit 0      # полный бэкап
  python3 fmu_dds_e2e_check.py --only cal --cal accel
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    ParamBridgeRequest, ParamBridgeResponse,
    VehicleCommand, VehicleStatus, BatteryStatus, VehicleAttitude,
    SensorGps, InputRc, LogMessage,
)

NAME_LEN = 17

QOS_IN = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
QOS_OUT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=50)

TYPE_NAMES = {0: 'INT32', 1: 'FLOAT', 255: 'UNKNOWN'}

# Есть ли в собранном px4_msgs новые поля ParamBridge (index/total/GET_INDEX)?
# Если нет — значит px4_msgs ещё не пересобран; param/backup пропускаем.
HAS_NEW_PARAM = 'index' in ParamBridgeRequest.get_fields_and_field_types()

# PREFLIGHT_CALIBRATION param mapping (which param slot triggers which sensor)
CAL_MAP = {
    'gyro':  dict(param1=1.0),
    'mag':   dict(param2=1.0),
    'baro':  dict(param3=1.0),
    'accel': dict(param5=1.0),
    'level': dict(param5=2.0),
}
CAL_TIMEOUTS = {'gyro': 30, 'baro': 20, 'level': 40, 'accel': 150, 'mag': 150}


def name_to_field(name):
    raw = name.encode('ascii')[:NAME_LEN - 1]
    return list(raw) + [0] * (NAME_LEN - len(raw))


def field_to_str(field):
    b = bytes(int(c) & 0xFF for c in field)
    return b.split(b'\x00', 1)[0].decode('ascii', 'replace')


def q_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


class E2ENode(Node):
    def __init__(self):
        super().__init__('fmu_dds_e2e_check')
        # publishers
        self._param_req = self.create_publisher(
            ParamBridgeRequest, '/fmu/in/param_bridge_request', QOS_IN)
        self._cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', QOS_IN)
        # param response correlation
        self._pending = None          # ('name', str) | ('index', int)
        self._resp = None
        self._ev = threading.Event()
        self.create_subscription(ParamBridgeResponse,
                                 '/fmu/out/param_bridge_response',
                                 self._on_param, QOS_OUT)
        # telemetry latest
        self.status = None
        self.batt = None
        self.att = None
        self.gps = None
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), QOS_OUT)
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status',
                                 lambda m: setattr(self, 'batt', m), QOS_OUT)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 lambda m: setattr(self, 'att', m), QOS_OUT)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position',
                                 lambda m: setattr(self, 'gps', m), QOS_OUT)
        # rc
        self.rc = None
        self.create_subscription(InputRc, '/fmu/out/input_rc',
                                 lambda m: setattr(self, 'rc', m), QOS_OUT)
        # calibration log stream
        self._print_cal = False
        self._cal_last = None
        self.create_subscription(LogMessage, '/fmu/out/log_message',
                                 self._on_log, QOS_OUT)

    # ---- callbacks ----
    def _on_param(self, m):
        if self._pending is None:
            return
        kind, key = self._pending
        if (kind == 'name' and field_to_str(m.name) == key) or \
           (kind == 'index' and m.index == key):
            self._resp = m
            self._ev.set()

    def _on_log(self, m):
        text = field_to_str(m.text)
        if self._print_cal and '[cal]' in text.lower():
            self._cal_last = text
            print(f'    [cal] >> {text}')

    # ---- helpers ----
    def _us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def param_call(self, operation, name='', index=0, value=None,
                   timeout=3.0, retries=3):
        req = ParamBridgeRequest()
        req.operation = operation
        req.name = name_to_field(name)
        req.index = index
        if value is not None:
            req.int_value = int(round(float(value)))
            req.float_value = float(value)
        self._pending = ('index', index) if \
            operation == ParamBridgeRequest.OPERATION_GET_INDEX else ('name', name)
        self._resp = None
        self._ev.clear()
        for _ in range(retries):
            req.timestamp = self._us()
            self._param_req.publish(req)
            if self._ev.wait(timeout / retries):
                break
        self._pending = None
        return self._resp

    def send_cmd(self, command, **params):
        m = VehicleCommand()
        m.timestamp = self._us()
        m.command = command
        for i in range(1, 8):
            setattr(m, f'param{i}', float(params.get(f'param{i}', 0.0)))
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self._cmd.publish(m)


# =====================================================================
# Tests
# =====================================================================

def test_telemetry(n, ok):
    print('\n== 1. TELEMETRY (собираю 4 c) ==')
    time.sleep(4.0)
    got = []
    if n.status is not None:
        print(f'  vehicle_status : arming={n.status.arming_state} '
              f'nav_state={n.status.nav_state} failsafe={n.status.failsafe}')
        got.append('status')
    if n.batt is not None:
        print(f'  battery        : {n.batt.voltage_v:.2f} V, '
              f'remaining={n.batt.remaining:.2f}')
        got.append('battery')
    if n.att is not None:
        r, p, y = q_to_rpy(n.att.q)
        print(f'  attitude       : roll={r:.1f} pitch={p:.1f} yaw={y:.1f} deg')
        got.append('attitude')
    if n.gps is not None:
        print(f'  gps            : fix={n.gps.fix_type} sats={n.gps.satellites_used}')
        got.append('gps')
    else:
        print('  gps            : нет данных (норм, если GPS не подключён)')
    ok['telemetry'] = 'status' in got and 'battery' in got and 'attitude' in got


def test_param(n, ok):
    print('\n== 2. PARAM BY-NAME (MPC_XY_CRUISE) ==')
    if not HAS_NEW_PARAM:
        print('  SKIP пропуск: px4_msgs без новых полей ParamBridge (index/total). '
              'Структура сообщения не совпадёт с прошивкой — нужен пересбор '
              'px4_msgs. Гоняй этот тест после пересборки.')
        ok['param'] = None
        return
    r = n.param_call(ParamBridgeRequest.OPERATION_GET, name='MPC_XY_CRUISE')
    if r is None or not r.found:
        print('  GET: НЕТ ОТВЕТА / не найден'); ok['param'] = False; return
    orig = r.float_value
    print(f'  GET  -> {orig} ({TYPE_NAMES.get(r.param_type)}), total={r.total}')
    newv = 8.0 if abs(orig - 8.0) > 0.1 else 6.0
    r = n.param_call(ParamBridgeRequest.OPERATION_SET, name='MPC_XY_CRUISE', value=newv)
    print(f'  SET {newv} -> success={r and r.success}, readback={r and r.float_value}')
    r = n.param_call(ParamBridgeRequest.OPERATION_RESET, name='MPC_XY_CRUISE')
    print(f'  RESET -> success={r and r.success}, value={r and r.float_value}')
    ok['param'] = bool(r and r.success)


def test_backup(n, ok, limit):
    print('\n== 3. PARAM GET_INDEX / BACKUP ==')
    if not HAS_NEW_PARAM:
        print('  SKIP пропуск: нужен пересобранный px4_msgs (GET_INDEX/index/total).')
        ok['backup'] = None
        return
    first = n.param_call(ParamBridgeRequest.OPERATION_GET_INDEX, index=0)
    if first is None:
        print('  index 0: НЕТ ОТВЕТА — GET_INDEX не работает в прошивке')
        ok['backup'] = False
        return
    total = first.total
    count = total if limit in (None, 0) else min(limit, total)
    print(f'  total={total}, читаю первые {count} по индексу (reliable)...')
    t0 = time.time()
    got = 0
    missed = []
    for i in range(count):
        r = n.param_call(ParamBridgeRequest.OPERATION_GET_INDEX, index=i,
                         timeout=2.0, retries=4)
        if r is None or not r.found:
            missed.append(i)
            continue
        got += 1
        if i < 5 or i % 50 == 0:
            v = r.int_value if r.param_type == 0 else r.float_value
            print(f'    [{i:4d}/{total}] {field_to_str(r.name):<16} = {v}')
    dt = time.time() - t0
    print(f'  получено {got}/{count} за {dt:.1f}s, пропусков (после ретраев): {len(missed)}')
    if missed:
        print(f'    missed indices: {missed[:20]}{"..." if len(missed) > 20 else ""}')
    ok['backup'] = (got == count)


def test_rc(n, ok, secs=15):
    print(f'\n== 4. INPUT_RC ({secs} c) — ПОДВИГАЙ СТИКИ ==')
    end = time.time() + secs
    seen = False
    last_print = 0
    while time.time() < end:
        if n.rc is not None:
            seen = True
            if time.time() - last_print > 0.5:
                ch = list(n.rc.values[:8])
                print(f'    RC ch1-8: {ch}   (count={n.rc.channel_count})')
                last_print = time.time()
        time.sleep(0.1)
    if not seen:
        print('  НЕТ данных input_rc — приёмник не подключён или топик не проброшен')
    ok['rc'] = seen


def test_calibration(n, ok, sensor):
    to = CAL_TIMEOUTS.get(sensor, 60)
    hint = {
        'gyro':  'держи дрон НЕПОДВИЖНО',
        'baro':  'ничего не делай',
        'level': 'поставь дрон РОВНО и держи неподвижно',
        'accel': 'ставь дрон по очереди на 6 сторон по подсказкам [cal]',
        'mag':   'ВРАЩАЙ дрон вокруг всех осей по подсказкам [cal]',
    }.get(sensor, '')
    print(f'\n== 5. CALIBRATION: {sensor.upper()} (таймаут {to} c) ==')
    print(f'  Действие: {hint}')
    try:
        input('  Дрон в дизарме, пропеллеры сняты. Enter — старт (Ctrl+C — пропустить)... ')
    except (EOFError, KeyboardInterrupt):
        print('  пропущено'); ok[f'cal_{sensor}'] = None; return

    n._cal_last = None
    n._print_cal = True
    n.send_cmd(VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION, **CAL_MAP[sensor])
    print(f'  команда отправлена (PREFLIGHT_CALIBRATION {CAL_MAP[sensor]}), слушаю [cal]...')

    end = time.time() + to
    done = None
    while time.time() < end:
        last = (n._cal_last or '').lower()
        if 'cal] calibration done' in last or ('done:' in last and '[cal]' in last):
            done = True; break
        if 'fail' in last and '[cal]' in last:
            done = False; break
        time.sleep(0.2)
    n._print_cal = False
    if done is True:
        print('  OK калибровка ЗАВЕРШЕНА')
    elif done is False:
        print('  FAIL калибровка FAILED (см. строки [cal] выше)')
    else:
        print(f'  timeout таймаут {to} c — не увидел "done/failed". '
              'Отменяю (PREFLIGHT_CALIBRATION 0).')
        n.send_cmd(VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION)  # all zero = cancel
    ok[f'cal_{sensor}'] = done


def test_motors(n, ok):
    print('\n== 6. MOTOR TEST (на кабеле не крутятся — проверяю отправку) ==')
    for motor in range(1, 5):
        n.send_cmd(VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                   param1=float(motor),  # motor number (1-based)
                   param2=0.0,           # throttle type: percent
                   param3=10.0,          # 10%
                   param4=1.0,           # timeout 1s
                   param5=0.0)           # single motor
        print(f'    отправлен DO_MOTOR_TEST motor={motor} throttle=10% timeout=1s')
        time.sleep(0.3)
    print('    (если бы был АКБ — моторы дали бы короткий импульс)')
    ok['motors'] = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='+',
                    choices=['telemetry', 'param', 'backup', 'rc', 'cal', 'motors'])
    ap.add_argument('--cal', nargs='+',
                    choices=['gyro', 'baro', 'level', 'accel', 'mag'],
                    default=['gyro', 'level'])
    ap.add_argument('--backup-limit', type=int, default=30,
                    help='сколько индексов читать (0 = все ~900)')
    ap.add_argument('--rc-secs', type=int, default=15)
    args = ap.parse_args()

    steps = args.only or ['telemetry', 'param', 'backup', 'rc', 'cal', 'motors']

    rclpy.init()
    node = E2ENode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    print('discovery: жду смэтчивания топиков (2.5 c)...')
    time.sleep(2.5)

    ok = {}
    try:
        if 'telemetry' in steps:
            test_telemetry(node, ok)
        if 'param' in steps:
            test_param(node, ok)
        if 'backup' in steps:
            test_backup(node, ok, args.backup_limit)
        if 'rc' in steps:
            test_rc(node, ok, args.rc_secs)
        if 'cal' in steps:
            for s in args.cal:
                test_calibration(node, ok, s)
        if 'motors' in steps:
            test_motors(node, ok)
    except KeyboardInterrupt:
        print('\nпрервано пользователем')

    print('\n===== ИТОГ =====')
    for k, v in ok.items():
        mark = 'OK' if v is True else ('SKIP' if v is None else 'FAIL')
        print(f'  {mark}  {k}')

    # Чистое завершение: сначала гасим executor и джойним spin-поток,
    # иначе он крутится на разрушаемом контексте -> terminate/core dump.
    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
