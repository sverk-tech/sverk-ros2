#!/usr/bin/env python3
"""
Скачать последний .ulg с SD полётника по чистому uXRCE-DDS (План B).

Надёжный ОКОННЫЙ pull через модуль log_download_bridge:
  1. OP_INFO  -> имя и размер последнего лога.
  2. OP_READ  -> куски по offset ПОТОКОМ (пайплайн), ответы собираются
     асинхронно, потерянные (best-effort) дозапрашиваются проходами.
Быстро и целиком.

Требует прошивку с модулем log_download_bridge + топиками
/fmu/in/log_download_request, /fmu/out/log_download_response, и px4_msgs с
LogDownloadRequest/Response.

Запуск:
  python3 log_download_receiver.py --out /home/sverk/last.ulg
  python3 log_download_receiver.py --out /home/sverk/last.ulg --window 12 --timeout 0.3
"""
import argparse
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import LogDownloadRequest, LogDownloadResponse

CHUNK = 240  # = uint8[240] data в LogDownloadResponse

QOS_IN = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
QOS_OUT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=50)


def _decode(field) -> str:
    return bytes(int(c) & 0xFF for c in field).split(b'\x00', 1)[0].decode('ascii', 'replace')


class LogDownloader(Node):
    def __init__(self):
        super().__init__('log_download_receiver')
        self._pub = self.create_publisher(
            LogDownloadRequest, '/fmu/in/log_download_request', QOS_IN)
        self.create_subscription(
            LogDownloadResponse, '/fmu/out/log_download_response',
            self._on_resp, QOS_OUT)

        # INFO correlation
        self._info = None
        self._info_ev = threading.Event()

        # bulk download state (заполняется асинхронно из колбэка)
        self._lock = threading.Lock()
        self._buf = None
        self._got = None
        self._size = 0
        self._n_got = 0

    def _on_resp(self, msg: LogDownloadResponse):
        if msg.operation == LogDownloadRequest.OP_INFO:
            self._info = msg
            self._info_ev.set()
            return
        if msg.operation != LogDownloadRequest.OP_READ:
            return
        if self._got is None or msg.result != LogDownloadResponse.RESULT_OK:
            return
        off = int(msg.offset)
        idx = off // CHUNK
        n = int(msg.length)
        with self._lock:
            if 0 <= idx < len(self._got) and not self._got[idx] and n > 0:
                data = bytes(int(b) & 0xFF for b in msg.data[:n])
                self._buf[off:off + len(data)] = data
                self._got[idx] = True
                self._n_got += 1

    def info(self, timeout=5.0, retries=15):
        req = LogDownloadRequest()
        req.operation = LogDownloadRequest.OP_INFO
        self._info = None
        self._info_ev.clear()
        for _ in range(retries):
            req.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self._pub.publish(req)
            if self._info_ev.wait(timeout / retries):
                break
        return self._info

    def prepare(self, size):
        with self._lock:
            self._size = size
            self._buf = bytearray(size)
            self._got = [False] * ((size + CHUNK - 1) // CHUNK)
            self._n_got = 0

    def fire_read(self, offset, length):
        req = LogDownloadRequest()
        req.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        req.operation = LogDownloadRequest.OP_READ
        req.offset = int(offset)
        req.length = int(length)
        self._pub.publish(req)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='last.ulg')
    ap.add_argument('--window', type=int, default=12,
                    help='макс. число запросов "в полёте" одновременно (flow control); '
                    '16 - подтверждённое свипом (16/20/24/32/48) плато throughput, '
                    'больше не ускоряет, только грузит CPU агента')
    ap.add_argument('--timeout', type=float, default=0.3,
                    help='таймаут ожидания ответа на один кусок, с (после него - ретрай)')
    ap.add_argument('--max-time', type=float, default=600.0,
                    help='общий дедлайн скачивания, с (180с оказалось мало для '
                    'логов больше ~4 МБ при ~27 КБ/с)')
    ap.add_argument('--tick', type=float, default=0.005,
                    help='период опроса очереди/таймаутов, с')
    args = ap.parse_args()

    rclpy.init()
    node = LogDownloader()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    print('жду смэтчивания request-канала (PX4 -> /fmu/in/log_download_request)...')
    deadline = time.time() + 8.0
    while time.time() < deadline and node._pub.get_subscription_count() < 1:
        time.sleep(0.1)
    print(f'  request subscription_count = {node._pub.get_subscription_count()}')
    time.sleep(0.5)

    info = node.info()
    if info is None:
        print('нет ответа на INFO. Проверь на FMU: log_download_bridge status.')
        rc = 1
    elif info.result == LogDownloadResponse.RESULT_NO_FILE:
        print('на SD нет .ulg')
        rc = 1
    elif info.result != LogDownloadResponse.RESULT_OK:
        print(f'INFO error: result={info.result}')
        rc = 1
    else:
        name = _decode(info.name)
        size = int(info.file_size)
        nchunks = (size + CHUNK - 1) // CHUNK
        print(f'последний лог: {name}  ({size} байт, {nchunks} кусков)')
        node.prepare(size)
        t0 = time.time()
        deadline = t0 + args.max_time

        # ack-driven windowed pipeline: не более args.window запросов "в
        # полёте" одновременно; следующий уходит, как только освобождается
        # слот (пришёл ответ) или истекает per-chunk timeout (ретрай).
        queue = deque(range(nchunks))
        inflight = {}  # idx -> sent_at
        last_print = 0.0
        while node._n_got < nchunks and time.time() < deadline:
            now = time.time()
            with node._lock:
                got = node._got
                while len(inflight) < args.window and queue:
                    idx = queue.popleft()
                    if got[idx]:
                        continue
                    off = idx * CHUNK
                    node.fire_read(off, min(CHUNK, size - off))
                    inflight[idx] = now
            for idx in list(inflight.keys()):
                with node._lock:
                    done_chunk = node._got[idx]
                if done_chunk:
                    del inflight[idx]
                elif now - inflight[idx] > args.timeout:
                    del inflight[idx]
                    queue.append(idx)
            if now - last_print > 1.0:
                with node._lock:
                    done = node._n_got
                print(f'  получено {done}/{nchunks} ({done*100//max(nchunks,1)}%), '
                      f'в полёте {len(inflight)}, в очереди {len(queue)}')
                last_print = now
            time.sleep(args.tick)

        with node._lock:
            done = node._n_got
            data = bytes(node._buf)
        with open(args.out, 'wb') as f:
            f.write(data)
        dt = time.time() - t0
        print('\n===== ИТОГ =====')
        print(f'  файл  : {args.out}')
        print(f'  куски : {done}/{nchunks}')
        print(f'  байт  : {size}')
        print(f'  время : {dt:.1f}s  (~{size/max(dt,0.1)/1024:.0f} КБ/с)')
        if done == nchunks:
            print('  OK файл собран ПОЛНОСТЬЮ. Проверь: ulog_info', args.out)
            rc = 0
        else:
            print(f'  WARNING не хватает {nchunks-done}. Увеличь --passes или --pace.')
            rc = 1

    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
