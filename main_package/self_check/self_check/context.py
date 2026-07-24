"""CheckContext — интерфейс, который нода даёт каждой проверке в BaseCheck.run(ctx).

Живёт ровно на время одного шага прогона: подписки создаются на время
вызова и уничтожаются (см. 00_contract.md / tz1 §2.1).
"""
from __future__ import annotations

import threading
import time

from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)


def qos_sensor_data(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_transient_local(depth: int = 1) -> QoSProfile:
    """Для латчнутых топиков вроде /aruco/map/markers — подписка ОБЯЗАНА быть такой."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class LatestSub:
    """Подписка, отдающая последнее полученное сообщение — для интерактивных
    проверок, которые в цикле опроса шлют живые метрики через ctx.instruct()."""

    def __init__(self, node, msg_type, topic: str, qos):
        self._node = node
        self._lock = threading.Lock()
        self._msg = None
        self._sub = node.create_subscription(msg_type, topic, self._cb, qos)

    def _cb(self, msg):
        with self._lock:
            self._msg = msg

    def get(self):
        with self._lock:
            return self._msg

    def close(self) -> None:
        try:
            self._node.destroy_subscription(self._sub)
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


def wait_latest(sub: LatestSub, timeout: float, aborted: threading.Event = None,
                 poll_s: float = 0.05):
    """Опрашивать LatestSub.get() пока не вернёт не-None или не истечёт timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if aborted is not None and aborted.is_set():
            return None
        val = sub.get()
        if val is not None:
            return val
        time.sleep(poll_s)
    return sub.get()


class CheckContext:
    def __init__(self, node, profile: dict, result, aborted: threading.Event,
                 feedback_cb=None, user_input_wait=None, confirm: str = '',
                 language: str = 'en'):
        self.node = node
        self.profile = profile
        self.result = result
        self.aborted = aborted
        self.confirm = confirm                   # RunChecks.Goal.confirm (для motor_spin)
        self.lang = language if language in ('en', 'ru') else 'en'
        self._feedback_cb = feedback_cb          # (instruction, metrics) -> None
        self._user_input_wait = user_input_wait  # (check_id, options, timeout) -> str

    # ------------------------------------------------------------------
    # i18n диагностики
    # ------------------------------------------------------------------

    def t(self, en: str, ru: str) -> str:
        """Выбрать язык диагностической строки. Каталог по умолчанию —
        английский; русский отдаётся только когда нода запущена с language='ru'.
        Подстановку живых чисел оставляем на месте вызова (обе ветки — f-строки).
        """
        return ru if self.lang == 'ru' else en

    # ------------------------------------------------------------------
    # Отчёт (см. report.CheckResult)
    # ------------------------------------------------------------------

    def report(self, level: str, text: str) -> None:
        self.result.report(level, text)

    def metric(self, key: str, value) -> None:
        self.result.metric(key, value)

    # ------------------------------------------------------------------
    # Топики
    # ------------------------------------------------------------------

    def wait_for_message(self, msg_type, topic: str, timeout: float, qos=None):
        """Дождаться одного сообщения в топике. None по таймауту."""
        got = threading.Event()
        box = {'msg': None}

        def cb(msg):
            box['msg'] = msg
            got.set()

        sub = self.node.create_subscription(msg_type, topic, cb, qos or qos_sensor_data())
        try:
            got.wait(timeout)
            return box['msg']
        finally:
            self.node.destroy_subscription(sub)

    def wait_for_updates(self, msg_type, topic: str, timeout: float,
                          min_msgs: int = 2, qos=None) -> bool:
        """Дождаться min_msgs сообщений (подтверждает, что топик реально обновляется)."""
        got = threading.Event()
        count = {'n': 0}

        def cb(_msg):
            count['n'] += 1
            if count['n'] >= min_msgs:
                got.set()

        sub = self.node.create_subscription(msg_type, topic, cb, qos or qos_sensor_data())
        try:
            return got.wait(timeout)
        finally:
            self.node.destroy_subscription(sub)

    def measure_rate(self, topic: str, msg_type, window_s: float, qos=None) -> float:
        """Частота публикации за окно window_s (Гц), 0.0 — если нет сообщений."""
        lock = threading.Lock()
        count = {'n': 0}

        def cb(_msg):
            with lock:
                count['n'] += 1

        sub = self.node.create_subscription(msg_type, topic, cb, qos or qos_sensor_data())
        stop_at = time.time() + window_s
        try:
            while time.time() < stop_at and not self.aborted.is_set():
                time.sleep(0.02)
        finally:
            self.node.destroy_subscription(sub)
        with lock:
            n = count['n']
        return n / window_s if n > 0 else 0.0

    def collect(self, topic: str, msg_type, duration: float, qos=None, max_samples: int = 2000):
        """Собрать все сообщения топика за duration секунд: [(time.time(), msg), ...]."""
        lock = threading.Lock()
        samples = []

        def cb(msg):
            with lock:
                if len(samples) < max_samples:
                    samples.append((time.time(), msg))

        sub = self.node.create_subscription(msg_type, topic, cb, qos or qos_sensor_data())
        stop_at = time.time() + duration
        try:
            while time.time() < stop_at and not self.aborted.is_set():
                time.sleep(0.01)
        finally:
            self.node.destroy_subscription(sub)
        with lock:
            return list(samples)

    def collect_multi(self, specs, duration: float, qos=None, max_samples: int = 2000):
        """Параллельный сбор нескольких топиков за общее окно duration.

        specs: [(key, topic, msg_type), ...]. Возвращает
        {key: [(time.time(), msg), ...]}.
        """
        lock = threading.Lock()
        buffers = {key: [] for key, _, _ in specs}
        subs = []

        def make_cb(key):
            def cb(msg):
                with lock:
                    if len(buffers[key]) < max_samples:
                        buffers[key].append((time.time(), msg))
            return cb

        for key, topic, msg_type in specs:
            subs.append(self.node.create_subscription(
                msg_type, topic, make_cb(key), qos or qos_sensor_data()))

        stop_at = time.time() + duration
        try:
            while time.time() < stop_at and not self.aborted.is_set():
                time.sleep(0.02)
        finally:
            for sub in subs:
                self.node.destroy_subscription(sub)
        with lock:
            return {k: list(v) for k, v in buffers.items()}

    def subscribe_latest(self, msg_type, topic: str, qos=None) -> LatestSub:
        """Подписка на "последнее сообщение" для циклов опроса (см. LatestSub)."""
        return LatestSub(self.node, msg_type, topic, qos or qos_sensor_data())

    # ------------------------------------------------------------------
    # Сервисы (например /fmu_control/*)
    # ------------------------------------------------------------------

    def call_service(self, srv_type, name: str, request, timeout: float):
        """Синхронный вызов сервиса из воркер-потока. None — недоступен/таймаут.

        Не использует rclpy.spin_until_future_complete: нода уже крутится в
        своём MultiThreadedExecutor, поэтому future просто дожидаемся опросом.
        """
        client = self.node.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(request)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline and not self.aborted.is_set():
                time.sleep(0.01)
            if not future.done():
                return None
            return future.result()
        finally:
            self.node.destroy_client(client)

    # ------------------------------------------------------------------
    # Интерактив
    # ------------------------------------------------------------------

    def instruct(self, text: str, metrics: dict = None) -> None:
        """Обновить instruction/metrics_json в feedback текущего шага."""
        if self._feedback_cb:
            self._feedback_cb(text, metrics or {})

    def wait_user(self, text: str, options=('ok', 'skip'), timeout_s: float = 120.0) -> str:
        """Перевести шаг в WAITING_USER и дождаться /self_check/user_input.

        Таймаут -> "skip".
        """
        if self._user_input_wait is None:
            self.instruct(text)
            return 'skip'
        return self._user_input_wait(text, options, timeout_s)

    # ------------------------------------------------------------------
    # Служебное
    # ------------------------------------------------------------------

    def last_vehicle_status(self):
        return self.node.last_vehicle_status()
