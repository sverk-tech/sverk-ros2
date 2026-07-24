"""self_check ROS-нода: оркестратор диагностики дрона (00_contract.md §3, tz1 §2.2).

Имена в ROS-графе: нода `self_check`, action `/self_check/run`,
сервисы `/self_check/list`, `/self_check/report`, `/self_check/user_input`.
"""
from __future__ import annotations

import json
import os
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import VehicleStatus
from self_check_interfaces.srv import ListChecks, UserInput
from self_check_interfaces.action import RunChecks

from .checks import REGISTRY, ALL_IDS
from .context import CheckContext
from .profile import build_profile
from .report import (
    CheckResult, STATE_RUNNING, STATE_WAITING_USER, STATE_SKIP, STATE_FAIL, STATE_WARN,
    TERMINAL_STATES, build_report, json_safe, manifest_entry, now_iso)

PASSIVE_POOL_SIZE = 4


class SelfCheckNode(Node):
    def __init__(self):
        super().__init__('self_check')

        self.declare_parameter('platform', 'RPI')
        self.declare_parameter('profile_file', '')
        # Язык диагностики (инструкции/сообщения/заголовки каталога).
        # По умолчанию English; 'ru' переключает на русские тексты.
        self.declare_parameter('language', 'en')
        platform = self.get_parameter('platform').get_parameter_value().string_value
        profile_file = self.get_parameter('profile_file').get_parameter_value().string_value
        lang = self.get_parameter('language').get_parameter_value().string_value
        self.language = lang if lang in ('en', 'ru') else 'en'

        config_dir = os.path.join(get_package_share_directory('self_check'), 'config')
        self.profile = build_profile(config_dir, platform, profile_file)
        self.get_logger().info(
            f"self_check: platform={self.profile['platform']}, "
            f"language={self.language}, config_dir={config_dir}")

        status_cbg = MutuallyExclusiveCallbackGroup()
        self._services_cbg = ReentrantCallbackGroup()
        self._action_cbg = ReentrantCallbackGroup()

        self._vehicle_status_lock = threading.Lock()
        self._vehicle_status = None
        # PX4 /fmu/out/* публикуются BestEffort+Volatile — reliable-подписка
        # (depth=10 по умолчанию) несовместима, кэш armed для motor_spin
        # оставался бы пустым.
        self.create_subscription(
            VehicleStatus, self.profile['topics']['vehicle_status'],
            self._on_vehicle_status, qos_profile_sensor_data,
            callback_group=status_cbg)

        self._run_lock = threading.Lock()
        self._running = False
        self._aborted = threading.Event()
        self._last_report = None

        # Корреляция WAITING_USER <-> /self_check/user_input (один активный шаг разом).
        self._wait_lock = threading.Lock()
        self._wait_check_id = None
        self._wait_options = ()
        self._wait_event = threading.Event()
        self._wait_response = None

        self.create_service(
            ListChecks, '/self_check/list', self._handle_list,
            callback_group=self._services_cbg)
        self.create_service(
            ListChecks, '/self_check/report', self._handle_report,
            callback_group=self._services_cbg)
        self.create_service(
            UserInput, '/self_check/user_input', self._handle_user_input,
            callback_group=self._services_cbg)

        self._action_server = ActionServer(
            self, RunChecks, '/self_check/run',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._action_cbg)

        self.get_logger().info(
            'self_check: action /self_check/run, services /self_check/list, '
            '/self_check/report, /self_check/user_input '
            f'({len(ALL_IDS)} проверок в каталоге)')

    # ------------------------------------------------------------------
    # Кэш последнего vehicle_status (для проверки armed в motor_spin)
    # ------------------------------------------------------------------

    def _t(self, en: str, ru: str) -> str:
        """i18n для собственных строк ноды (фолбэки/сервисные ответы)."""
        return ru if self.language == 'ru' else en

    def _on_vehicle_status(self, msg):
        with self._vehicle_status_lock:
            self._vehicle_status = msg

    def last_vehicle_status(self):
        with self._vehicle_status_lock:
            return self._vehicle_status

    # ------------------------------------------------------------------
    # Сервисы
    # ------------------------------------------------------------------

    def _manifest_list(self):
        manifests = []
        for cid in ALL_IDS:
            check = REGISTRY[cid]
            available, reason = check.available(self.profile, self.language)
            manifests.append(manifest_entry(check, available, reason, self.language))
        return manifests

    def _handle_list(self, _request, response):
        response.checks_json = json.dumps(self._manifest_list(), ensure_ascii=False)
        return response

    def _handle_report(self, _request, response):
        response.checks_json = (
            json.dumps(json_safe(self._last_report), ensure_ascii=False, allow_nan=False)
            if self._last_report else 'null')
        return response

    def _handle_user_input(self, request, response):
        with self._wait_lock:
            if self._wait_check_id is None or self._wait_check_id != request.check_id:
                response.accepted = False
                response.message = self._t(
                    f'no active WAITING_USER step with id={request.check_id}',
                    f'нет активного шага WAITING_USER с id={request.check_id}')
                return response
            # Ответ вне options шага не принимаем: иначе проверка молча
            # трактует его как skip (например, "yes" на подтверждении
            # "пропеллеры сняты" тихо пропускала бы тест моторов).
            if self._wait_options and request.response not in self._wait_options:
                response.accepted = False
                response.message = self._t(
                    f'response "{request.response}" is not in step options: '
                    f'{", ".join(self._wait_options)}',
                    f'ответ "{request.response}" не входит в options шага: '
                    f'{", ".join(self._wait_options)}')
                return response
            self._wait_response = request.response
            self._wait_event.set()
        response.accepted = True
        response.message = 'ok'
        return response

    # ------------------------------------------------------------------
    # Action /self_check/run
    # ------------------------------------------------------------------

    def _goal_cb(self, goal_request):
        requested = list(goal_request.check_ids) or ALL_IDS
        unknown = [cid for cid in requested if cid not in REGISTRY]
        if unknown:
            self.get_logger().warn(f'RunChecks: unknown check ids rejected: {unknown}')
            return GoalResponse.REJECT
        with self._run_lock:
            if self._running:
                self.get_logger().warn('RunChecks: goal rejected, a run is already in progress')
                return GoalResponse.REJECT
            self._running = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self._aborted.set()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._aborted.clear()
        with self._wait_lock:
            self._wait_check_id = None
            self._wait_options = ()
            self._wait_event.clear()
            self._wait_response = None

        goal = goal_handle.request
        requested_ids = list(goal.check_ids) or list(ALL_IDS)
        interactive = bool(goal.interactive)
        confirm = goal.confirm or ''

        # Порядок: пассивные (по каталогу §4), затем интерактивные;
        # interactive=false в goal — интерактивные не включаются вовсе.
        run_ids = [cid for cid in ALL_IDS if cid in requested_ids and not REGISTRY[cid].interactive]
        if interactive:
            run_ids += [cid for cid in ALL_IDS if cid in requested_ids and REGISTRY[cid].interactive]

        started_at = now_iso()
        results = {}

        try:
            passive_ids = [cid for cid in run_ids if not REGISTRY[cid].interactive]
            interactive_ids = [cid for cid in run_ids if REGISTRY[cid].interactive]

            self._run_batch(passive_ids, results, goal_handle, confirm, run_ids, parallel=True)
            if not self._aborted.is_set():
                self._run_batch(interactive_ids, results, goal_handle, confirm, run_ids, parallel=False)
        finally:
            aborted_flag = self._aborted.is_set()
            with self._run_lock:
                self._running = False

        finished_at = now_iso()
        ordered_results = [results[cid] for cid in run_ids if cid in results]
        report = build_report(
            started_at, finished_at, self.profile['platform'], interactive,
            aborted_flag, ordered_results)
        self._last_report = report

        result = RunChecks.Result()
        result.report_json = json.dumps(
            json_safe(report), ensure_ascii=False, allow_nan=False)
        if aborted_flag:
            goal_handle.canceled()
            result.success = False
        else:
            goal_handle.succeed()
            result.success = True
        return result

    def _run_batch(self, ids, results, goal_handle, confirm, run_ids, parallel):
        if not ids:
            return
        total = len(run_ids)

        def run_one(cid):
            self._run_single_check(cid, results, goal_handle, confirm, run_ids, total)

        if parallel and len(ids) > 1:
            with ThreadPoolExecutor(max_workers=min(PASSIVE_POOL_SIZE, len(ids))) as pool:
                list(pool.map(run_one, ids))
        else:
            for cid in ids:
                if self._aborted.is_set():
                    self._mark_skip(cid, results, goal_handle, run_ids, total,
                                    self._t('run aborted', 'прогон прерван'))
                    continue
                run_one(cid)

    def _mark_skip(self, cid, results, goal_handle, run_ids, total, reason):
        check = REGISTRY[cid]
        result = CheckResult(id=cid, category=check.category, state=STATE_SKIP)
        result.report('warn', reason)
        results[cid] = result
        self._publish_feedback(goal_handle, cid, STATE_SKIP, run_ids, total, '', {})

    def _run_single_check(self, cid, results, goal_handle, confirm, run_ids, total):
        check = REGISTRY[cid]
        available, reason = check.available(self.profile, self.language)
        result = CheckResult(id=cid, category=check.category)

        if not available:
            result.state = STATE_SKIP
            result.report('info', reason or self._t(
                'check not available for this platform',
                'проверка недоступна для этой платформы'))
            results[cid] = result
            self._publish_feedback(goal_handle, cid, STATE_SKIP, run_ids, total, '', {})
            return

        self._publish_feedback(goal_handle, cid, STATE_RUNNING, run_ids, total, '', {})

        def feedback_cb(instruction, metrics):
            # instruct() — живой прогресс интерактивной проверки, не блокирующий
            # ответ пользователя (WAITING_USER — только для wait_user() ниже).
            self._publish_feedback(
                goal_handle, cid, STATE_RUNNING, run_ids, total, instruction, metrics)

        def user_input_wait(text, options, timeout_s):
            with self._wait_lock:
                self._wait_check_id = cid
                self._wait_options = tuple(options)
                self._wait_event.clear()
                self._wait_response = None
            # options уходят в metrics_json — UI рисует по ним кнопки шага.
            self._publish_feedback(
                goal_handle, cid, STATE_WAITING_USER, run_ids, total, text,
                {'options': list(options)})
            got = self._wait_event.wait(timeout_s)
            with self._wait_lock:
                response = self._wait_response if got else 'skip'
                self._wait_check_id = None
                self._wait_options = ()
            return response

        ctx = CheckContext(
            self, self.profile, result, self._aborted,
            feedback_cb=feedback_cb, user_input_wait=user_input_wait, confirm=confirm,
            language=self.language)

        start = time.monotonic()
        try:
            check.run(ctx)
        except Exception:
            tb = traceback.format_exc()
            self.get_logger().error(f'[{cid}] exception:\n{tb}')
            result.report('error', self._t(
                'internal check error (see node log)',
                'внутренняя ошибка проверки (см. лог ноды)'))
        finally:
            result.duration_s = time.monotonic() - start
            if result.state not in TERMINAL_STATES:
                has_error = any(m['level'] == 'error' for m in result.messages)
                result.state = STATE_FAIL if has_error else STATE_WARN

        results[cid] = result
        self._publish_feedback(goal_handle, cid, result.state, run_ids, total, '', dict(result.metrics))

    def _publish_feedback(self, goal_handle, cid, state, run_ids, total, instruction, metrics):
        try:
            idx = run_ids.index(cid)
        except ValueError:
            idx = 0
        progress = (idx + (1.0 if state in TERMINAL_STATES else 0.0)) / max(1, total)
        fb = RunChecks.Feedback()
        fb.check_id = cid
        fb.state = state
        fb.progress = float(min(1.0, max(0.0, progress)))
        fb.instruction = instruction
        # json_safe: NaN/Inf из телеметрии дают невалидный JSON (браузерный
        # JSON.parse падает), allow_nan=False — страховка от новых источников.
        fb.metrics_json = (
            json.dumps(json_safe(metrics), ensure_ascii=False, allow_nan=False)
            if metrics else '{}')
        try:
            goal_handle.publish_feedback(fb)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SelfCheckNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
