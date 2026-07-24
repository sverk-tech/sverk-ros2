#!/usr/bin/env python3
"""CLI-клиент action /self_check/run.

ros2 run self_check selfcheck [--only id,id] [--category cat,cat]
    [--interactive] [--confirm-props-off] [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from self_check_interfaces.action import RunChecks
from self_check_interfaces.srv import ListChecks, UserInput

from .report import STATE_FAIL, STATE_WARN, STATE_SKIP, STATE_PASS, STATE_WAITING_USER

CONFIRM_PHRASE = 'I_UNDERSTAND_PROPS_ARE_OFF'

if sys.stderr.isatty():
    RED, YELLOW, GREEN, END = '\033[31m', '\033[33m', '\033[32m', '\033[0m'
else:
    RED = YELLOW = GREEN = END = ''

STATE_COLOR = {
    STATE_FAIL: RED,
    STATE_WARN: YELLOW,
    STATE_SKIP: YELLOW,
    STATE_PASS: GREEN,
}


class SelfCheckCliNode(Node):
    def __init__(self, interactive_stdin: bool):
        super().__init__('selfcheck_cli')
        self._interactive_stdin = interactive_stdin
        self._client = ActionClient(self, RunChecks, '/self_check/run')
        self._list_client = self.create_client(ListChecks, '/self_check/list')
        self._user_input_client = self.create_client(UserInput, '/self_check/user_input')
        self._waiting_ids = set()
        self._last_instruction = {}

    def ids_for_categories(self, categories, interactive: bool):
        """id проверок выбранных категорий (через /self_check/list).

        Берём только available; интерактивные — только при --interactive.
        """
        if not self._list_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/self_check/list недоступен (нода self_check не запущена?)')
            return None
        future = self._list_client.call_async(ListChecks.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('/self_check/list: нет ответа')
            return None
        manifests = json.loads(future.result().checks_json)
        known = sorted({m['category'] for m in manifests})
        unknown = [c for c in categories if c not in known]
        if unknown:
            self.get_logger().error(
                f"неизвестные категории: {', '.join(unknown)} (есть: {', '.join(known)})")
            return None
        return [
            m['id'] for m in manifests
            if m['category'] in categories and m.get('available', True)
            and (interactive or not m.get('interactive', False))
        ]

    def run(self, check_ids, interactive: bool, confirm: str, discover_timeout_s: float = 15.0):
        if not self._client.wait_for_server(timeout_sec=discover_timeout_s):
            self.get_logger().error(
                'action /self_check/run недоступен (нода self_check не запущена?)')
            return None

        goal = RunChecks.Goal()
        goal.check_ids = check_ids
        goal.interactive = interactive
        goal.confirm = confirm

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                'RunChecks: goal отклонён нодой (уже идёт прогон или неизвестные id)')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        color = STATE_COLOR.get(fb.state, '')
        if fb.state == STATE_WAITING_USER:
            print(f'{color}[{fb.check_id}] WAITING_USER: {fb.instruction}{END}')
            if self._interactive_stdin and fb.check_id not in self._waiting_ids:
                self._waiting_ids.add(fb.check_id)
                threading.Thread(
                    target=self._ask_stdin, args=(fb.check_id, fb.instruction), daemon=True
                ).start()
        elif fb.state == 'RUNNING':
            # instruct() шлёт живой прогресс (~5 Гц) — печатать только при
            # смене текста инструкции, чтобы не заспамить лог.
            if fb.instruction and self._last_instruction.get(fb.check_id) != fb.instruction:
                self._last_instruction[fb.check_id] = fb.instruction
                print(f'[{fb.check_id}] {fb.instruction}')
        else:
            print(f'{color}[{fb.check_id}] {fb.state}{END}')

    def _ask_stdin(self, check_id: str, instruction: str) -> None:
        try:
            answer = input(f'{instruction} [ok/yes/no/skip]: ').strip().lower()
        except EOFError:
            answer = 'skip'
        if answer not in ('ok', 'yes', 'no', 'skip'):
            answer = 'skip'
        if self._user_input_client.wait_for_service(timeout_sec=2.0):
            req = UserInput.Request(check_id=check_id, response=answer)
            self._user_input_client.call_async(req)
        else:
            self.get_logger().warn('/self_check/user_input недоступен')
        self._waiting_ids.discard(check_id)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='CLI-клиент self_check (action /self_check/run)')
    p.add_argument('--only', default='', help='список id через запятую (пусто = все применимые)')
    p.add_argument(
        '--category', default='',
        help='категории через запятую (link,power,sensors,vision,motors,sbc,prearm); '
             'объединяется с --only')
    p.add_argument('--interactive', action='store_true', help='включить интерактивные проверки')
    p.add_argument(
        '--confirm-props-off', action='store_true',
        help='подтвердить снятые пропеллеры (обязательно для motor_spin)')
    p.add_argument('--json', default='', help='сохранить report_json в файл')
    return p


def _print_report(report: dict) -> int:
    summary = report.get('summary', {})
    print()
    print(
        f"Итог: pass={summary.get('pass', 0)} warn={summary.get('warn', 0)} "
        f"fail={summary.get('fail', 0)} skip={summary.get('skip', 0)}")
    for check in report.get('checks', []):
        color = STATE_COLOR.get(check['state'], '')
        print(f"{color}[{check['id']}] {check['state']} ({check['duration_s']:.2f}с){END}")
        for m in check.get('messages', []):
            level = m.get('level', 'info')
            mcolor = RED if level == 'error' else (YELLOW if level == 'warn' else '')
            print(f"    {mcolor}{level.upper()}: {m['text']}{END}")
    return 1 if summary.get('fail', 0) > 0 else 0


def main(args=None):
    parser = build_arg_parser()
    ns, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = SelfCheckCliNode(interactive_stdin=ns.interactive)

    check_ids = [s.strip() for s in ns.only.split(',') if s.strip()]
    categories = [s.strip() for s in ns.category.split(',') if s.strip()]
    if categories:
        cat_ids = node.ids_for_categories(categories, ns.interactive)
        if cat_ids is None:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(2)
        check_ids = list(dict.fromkeys(check_ids + cat_ids))
        if not check_ids:
            node.get_logger().error('в выбранных категориях нет доступных проверок')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(2)
    confirm = CONFIRM_PHRASE if ns.confirm_props_off else ''

    result = node.run(check_ids, ns.interactive, confirm)

    exit_code = 2
    if result is not None and result.report_json:
        report = json.loads(result.report_json)
        if ns.json:
            with open(ns.json, 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
        exit_code = _print_report(report)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
