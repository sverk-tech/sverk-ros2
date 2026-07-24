"""Модель результата проверки и сборка report_json (00_contract.md §6).

Модуль не зависит от rclpy — используется и нодой, и pytest напрямую.
"""
from __future__ import annotations

import datetime
import math
from dataclasses import dataclass, field

# Состояния проверки. WAITING_USER и RUNNING — промежуточные,
# в CheckResult.state попадает только один из терминальных.
STATE_RUNNING = 'RUNNING'
STATE_WAITING_USER = 'WAITING_USER'
STATE_PASS = 'PASS'
STATE_WARN = 'WARN'
STATE_FAIL = 'FAIL'
STATE_SKIP = 'SKIP'

TERMINAL_STATES = (STATE_PASS, STATE_WARN, STATE_FAIL, STATE_SKIP)

REPORT_VERSION = 1


def now_iso() -> str:
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


def json_safe(value):
    """Рекурсивно заменить нефинитные float (NaN/Inf) на None.

    python json.dumps пишет голые NaN/Infinity — это невалидный JSON:
    браузерный JSON.parse на нём падает (например, temperature_c=NaN из
    BatteryStatus ронял разбор SSE-событий во вкладке «Проверка системы»).
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


@dataclass
class CheckResult:
    """Накопитель результата одной проверки: сообщения + метрики + итог."""

    id: str
    category: str
    state: str = STATE_RUNNING
    duration_s: float = 0.0
    messages: list = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    def report(self, level: str, text: str) -> None:
        if level not in ('info', 'warn', 'error'):
            level = 'info'
        self.messages.append({'level': level, 'text': text})

    def metric(self, key: str, value) -> None:
        self.metrics[key] = json_safe(value)

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'category': self.category,
            'state': self.state,
            'duration_s': round(self.duration_s, 3),
            'messages': list(self.messages),
            'metrics': dict(self.metrics),
        }


def manifest_entry(check, available: bool, reason: str, lang: str = 'en') -> dict:
    return {
        'id': check.id,
        'category': check.category,
        'title': check.localized_title(lang),
        'description': check.localized_description(lang),
        'interactive': bool(check.interactive),
        'dangerous': bool(check.dangerous),
        'available': bool(available),
        'unavailable_reason': '' if available else str(reason or ''),
    }


def build_report(started_at: str, finished_at: str, platform: str,
                  interactive: bool, aborted: bool,
                  results) -> dict:
    """Собрать report_json (00_contract.md §6) из списка CheckResult."""
    summary = {'pass': 0, 'warn': 0, 'fail': 0, 'skip': 0}
    checks = []
    for r in results:
        state = r.state if r.state in TERMINAL_STATES else STATE_FAIL
        key = state.lower()
        if key in summary:
            summary[key] += 1
        checks.append(r.to_dict())

    return {
        'version': REPORT_VERSION,
        'started_at': started_at,
        'finished_at': finished_at,
        'platform': platform,
        'interactive': bool(interactive),
        'aborted': bool(aborted),
        'summary': summary,
        'checks': checks,
    }
