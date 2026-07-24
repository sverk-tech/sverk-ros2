"""Модель результата и сборка report_json / манифеста (00_contract.md §5-6)."""
from self_check.checks.base import BaseCheck
from self_check.report import (
    CheckResult, STATE_FAIL, STATE_PASS, STATE_SKIP, STATE_WARN,
    build_report, manifest_entry,
)


class DummyCheck(BaseCheck):
    id = 'dummy_check'
    category = 'sbc'
    title = 'Тестовая проверка'
    description = 'Только для теста'
    interactive = False
    dangerous = False


def test_check_result_report_and_metric():
    r = CheckResult(id='x', category='sbc')
    r.report('info', 'hello')
    r.report('warn', 'careful')
    r.report('not_a_level', 'falls back to info')
    r.metric('voltage_v', 15.9)

    d = r.to_dict()
    assert d['messages'] == [
        {'level': 'info', 'text': 'hello'},
        {'level': 'warn', 'text': 'careful'},
        {'level': 'info', 'text': 'falls back to info'},
    ]
    assert d['metrics'] == {'voltage_v': 15.9}
    assert d['id'] == 'x'
    assert d['category'] == 'sbc'


def test_manifest_entry_available():
    check = DummyCheck()
    m = manifest_entry(check, True, '')
    assert m == {
        'id': 'dummy_check',
        'category': 'sbc',
        'title': 'Тестовая проверка',
        'description': 'Только для теста',
        'interactive': False,
        'dangerous': False,
        'available': True,
        'unavailable_reason': '',
    }


def test_manifest_entry_unavailable_has_reason():
    check = DummyCheck()
    m = manifest_entry(check, False, 'LED отключён в профиле')
    assert m['available'] is False
    assert m['unavailable_reason'] == 'LED отключён в профиле'


def test_build_report_summary_counts():
    results = [
        CheckResult(id='a', category='sbc', state=STATE_PASS),
        CheckResult(id='b', category='sbc', state=STATE_WARN),
        CheckResult(id='c', category='motors', state=STATE_FAIL),
        CheckResult(id='d', category='vision', state=STATE_SKIP),
    ]
    report = build_report('2026-07-14T12:00:00+03:00', '2026-07-14T12:00:05+03:00',
                           'RPI', True, False, results)

    assert report['version'] == 1
    assert report['platform'] == 'RPI'
    assert report['interactive'] is True
    assert report['aborted'] is False
    assert report['summary'] == {'pass': 1, 'warn': 1, 'fail': 1, 'skip': 1}
    assert [c['id'] for c in report['checks']] == ['a', 'b', 'c', 'd']


def test_build_report_non_terminal_state_counts_as_fail():
    stuck = CheckResult(id='e', category='sbc', state='RUNNING')
    report = build_report('t0', 't1', 'RPI', False, True, [stuck])
    assert report['summary'] == {'pass': 0, 'warn': 0, 'fail': 1, 'skip': 0}
    assert report['aborted'] is True
