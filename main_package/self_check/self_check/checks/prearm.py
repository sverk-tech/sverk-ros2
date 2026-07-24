"""Проверки категории prearm: failsafe-флаги, состояние EKF-эстиматора."""
from __future__ import annotations

from px4_msgs.msg import VehicleStatus, FailsafeFlags, EstimatorStatusFlags

from ..report import STATE_PASS, STATE_WARN, STATE_FAIL
from .base import BaseCheck


def _active_bool_flags(msg, exclude=('timestamp',)) -> list:
    active = []
    for attr in dir(msg):
        if attr.startswith('_') or attr in exclude:
            continue
        try:
            value = getattr(msg, attr)
        except Exception:
            continue
        if isinstance(value, bool) and value:
            active.append(attr)
    return sorted(active)


class PrearmFailsafeCheck(BaseCheck):
    id = 'prearm_failsafe'
    category = 'prearm'
    title = 'Failsafe / pre-arm'
    title_ru = 'Failsafe / pre-arm'
    description = 'failsafe_flags: active flags; vehicle_status.pre_flight_checks_pass'
    description_ru = 'failsafe_flags: активные флаги; vehicle_status.pre_flight_checks_pass'

    def run(self, ctx):
        topics = ctx.profile['topics']
        fs = ctx.wait_for_message(FailsafeFlags, topics['failsafe_flags'], 3.0)
        vs = ctx.wait_for_message(VehicleStatus, topics['vehicle_status'], 3.0)

        if fs is None and vs is None:
            ctx.report('error', ctx.t(
                'failsafe_flags and vehicle_status unavailable',
                'failsafe_flags и vehicle_status недоступны'))
            ctx.result.state = STATE_FAIL
            return

        state = STATE_PASS

        if fs is None:
            ctx.report('warn', ctx.t(
                f"{topics['failsafe_flags']}: no messages",
                f"{topics['failsafe_flags']}: нет сообщений"))
            state = STATE_WARN
        else:
            active = _active_bool_flags(fs)
            ctx.metric('active_flags', active)
            if active:
                ctx.report('warn', ctx.t(
                    f'active failsafe flags: {", ".join(active)}',
                    f'активные failsafe-флаги: {", ".join(active)}'))
                state = STATE_WARN
            else:
                ctx.report('info', ctx.t(
                    'failsafe flags: none active',
                    'failsafe-флаги: активных нет'))

        if vs is None:
            ctx.report('warn', ctx.t(
                f"{topics['vehicle_status']}: no messages",
                f"{topics['vehicle_status']}: нет сообщений"))
            state = STATE_WARN
        else:
            passed = bool(vs.pre_flight_checks_pass)
            ctx.metric('pre_flight_checks_pass', passed)
            if passed:
                ctx.report('info', 'pre_flight_checks_pass: true')
            else:
                ctx.report('warn', 'pre_flight_checks_pass: false')
                state = STATE_WARN

        ctx.result.state = state


class EstimatorFlagsCheck(BaseCheck):
    id = 'estimator_flags'
    category = 'prearm'
    title = 'Estimator status'
    title_ru = 'Состояние эстиматора'
    description = 'estimator_status_flags: fail/fault/invalid + cs_ev_pos/yaw/hgt (does EKF accept vision)'
    description_ru = 'estimator_status_flags: fail/fault/invalid + cs_ev_pos/yaw/hgt (принимает ли EKF vision)'

    def run(self, ctx):
        topic = ctx.profile['topics']['estimator_status_flags']
        msg = ctx.wait_for_message(EstimatorStatusFlags, topic, 3.0)
        if msg is None:
            ctx.report('error', ctx.t(
                f'{topic}: no messages',
                f'{topic}: нет сообщений'))
            ctx.result.state = STATE_FAIL
            return

        # cs_ev_pos/cs_ev_yaw/cs_ev_hgt — это флаги "намерения" фьюжна (см.
        # ниже), не признаки неисправности, хоть и содержат "ev_"; исключаем
        # их поимённо, а не по подстроке — иначе под фильтр случайно попадали
        # бы и настоящие фолты вроде cs_ev_yaw_fault/cs_gps_yaw_fault.
        intent_flags = {'cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt'}
        bad = [a for a in _active_bool_flags(msg)
               if ('fail' in a or 'fault' in a or 'invalid' in a) and a not in intent_flags]
        ctx.metric('bad_flags', bad)
        ctx.metric('cs_ev_pos', bool(msg.cs_ev_pos))
        ctx.metric('cs_ev_yaw', bool(msg.cs_ev_yaw))
        ctx.metric('cs_ev_hgt', bool(msg.cs_ev_hgt))

        state = STATE_PASS
        if bad:
            ctx.report('warn', ctx.t(
                f'fail/fault/invalid flags active: {", ".join(bad)}',
                f'fail/fault/invalid флаги активны: {", ".join(bad)}'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'fail/fault/invalid flags: none active',
                'fail/fault/invalid флаги: активных нет'))

        ctx.report('info', ctx.t(
            f'EV fusion: pos={msg.cs_ev_pos}, yaw={msg.cs_ev_yaw}, hgt={msg.cs_ev_hgt}',
            f'EV-фьюжн: pos={msg.cs_ev_pos}, yaw={msg.cs_ev_yaw}, hgt={msg.cs_ev_hgt}'))

        ctx.result.state = state


CHECKS = [PrearmFailsafeCheck(), EstimatorFlagsCheck()]
