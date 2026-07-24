"""Проверки категории link: FMU-связь, offboard-нода, стики RC."""
from __future__ import annotations

from px4_msgs.msg import VehicleStatus, TimesyncStatus, InputRc
from offboard_interfaces.msg import OffboardStatus

from ..report import STATE_PASS, STATE_WARN, STATE_FAIL
from ..util import rc_channel_span_ok
from .base import BaseCheck

RC_MIN, RC_MAX = 1000, 2000
RC_MIN_SPAN_PCT = 60.0


class LinkFmuCheck(BaseCheck):
    id = 'link_fmu'
    category = 'link'
    title = 'FMU link'
    title_ru = 'Связь с FMU'
    description = 'vehicle_status updates; rate; time desync magnitude'
    description_ru = 'vehicle_status обновляется; частота; величина рассинхрона времени'

    def run(self, ctx):
        topic = ctx.profile['topics']['vehicle_status']
        timeout = 3.0
        ok = ctx.wait_for_updates(VehicleStatus, topic, timeout, min_msgs=2)
        if not ok:
            ctx.report('error', ctx.t(
                f'{topic}: no updates within {timeout:.0f} s',
                f'{topic}: нет обновлений за {timeout:.0f} с'))
            ctx.result.state = STATE_FAIL
            return
        rate = ctx.measure_rate(topic, VehicleStatus, 1.0)
        ctx.metric('rate_hz', round(rate, 1))
        ctx.report('info', ctx.t(
            f'vehicle_status: {rate:.1f} Hz',
            f'vehicle_status: {rate:.1f} Гц'))

        ts_topic = ctx.profile['topics']['timesync_status']
        ts = ctx.wait_for_message(TimesyncStatus, ts_topic, 2.0)
        state = STATE_PASS
        if ts is None:
            ctx.report('warn', ctx.t(
                f'{ts_topic}: no time-sync data',
                f'{ts_topic}: нет данных о синхронизации времени'))
            state = STATE_WARN
        else:
            offset_ms = abs(ts.estimated_offset) / 1000.0
            ctx.metric('timesync_offset_ms', round(offset_ms, 2))
            max_offset = ctx.profile['thresholds']['timesync_offset_ms']
            ctx.report('info', ctx.t(
                f'time desync: {offset_ms:.1f} ms (threshold {max_offset:.0f} ms)',
                f'рассинхрон времени: {offset_ms:.1f} мс (порог {max_offset:.0f} мс)'))
            if offset_ms > max_offset:
                ctx.report('warn', ctx.t(
                    f'time desync exceeds threshold: {offset_ms:.1f} ms',
                    f'рассинхрон времени превышает порог: {offset_ms:.1f} мс'))
                state = STATE_WARN
        ctx.result.state = state


class OffboardStatusCheck(BaseCheck):
    id = 'offboard_status'
    category = 'link'
    title = 'Offboard node'
    title_ru = 'Offboard-нода'
    description = '/offboard_control/status alive; telemetry_age_s < 1s; last_abort_reason empty'
    description_ru = '/offboard_control/status жив; telemetry_age_s < 1с; last_abort_reason пуст'

    def run(self, ctx):
        topic = ctx.profile['topics']['offboard_status']
        msg = ctx.wait_for_message(OffboardStatus, topic, 3.0)
        if msg is None:
            ctx.report('error', ctx.t(
                f'{topic}: no messages (offboard_control not running?)',
                f'{topic}: нет сообщений (offboard_control не запущена?)'))
            ctx.result.state = STATE_FAIL
            return
        ctx.metric('armed', bool(msg.armed))
        ctx.metric('nav_mode', msg.nav_mode)
        ctx.metric('telemetry_age_s', round(float(msg.telemetry_age_s), 2))
        ctx.metric('battery_pct', round(float(msg.battery_pct), 1))
        ctx.report('info', ctx.t(
            f'nav_mode={msg.nav_mode}, armed={msg.armed}, telemetry_age={msg.telemetry_age_s:.2f}s',
            f'nav_mode={msg.nav_mode}, armed={msg.armed}, telemetry_age={msg.telemetry_age_s:.2f}с'))

        state = STATE_PASS
        if msg.telemetry_age_s >= 1.0:
            ctx.report('warn', ctx.t(
                f'telemetry_age_s={msg.telemetry_age_s:.2f} >= 1s',
                f'telemetry_age_s={msg.telemetry_age_s:.2f} >= 1с'))
            state = STATE_WARN
        if msg.last_abort_reason:
            ctx.report('warn', ctx.t(
                f'last_abort_reason: {msg.last_abort_reason}',
                f'last_abort_reason: {msg.last_abort_reason}'))
            state = STATE_WARN
        ctx.result.state = state


class RcSticksCheck(BaseCheck):
    id = 'rc_sticks'
    category = 'link'
    title = 'RC sticks'
    title_ru = 'Стики RC'
    description = 'Move both sticks to their extents — span of channels 1-4'
    description_ru = 'Подвигать обоими стиками до упоров — размах каналов 1-4'
    interactive = True

    def available(self, profile, lang='en'):
        if not profile.get('features', {}).get('rc', True):
            return False, (
                'RC отключён в профиле платформы' if lang == 'ru'
                else 'RC disabled in the platform profile')
        return True, ''

    def run(self, ctx):
        topic = ctx.profile['topics']['input_rc']
        duration = 8.0
        ctx.instruct(ctx.t(
            'Move both RC sticks to their extents on all axes',
            'Подвигайте обоими стиками пульта до упоров по всем осям'))
        samples = ctx.collect(topic, InputRc, duration)
        if not samples:
            ctx.report('error', ctx.t(
                f'{topic}: no InputRc messages within {duration:.0f}s',
                f'{topic}: нет сообщений InputRc за {duration:.0f}с'))
            ctx.result.state = STATE_FAIL
            return

        channels = {ch: [] for ch in range(4)}
        for _, msg in samples:
            for ch in range(min(4, len(msg.values))):
                channels[ch].append(int(msg.values[ch]))

        bad = []
        for ch in range(4):
            vals = channels[ch]
            ok = rc_channel_span_ok(vals, RC_MIN, RC_MAX, RC_MIN_SPAN_PCT) if vals else False
            ctx.metric(f'ch{ch + 1}_span', (max(vals) - min(vals)) if vals else 0)
            if not ok:
                bad.append(ch + 1)

        if bad:
            ctx.report('warn', ctx.t(
                f'insufficient channel span: {bad}',
                f'недостаточный размах каналов: {bad}'))
            ctx.result.state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'all 4 channels span >= 60% of range',
                'все 4 канала имеют размах >= 60% диапазона'))
            ctx.result.state = STATE_PASS


CHECKS = [LinkFmuCheck(), OffboardStatusCheck(), RcSticksCheck()]
