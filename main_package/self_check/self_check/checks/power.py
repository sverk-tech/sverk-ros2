"""Проверки категории power: батарея."""
from __future__ import annotations

from px4_msgs.msg import BatteryStatus

from ..report import STATE_PASS, STATE_WARN, STATE_FAIL
from ..util import battery_cell_voltage
from .base import BaseCheck


class PowerBatteryCheck(BaseCheck):
    id = 'power_battery'
    category = 'power'
    title = 'Battery'
    title_ru = 'Батарея'
    description = 'battery_status: V/cell, remaining, warning, current, temperature'
    description_ru = 'battery_status: В/банка, remaining, warning, ток, температура'

    def run(self, ctx):
        topic = ctx.profile['topics']['battery_status']
        msg = ctx.wait_for_message(BatteryStatus, topic, 3.0)
        if msg is None:
            ctx.report('error', ctx.t(
                f'{topic}: no BatteryStatus messages',
                f'{topic}: нет сообщений BatteryStatus'))
            ctx.result.state = STATE_FAIL
            return

        cells = int(ctx.profile.get('battery_cells', 0))
        voltage = float(msg.voltage_v)
        cell_v = battery_cell_voltage(voltage, cells)
        ctx.metric('voltage_v', round(voltage, 2))
        if cell_v is not None:
            ctx.metric('cell_v', round(cell_v, 2))
        ctx.metric('cells', cells)
        ctx.metric('remaining', round(float(msg.remaining), 2))
        ctx.metric('current_a', round(float(msg.current_a), 2))
        ctx.metric('temperature_c', round(float(msg.temperature), 1))
        ctx.metric('warning', int(msg.warning))

        state = STATE_PASS
        if cell_v is not None:
            ctx.report('info', ctx.t(
                f'{voltage:.2f} V ({cell_v:.2f} V/cell, {cells} cells)',
                f'{voltage:.2f} В ({cell_v:.2f} В/банка, {cells} банки)'))
            min_cell_v = ctx.profile['thresholds']['min_cell_v']
            if cell_v < min_cell_v:
                ctx.report('warn', ctx.t(
                    f'low per-cell voltage: {cell_v:.2f} V (min {min_cell_v:.2f} V)',
                    f'низкое напряжение на банку: {cell_v:.2f} В (мин {min_cell_v:.2f} В)'))
                state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'{voltage:.2f} V (battery_cells not set in profile)',
                f'{voltage:.2f} В (battery_cells не задан в профиле)'))

        if msg.remaining >= 0.0 and msg.remaining < 0.20:
            ctx.report('warn', ctx.t(
                f'low charge remaining: {msg.remaining * 100:.0f}%',
                f'малый остаток заряда: {msg.remaining * 100:.0f}%'))
            state = STATE_WARN

        if msg.warning >= BatteryStatus.BATTERY_WARNING_LOW:
            ctx.report('warn', ctx.t(
                f'battery warning={msg.warning}',
                f'battery warning={msg.warning}'))
            state = STATE_WARN

        ctx.result.state = state


CHECKS = [PowerBatteryCheck()]
