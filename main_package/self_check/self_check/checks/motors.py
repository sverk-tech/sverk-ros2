"""Проверка категории motors: тест раскрутки моторов (опасная, 00_contract.md §8)."""
from __future__ import annotations

from px4_msgs.msg import VehicleStatus
from std_srvs.srv import Trigger
from offboard_interfaces.srv import MotorTest

from ..report import STATE_PASS, STATE_WARN, STATE_FAIL, STATE_SKIP
from .base import BaseCheck

CONFIRM_PHRASE = 'I_UNDERSTAND_PROPS_ARE_OFF'
# PX4 quad X: моторы 1,2 CCW / 3,4 CW.
MOTOR_DIRECTION = {1: 'CCW', 2: 'CCW', 3: 'CW', 4: 'CW'}


def _is_armed(vs) -> bool:
    return vs is not None and int(vs.arming_state) == VehicleStatus.ARMING_STATE_ARMED


class MotorSpinCheck(BaseCheck):
    id = 'motor_spin'
    category = 'motors'
    title = 'Motor test'
    title_ru = 'Тест моторов'
    description = 'Spin each motor at minimum throttle, confirm direction'
    description_ru = 'Вращение каждого мотора на минимальном газу, подтверждение направления'
    interactive = True
    dangerous = True

    def run(self, ctx):
        # §8: без confirm в goal — мгновенный FAIL, моторы не трогать.
        if ctx.confirm != CONFIRM_PHRASE:
            ctx.report('error', ctx.t(
                'no confirmation that propellers are removed',
                'нет подтверждения снятых пропеллеров'))
            ctx.result.state = STATE_FAIL
            return

        if _is_armed(ctx.last_vehicle_status()):
            ctx.report('error', ctx.t(
                'drone is armed — motor test cancelled',
                'дрон армирован — тест моторов отменён'))
            ctx.result.state = STATE_FAIL
            return

        confirmed = ctx.wait_user(
            ctx.t('DANGER: motor test. Confirm the propellers are removed',
                  'ОПАСНО: тест моторов. Подтвердите, что пропеллеры сняты'),
            options=('ok', 'skip'), timeout_s=120.0)
        if confirmed != 'ok':
            ctx.report('warn', ctx.t(
                'motor test skipped by user',
                'тест моторов пропущен пользователем'))
            ctx.result.state = STATE_SKIP
            return

        motors_cfg = ctx.profile.get('motors', {})
        count = int(motors_cfg.get('count', 4))
        throttle = float(motors_cfg.get('test_throttle', 0.15))
        timeout_s = float(motors_cfg.get('test_timeout_s', 3.0))
        # Времени на «посмотреть на мотор и оценить направление» нужно заметно
        # больше, чем длится спин: 18с (spin+15) на практике не хватает.
        question_timeout_s = float(motors_cfg.get('question_timeout_s', 45.0))

        state = STATE_PASS
        try:
            for motor_id in range(1, count + 1):
                if ctx.aborted.is_set():
                    break

                if _is_armed(ctx.last_vehicle_status()):
                    ctx.report('error', ctx.t(
                        f'drone armed before motor {motor_id} — test stopped',
                        f'дрон армирован перед мотором {motor_id} — тест остановлен'))
                    state = STATE_FAIL
                    break

                req = MotorTest.Request(motor_id=motor_id, throttle=throttle, timeout_s=timeout_s)
                resp = ctx.call_service(MotorTest, '/fmu_control/motor_test', req, timeout=3.0)
                if resp is None or not resp.success:
                    text = resp.message if resp is not None else ctx.t(
                        'timeout /fmu_control/motor_test', 'таймаут /fmu_control/motor_test')
                    ctx.report('error', ctx.t(
                        f'motor {motor_id}: command failed ({text})',
                        f'мотор {motor_id}: команда не выполнена ({text})'))
                    state = STATE_FAIL
                    continue

                direction = MOTOR_DIRECTION.get(motor_id, '?')
                answer = ctx.wait_user(
                    ctx.t(f'Is motor {motor_id} spinning? Correct direction ({direction} per quad X)?',
                          f'Мотор {motor_id} крутится? Направление верное ({direction} по схеме quad X)?'),
                    options=('yes', 'no', 'skip'), timeout_s=question_timeout_s)
                ctx.metric(f'motor{motor_id}_answer', answer)
                if answer == 'yes':
                    ctx.report('info', ctx.t(
                        f'motor {motor_id}: confirmed ({direction})',
                        f'мотор {motor_id}: подтверждён ({direction})'))
                elif answer == 'no':
                    ctx.report('warn', ctx.t(
                        f'motor {motor_id}: not spinning or wrong direction',
                        f'мотор {motor_id}: не крутится или неверное направление'))
                    state = STATE_WARN
                else:
                    ctx.report('warn', ctx.t(
                        f'motor {motor_id}: skipped by user/timeout',
                        f'мотор {motor_id}: пропущен пользователем/таймаут'))
                    state = STATE_WARN
        finally:
            # §8: остановка моторов при завершении/abort/исключении — best effort.
            stop_resp = ctx.call_service(Trigger, '/fmu_control/stop_all_motors', Trigger.Request(), timeout=2.0)
            if stop_resp is None or not stop_resp.success:
                ctx.report('warn', ctx.t(
                    'stop_all_motors: could not confirm stop (best effort)',
                    'stop_all_motors: не удалось подтвердить остановку (best effort)'))

        if ctx.aborted.is_set():
            ctx.result.state = STATE_SKIP
            return

        ctx.result.state = state


CHECKS = [MotorSpinCheck()]
