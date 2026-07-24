"""Проверки категории sbc: здоровье платы, ROS-граф, веб-сервисы, LED-тест."""
from __future__ import annotations

import glob
import os
import subprocess
import shutil
import time
import urllib.request

# led_interfaces — опциональный пакет: в SITL/слим-образах led_control не
# собирается, нода self_check при этом обязана подниматься (led_test -> SKIP).
try:
    from led_interfaces.srv import SetLEDEffect
except ImportError:
    SetLEDEffect = None

from ..report import STATE_PASS, STATE_WARN, STATE_FAIL, STATE_SKIP
from .base import BaseCheck


def _cpu_usage_pct(sample_dt: float = 0.2) -> float:
    def read():
        with open('/proc/stat', 'r', encoding='utf-8') as f:
            parts = f.readline().split()[1:]
        vals = [int(x) for x in parts[:8]]
        idle = vals[3] + vals[4]
        total = sum(vals)
        return idle, total
    i0, t0 = read()
    time.sleep(sample_dt)
    i1, t1 = read()
    dt, di = (t1 - t0), (i1 - i0)
    return 0.0 if dt <= 0 else 100.0 * (dt - di) / dt


def _mem_usage_pct():
    info = {}
    try:
        with open('/proc/meminfo', 'r', encoding='utf-8') as f:
            for line in f:
                key, _, rest = line.partition(':')
                parts = rest.strip().split()
                if parts:
                    info[key] = int(parts[0])
    except OSError:
        return None
    total = info.get('MemTotal')
    avail = info.get('MemAvailable')
    if not total or avail is None:
        return None
    return 100.0 * (total - avail) / total


def _max_temp_c():
    max_temp = None
    for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                raw = int(f.read().strip())
            temp = raw / 1000.0 if raw > 1000 else float(raw)
            if max_temp is None or temp > max_temp:
                max_temp = temp
        except (OSError, ValueError):
            continue
    return max_temp


# Флаги vcgencmd get_throttled (только RPi). Технические строки прошивки —
# оставлены на английском в обоих языках (аппаратные диагностические флаги).
_THROTTLE_FLAGS = (
    (0x4, 'system throttled to prevent damage'),
    (0x40000, 'system was throttled previously'),
    (0x1, 'undervoltage now (power insufficient)'),
    (0x10000, 'undervoltage occurred previously'),
    (0x2, 'frequency capped now (thermal limit)'),
    (0x20000, 'frequency capped occurred previously'),
    (0x8, 'soft thermal limit now (freq reduced)'),
    (0x80000, 'soft thermal limit occurred previously'),
)


class SbcHealthCheck(BaseCheck):
    id = 'sbc_health'
    category = 'sbc'
    title = 'Onboard computer health'
    title_ru = 'Здоровье платы'
    description = 'Disk, RAM, CPU, temperature, (RPi) vcgencmd get_throttled'
    description_ru = 'Диск, RAM, CPU, температура, (RPi) vcgencmd get_throttled'

    def run(self, ctx):
        state = STATE_PASS
        th = ctx.profile['thresholds']

        try:
            _, _, free = shutil.disk_usage('/')
            free_gb = free / (1024 ** 3)
            ctx.metric('disk_free_gb', round(free_gb, 2))
            ctx.report('info', ctx.t(
                f'disk free: {free_gb:.2f} GB',
                f'диск свободно: {free_gb:.2f} ГБ'))
            if free_gb < th.get('min_disk_gb', 1.0):
                ctx.report('warn', ctx.t(
                    f'low disk space: {free_gb:.2f} GB',
                    f'мало места на диске: {free_gb:.2f} ГБ'))
                state = STATE_WARN
        except OSError as exc:
            ctx.report('warn', ctx.t(
                f'could not check disk: {exc}',
                f'не удалось проверить диск: {exc}'))
            state = STATE_WARN

        mem_pct = _mem_usage_pct()
        if mem_pct is not None:
            ctx.metric('mem_used_pct', round(mem_pct, 1))
            ctx.report('info', ctx.t(
                f'RAM used: {mem_pct:.1f}%',
                f'RAM занято: {mem_pct:.1f}%'))
        else:
            ctx.report('warn', ctx.t(
                '/proc/meminfo unavailable',
                '/proc/meminfo недоступен'))
            state = STATE_WARN

        cpu_pct = _cpu_usage_pct()
        ctx.metric('cpu_pct', round(cpu_pct, 1))
        ctx.report('info', f'CPU: {cpu_pct:.1f}%')
        if cpu_pct > th.get('max_cpu_pct', 85):
            ctx.report('warn', ctx.t(
                f'high CPU load: {cpu_pct:.1f}%',
                f'высокая загрузка CPU: {cpu_pct:.1f}%'))
            state = STATE_WARN

        temp = _max_temp_c()
        if temp is not None:
            ctx.metric('temp_c', round(temp, 1))
            ctx.report('info', ctx.t(
                f'temperature (max over zones): {temp:.1f}°C',
                f'температура (макс. по зонам): {temp:.1f}°C'))
            if temp > th.get('max_temp_c', 80):
                ctx.report('warn', ctx.t(
                    f'high temperature: {temp:.1f}°C',
                    f'высокая температура: {temp:.1f}°C'))
                state = STATE_WARN
        else:
            ctx.report('warn', ctx.t(
                'thermal_zone* unavailable',
                'thermal_zone* недоступны'))
            state = STATE_WARN

        if not os.path.exists('/dev/vchiq'):
            ctx.report('info', ctx.t(
                'vcgencmd get_throttled: skipped (no /dev/vchiq)',
                'vcgencmd get_throttled: пропущено (нет /dev/vchiq)'))
        else:
            try:
                out = subprocess.check_output(['vcgencmd', 'get_throttled'], text=True).strip()
                mask = int(out.split('=')[1], 16)
                ctx.metric('throttled_mask', hex(mask))
                any_flag = False
                for flag, desc in _THROTTLE_FLAGS:
                    if mask & flag:
                        any_flag = True
                        ctx.report('warn', desc)
                if any_flag:
                    state = STATE_WARN
                else:
                    ctx.report('info', ctx.t(
                        'throttle flags: none set',
                        'throttle-флаги: не установлены'))
            except (OSError, subprocess.SubprocessError, ValueError, IndexError) as exc:
                ctx.report('warn', ctx.t(
                    f'vcgencmd get_throttled unavailable: {exc}',
                    f'vcgencmd get_throttled недоступен: {exc}'))

        ctx.result.state = state


class RosGraphCheck(BaseCheck):
    id = 'ros_graph'
    category = 'sbc'
    title = 'ROS graph'
    title_ru = 'ROS-граф'
    description = 'All expected nodes from the profile present, required services visible'
    description_ru = 'Все ожидаемые ноды из профиля присутствуют, обязательные сервисы видны'

    def run(self, ctx):
        expected_nodes = ctx.profile.get('expected_nodes', [])
        expected_services = ctx.profile.get('expected_services', [])

        present = {name for name, _ns in ctx.node.get_node_names_and_namespaces()}
        missing_nodes = [n for n in expected_nodes if n not in present]
        ctx.metric('nodes_missing', missing_nodes)

        service_names = {name for name, _types in ctx.node.get_service_names_and_types()}
        missing_services = [s for s in expected_services if s not in service_names]
        ctx.metric('services_missing', missing_services)

        state = STATE_PASS
        if missing_nodes:
            ctx.report('warn', ctx.t(
                f'missing nodes: {", ".join(missing_nodes)}',
                f'отсутствуют ноды: {", ".join(missing_nodes)}'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'all expected nodes present',
                'все ожидаемые ноды присутствуют'))

        if missing_services:
            ctx.report('warn', ctx.t(
                f'missing services: {", ".join(missing_services)}',
                f'отсутствуют сервисы: {", ".join(missing_services)}'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'all required services visible',
                'все обязательные сервисы видны'))

        ctx.result.state = state


WEB_SERVICE_PATHS = {
    9090: '/api/services',  # ros_services_bridge
}


class WebServicesCheck(BaseCheck):
    id = 'web_services'
    category = 'sbc'
    title = 'Web services'
    title_ru = 'Веб-сервисы'
    description = 'HTTP GET localhost on ports from the profile (ros_services_bridge/rosboard/web_video_server)'
    description_ru = 'HTTP GET localhost на портах из профиля (ros_services_bridge/rosboard/web_video_server)'

    def run(self, ctx):
        ports = ctx.profile.get('web_ports', [])
        if not ports:
            ctx.result.state = STATE_SKIP
            return

        state = STATE_PASS
        for port in ports:
            path = WEB_SERVICE_PATHS.get(port, '/')
            url = f'http://127.0.0.1:{port}{path}'
            try:
                with urllib.request.urlopen(url, timeout=2.0) as resp:
                    ctx.metric(f'port_{port}', resp.status)
                    ctx.report('info', f'{url}: HTTP {resp.status}')
            except Exception as exc:  # best-effort probe: любая сетевая ошибка -> WARN
                ctx.metric(f'port_{port}', None)
                ctx.report('warn', ctx.t(
                    f'{url}: unreachable ({exc})',
                    f'{url}: недоступен ({exc})'))
                state = STATE_WARN

        ctx.result.state = state


class LedTestCheck(BaseCheck):
    id = 'led_test'
    category = 'sbc'
    title = 'LED test'
    title_ru = 'Тест LED'
    description = 'Trigger an effect via led_control, ask the user whether the strip blinks'
    description_ru = 'Включить эффект через led_control, спросить у пользователя, мигает ли лента'
    interactive = True

    def available(self, profile, lang='en'):
        if SetLEDEffect is None:
            return False, (
                'led_interfaces не собран в этом образе' if lang == 'ru'
                else 'led_interfaces not built in this image')
        if not profile.get('features', {}).get('led', True):
            return False, (
                'LED отключён в профиле платформы' if lang == 'ru'
                else 'LED disabled in the platform profile')
        return True, ''

    def run(self, ctx):
        if SetLEDEffect is None:
            ctx.report('warn', ctx.t(
                'led_interfaces not built — check skipped',
                'led_interfaces не собран — проверка пропущена'))
            ctx.result.state = STATE_SKIP
            return
        req = SetLEDEffect.Request(effect='blink', r=0, g=255, b=0)
        resp = ctx.call_service(SetLEDEffect, '/led_control/set_effect', req, timeout=2.0)
        if resp is None or not resp.success:
            ctx.report('warn', ctx.t(
                '/led_control/set_effect unavailable (led_control not running?)',
                '/led_control/set_effect недоступен (led_control не запущена?)'))
            ctx.result.state = STATE_SKIP
            return

        answer = ctx.wait_user(
            ctx.t('Is the strip blinking green?', 'Лента мигает зелёным?'),
            options=('yes', 'no', 'skip'), timeout_s=30.0)
        ctx.metric('answer', answer)
        if answer == 'yes':
            ctx.report('info', ctx.t(
                'strip is blinking — confirmed by user',
                'лента мигает — подтверждено пользователем'))
            ctx.result.state = STATE_PASS
        elif answer == 'no':
            ctx.report('warn', ctx.t(
                'user did not see the strip blink',
                'пользователь не увидел мигание ленты'))
            ctx.result.state = STATE_WARN
        else:
            ctx.report('warn', ctx.t(
                'skipped by user/timeout',
                'пропущено пользователем/таймаут'))
            ctx.result.state = STATE_SKIP


CHECKS = [SbcHealthCheck(), RosGraphCheck(), WebServicesCheck(), LedTestCheck()]
