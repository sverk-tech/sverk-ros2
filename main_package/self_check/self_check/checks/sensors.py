"""Проверки категории sensors: калибровка, горизонт, периферия + интерактив."""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import (
    VehicleAttitude, SensorCombined, VehicleLocalPosition, DistanceSensor)
from offboard_interfaces.srv import GetParam

from ..context import wait_latest
from ..report import STATE_PASS, STATE_WARN, STATE_FAIL, STATE_SKIP
from ..util import quat_to_euler_rpy, yaw_diff_deg, evaluate_tilt
from .base import BaseCheck

CAL_PARAMS = ('CAL_GYRO0_ID', 'CAL_ACC0_ID', 'CAL_MAG0_ID')


def _yaw_from_attitude(msg) -> float:
    qw, qx, qy, qz = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
    _, _, yaw = quat_to_euler_rpy(qx, qy, qz, qw)
    return yaw


def _get_z(sub):
    msg = sub.get()
    return float(msg.z) if msg is not None else None


# ---------------------------------------------------------------------
# Пассивные
# ---------------------------------------------------------------------

class CalibParamsCheck(BaseCheck):
    id = 'calib_params'
    category = 'sensors'
    title = 'Calibration parameters'
    title_ru = 'Параметры калибровки'
    description = 'CAL_GYRO0_ID/CAL_ACC0_ID/CAL_MAG0_ID != 0; SENS_BOARD_ROT value'
    description_ru = 'CAL_GYRO0_ID/CAL_ACC0_ID/CAL_MAG0_ID != 0; значение SENS_BOARD_ROT'

    def run(self, ctx):
        service = '/fmu_control/get_param'
        first = ctx.call_service(GetParam, service, GetParam.Request(name=CAL_PARAMS[0]), timeout=2.0)
        if first is None:
            ctx.report('warn', ctx.t(
                f'{service} unavailable (fmu_control not running?)',
                f'{service} недоступен (fmu_control не запущена?)'))
            ctx.result.state = STATE_SKIP
            return

        state = STATE_PASS
        results = {CAL_PARAMS[0]: first}
        for name in CAL_PARAMS[1:]:
            results[name] = ctx.call_service(GetParam, service, GetParam.Request(name=name), timeout=2.0)

        for name in CAL_PARAMS:
            resp = results[name]
            if resp is None or not resp.success:
                ctx.report('warn', ctx.t(
                    f'{name}: could not read',
                    f'{name}: не удалось прочитать'))
                state = STATE_WARN
                continue
            ctx.metric(name.lower(), resp.value)
            if int(resp.value) == 0:
                ctx.report('warn', ctx.t(
                    f'{name} = 0 (sensor not calibrated)',
                    f'{name} = 0 (сенсор не откалиброван)'))
                state = STATE_WARN
            else:
                ctx.report('info', f'{name} = {int(resp.value)}')

        rot = ctx.call_service(GetParam, service, GetParam.Request(name='SENS_BOARD_ROT'), timeout=2.0)
        if rot is not None and rot.success:
            ctx.metric('sens_board_rot', rot.value)
            ctx.report('info', f'SENS_BOARD_ROT = {int(rot.value)}')

        ctx.result.state = state


class AttitudeLevelCheck(BaseCheck):
    id = 'attitude_level'
    category = 'sensors'
    title = 'Horizon'
    title_ru = 'Горизонт'
    description = 'vehicle_attitude: roll/pitch < threshold (drone sitting level)'
    description_ru = 'vehicle_attitude: roll/pitch < порога (дрон стоит ровно)'

    def run(self, ctx):
        topic = ctx.profile['topics']['vehicle_attitude']
        msg = ctx.wait_for_message(VehicleAttitude, topic, 3.0)
        if msg is None:
            ctx.report('error', ctx.t(
                f'{topic}: no messages',
                f'{topic}: нет сообщений'))
            ctx.result.state = STATE_FAIL
            return
        roll, pitch, _ = quat_to_euler_rpy(
            float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0]))
        roll_deg, pitch_deg = math.degrees(roll), math.degrees(pitch)
        ctx.metric('roll_deg', round(roll_deg, 2))
        ctx.metric('pitch_deg', round(pitch_deg, 2))
        max_tilt = ctx.profile['thresholds']['tilt_deg']
        ctx.report('info', ctx.t(
            f'roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}° (threshold {max_tilt:.1f}°)',
            f'roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}° (порог {max_tilt:.1f}°)'))
        if evaluate_tilt(roll_deg, pitch_deg, max_tilt):
            ctx.result.state = STATE_PASS
        else:
            ctx.report('warn', ctx.t(
                'drone is not level — set it horizontal',
                'дрон стоит не ровно — поставьте горизонтально'))
            ctx.result.state = STATE_WARN


class PeripheralsCheck(BaseCheck):
    id = 'peripherals'
    category = 'sensors'
    title = 'Peripherals'
    title_ru = 'Периферия'
    description = 'LaserScan /scan and DistanceSensor — per profile flags'
    description_ru = 'LaserScan /scan и DistanceSensor — по флагам профиля'

    def available(self, profile, lang='en'):
        feats = profile.get('features', {})
        if not feats.get('lidar', False) and not feats.get('rangefinder', False):
            return False, (
                'lidar и rangefinder отключены в профиле' if lang == 'ru'
                else 'lidar and rangefinder disabled in the profile')
        return True, ''

    def run(self, ctx):
        feats = ctx.profile.get('features', {})
        topics = ctx.profile['topics']
        state = STATE_PASS

        if feats.get('lidar', False):
            topic = topics['scan']
            ok = ctx.wait_for_updates(LaserScan, topic, 3.0, min_msgs=2)
            if ok:
                ctx.report('info', ctx.t(
                    f'{topic}: updating',
                    f'{topic}: обновляется'))
            else:
                ctx.report('warn', ctx.t(
                    f'{topic}: no updates (lidar enabled in profile)',
                    f'{topic}: нет обновлений (lidar включен в профиле)'))
                state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'lidar disabled in profile — skipped',
                'lidar отключен в профиле — пропущено'))

        if feats.get('rangefinder', False):
            topic = topics['distance_sensor']
            msg = ctx.wait_for_message(DistanceSensor, topic, 3.0)
            if msg is not None:
                ctx.metric('distance_m', round(float(msg.current_distance), 3))
                ctx.report('info', ctx.t(
                    f'{topic}: {msg.current_distance:.2f} m',
                    f'{topic}: {msg.current_distance:.2f} м'))
            else:
                ctx.report('warn', ctx.t(
                    f'{topic}: no messages (rangefinder enabled in profile)',
                    f'{topic}: нет сообщений (rangefinder включен в профиле)'))
                state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'rangefinder disabled in profile — skipped',
                'rangefinder отключен в профиле — пропущено'))

        ctx.result.state = state


# ---------------------------------------------------------------------
# Интерактивные
# ---------------------------------------------------------------------

class HoldStillCheck(BaseCheck):
    id = 'hold_still'
    category = 'sensors'
    title = 'Hold still'
    title_ru = 'Стоять неподвижно'
    description = 'Drone level for 5 seconds: gyro noise, tilt, ArUco pose drift'
    description_ru = 'Дрон стоит ровно 5 секунд: шум gyro, наклон, дрейф ArUco-позы'
    interactive = True

    def run(self, ctx):
        duration = 5.0
        use_aruco = ctx.profile.get('features', {}).get('aruco', True)
        ctx.instruct(ctx.t(
            'Place the drone level and do not touch it for 5 seconds',
            'Поставьте дрон ровно и не трогайте 5 секунд'))

        specs = [
            ('attitude', ctx.profile['topics']['vehicle_attitude'], VehicleAttitude),
            ('sensor_combined', ctx.profile['topics']['sensor_combined'], SensorCombined),
        ]
        if use_aruco:
            specs.append(('world_pose', ctx.profile['topics']['world_pose'], PoseWithCovarianceStamped))

        samples = ctx.collect_multi(specs, duration)
        state = STATE_PASS

        att = samples.get('attitude', [])
        if not att:
            ctx.report('error', ctx.t(
                'no vehicle_attitude data',
                'нет данных vehicle_attitude'))
            ctx.result.state = STATE_FAIL
            return
        max_roll = max_pitch = 0.0
        for _, msg in att:
            roll, pitch, _ = quat_to_euler_rpy(
                float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0]))
            max_roll = max(max_roll, abs(math.degrees(roll)))
            max_pitch = max(max_pitch, abs(math.degrees(pitch)))
        max_tilt = ctx.profile['thresholds']['tilt_deg']
        ctx.metric('max_roll_deg', round(max_roll, 2))
        ctx.metric('max_pitch_deg', round(max_pitch, 2))
        if max_roll > max_tilt or max_pitch > max_tilt:
            ctx.report('warn', ctx.t(
                f'tilt exceeds threshold: roll={max_roll:.2f}, pitch={max_pitch:.2f}',
                f'наклон превышает порог: roll={max_roll:.2f}, pitch={max_pitch:.2f}'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'tilt within limits: roll={max_roll:.2f}°, pitch={max_pitch:.2f}°',
                f'наклон в норме: roll={max_roll:.2f}°, pitch={max_pitch:.2f}°'))

        gyro = samples.get('sensor_combined', [])
        if gyro:
            max_gyro = max(math.sqrt(sum(v * v for v in msg.gyro_rad)) for _, msg in gyro)
            ctx.metric('max_gyro_rad_s', round(max_gyro, 4))
            ctx.report('info', ctx.t(
                f'gyro noise: {max_gyro:.4f} rad/s',
                f'шум gyro: {max_gyro:.4f} рад/с'))
            if max_gyro > 0.05:
                ctx.report('warn', ctx.t(
                    f'gyro noise higher than expected: {max_gyro:.4f} rad/s',
                    f'шум gyro выше ожидаемого: {max_gyro:.4f} рад/с'))
                state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'sensor_combined unavailable — gyro noise estimated from attitude derivative',
                'sensor_combined недоступен — шум gyro оценён по производной attitude'))
            if len(att) >= 2:
                max_rate = 0.0
                for (t0, m0), (t1, m1) in zip(att[:-1], att[1:]):
                    dt = t1 - t0
                    if dt <= 1e-3:
                        continue
                    y0 = _yaw_from_attitude(m0)
                    y1 = _yaw_from_attitude(m1)
                    max_rate = max(max_rate, abs(yaw_diff_deg(y1, y0)) / dt)
                ctx.metric('max_yaw_rate_deg_s', round(max_rate, 2))

        if use_aruco:
            pose = samples.get('world_pose', [])
            if len(pose) >= 2:
                max_v = 0.0
                for (t0, m0), (t1, m1) in zip(pose[:-1], pose[1:]):
                    dt = t1 - t0
                    if dt <= 1e-3:
                        continue
                    p0, p1 = m0.pose.pose.position, m1.pose.pose.position
                    v = math.dist((p0.x, p0.y, p0.z), (p1.x, p1.y, p1.z)) / dt
                    max_v = max(max_v, v)
                ctx.metric('aruco_drift_ms', round(max_v, 3))
                still_vel = ctx.profile['thresholds']['still_vel_ms']
                if max_v > still_vel:
                    ctx.report('warn', ctx.t(
                        f'ArUco pose drift: {max_v:.3f} m/s (threshold {still_vel:.2f})',
                        f'дрейф ArUco-позы: {max_v:.3f} м/с (порог {still_vel:.2f})'))
                    state = STATE_WARN
                else:
                    ctx.report('info', ctx.t(
                        f'ArUco pose drift within limits: {max_v:.3f} m/s',
                        f'дрейф ArUco-позы в норме: {max_v:.3f} м/с'))
            else:
                ctx.report('info', ctx.t(
                    'not enough world_pose data to estimate drift',
                    'недостаточно данных world_pose для оценки дрейфа'))

        ctx.result.state = state


class OrientationTiltCheck(BaseCheck):
    id = 'orientation_tilt'
    category = 'sensors'
    title = 'Axis tilts'
    title_ru = 'Наклоны по осям'
    description = '4 substeps: nose down/up, left/right side down'
    description_ru = '4 подшага: нос вниз/вверх, левый/правый бок вниз'
    interactive = True

    # (label_en, label_ru, cond)
    _STEPS = (
        ('nose down (~30°)', 'носом вниз (~30°)', lambda pitch, roll: pitch < -15.0),
        ('nose up (~30°)', 'носом вверх (~30°)', lambda pitch, roll: pitch > 15.0),
        ('left side down (~30°)', 'левым боком вниз (~30°)', lambda pitch, roll: roll < -15.0),
        ('right side down (~30°)', 'правым боком вниз (~30°)', lambda pitch, roll: roll > 15.0),
    )

    def run(self, ctx):
        topic = ctx.profile['topics']['vehicle_attitude']
        state = STATE_PASS
        done_steps = 0
        with ctx.subscribe_latest(VehicleAttitude, topic) as sub:
            for label_en, label_ru, cond in self._STEPS:
                label = ctx.t(label_en, label_ru)
                instruction = ctx.t(f'Tilt the drone {label_en}', f'Наклоните дрон {label_ru}')
                ctx.instruct(instruction)
                deadline = time.time() + 30.0
                ok = False
                while time.time() < deadline and not ctx.aborted.is_set():
                    msg = sub.get()
                    if msg is not None:
                        roll, pitch, _ = quat_to_euler_rpy(
                            float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0]))
                        roll_deg, pitch_deg = math.degrees(roll), math.degrees(pitch)
                        ctx.instruct(instruction, {'roll_deg': round(roll_deg, 1), 'pitch_deg': round(pitch_deg, 1)})
                        if cond(pitch_deg, roll_deg):
                            ok = True
                            break
                    time.sleep(0.2)
                if ctx.aborted.is_set():
                    ctx.report('warn', ctx.t(f'{label}: aborted', f'{label}: прервано'))
                    ctx.result.state = STATE_SKIP
                    return
                if ok:
                    done_steps += 1
                    ctx.report('info', f'{label}: OK')
                else:
                    # §4: таймаут подшага -> FAIL (единственное терминальное
                    # состояние на всю проверку, поэтому пробрасываем сюда).
                    ctx.report('error', ctx.t(
                        f'{label}: not completed within 30s (substep timeout)',
                        f'{label}: не выполнено за 30с (таймаут подшага)'))
                    state = STATE_FAIL
        ctx.metric('steps_done', done_steps)
        ctx.result.state = state


class YawRotationCheck(BaseCheck):
    id = 'yaw_rotation'
    category = 'sensors'
    title = 'Yaw rotation'
    title_ru = 'Поворот по yaw'
    description = 'Rotate the drone clockwise ~90° — heading (+ArUco if active)'
    description_ru = 'Повернуть дрон по часовой стрелке на ~90° — heading (+ArUco, если активен)'
    interactive = True

    def run(self, ctx):
        att_topic = ctx.profile['topics']['vehicle_attitude']
        use_aruco = ctx.profile.get('features', {}).get('aruco', True)
        world_pose_topic = ctx.profile['topics']['world_pose']
        instruction = ctx.t(
            'Rotate the drone clockwise by about 90°',
            'Поверните дрон по часовой стрелке примерно на 90°')

        att_sub = ctx.subscribe_latest(VehicleAttitude, att_topic)
        pose_sub = ctx.subscribe_latest(PoseWithCovarianceStamped, world_pose_topic) if use_aruco else None
        best_delta = 0.0
        pose0 = pose1 = None
        try:
            start_msg = wait_latest(att_sub, 3.0, ctx.aborted)
            if start_msg is None:
                ctx.report('error', ctx.t(
                    f'{att_topic}: no attitude data',
                    f'{att_topic}: нет данных attitude'))
                ctx.result.state = STATE_FAIL
                return
            yaw0 = _yaw_from_attitude(start_msg)
            pose0 = pose_sub.get() if pose_sub else None

            ctx.instruct(instruction)
            deadline = time.time() + 30.0
            while time.time() < deadline and not ctx.aborted.is_set():
                msg = att_sub.get()
                if msg is not None:
                    delta = yaw_diff_deg(_yaw_from_attitude(msg), yaw0)
                    if abs(delta) > abs(best_delta):
                        best_delta = delta
                    ctx.instruct(instruction, {'yaw_delta_deg': round(delta, 1)})
                    if 60.0 <= abs(delta) <= 120.0:
                        break
                time.sleep(0.2)
            pose1 = pose_sub.get() if pose_sub else None
        finally:
            att_sub.close()
            if pose_sub:
                pose_sub.close()

        if ctx.aborted.is_set():
            ctx.result.state = STATE_SKIP
            return

        ctx.metric('yaw_delta_deg', round(best_delta, 1))
        state = STATE_PASS
        if 60.0 <= abs(best_delta) <= 120.0:
            ctx.report('info', ctx.t(
                f'yaw rotation (attitude): {best_delta:.1f}°',
                f'поворот по yaw (attitude): {best_delta:.1f}°'))
        else:
            ctx.report('warn', ctx.t(
                f'yaw rotation outside 60-120° range: {best_delta:.1f}°',
                f'поворот по yaw вне диапазона 60-120°: {best_delta:.1f}°'))
            state = STATE_WARN

        if pose0 is not None and pose1 is not None:
            q0, q1 = pose0.pose.pose.orientation, pose1.pose.pose.orientation
            _, _, yaw_a0 = quat_to_euler_rpy(q0.x, q0.y, q0.z, q0.w)
            _, _, yaw_a1 = quat_to_euler_rpy(q1.x, q1.y, q1.z, q1.w)
            aruco_delta = yaw_diff_deg(yaw_a1, yaw_a0)
            ctx.metric('aruco_yaw_delta_deg', round(aruco_delta, 1))
            ctx.report('info', ctx.t(
                f'yaw rotation (ArUco): {aruco_delta:.1f}°',
                f'поворот по yaw (ArUco): {aruco_delta:.1f}°'))

        ctx.result.state = state


class RaiseLowerCheck(BaseCheck):
    id = 'raise_lower'
    category = 'sensors'
    title = 'Raise / lower'
    title_ru = 'Подъём/опускание'
    description = 'Raise the drone ~1m and lower — baro/ArUco/rangefinder consistency'
    description_ru = 'Поднять дрон на ~1м и опустить — согласованность baro/ArUco/дальномера'
    interactive = True

    def run(self, ctx):
        use_aruco = ctx.profile.get('features', {}).get('aruco', True)
        use_range = ctx.profile.get('features', {}).get('rangefinder', False)
        topics = ctx.profile['topics']
        instruction = ctx.t(
            'Raise the drone about 1 meter and lower it again',
            'Поднимите дрон примерно на 1 метр и снова опустите')

        lp_sub = ctx.subscribe_latest(VehicleLocalPosition, topics['vehicle_local_position'])
        pose_sub = ctx.subscribe_latest(PoseWithCovarianceStamped, topics['world_pose']) if use_aruco else None
        range_sub = ctx.subscribe_latest(DistanceSensor, topics['distance_sensor']) if use_range else None

        # "Вверх" считается по знаку своей оси у каждого источника: lp.z — NED
        # (down positive) -> вверх = z0 - z; world_pose и дальномер (смотрит
        # вниз) — вверх = рост значения. Проверяем именно знак (§4 «проверка
        # знака оси»), а не просто модуль изменения — иначе перепутанная
        # полярность оси прошла бы проверку.
        max_lp_up = max_pose_up = max_range_up = float('-inf')
        try:
            z0 = _get_z(lp_sub)
            pose0_msg = pose_sub.get() if pose_sub else None
            pose_z0 = pose0_msg.pose.pose.position.z if pose0_msg else None
            range0_msg = range_sub.get() if range_sub else None
            range0 = float(range0_msg.current_distance) if range0_msg else None

            ctx.instruct(instruction)
            deadline = time.time() + 30.0
            while time.time() < deadline and not ctx.aborted.is_set():
                z = _get_z(lp_sub)
                if z is not None and z0 is not None:
                    max_lp_up = max(max_lp_up, z0 - z)
                pose_msg = pose_sub.get() if pose_sub else None
                if pose_msg is not None and pose_z0 is not None:
                    max_pose_up = max(max_pose_up, pose_msg.pose.pose.position.z - pose_z0)
                range_msg = range_sub.get() if range_sub else None
                if range_msg is not None and range0 is not None:
                    max_range_up = max(max_range_up, float(range_msg.current_distance) - range0)
                metrics = {'lp_up_m': round(max(max_lp_up, 0.0), 2)}
                if pose_sub:
                    metrics['aruco_up_m'] = round(max(max_pose_up, 0.0), 2)
                if range_sub:
                    metrics['range_up_m'] = round(max(max_range_up, 0.0), 2)
                ctx.instruct(instruction, metrics)
                if max_lp_up > 0.3:
                    break
                time.sleep(0.2)
        finally:
            lp_sub.close()
            if pose_sub:
                pose_sub.close()
            if range_sub:
                range_sub.close()

        if ctx.aborted.is_set():
            ctx.result.state = STATE_SKIP
            return

        ctx.metric('lp_up_m', round(max_lp_up, 2))
        state = STATE_PASS
        if max_lp_up < 0.3:
            ctx.report('warn', ctx.t(
                f'local position (baro/EKF) did not show a climb: {max_lp_up:.2f} m',
                f'локальная позиция (baro/EKF) не показала подъём: {max_lp_up:.2f} м'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'local position showed a climb of {max_lp_up:.2f} m',
                f'локальная позиция показала подъём на {max_lp_up:.2f} м'))

        if pose_sub:
            ctx.metric('aruco_up_m', round(max_pose_up, 2))
            if max_pose_up < 0.3:
                ctx.report('warn', ctx.t(
                    f'ArUco world_pose.z did not show a climb: {max_pose_up:.2f} m',
                    f'ArUco world_pose.z не показал подъём: {max_pose_up:.2f} м'))
                state = STATE_WARN
            else:
                ctx.report('info', ctx.t(
                    f'ArUco world_pose.z showed a climb of {max_pose_up:.2f} m',
                    f'ArUco world_pose.z показал подъём на {max_pose_up:.2f} м'))

        if range_sub:
            ctx.metric('range_up_m', round(max_range_up, 2))
            if max_range_up < 0.3:
                ctx.report('warn', ctx.t(
                    f'rangefinder did not show a distance increase: {max_range_up:.2f} m',
                    f'дальномер не показал рост дистанции: {max_range_up:.2f} м'))
                state = STATE_WARN
            else:
                ctx.report('info', ctx.t(
                    f'rangefinder showed a distance increase of {max_range_up:.2f} m',
                    f'дальномер показал рост дистанции на {max_range_up:.2f} м'))

        ctx.result.state = state


CHECKS = [
    CalibParamsCheck(), AttitudeLevelCheck(), PeripheralsCheck(),
    HoldStillCheck(), OrientationTiltCheck(), YawRotationCheck(), RaiseLowerCheck(),
]
