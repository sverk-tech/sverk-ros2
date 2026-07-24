"""Проверки категории vision: камера, ArUco-конвейер, VPE + интерактив."""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import UInt8
from px4_msgs.msg import VehicleOdometry
from aruco_det_loc.msg import MarkerArray

from ..context import qos_transient_local, wait_latest
from ..report import STATE_PASS, STATE_WARN, STATE_FAIL, STATE_SKIP
from ..util import frames_differ, in_range_pct, evaluate_vpe, quat_to_euler_rpy, yaw_diff_deg
from .base import BaseCheck


def _aruco_off_reason(lang):
    return ('ArUco отключён в профиле платформы' if lang == 'ru'
            else 'ArUco disabled in the platform profile')


class CameraStreamCheck(BaseCheck):
    id = 'camera_stream'
    category = 'vision'
    title = 'Camera stream'
    title_ru = 'Видеопоток камеры'
    description = 'Frames flowing, actual FPS, match with camera_info, frame not frozen'
    description_ru = 'Кадры идут, фактический FPS, совпадение с camera_info, кадр не замёрз'

    def run(self, ctx):
        image_topic = ctx.profile['camera']['image_topic']
        info_topic = ctx.profile['camera']['info_topic']
        expected_fps = float(ctx.profile['camera'].get('expected_fps', 30))

        samples = ctx.collect(image_topic, Image, 2.0, max_samples=200)
        if len(samples) < 2:
            ctx.report('error', ctx.t(
                f'{image_topic}: not enough frames in 2s ({len(samples)})',
                f'{image_topic}: недостаточно кадров за 2с ({len(samples)})'))
            ctx.result.state = STATE_FAIL
            return

        actual_fps = (len(samples) - 1) / max(1e-3, samples[-1][0] - samples[0][0])
        ctx.metric('fps', round(actual_fps, 1))
        tol = ctx.profile['thresholds'].get('camera_fps_tolerance_pct', 30.0)
        state = STATE_PASS
        if not in_range_pct(actual_fps, expected_fps, tol):
            ctx.report('warn', ctx.t(
                f'FPS outside expected range: {actual_fps:.1f} (expected ~{expected_fps:.0f})',
                f'FPS вне ожидаемого диапазона: {actual_fps:.1f} (ожидалось ~{expected_fps:.0f})'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'FPS: {actual_fps:.1f} (expected ~{expected_fps:.0f})',
                f'FPS: {actual_fps:.1f} (ожидалось ~{expected_fps:.0f})'))

        first_frame, last_frame = samples[0][1], samples[-1][1]
        sample_a = bytes(first_frame.data[:4096])
        sample_b = bytes(last_frame.data[:4096])
        if not frames_differ(sample_a, sample_b):
            ctx.report('warn', ctx.t(
                'frame looks frozen (adjacent frames almost identical)',
                'кадр выглядит замёрзшим (соседние кадры почти идентичны)'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                'frames are changing (not frozen)',
                'кадры меняются (не замёрзли)'))

        info = ctx.wait_for_message(CameraInfo, info_topic, 2.0)
        if info is None:
            ctx.report('warn', ctx.t(
                f'{info_topic}: no camera_info',
                f'{info_topic}: нет camera_info'))
            state = STATE_WARN
        elif info.width != last_frame.width or info.height != last_frame.height:
            ctx.report('warn', ctx.t(
                f'camera_info {info.width}x{info.height} != frame {last_frame.width}x{last_frame.height}',
                f'camera_info {info.width}x{info.height} != кадр {last_frame.width}x{last_frame.height}'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'camera_info matches frame: {info.width}x{info.height}',
                f'camera_info совпадает с кадром: {info.width}x{info.height}'))

        ctx.result.state = state


class ArucoPipelineCheck(BaseCheck):
    id = 'aruco_pipeline'
    category = 'vision'
    title = 'ArUco pipeline'
    title_ru = 'ArUco-конвейер'
    description = '/aruco/map/markers received, markers/pose_cov/world_pose rates, reset_counter present'
    description_ru = '/aruco/map/markers получена, частоты markers/pose_cov/world_pose, reset_counter присутствует'

    def available(self, profile, lang='en'):
        if not profile.get('features', {}).get('aruco', True):
            return False, _aruco_off_reason(lang)
        return True, ''

    def run(self, ctx):
        topics = ctx.profile['topics']
        state = STATE_PASS

        map_msg = ctx.wait_for_message(
            MarkerArray, topics['map_markers'], 3.0, qos=qos_transient_local())
        if map_msg is None:
            ctx.report('error', ctx.t(
                f"{topics['map_markers']}: marker map not received (TRANSIENT_LOCAL)",
                f"{topics['map_markers']}: карта меток не получена (TRANSIENT_LOCAL)"))
            ctx.result.state = STATE_FAIL
            return
        n_markers = len(map_msg.markers)
        ctx.metric('map_markers_count', n_markers)
        ctx.report('info', ctx.t(
            f'map_markers: received ({n_markers} markers)',
            f'map_markers: получена ({n_markers} меток)'))

        for key, msg_type in (
                ('markers', MarkerArray),
                ('pose_cov', PoseWithCovarianceStamped),
                ('world_pose', PoseWithCovarianceStamped)):
            topic = topics[key]
            rate = ctx.measure_rate(topic, msg_type, 1.5)
            ctx.metric(f'{key}_rate_hz', round(rate, 1))
            if rate <= 0.0:
                ctx.report('warn', ctx.t(
                    f'{topic}: no updates',
                    f'{topic}: нет обновлений'))
                state = STATE_WARN
            else:
                ctx.report('info', ctx.t(
                    f'{topic}: {rate:.1f} Hz',
                    f'{topic}: {rate:.1f} Гц'))

        rc = ctx.wait_for_message(UInt8, topics['reset_counter'], 2.0, qos=qos_transient_local())
        if rc is None:
            ctx.report('warn', ctx.t(
                f"{topics['reset_counter']}: no data",
                f"{topics['reset_counter']}: нет данных"))
            state = STATE_WARN
        else:
            ctx.metric('reset_counter', int(rc.data))
            ctx.report('info', f'reset_counter = {rc.data}')

        ctx.result.state = state


class VpeConsistencyCheck(BaseCheck):
    id = 'vpe_consistency'
    category = 'vision'
    title = 'VPE consistency'
    title_ru = 'Согласованность VPE'
    description = 'vehicle_visual_odometry flowing; divergence from vehicle_odometry in position/yaw'
    description_ru = 'vehicle_visual_odometry идёт; расхождение с vehicle_odometry по позиции/yaw'

    def available(self, profile, lang='en'):
        if not profile.get('features', {}).get('vio', True):
            return False, (
                'VIO отключён в профиле платформы' if lang == 'ru'
                else 'VIO disabled in the platform profile')
        return True, ''

    def run(self, ctx):
        topics = ctx.profile['topics']
        in_topic = topics['visual_odometry']
        out_topic = topics['vehicle_odometry']

        if not ctx.wait_for_updates(VehicleOdometry, in_topic, 3.0, min_msgs=2):
            ctx.report('error', ctx.t(
                f'{in_topic}: no updates (VIO/px4_local_pose_publisher not running?)',
                f'{in_topic}: нет обновлений (VIO/px4_local_pose_publisher не работает?)'))
            ctx.result.state = STATE_FAIL
            return
        if not ctx.wait_for_updates(VehicleOdometry, out_topic, 3.0, min_msgs=2):
            ctx.report('error', ctx.t(
                f'{out_topic}: no updates',
                f'{out_topic}: нет обновлений'))
            ctx.result.state = STATE_FAIL
            return

        samples = ctx.collect_multi(
            [('vin', in_topic, VehicleOdometry), ('vout', out_topic, VehicleOdometry)], 1.5)
        vin = samples['vin'][-1][1] if samples['vin'] else None
        vout = samples['vout'][-1][1] if samples['vout'] else None
        if vin is None or vout is None:
            ctx.report('warn', ctx.t(
                'could not obtain odometry for comparison',
                'не удалось получить одометрию для сравнения'))
            ctx.result.state = STATE_WARN
            return

        dx = float(vout.position[0]) - float(vin.position[0])
        dy = float(vout.position[1]) - float(vin.position[1])
        dz = float(vout.position[2]) - float(vin.position[2])
        horiz = math.hypot(dx, dy)
        vert = abs(dz)
        ctx.metric('horiz_diff_m', round(horiz, 3))
        ctx.metric('vert_diff_m', round(vert, 3))

        yaw_deg = None
        qin, qout = vin.q, vout.q
        if qin is not None and qout is not None and len(qin) == 4 and len(qout) == 4:
            _, _, yin = quat_to_euler_rpy(float(qin[1]), float(qin[2]), float(qin[3]), float(qin[0]))
            _, _, yout = quat_to_euler_rpy(float(qout[1]), float(qout[2]), float(qout[3]), float(qout[0]))
            yaw_deg = yaw_diff_deg(yout, yin)
            ctx.metric('yaw_diff_deg', round(yaw_deg, 2))

        pos_th = ctx.profile['thresholds']['vpe_pos_m']
        yaw_th = ctx.profile['thresholds']['vpe_yaw_deg']
        yaw_str = f'{yaw_deg:.2f}°' if yaw_deg is not None else '?'
        ctx.report('info', ctx.t(
            f'VPE/PX4 divergence: horiz={horiz:.3f}m vert={vert:.3f}m yaw={yaw_str}',
            f'расхождение VPE/PX4: horiz={horiz:.3f}м vert={vert:.3f}м yaw={yaw_str}'))
        if evaluate_vpe(horiz, vert, yaw_deg, pos_th, yaw_th):
            ctx.result.state = STATE_PASS
        else:
            ctx.report('warn', ctx.t(
                'VPE/PX4 divergence exceeds threshold',
                'расхождение VPE/PX4 превышает порог'))
            ctx.result.state = STATE_WARN


class ArucoShowMapCheck(BaseCheck):
    id = 'aruco_show_map'
    category = 'vision'
    title = 'Show ArUco map'
    title_ru = 'Показать карту ArUco'
    description = 'Point the camera at the marker map — detect >=2 markers, pose_cov, world_pose, VPE divergence OK'
    description_ru = 'Направить камеру на карту меток — детект >=2 меток, pose_cov, world_pose, VPE-расхождение в норме'
    interactive = True

    def available(self, profile, lang='en'):
        if not profile.get('features', {}).get('aruco', True):
            return False, _aruco_off_reason(lang)
        return True, ''

    def run(self, ctx):
        topics = ctx.profile['topics']
        instruction = ctx.t(
            'Point the drone camera at the ArUco marker map',
            'Направьте камеру дрона на карту ArUco-меток')
        ctx.instruct(instruction)

        markers_sub = ctx.subscribe_latest(MarkerArray, topics['markers'])
        pose_sub = ctx.subscribe_latest(PoseWithCovarianceStamped, topics['pose_cov'])
        world_sub = ctx.subscribe_latest(PoseWithCovarianceStamped, topics['world_pose'])
        n_markers = 0
        got_pose = got_world = False
        try:
            deadline = time.time() + 60.0
            while time.time() < deadline and not ctx.aborted.is_set():
                m = markers_sub.get()
                if m is not None:
                    n_markers = len(m.markers)
                got_pose = pose_sub.get() is not None
                got_world = world_sub.get() is not None
                ctx.instruct(instruction, {'markers': n_markers, 'pose_cov': got_pose, 'world_pose': got_world})
                if n_markers >= 2 and got_pose and got_world:
                    break
                time.sleep(0.2)
        finally:
            markers_sub.close()
            pose_sub.close()
            world_sub.close()

        if ctx.aborted.is_set():
            ctx.result.state = STATE_SKIP
            return

        ctx.metric('markers', n_markers)
        ctx.metric('pose_cov', got_pose)
        ctx.metric('world_pose', got_world)
        if not (n_markers >= 2 and got_pose and got_world):
            ctx.report('warn', ctx.t(
                f'could not recognize the map in 60s (markers={n_markers}, pose_cov={got_pose}, world_pose={got_world})',
                f'не удалось распознать карту за 60с (markers={n_markers}, pose_cov={got_pose}, world_pose={got_world})'))
            ctx.result.state = STATE_WARN
            return

        ctx.report('info', ctx.t(
            f'map recognized: {n_markers} markers, pose_cov and world_pose present',
            f'карта распознана: {n_markers} меток, pose_cov и world_pose есть'))

        if not ctx.profile.get('features', {}).get('vio', True):
            ctx.report('info', ctx.t(
                'VIO disabled in profile — VPE divergence not checked',
                'VIO отключён в профиле — VPE-расхождение не проверялось'))
            ctx.result.state = STATE_PASS
            return

        topics = ctx.profile['topics']
        samples = ctx.collect_multi(
            [('vin', topics['visual_odometry'], VehicleOdometry),
             ('vout', topics['vehicle_odometry'], VehicleOdometry)], 1.0)
        vin = samples['vin'][-1][1] if samples['vin'] else None
        vout = samples['vout'][-1][1] if samples['vout'] else None
        if vin is None or vout is None:
            ctx.report('warn', ctx.t(
                'could not obtain odometry to check VPE divergence',
                'не удалось получить одометрию для проверки VPE-расхождения'))
            ctx.result.state = STATE_WARN
            return

        dx = float(vout.position[0]) - float(vin.position[0])
        dy = float(vout.position[1]) - float(vin.position[1])
        dz = float(vout.position[2]) - float(vin.position[2])
        horiz, vert = math.hypot(dx, dy), abs(dz)
        pos_th = ctx.profile['thresholds']['vpe_pos_m']
        ctx.metric('vpe_horiz_diff_m', round(horiz, 3))
        ctx.metric('vpe_vert_diff_m', round(vert, 3))
        if horiz <= pos_th and vert <= pos_th:
            ctx.report('info', ctx.t(
                f'VPE divergence within limits: horiz={horiz:.3f}m vert={vert:.3f}m',
                f'VPE-расхождение в норме: horiz={horiz:.3f}м vert={vert:.3f}м'))
            ctx.result.state = STATE_PASS
        else:
            ctx.report('warn', ctx.t(
                f'VPE divergence above threshold: horiz={horiz:.3f}m vert={vert:.3f}m',
                f'VPE-расхождение выше порога: horiz={horiz:.3f}м vert={vert:.3f}м'))
            ctx.result.state = STATE_WARN


class Move1mCheck(BaseCheck):
    id = 'move_1m'
    category = 'vision'
    title = 'Move 1 meter'
    title_ru = 'Перенос на 1 метр'
    description = 'Move the drone ~1m aside and back — displacement per world_pose'
    description_ru = 'Перенести дрон на ~1м в сторону и вернуть — смещение по world_pose'
    interactive = True

    def available(self, profile, lang='en'):
        if not profile.get('features', {}).get('aruco', True):
            return False, _aruco_off_reason(lang)
        return True, ''

    def run(self, ctx):
        topic = ctx.profile['topics']['world_pose']
        pose_sub = ctx.subscribe_latest(PoseWithCovarianceStamped, topic)
        max_dist = final_d = 0.0
        try:
            start = wait_latest(pose_sub, 3.0, ctx.aborted)
            if start is None:
                ctx.report('error', ctx.t(
                    f'{topic}: no world_pose data',
                    f'{topic}: нет данных world_pose'))
                ctx.result.state = STATE_FAIL
                return
            p0 = start.pose.pose.position

            instr_out = ctx.t(
                'Move the drone about 1 meter to the side',
                'Перенесите дрон примерно на 1 метр в сторону')
            ctx.instruct(instr_out)
            deadline = time.time() + 30.0
            while time.time() < deadline and not ctx.aborted.is_set():
                msg = pose_sub.get()
                if msg is not None:
                    p = msg.pose.pose.position
                    d = math.dist((p0.x, p0.y, p0.z), (p.x, p.y, p.z))
                    max_dist = max(max_dist, d)
                    ctx.instruct(instr_out, {'dist_m': round(d, 2)})
                    if 0.5 <= d <= 2.0:
                        break
                time.sleep(0.2)

            instr_back = ctx.t(
                'Return the drone to its original position',
                'Верните дрон обратно на исходное место')
            ctx.instruct(instr_back)
            deadline = time.time() + 30.0
            final_d = max_dist
            while time.time() < deadline and not ctx.aborted.is_set():
                msg = pose_sub.get()
                if msg is not None:
                    p = msg.pose.pose.position
                    final_d = math.dist((p0.x, p0.y, p0.z), (p.x, p.y, p.z))
                    ctx.instruct(instr_back, {'dist_m': round(final_d, 2)})
                    if final_d <= 0.3:
                        break
                time.sleep(0.15)
        finally:
            pose_sub.close()

        if ctx.aborted.is_set():
            ctx.result.state = STATE_SKIP
            return

        ctx.metric('max_dist_m', round(max_dist, 2))
        ctx.metric('return_dist_m', round(final_d, 2))
        state = STATE_PASS
        if not (0.5 <= max_dist <= 2.0):
            ctx.report('warn', ctx.t(
                f'displacement outside 0.5-2m range: {max_dist:.2f}m',
                f'смещение вне диапазона 0.5-2м: {max_dist:.2f}м'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'displacement: {max_dist:.2f}m',
                f'смещение: {max_dist:.2f}м'))
        if final_d > 0.3:
            ctx.report('warn', ctx.t(
                f'drone did not return to the start point: {final_d:.2f}m',
                f'дрон не вернулся в исходную точку: {final_d:.2f}м'))
            state = STATE_WARN
        else:
            ctx.report('info', ctx.t(
                f'returned to the start point: {final_d:.2f}m',
                f'возврат в исходную точку: {final_d:.2f}м'))
        ctx.result.state = state


CHECKS = [
    CameraStreamCheck(), ArucoPipelineCheck(), VpeConsistencyCheck(),
    ArucoShowMapCheck(), Move1mCheck(),
]
