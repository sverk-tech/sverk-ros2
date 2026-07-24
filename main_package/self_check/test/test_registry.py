"""Реестр проверок и манифесты (00_contract.md §4-5).

Модули checks/* импортируют типы сообщений/сервисов (px4_msgs,
offboard_interfaces, aruco_det_loc, led_interfaces, self_check_interfaces) —
это генерируемые ROS-пакеты, доступные только после colcon build текущего
воркспейса. В окружении без собранного воркспейса этот файл пропускается
целиком (importorskip), остальные test_*.py в этом пакете от rclpy/ROS не
зависят и работают всегда.
"""
import os

import pytest

rclpy = pytest.importorskip('rclpy')
px4_msgs = pytest.importorskip('px4_msgs')

from self_check.checks import ALL_IDS, INTERACTIVE_IDS, PASSIVE_IDS, REGISTRY  # noqa: E402
from self_check.profile import build_profile  # noqa: E402
from self_check.report import manifest_entry  # noqa: E402

EXPECTED_PASSIVE_IDS = {
    'link_fmu', 'power_battery', 'calib_params', 'prearm_failsafe',
    'estimator_flags', 'camera_stream', 'aruco_pipeline', 'vpe_consistency',
    'offboard_status', 'attitude_level', 'sbc_health', 'ros_graph',
    'web_services', 'peripherals',
}

EXPECTED_INTERACTIVE_IDS = {
    'hold_still', 'orientation_tilt', 'yaw_rotation', 'raise_lower',
    'rc_sticks', 'motor_spin', 'aruco_show_map', 'move_1m', 'led_test',
}

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config')


def test_registry_has_all_contract_ids():
    assert set(PASSIVE_IDS) == EXPECTED_PASSIVE_IDS
    assert set(INTERACTIVE_IDS) == EXPECTED_INTERACTIVE_IDS
    assert set(ALL_IDS) == EXPECTED_PASSIVE_IDS | EXPECTED_INTERACTIVE_IDS
    assert len(ALL_IDS) == len(set(ALL_IDS))  # без дублей


def test_only_motor_spin_is_dangerous():
    dangerous = [cid for cid, check in REGISTRY.items() if check.dangerous]
    assert dangerous == ['motor_spin']


def test_interactive_flag_matches_partition():
    for cid in PASSIVE_IDS:
        assert REGISTRY[cid].interactive is False
    for cid in INTERACTIVE_IDS:
        assert REGISTRY[cid].interactive is True


@pytest.mark.parametrize('platform', ['RPI', 'RADXA', 'RDX', 'ORANGE', 'ORANGE_3W'])
def test_manifest_list_matches_schema(platform):
    profile = build_profile(CONFIG_DIR, platform)
    for cid in ALL_IDS:
        check = REGISTRY[cid]
        available, reason = check.available(profile)
        m = manifest_entry(check, available, reason)
        assert set(m.keys()) == {
            'id', 'category', 'title', 'description', 'interactive',
            'dangerous', 'available', 'unavailable_reason',
        }
        assert m['id'] == cid
        assert isinstance(m['available'], bool)
        if m['available']:
            assert m['unavailable_reason'] == ''


def test_peripherals_unavailable_when_lidar_and_rangefinder_off():
    profile = build_profile(CONFIG_DIR, 'RPI')
    check = REGISTRY['peripherals']
    available, reason = check.available(profile)
    assert available is False
    assert reason
