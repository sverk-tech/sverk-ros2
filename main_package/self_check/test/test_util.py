"""Пороговые функции self_check.util — банки, наклон, расхождение VPE и т.д."""
import math

from self_check.util import (
    battery_cell_voltage, evaluate_tilt, evaluate_vpe, frames_differ,
    in_range_pct, quat_to_euler_rpy, rc_channel_span_ok, yaw_diff_deg,
)


def test_battery_cell_voltage():
    assert battery_cell_voltage(15.8, 4) == 3.95
    assert battery_cell_voltage(15.8, 0) is None
    assert battery_cell_voltage(15.8, None) is None


def test_evaluate_tilt():
    assert evaluate_tilt(0.5, -1.0, 2.0) is True
    assert evaluate_tilt(2.5, 0.0, 2.0) is False
    assert evaluate_tilt(0.0, -2.1, 2.0) is False


def test_evaluate_vpe_position_only():
    assert evaluate_vpe(0.2, 0.1, None, 0.5, 8.0) is True
    assert evaluate_vpe(0.6, 0.1, None, 0.5, 8.0) is False
    assert evaluate_vpe(0.1, 0.6, None, 0.5, 8.0) is False


def test_evaluate_vpe_with_yaw():
    assert evaluate_vpe(0.1, 0.1, 5.0, 0.5, 8.0) is True
    assert evaluate_vpe(0.1, 0.1, 9.0, 0.5, 8.0) is False


def test_frames_differ():
    a = bytes([10, 20, 30, 40])
    same = bytes([10, 20, 30, 40])
    different = bytes([200, 20, 30, 40])
    assert frames_differ(a, same) is False
    assert frames_differ(a, different) is True
    assert frames_differ(b'', b'') is False


def test_in_range_pct():
    assert in_range_pct(28.0, 30.0, 30.0) is True
    assert in_range_pct(10.0, 30.0, 30.0) is False
    assert in_range_pct(1.0, 0.0, 10.0) is True  # expected<=0 -> всегда норма


def test_rc_channel_span_ok():
    vals = [1000, 1500, 2000, 1200]
    assert rc_channel_span_ok(vals, 1000, 2000, 60.0) is True
    vals_small = [1490, 1500, 1510]
    assert rc_channel_span_ok(vals_small, 1000, 2000, 60.0) is False
    assert rc_channel_span_ok([], 1000, 2000, 60.0) is False


def test_quat_to_euler_rpy_identity():
    roll, pitch, yaw = quat_to_euler_rpy(0.0, 0.0, 0.0, 1.0)
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw) < 1e-9


def test_quat_to_euler_rpy_yaw_90():
    # Поворот на +90° вокруг Z: qz=sin(45°), qw=cos(45°)
    half = math.radians(45.0)
    roll, pitch, yaw = quat_to_euler_rpy(0.0, 0.0, math.sin(half), math.cos(half))
    assert abs(roll) < 1e-6
    assert abs(pitch) < 1e-6
    assert abs(math.degrees(yaw) - 90.0) < 1e-4


def test_yaw_diff_deg_wraps():
    assert abs(yaw_diff_deg(math.radians(179), math.radians(-179)) - (-2.0)) < 1e-6
    assert abs(yaw_diff_deg(math.radians(10), math.radians(0)) - 10.0) < 1e-6
