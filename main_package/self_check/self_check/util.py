"""Чистые вспомогательные функции без зависимости на rclpy — тестируются pytest."""
from __future__ import annotations

import math


def quat_to_euler_rpy(qx: float, qy: float, qz: float, qw: float):
    """PX4 FRD-кватернион (x,y,z,w) -> (roll, pitch, yaw) в радианах."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_diff_deg(yaw_a_rad: float, yaw_b_rad: float) -> float:
    """Кратчайшая угловая разница в градусах, в диапазоне (-180, 180]."""
    d = math.degrees(yaw_a_rad - yaw_b_rad)
    return (d + 180.0) % 360.0 - 180.0


def battery_cell_voltage(voltage_v: float, cells: int):
    """В/банка, либо None, если число банок не задано в профиле."""
    if not cells or cells <= 0:
        return None
    return voltage_v / float(cells)


def evaluate_tilt(roll_deg: float, pitch_deg: float, max_tilt_deg: float) -> bool:
    """True, если наклон в пределах допуска (дрон стоит ровно)."""
    return abs(roll_deg) <= max_tilt_deg and abs(pitch_deg) <= max_tilt_deg


def evaluate_vpe(horiz_m: float, vert_m: float, yaw_deg,
                  pos_threshold_m: float, yaw_threshold_deg: float) -> bool:
    """True, если расхождение VPE/PX4 (позиция + при наличии yaw) в норме."""
    if horiz_m > pos_threshold_m or vert_m > pos_threshold_m:
        return False
    if yaw_deg is not None and abs(yaw_deg) > yaw_threshold_deg:
        return False
    return True


def frames_differ(sample_a: bytes, sample_b: bytes, min_abs_diff_sum: int = 50) -> bool:
    """«Кадр не замёрзший», если сумма абсолютной разности сэмпла байт выше порога."""
    n = min(len(sample_a), len(sample_b))
    if n == 0:
        return False
    diff = sum(abs(a - b) for a, b in zip(sample_a[:n], sample_b[:n]))
    return diff >= min_abs_diff_sum


def in_range_pct(value: float, expected: float, tolerance_pct: float) -> bool:
    """True, если value отклоняется от expected не больше чем на tolerance_pct%."""
    if expected <= 0:
        return True
    return abs(value - expected) / expected * 100.0 <= tolerance_pct


def rc_channel_span_ok(values, min_v: int, max_v: int, min_span_pct: float) -> bool:
    """True, если размах канала RC (max-min среди значений) >= min_span_pct% диапазона."""
    if not values:
        return False
    span = max(values) - min(values)
    full_range = max(1, max_v - min_v)
    return (span / full_range) * 100.0 >= min_span_pct
