"""Загрузка и слияние профиля платформы (config/default.yaml + config/<platform>.yaml).

Без зависимости на rclpy — тестируется pytest напрямую.
"""
from __future__ import annotations

import copy
import os

import yaml

PLATFORMS = ('RPI', 'RADXA', 'RDX', 'ORANGE', 'ORANGE_3W')

PLATFORM_FILE = {
    'RPI': 'rpi.yaml',
    'RADXA': 'radxa.yaml',
    'RDX': 'rdx.yaml',
    'ORANGE': 'orange.yaml',
    'ORANGE_3W': 'orange_3w.yaml',
}


def deep_merge(base: dict, override: dict) -> dict:
    """Рекурсивное слияние словарей: override поверх base, вложенные dict — тоже."""
    result = copy.deepcopy(base or {})
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_yaml_file(path: str) -> dict:
    if not path or not os.path.isfile(path):
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data or {}


def build_profile(config_dir: str, platform: str, override_file: str = '') -> dict:
    """Собрать профиль: default.yaml + оверрайд платформы (deep-merge).

    override_file, если задан — заменяет собой платформенный файл целиком
    (соответствует параметру ноды `profile_file`).
    """
    platform_name = (platform or 'RPI').upper()
    default = load_yaml_file(os.path.join(config_dir, 'default.yaml'))
    platform_path = override_file or os.path.join(
        config_dir, PLATFORM_FILE.get(platform_name, 'rpi.yaml'))
    override = load_yaml_file(platform_path)
    profile = deep_merge(default, override)
    profile['platform'] = platform_name
    return profile
