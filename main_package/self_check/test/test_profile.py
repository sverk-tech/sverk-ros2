"""Загрузка и слияние профиля платформы (self_check.profile)."""
from self_check.profile import build_profile, deep_merge, load_yaml_file


def test_deep_merge_overrides_nested_keys():
    base = {'a': 1, 'thresholds': {'x': 1, 'y': 2}, 'features': {'led': True}}
    override = {'thresholds': {'y': 20, 'z': 30}}
    merged = deep_merge(base, override)
    assert merged == {
        'a': 1,
        'thresholds': {'x': 1, 'y': 20, 'z': 30},
        'features': {'led': True},
    }
    # base не мутируется
    assert base['thresholds']['y'] == 2


def test_deep_merge_replaces_non_dict_values():
    base = {'features': {'lidar': False}}
    override = {'features': {'lidar': True}}
    assert deep_merge(base, override)['features']['lidar'] is True


def test_deep_merge_empty_override():
    base = {'a': 1}
    assert deep_merge(base, None) == base
    assert deep_merge(base, {}) == base


def test_load_yaml_file_missing_returns_empty(tmp_path):
    assert load_yaml_file(str(tmp_path / 'missing.yaml')) == {}
    assert load_yaml_file('') == {}


def test_build_profile_merges_default_and_platform(tmp_path):
    (tmp_path / 'default.yaml').write_text(
        'battery_cells: 4\n'
        'thresholds:\n'
        '  tilt_deg: 2.0\n'
        '  max_cpu_pct: 85\n'
        'features:\n'
        '  led: true\n'
        '  lidar: false\n',
        encoding='utf-8',
    )
    (tmp_path / 'orange_3w.yaml').write_text(
        'thresholds:\n'
        '  max_cpu_pct: 95\n',
        encoding='utf-8',
    )

    profile = build_profile(str(tmp_path), 'orange_3w')

    assert profile['platform'] == 'ORANGE_3W'
    assert profile['battery_cells'] == 4
    assert profile['thresholds']['tilt_deg'] == 2.0
    assert profile['thresholds']['max_cpu_pct'] == 95  # переопределено платформой
    assert profile['features']['led'] is True


def test_build_profile_unknown_platform_falls_back_to_rpi_file(tmp_path):
    (tmp_path / 'default.yaml').write_text('battery_cells: 4\n', encoding='utf-8')
    (tmp_path / 'rpi.yaml').write_text('battery_cells: 6\n', encoding='utf-8')

    profile = build_profile(str(tmp_path), 'not_a_real_platform')

    assert profile['platform'] == 'NOT_A_REAL_PLATFORM'
    assert profile['battery_cells'] == 6


def test_build_profile_override_file_replaces_platform_file(tmp_path):
    (tmp_path / 'default.yaml').write_text('battery_cells: 4\n', encoding='utf-8')
    custom = tmp_path / 'custom.yaml'
    custom.write_text('battery_cells: 12\n', encoding='utf-8')

    profile = build_profile(str(tmp_path), 'RPI', override_file=str(custom))

    assert profile['battery_cells'] == 12
