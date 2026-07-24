"""
Unit tests for ulog_adapter._datasets_to_frames.

Run from the package root (no ROS, no pyulog, no real .ulg file needed):
    python -m pytest test/test_ulog_adapter.py -v

These exercise the pure dataset → frames/metadata conversion with hand-built
fake datasets. Regression focus: the old `_resolve_field(...) or default`
idiom raised `ValueError: truth value of an array ... is ambiguous` on every
real log (multi-sample vehicle_attitude). The fix must return frames instead.
"""
import sys
import os
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest

from pid_tuning_assistant.ulog_adapter import _datasets_to_frames


def _ds(name: str, data: dict, multi_id: int = 0):
    """Build a fake pyulog dataset. `multi_id` is required — _find_topic checks it."""
    return types.SimpleNamespace(name=name, multi_id=multi_id, data=data)


def _ang_vel(n: int = 5, dt_us: int = 2500):
    """vehicle_angular_velocity — the primary time axis for every real log."""
    t = np.arange(n, dtype=np.int64) * dt_us
    return _ds("vehicle_angular_velocity", {
        "timestamp": t,
        "xyz[0]": np.linspace(0.0, 0.4, n),
        "xyz[1]": np.linspace(0.0, -0.3, n),
        "xyz[2]": np.zeros(n),
    })


def _attitude(n: int = 5, dt_us: int = 2500):
    """vehicle_attitude with >1 sample — identity-ish quaternion."""
    t = np.arange(n, dtype=np.int64) * dt_us
    return _ds("vehicle_attitude", {
        "timestamp": t,
        "q[0]": np.ones(n),
        "q[1]": np.zeros(n),
        "q[2]": np.zeros(n),
        "q[3]": np.zeros(n),
    })


def test_multisample_attitude_does_not_raise():
    """Regression: multi-sample attitude used to trigger `ndarray or` ValueError."""
    warnings: list = []
    datasets = [_ang_vel(n=5), _attitude(n=5)]

    # Must NOT raise ValueError — must build frames.
    result = _datasets_to_frames(datasets, warnings)

    assert len(result.frames) == 5
    # Identity quaternion → roll/pitch/yaw are real floats (≈0), not None.
    f = result.frames[0]
    assert f.roll is not None and abs(f.roll) < 1e-6
    assert f.pitch is not None
    assert f.yaw is not None
    assert result.metadata.n_frames == 5


def test_missing_battery_is_tolerated():
    """A log without battery_status yields frames with v_bat = None, no exception."""
    warnings: list = []
    datasets = [_ang_vel(n=4), _attitude(n=4)]   # no battery_status

    result = _datasets_to_frames(datasets, warnings)

    assert len(result.frames) == 4
    assert all(f.v_bat is None for f in result.frames)
    assert all(f.bat_remaining is None for f in result.frames)


def test_missing_attitude_warns_and_nulls_euler():
    """No vehicle_attitude → warning recorded, roll/pitch/yaw = None in frames."""
    warnings: list = []
    datasets = [_ang_vel(n=3)]   # no vehicle_attitude

    result = _datasets_to_frames(datasets, warnings)

    assert len(result.frames) == 3
    assert any("vehicle_attitude not found" in w for w in warnings)
    for f in result.frames:
        assert f.roll is None
        assert f.pitch is None
        assert f.yaw is None
    # Rate fields still come from the primary axis.
    assert result.frames[0].roll_rate is not None


def test_missing_primary_axis_returns_empty():
    """No vehicle_angular_velocity → empty frames + explanatory warning."""
    warnings: list = []
    datasets = [_attitude(n=3)]   # no primary axis

    result = _datasets_to_frames(datasets, warnings)

    assert result.frames == []
    assert result.metadata.n_frames == 0
    assert any("vehicle_angular_velocity not found" in w for w in warnings)
