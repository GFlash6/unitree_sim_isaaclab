from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "holoagent_bridge" / "record_ros_odometry.py"
SPEC = importlib.util.spec_from_file_location("record_ros_odometry", SCRIPT)
assert SPEC and SPEC.loader
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


def test_format_pose_matches_evaluator_trajectory_format() -> None:
    line = recorder.format_pose(1_250_000_000, [1, 2, 3], [0.1, 0.2, 0.3, 0.9])
    assert [float(value) for value in line.split()] == [1.25, 1, 2, 3, 0.1, 0.2, 0.3, 0.9]


def test_timestamp_guard_skips_equal_updates_but_rejects_regression() -> None:
    guard = recorder.TimestampGuard()
    assert guard.check(10)
    assert guard.check(11)
    assert not guard.check(11)
    with pytest.raises(recorder.TimestampRegressionError):
        guard.check(9)
