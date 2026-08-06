from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from tools.ground_truth_shared_memory_utils import GroundTruthSample


SCRIPT = Path(__file__).resolve().parents[1] / "holoagent_bridge" / "record_ground_truth.py"
SPEC = importlib.util.spec_from_file_location("record_ground_truth", SCRIPT)
assert SPEC and SPEC.loader
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


def test_format_sample_uses_seconds_and_xyzw_quaternion() -> None:
    sample = GroundTruthSample(
        2,
        1_250_000_000,
        np.array([1.0, 2.0, 3.0]),
        np.array([0.5, 0.5, -0.5, 0.5]),
    )
    values = [float(value) for value in recorder.format_sample(sample).split()]
    np.testing.assert_allclose(values, [1.25, 1, 2, 3, 0.5, -0.5, 0.5, 0.5])


def test_timestamp_guard_rejects_duplicate_and_regression() -> None:
    guard = recorder.TimestampGuard()
    guard.check(10)
    guard.check(11)
    with pytest.raises(ValueError):
        guard.check(11)
    with pytest.raises(ValueError):
        guard.check(9)

