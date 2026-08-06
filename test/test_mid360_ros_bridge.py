#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).with_name("mid360_to_ros2_topic.py")
spec = importlib.util.spec_from_file_location("mid360_to_ros2_topic", SCRIPT)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_parse_args() -> None:
    args = bridge.parse_args(["--topic", "/mid360", "--rate", "5", "--once"])
    assert args.topic == "/mid360"
    assert args.rate == 5.0
    assert args.once is True


def test_prepare_points_filters_invalid_and_limits() -> None:
    points = np.array(
        [
            [1.0, 2.0, 3.0],
            [np.inf, 0.0, 0.0],
            [4.0, 5.0, 6.0],
            [np.nan, 1.0, 1.0],
        ],
        dtype=np.float64,
    )

    prepared = bridge.prepare_points(points, max_points=1)

    assert prepared.dtype == np.float32
    assert prepared.shape == (1, 3)
    np.testing.assert_allclose(prepared[0], [1.0, 2.0, 3.0])


def test_timestamp_guard_rejects_duplicate_or_regression() -> None:
    guard = bridge.TimestampGuard()
    guard.check(10)
    guard.check(11)
    with pytest.raises(bridge.TimestampRegressionError):
        guard.check(11)
    with pytest.raises(bridge.TimestampRegressionError):
        guard.check(9)


if __name__ == "__main__":
    test_parse_args()
    test_prepare_points_filters_invalid_and_limits()
