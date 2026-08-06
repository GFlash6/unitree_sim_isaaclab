from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from tools.imu_shared_memory_utils import ImuSample


SCRIPT = Path(__file__).resolve().parents[1] / "holoagent_bridge" / "imu_to_ros2_topic.py"
SPEC = importlib.util.spec_from_file_location("imu_to_ros2_topic", SCRIPT)
assert SPEC and SPEC.loader
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


def sample(timestamp_ns: int = 1_234_567_890) -> ImuSample:
    return ImuSample(
        sequence=2,
        timestamp_ns=timestamp_ns,
        quaternion_wxyz=np.array([0.5, 0.5, -0.5, 0.5], dtype=np.float32),
        linear_acceleration=np.array([1.0, 2.0, 3.0], dtype=np.float32),
        angular_velocity=np.array([-0.1, 0.2, 0.3], dtype=np.float32),
    )


def test_sample_to_fields_converts_wxyz_to_ros_xyzw() -> None:
    fields = bridge.sample_to_fields(sample())
    np.testing.assert_allclose(fields.orientation_xyzw, [0.5, -0.5, 0.5, 0.5])
    np.testing.assert_allclose(fields.linear_acceleration, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(fields.angular_velocity, [-0.1, 0.2, 0.3])


def test_split_timestamp_preserves_nanoseconds() -> None:
    assert bridge.split_timestamp_ns(1_234_567_890) == (1, 234_567_890)


@pytest.mark.parametrize(
    "field,index",
    [("quaternion_wxyz", 0), ("linear_acceleration", 1), ("angular_velocity", 2)],
)
def test_sample_to_fields_rejects_non_finite_values(field: str, index: int) -> None:
    values = list(sample())
    vector = values[index + 2].copy()
    vector[0] = np.nan
    values[index + 2] = vector
    with pytest.raises(ValueError, match="finite"):
        bridge.sample_to_fields(ImuSample(*values))


def test_sample_to_fields_rejects_invalid_quaternion_norm() -> None:
    bad = sample()._replace(quaternion_wxyz=np.array([1.0, 1.0, 0.0, 0.0]))
    with pytest.raises(ValueError, match="quaternion"):
        bridge.sample_to_fields(bad)


def test_timestamp_guard_rejects_regression_and_duplicate() -> None:
    guard = bridge.TimestampGuard()
    guard.check(100)
    guard.check(101)
    with pytest.raises(bridge.TimestampRegressionError):
        guard.check(101)
    with pytest.raises(bridge.TimestampRegressionError):
        guard.check(99)


def test_publisher_qos_matches_fast_livo_reliable_subscription() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "QoSReliabilityPolicy.RELIABLE" in source
    assert "depth=2000" in source
    assert "qos_profile_sensor_data" not in source
