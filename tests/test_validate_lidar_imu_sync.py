from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "holoagent_bridge" / "validate_lidar_imu_sync.py"
SPEC = importlib.util.spec_from_file_location("validate_lidar_imu_sync", SCRIPT)
assert SPEC and SPEC.loader
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def valid_inputs():
    lidar = np.arange(0.0, 10.0, 0.05)
    imu = np.arange(0.0, 10.0, 0.01)
    acceleration = np.tile([0.0, 0.0, 9.81], (imu.size, 1))
    angular = np.zeros((imu.size, 3))
    return lidar, imu, acceleration, angular


def analyze(**changes):
    lidar, imu, acceleration, angular = valid_inputs()
    values = {
        "lidar_timestamps_s": lidar,
        "imu_timestamps_s": imu,
        "accelerations": acceleration,
        "angular_velocities": angular,
        "lidar_last_receive_s": 99.8,
        "imu_last_receive_s": 99.9,
        "now_s": 100.0,
        "min_span_s": 8.0,
    }
    values.update(changes)
    return validator.analyze_streams(**values)


def test_valid_synchronized_stationary_streams_pass() -> None:
    report = analyze()
    assert report.ok
    assert report.lidar_rate_hz > 19.0
    assert report.imu_rate_hz > 99.0
    assert report.max_nearest_offset_s < 1e-9
    assert abs(report.acceleration_norm_median - 9.81) < 1e-9


def test_non_monotonic_timestamp_fails() -> None:
    lidar, *_ = valid_inputs()
    lidar[5] = lidar[4]
    report = analyze(lidar_timestamps_s=lidar)
    assert not report.ok
    assert any("LiDAR timestamps" in error for error in report.errors)


def test_missing_time_overlap_fails() -> None:
    _, imu, acceleration, angular = valid_inputs()
    report = analyze(
        lidar_timestamps_s=np.arange(20.0, 30.0, 0.05),
        imu_timestamps_s=imu,
        accelerations=acceleration,
        angular_velocities=angular,
    )
    assert not report.ok
    assert any("overlap" in error for error in report.errors)


def test_implausible_gravity_and_gyro_bias_fail() -> None:
    _, imu, _, _ = valid_inputs()
    report = analyze(
        accelerations=np.tile([0.0, 0.0, 2.0], (imu.size, 1)),
        angular_velocities=np.tile([0.0, 0.0, 0.8], (imu.size, 1)),
    )
    assert not report.ok
    assert any("acceleration norm" in error for error in report.errors)
    assert any("gyro norm" in error for error in report.errors)


def test_non_finite_measurement_fails() -> None:
    _, imu, acceleration, _ = valid_inputs()
    acceleration[3, 1] = np.nan
    report = analyze(accelerations=acceleration)
    assert not report.ok
    assert any("finite" in error for error in report.errors)


def test_stale_receive_time_fails() -> None:
    report = analyze(lidar_last_receive_s=97.0)
    assert not report.ok
    assert any("stale" in error for error in report.errors)


def test_failed_report_is_strict_json_serializable(tmp_path) -> None:
    report = analyze(lidar_timestamps_s=[], imu_timestamps_s=[], accelerations=[], angular_velocities=[])
    payload = validator.json_payload(report)
    assert payload["max_nearest_offset_s"] is None
    validator.json.dumps(payload, allow_nan=False)
    output = tmp_path / "nested" / "report.json"
    validator.write_json_report(output, payload)
    assert validator.json.loads(output.read_text())["ok"] is False


def test_fast_livo_overlay_enables_real_imu_and_declares_measured_extrinsic() -> None:
    config = (SCRIPT.parent / "fast_livo_mid360_sim.yaml").read_text(encoding="utf-8")
    assert 'imu_topic: "/livox/imu"' in config
    assert "imu_en: true" in config
    assert "img_en: 0" in config
    assert "enable_wheel_odom: false" in config
    assert "extrinsic_T: [0.0398735, 0.00227, 0.26826]" in config
    assert "0.999194395, 0.0, 0.040131795" in config
