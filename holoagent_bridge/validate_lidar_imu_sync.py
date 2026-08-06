#!/usr/bin/env python3
"""Validate live real LiDAR/IMU streams before allowing FAST-LIVO use."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import NamedTuple, Sequence

import numpy as np


class StreamReport(NamedTuple):
    ok: bool
    errors: tuple[str, ...]
    lidar_count: int
    imu_count: int
    lidar_rate_hz: float
    imu_rate_hz: float
    overlap_s: float
    max_nearest_offset_s: float
    acceleration_norm_median: float
    gyro_norm_median: float


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate stationary IsaacLab LiDAR/IMU rate, time alignment, and measurements."
    )
    parser.add_argument("--lidar-topic", default="/mid360/points")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--duration", type=positive_float, default=10.0, help="Required overlap in simulator seconds.")
    parser.add_argument("--wall-timeout", type=positive_float, default=120.0)
    parser.add_argument("--output-json", help="Optional path for the machine-readable report.")
    return parser.parse_args(argv)


def _rate(timestamps: np.ndarray) -> float:
    if timestamps.size < 2:
        return 0.0
    span = float(timestamps[-1] - timestamps[0])
    return float((timestamps.size - 1) / span) if span > 0 else 0.0


def _nearest_offset(reference: np.ndarray, samples: np.ndarray) -> float:
    in_range = reference[(reference >= samples[0]) & (reference <= samples[-1])]
    if in_range.size == 0:
        return math.inf
    right = np.searchsorted(samples, in_range, side="left")
    right = np.clip(right, 0, samples.size - 1)
    left = np.maximum(right - 1, 0)
    offsets = np.minimum(np.abs(samples[right] - in_range), np.abs(samples[left] - in_range))
    return float(np.max(offsets))


def collection_complete(
    lidar_timestamps_s: Sequence[float], imu_timestamps_s: Sequence[float], required_overlap_s: float
) -> bool:
    if len(lidar_timestamps_s) < 2 or len(imu_timestamps_s) < 2:
        return False
    overlap = min(lidar_timestamps_s[-1], imu_timestamps_s[-1]) - max(
        lidar_timestamps_s[0], imu_timestamps_s[0]
    )
    return overlap >= required_overlap_s


def points_xyz_array(points) -> np.ndarray:
    values = np.asarray(points)
    if values.dtype.names and {"x", "y", "z"}.issubset(values.dtype.names):
        return np.column_stack((values["x"], values["y"], values["z"])).astype(
            np.float32, copy=False
        )
    values = np.asarray(list(points) if values.ndim == 0 else values, dtype=np.float32)
    return values.reshape((-1, 3))


def analyze_streams(
    *,
    lidar_timestamps_s: Sequence[float],
    imu_timestamps_s: Sequence[float],
    accelerations: Sequence[Sequence[float]],
    angular_velocities: Sequence[Sequence[float]],
    lidar_last_receive_s: float,
    imu_last_receive_s: float,
    now_s: float,
    min_span_s: float,
    lidar_point_counts: Sequence[int] | None = None,
    lidar_finite_flags: Sequence[bool] | None = None,
) -> StreamReport:
    lidar = np.asarray(lidar_timestamps_s, dtype=np.float64)
    imu = np.asarray(imu_timestamps_s, dtype=np.float64)
    acceleration = np.asarray(accelerations, dtype=np.float64)
    angular = np.asarray(angular_velocities, dtype=np.float64)
    errors: list[str] = []

    for label, stamps, minimum_count in (("LiDAR", lidar, 5), ("IMU", imu, 20)):
        if stamps.ndim != 1 or stamps.size < minimum_count:
            errors.append(f"{label} stream has too few samples")
        elif not np.isfinite(stamps).all():
            errors.append(f"{label} timestamps must be finite")
        elif np.any(np.diff(stamps) <= 0):
            errors.append(f"{label} timestamps are not strictly increasing")

    measurements_valid = (
        acceleration.shape == (imu.size, 3)
        and angular.shape == (imu.size, 3)
    )
    if not measurements_valid:
        errors.append("IMU measurement shapes do not match IMU timestamps")
    elif not np.isfinite(acceleration).all() or not np.isfinite(angular).all():
        errors.append("IMU measurements must be finite")

    if lidar_point_counts is not None:
        counts = np.asarray(lidar_point_counts)
        if counts.shape != lidar.shape or np.any(counts <= 0):
            errors.append("LiDAR frames must contain points")
    if lidar_finite_flags is not None:
        flags = np.asarray(lidar_finite_flags, dtype=bool)
        if flags.shape != lidar.shape or not np.all(flags):
            errors.append("LiDAR points must be finite")

    if now_s - lidar_last_receive_s > 1.0:
        errors.append("LiDAR stream is stale")
    if now_s - imu_last_receive_s > 1.0:
        errors.append("IMU stream is stale")

    lidar_rate = _rate(lidar) if lidar.size >= 2 and np.isfinite(lidar).all() else 0.0
    imu_rate = _rate(imu) if imu.size >= 2 and np.isfinite(imu).all() else 0.0
    if lidar_rate < 5.0:
        errors.append(f"LiDAR rate is too low: {lidar_rate:.3f} Hz")
    if imu_rate < 20.0:
        errors.append(f"IMU rate is too low: {imu_rate:.3f} Hz")

    overlap = 0.0
    nearest_offset = math.inf
    timestamp_arrays_valid = (
        lidar.size > 0
        and imu.size > 0
        and np.isfinite(lidar).all()
        and np.isfinite(imu).all()
    )
    if timestamp_arrays_valid:
        overlap = max(0.0, min(float(lidar[-1]), float(imu[-1])) - max(float(lidar[0]), float(imu[0])))
        if overlap < min_span_s:
            errors.append(f"LiDAR/IMU time overlap is too short: {overlap:.3f} s")
        nearest_offset = _nearest_offset(lidar, imu) if overlap > 0 else math.inf
        if nearest_offset > 0.05:
            errors.append(f"LiDAR/IMU nearest timestamp offset is too large: {nearest_offset:.6f} s")

    acceleration_norm = math.nan
    gyro_norm = math.nan
    if measurements_valid and np.isfinite(acceleration).all() and np.isfinite(angular).all():
        acceleration_norm = float(np.median(np.linalg.norm(acceleration, axis=1)))
        gyro_norm = float(np.median(np.linalg.norm(angular, axis=1)))
        if not 7.0 <= acceleration_norm <= 13.0:
            errors.append(f"stationary acceleration norm is implausible: {acceleration_norm:.3f} m/s^2")
        if gyro_norm > 0.2:
            errors.append(f"stationary gyro norm is too large: {gyro_norm:.3f} rad/s")

    return StreamReport(
        ok=not errors,
        errors=tuple(errors),
        lidar_count=int(lidar.size),
        imu_count=int(imu.size),
        lidar_rate_hz=lidar_rate,
        imu_rate_hz=imu_rate,
        overlap_s=overlap,
        max_nearest_offset_s=nearest_offset,
        acceleration_norm_median=acceleration_norm,
        gyro_norm_median=gyro_norm,
    )


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def json_payload(report: StreamReport) -> dict[str, object]:
    payload = report._asdict()
    for key, value in payload.items():
        if isinstance(value, float) and not math.isfinite(value):
            payload[key] = None
    return payload


def write_json_report(path: str | Path, payload: dict[str, object]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, PointCloud2
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:
        raise SystemExit("ROS 2 Python packages are unavailable. Source ROS 2 first.") from exc

    lidar_timestamps: list[float] = []
    imu_timestamps: list[float] = []
    accelerations: list[tuple[float, float, float]] = []
    angular_velocities: list[tuple[float, float, float]] = []
    lidar_counts: list[int] = []
    lidar_finite: list[bool] = []
    lidar_last_receive = -math.inf
    imu_last_receive = -math.inf

    rclpy.init()
    node = Node("lidar_imu_sync_validator")

    def lidar_callback(msg: PointCloud2) -> None:
        nonlocal lidar_last_receive
        xyz = points_xyz_array(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        )
        lidar_timestamps.append(_stamp_seconds(msg.header.stamp))
        lidar_counts.append(int(xyz.shape[0]))
        lidar_finite.append(bool(xyz.ndim == 2 and xyz.shape[1] == 3 and np.isfinite(xyz).all()))
        lidar_last_receive = time.monotonic()

    def imu_callback(msg: Imu) -> None:
        nonlocal imu_last_receive
        imu_timestamps.append(_stamp_seconds(msg.header.stamp))
        accelerations.append(
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        )
        angular_velocities.append((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        imu_last_receive = time.monotonic()

    node.create_subscription(PointCloud2, args.lidar_topic, lidar_callback, qos_profile_sensor_data)
    node.create_subscription(Imu, args.imu_topic, imu_callback, qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while (
            rclpy.ok()
            and time.monotonic() - start < args.wall_timeout
            and not collection_complete(lidar_timestamps, imu_timestamps, args.duration)
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    now = time.monotonic()
    report = analyze_streams(
        lidar_timestamps_s=lidar_timestamps,
        imu_timestamps_s=imu_timestamps,
        accelerations=accelerations,
        angular_velocities=angular_velocities,
        lidar_last_receive_s=lidar_last_receive,
        imu_last_receive_s=imu_last_receive,
        now_s=now,
        min_span_s=args.duration * 0.8,
        lidar_point_counts=lidar_counts,
        lidar_finite_flags=lidar_finite,
    )
    payload = json_payload(report)
    print(json.dumps(payload, indent=2, allow_nan=False), flush=True)
    if args.output_json:
        write_json_report(args.output_json, payload)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if report.ok else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
