#!/usr/bin/env python3
"""Forward fresh IsaacLab IMU records to ROS 2 without changing their time domain."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import NamedTuple

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.imu_shared_memory_utils import ImuReader, ImuSample


class ImuFields(NamedTuple):
    orientation_xyzw: np.ndarray
    linear_acceleration: np.ndarray
    angular_velocity: np.ndarray


class TimestampRegressionError(RuntimeError):
    pass


class TimestampGuard:
    def __init__(self) -> None:
        self.last_timestamp_ns = 0

    def check(self, timestamp_ns: int) -> None:
        if timestamp_ns <= self.last_timestamp_ns:
            raise TimestampRegressionError(
                f"IMU timestamp did not increase: {timestamp_ns} <= {self.last_timestamp_ns}"
            )
        self.last_timestamp_ns = timestamp_ns


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Forward IsaacLab IMU shared memory to ROS 2.")
    parser.add_argument("--topic", default="/livox/imu", help="ROS 2 sensor_msgs/Imu topic.")
    parser.add_argument("--frame-id", default="imu_link", help="IMU body frame.")
    parser.add_argument("--poll-rate", type=positive_float, default=200.0, help="Shared-memory poll rate in Hz.")
    parser.add_argument("--once", action="store_true", help="Publish one fresh sample and exit.")
    return parser.parse_args(argv)


def split_timestamp_ns(timestamp_ns: int) -> tuple[int, int]:
    if timestamp_ns <= 0:
        raise ValueError("timestamp_ns must be positive")
    return divmod(int(timestamp_ns), 1_000_000_000)


def sample_to_fields(sample: ImuSample) -> ImuFields:
    quaternion = np.asarray(sample.quaternion_wxyz, dtype=np.float64)
    acceleration = np.asarray(sample.linear_acceleration, dtype=np.float64)
    angular = np.asarray(sample.angular_velocity, dtype=np.float64)
    if quaternion.shape != (4,) or acceleration.shape != (3,) or angular.shape != (3,):
        raise ValueError("invalid IMU vector shape")
    if not all(np.isfinite(vector).all() for vector in (quaternion, acceleration, angular)):
        raise ValueError("IMU fields must be finite")
    quaternion_norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(quaternion_norm) or not 0.99 <= quaternion_norm <= 1.01:
        raise ValueError(f"invalid IMU quaternion norm: {quaternion_norm}")
    return ImuFields(
        orientation_xyzw=np.array(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]], dtype=np.float64
        ),
        linear_acceleration=acceleration.copy(),
        angular_velocity=angular.copy(),
    )


def timestamp_msg(timestamp_ns: int):
    from builtin_interfaces.msg import Time

    seconds, nanoseconds = split_timestamp_ns(timestamp_ns)
    stamp = Time()
    stamp.sec = seconds
    stamp.nanosec = nanoseconds
    return stamp


def imu_msg(sample: ImuSample, frame_id: str):
    from sensor_msgs.msg import Imu

    fields = sample_to_fields(sample)
    msg = Imu()
    msg.header.stamp = timestamp_msg(sample.timestamp_ns)
    msg.header.frame_id = frame_id
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = fields.orientation_xyzw
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = fields.linear_acceleration
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = fields.angular_velocity
    return msg


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        raise SystemExit("rclpy is not importable. Source the ROS 2 environment first.") from exc

    rclpy.init()
    node = Node("isaac_imu_shared_memory_bridge")
    publisher = node.create_publisher(Imu, args.topic, qos_profile_sensor_data)
    reader = ImuReader()
    guard = TimestampGuard()
    published_once = False
    fatal_error: str | None = None
    last_log = 0.0

    def publish() -> None:
        nonlocal published_once, fatal_error, last_log
        sample = reader.read_sample()
        if sample is None:
            return
        try:
            guard.check(sample.timestamp_ns)
            msg = imu_msg(sample, args.frame_id)
        except (TimestampRegressionError, ValueError) as exc:
            fatal_error = str(exc)
            node.get_logger().fatal(fatal_error)
            timer.cancel()
            return
        publisher.publish(msg)
        published_once = True
        now = time.monotonic()
        if now - last_log >= 1.0:
            node.get_logger().info(
                f"published fresh IMU seq={sample.sequence} stamp_ns={sample.timestamp_ns} on {args.topic}"
            )
            last_log = now

    timer = node.create_timer(1.0 / args.poll_rate, publish)
    try:
        while rclpy.ok() and fatal_error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.once and published_once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        reader.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 2 if fatal_error is not None else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
