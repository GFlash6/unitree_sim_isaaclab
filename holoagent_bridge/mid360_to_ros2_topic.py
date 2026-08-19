#!/usr/bin/env python3
"""Forward fresh Isaac MID360 records to the canonical ROS sensor topic."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.pointcloud_shared_memory_utils import PointCloudReader


class TimestampRegressionError(RuntimeError):
    pass


class TimestampGuard:
    def __init__(self) -> None:
        self.last_timestamp_ns = 0

    def check(self, timestamp_ns: int) -> None:
        if timestamp_ns <= self.last_timestamp_ns:
            raise TimestampRegressionError(
                f"MID360 timestamp did not increase: {timestamp_ns} <= {self.last_timestamp_ns}"
            )
        self.last_timestamp_ns = timestamp_ns


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward Isaac MID360 shared-memory points to ROS 2."
    )
    parser.add_argument(
        "--topic", default="sensors/lidar/points", help="PointCloud2 topic."
    )
    parser.add_argument("--frame-id", default="mid360_link")
    parser.add_argument("--rate", type=positive_float, default=10.0)
    parser.add_argument("--max-points", type=positive_int, default=20000)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args(argv)


def prepare_points(points: np.ndarray, max_points: int) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        return np.empty((0, 3), dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    return np.ascontiguousarray(points[:max_points], dtype=np.float32)


def cloud_msg(points: np.ndarray, frame_id: str, stamp):
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return point_cloud2.create_cloud_xyz32(header, points.tolist())


def timestamp_msg(timestamp_ns: int):
    from builtin_interfaces.msg import Time

    stamp = Time()
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000
    return stamp


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        raise SystemExit(
            "rclpy is not importable. Source the ROS 2 environment first."
        ) from exc

    rclpy.init()
    node = Node("mid360_shared_memory_bridge")
    publisher = node.create_publisher(PointCloud2, args.topic, 10)
    reader = PointCloudReader()
    published_once = False
    fatal_error: str | None = None
    timestamp_guard = TimestampGuard()
    last_log = 0.0

    def publish() -> None:
        nonlocal last_log, published_once, fatal_error
        raw = reader.read_points()
        if raw is None:
            return
        try:
            timestamp_guard.check(reader.last_timestamp_ns)
        except TimestampRegressionError as exc:
            fatal_error = str(exc)
            node.get_logger().fatal(fatal_error)
            timer.cancel()
            return
        points = prepare_points(raw, args.max_points)
        if points.size == 0:
            return
        publisher.publish(
            cloud_msg(points, args.frame_id, timestamp_msg(reader.last_timestamp_ns))
        )
        now = time.monotonic()
        if now - last_log > 1.0:
            node.get_logger().info(
                f"published seq={reader.last_sequence} stamp_ns={reader.last_timestamp_ns} "
                f"points={points.shape[0]} on {args.topic}"
            )
            last_log = now
        published_once = True

    timer = node.create_timer(1.0 / args.rate, publish)
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
