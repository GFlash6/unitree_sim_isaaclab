#!/usr/bin/env python3
"""Record timestamped ROS 2 odometry in the localization evaluator format."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


class TimestampRegressionError(RuntimeError):
    pass


class TimestampGuard:
    def __init__(self) -> None:
        self.last_timestamp_ns = 0

    def check(self, timestamp_ns: int) -> bool:
        if timestamp_ns < self.last_timestamp_ns:
            raise TimestampRegressionError(
                f"odometry timestamp regressed: {timestamp_ns} < {self.last_timestamp_ns}"
            )
        if timestamp_ns == self.last_timestamp_ns:
            return False
        self.last_timestamp_ns = timestamp_ns
        return True


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record nav_msgs/Odometry without changing source timestamps.")
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/aft_mapped_to_init")
    parser.add_argument("--duration", type=nonnegative_float, default=0.0, help="Wall seconds; zero records until interrupted.")
    return parser.parse_args(argv)


def format_pose(timestamp_ns: int, position, quaternion_xyzw) -> str:
    timestamp_s = timestamp_ns * 1e-9
    return (
        f"{timestamp_s:.9f} "
        f"{float(position[0]):.9f} {float(position[1]):.9f} {float(position[2]):.9f} "
        f"{float(quaternion_xyzw[0]):.9f} {float(quaternion_xyzw[1]):.9f} "
        f"{float(quaternion_xyzw[2]):.9f} {float(quaternion_xyzw[3]):.9f}"
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except ImportError as exc:
        raise SystemExit("ROS 2 Python packages are unavailable. Source ROS 2 first.") from exc

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("odometry_trajectory_recorder")
    guard = TimestampGuard()
    count = 0
    fatal_error: str | None = None
    started = time.monotonic()
    output = args.output.open("w", encoding="utf-8", buffering=1)

    def callback(message: Odometry) -> None:
        nonlocal count, fatal_error
        timestamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if not all(math.isfinite(value) for value in values) or not 0.99 <= norm <= 1.01:
            fatal_error = "odometry contains invalid pose values"
            return
        try:
            is_fresh_time = guard.check(timestamp_ns)
        except TimestampRegressionError as exc:
            fatal_error = str(exc)
            return
        if not is_fresh_time:
            return
        output.write(
            format_pose(timestamp_ns, values[:3], values[3:]) + "\n"
        )
        count += 1

    node.create_subscription(Odometry, args.topic, callback, 100)
    try:
        while (
            rclpy.ok()
            and fatal_error is None
            and (args.duration == 0.0 or time.monotonic() - started < args.duration)
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        output.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fatal_error:
        print(f"ERROR: {fatal_error}", file=sys.stderr, flush=True)
        return 2
    print(f"recorded_odometry_poses={count} topic={args.topic} output={args.output}", flush=True)
    return 0 if count >= 2 else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
