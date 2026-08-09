#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate a live, real HoloAgent relocalization stream.")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--min-poses", type=int, default=10)
    parser.add_argument("--min-clouds", type=int, default=10)
    parser.add_argument("--max-pose-jump", type=float, default=0.75)
    parser.add_argument("--max-score", type=float, default=0.35)
    return parser.parse_args()


def main() -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, Float64

    args = parse_args()
    rclpy.init()
    node = Node("validate_real_relocalization")
    poses: list[tuple[float, float, float, float]] = []
    cloud_count = 0
    scores: list[float] = []
    successes = 0
    errors: list[str] = []

    def pose_cb(message: Odometry) -> None:
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        quaternion = message.pose.pose.orientation
        qnorm = math.sqrt(
            quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2
        )
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(value) for value in (*values, qnorm, stamp)):
            errors.append("non-finite pose")
            return
        if abs(qnorm - 1.0) > 1e-3:
            errors.append(f"non-unit pose quaternion: {qnorm}")
        if poses and stamp <= poses[-1][0]:
            errors.append(f"non-increasing pose timestamp: {stamp} <= {poses[-1][0]}")
        if poses:
            jump = math.dist(values, poses[-1][1:])
            if jump > args.max_pose_jump:
                errors.append(f"pose jump {jump:.3f} m exceeds {args.max_pose_jump:.3f} m")
        poses.append((stamp, *values))

    def cloud_cb(message: PointCloud2) -> None:
        nonlocal cloud_count
        fields = {field.name for field in message.fields}
        if not {"x", "y", "z"}.issubset(fields) or message.width * message.height == 0:
            errors.append("invalid or empty reloc_body_cloud")
            return
        cloud_count += 1

    def score_cb(message: Float64) -> None:
        if not math.isfinite(message.data):
            errors.append("non-finite registration score")
        scores.append(message.data)

    def success_cb(message: Bool) -> None:
        nonlocal successes
        successes += int(message.data)

    node.create_subscription(Odometry, "/pose", pose_cb, 10)
    node.create_subscription(PointCloud2, "/reloc_body_cloud", cloud_cb, 10)
    node.create_subscription(Float64, "/relocalization/fitness_score", score_cb, 10)
    node.create_subscription(Bool, "/relocalization/registration_success", success_cb, 10)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    valid_scores = [score for score in scores if score <= args.max_score]
    if len(poses) < args.min_poses:
        errors.append(f"only {len(poses)} poses, expected at least {args.min_poses}")
    if cloud_count < args.min_clouds:
        errors.append(f"only {cloud_count} body clouds, expected at least {args.min_clouds}")
    if successes < 1 or not valid_scores:
        errors.append("no successful threshold-valid registration was observed")
    print(
        f"poses={len(poses)} clouds={cloud_count} attempts={len(scores)} "
        f"successes={successes} best_score={min(scores, default=math.inf):.6f}"
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
