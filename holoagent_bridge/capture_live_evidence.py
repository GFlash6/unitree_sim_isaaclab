#!/usr/bin/env python3
"""Capture one synchronized-enough set of live ROS sensor evidence.

The command fails unless RGB, depth, lidar, and localization messages all arrive
from the running graph.  It never fabricates missing inputs.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--timeout", type=float, default=20.0)
    return parser.parse_args()


def stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def decode_rgb(message: Image) -> np.ndarray:
    channels = 4 if message.encoding in {"rgba8", "bgra8"} else 3
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = raw[:, : message.width * channels].reshape(
        message.height, message.width, channels
    )
    if message.encoding in {"bgr8", "bgra8"}:
        image = image[..., [2, 1, 0, 3] if channels == 4 else [2, 1, 0]]
    return image[..., :3].copy()


def decode_depth_m(message: Image) -> np.ndarray:
    if message.encoding == "32FC1":
        dtype, scale = np.dtype("<f4"), 1.0
    elif message.encoding in {"16UC1", "mono16"}:
        dtype, scale = np.dtype("<u2"), 0.001
    else:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    if message.is_bigendian:
        dtype = dtype.newbyteorder(">")
    columns = message.step // dtype.itemsize
    raw = np.frombuffer(message.data, dtype=dtype).reshape(message.height, columns)
    return raw[:, : message.width].astype(np.float32) * scale


class LiveEvidence(Node):
    def __init__(self) -> None:
        super().__init__("capture_live_evidence")
        self.messages = {}
        self.create_subscription(
            Image, "/sensors/front/rgb", lambda msg: self._store("rgb", msg), SENSOR_QOS
        )
        self.create_subscription(
            Image,
            "/sensors/front/depth",
            lambda msg: self._store("depth", msg),
            SENSOR_QOS,
        )
        self.create_subscription(
            PointCloud2,
            "/sensors/lidar/points",
            lambda msg: self._store("lidar", msg),
            SENSOR_QOS,
        )
        self.create_subscription(
            Odometry,
            "/localization/odom",
            lambda msg: self._store("odom", msg),
            10,
        )

    def _store(self, name: str, message) -> None:
        if name not in self.messages:
            self.messages[name] = message


def save_capture(output_dir: Path, messages: dict) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    rgb = decode_rgb(messages["rgb"])
    depth = decode_depth_m(messages["depth"])
    points = point_cloud2.read_points_numpy(
        messages["lidar"], field_names=["x", "y", "z"], skip_nans=True
    ).astype(np.float32)

    PilImage.fromarray(rgb).save(output_dir / "live_rgb.png")
    depth_mm = np.clip(np.nan_to_num(depth) * 1000.0, 0, 65535).astype(np.uint16)
    PilImage.fromarray(depth_mm, mode="I;16").save(output_dir / "live_depth_mm.png")
    np.savez_compressed(output_dir / "live_lidar.npz", xyz=points)

    finite_depth = depth[np.isfinite(depth) & (depth > 0)]
    lower, upper = np.percentile(finite_depth, [1, 99])
    plt.figure(figsize=(8, 6))
    plt.imshow(depth, cmap="turbo", vmin=lower, vmax=upper)
    plt.colorbar(label="Depth (m)")
    plt.title("Live Isaac RGB-D depth")
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(output_dir / "live_depth_color.png", dpi=160)
    plt.close()

    stride = max(1, len(points) // 25000)
    shown = points[::stride]
    plt.figure(figsize=(7, 7))
    plt.scatter(shown[:, 0], shown[:, 1], c=shown[:, 2], s=0.7, cmap="viridis")
    plt.colorbar(label="Z (m)")
    plt.axis("equal")
    plt.xlabel("Lidar X (m)")
    plt.ylabel("Lidar Y (m)")
    plt.title(f"Live MID360 cloud ({len(points):,} points)")
    plt.tight_layout()
    plt.savefig(output_dir / "live_lidar_topdown.png", dpi=180)
    plt.close()

    odom = messages["odom"].pose.pose
    stamps = {name: stamp_seconds(message) for name, message in messages.items()}
    evidence = {
        "capture_wall_time": time.time(),
        "source_topics": {
            "rgb": "/sensors/front/rgb",
            "depth": "/sensors/front/depth",
            "lidar": "/sensors/lidar/points",
            "odom": "/localization/odom",
        },
        "source_stamps_sec": stamps,
        "stamp_span_sec": max(stamps.values()) - min(stamps.values()),
        "rgb_shape": list(rgb.shape),
        "depth_shape": list(depth.shape),
        "depth_valid_count": int(finite_depth.size),
        "depth_range_m": [float(finite_depth.min()), float(finite_depth.max())],
        "lidar_point_count": int(len(points)),
        "lidar_frame": messages["lidar"].header.frame_id,
        "localization_pose_map": [
            odom.position.x,
            odom.position.y,
            odom.position.z,
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w,
        ],
    }
    (output_dir / "live_capture.json").write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return evidence


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    rclpy.init()
    node = LiveEvidence()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and len(node.messages) < 4 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = sorted({"rgb", "depth", "lidar", "odom"} - node.messages.keys())
        if missing:
            print(f"ERROR: live topics missing before timeout: {missing}")
            return 2
        print(json.dumps(save_capture(args.output_dir, node.messages), indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
