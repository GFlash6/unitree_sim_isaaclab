#!/usr/bin/env python3
"""Wait for fresh, advancing Isaac LiDAR, IMU, and synchronized RGB-D records."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from tools.imu_shared_memory_utils import ImuReader
from tools.pointcloud_shared_memory_utils import PointCloudReader
from tools.shared_memory_utils import MultiImageReader, get_shm_name


def advance_count(previous: int | None, current: int, count: int) -> tuple[int, int]:
    if previous is None or current <= previous:
        return current, 1
    return current, count + 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    lidar_reader = PointCloudReader()
    imu_reader = ImuReader()
    image_reader: MultiImageReader | None = None
    image_shm_paths = [
        Path("/dev/shm") / get_shm_name(name)
        for name in ("head", "head_depth", "head_pose")
    ]
    image_reader_opened_at = 0.0
    lidar_stamp = imu_stamp = rgbd_stamp = None
    lidar_count = imu_count = rgbd_count = 0
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            points = lidar_reader.read_points()
            if points is not None and len(points):
                lidar_stamp, lidar_count = advance_count(
                    lidar_stamp, lidar_reader.last_timestamp_ns, lidar_count
                )

            sample = imu_reader.read_sample()
            if sample is not None:
                imu_stamp, imu_count = advance_count(
                    imu_stamp, sample.timestamp_ns, imu_count
                )

            if image_reader is None and all(path.exists() for path in image_shm_paths):
                image_reader = MultiImageReader()
                image_reader_opened_at = time.monotonic()
            if image_reader is not None:
                images = [
                    image_reader.read_single_image(name)
                    for name in ("head", "head_depth", "head_pose")
                ]
                stamps = [
                    image_reader.last_timestamps.get(name, 0)
                    for name in ("head", "head_depth", "head_pose")
                ]
                if all(image is not None for image in images) and stamps[0] > 0 and len(set(stamps)) == 1:
                    if stamps[0] != rgbd_stamp:
                        rgbd_stamp, rgbd_count = advance_count(
                            rgbd_stamp, stamps[0], rgbd_count
                        )
                if rgbd_count < 2 and time.monotonic() - image_reader_opened_at > 5.0:
                    image_reader.close()
                    image_reader = None
                    rgbd_stamp, rgbd_count = None, 0

            if min(lidar_count, imu_count, rgbd_count) >= 2:
                print(
                    "Isaac real data ready: "
                    f"lidar_ns={lidar_stamp} imu_ns={imu_stamp} rgbd_ms={rgbd_stamp}"
                )
                return 0
            time.sleep(0.05)
    finally:
        lidar_reader.close()
        imu_reader.close()
        if image_reader is not None:
            image_reader.close()

    print(
        "Timed out waiting for advancing Isaac data: "
        f"lidar={lidar_count} imu={imu_count} rgbd={rgbd_count}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
