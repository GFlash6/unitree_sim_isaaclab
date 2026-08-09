#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.pointcloud_shared_memory_utils import PointCloudReader


def cloud_path(output_dir: Path, frame_index: int) -> Path:
    return output_dir / f"mid360_{frame_index:06d}.npy"


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
    parser = argparse.ArgumentParser(description="Read MID360 point clouds from Unitree sim shared memory.")
    parser.add_argument("--output-dir", type=Path, default=Path("mid360_clouds"), help="Point cloud output directory.")
    parser.add_argument("--interval", type=positive_float, default=0.2, help="Read interval in seconds.")
    parser.add_argument("--duration", type=positive_float, default=None, help="Run seconds; omit for unlimited.")
    parser.add_argument("--once", action="store_true", help="Save one point cloud and exit.")
    parser.add_argument("--timeout", type=positive_float, default=10.0, help="Wait seconds for --once.")
    parser.add_argument("--max-points", type=positive_int, default=20000, help="Print/save at most this many points.")
    parser.add_argument("--no-save", action="store_true", help="Only print point cloud shape.")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    reader = PointCloudReader()
    start = time.monotonic()
    frame_index = 0
    try:
        while True:
            points = reader.read_points()
            if points is not None:
                points = points[: args.max_points]
                print(f"frame={frame_index} mid360:{tuple(points.shape)}", flush=True)
                if not args.no_save:
                    path = cloud_path(args.output_dir, frame_index)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    import numpy as np

                    np.save(path, points)
                frame_index += 1
                if args.once:
                    return 0

            elapsed = time.monotonic() - start
            if args.duration is not None and elapsed >= args.duration:
                return 0
            if args.once and elapsed >= args.timeout:
                print("mid360 point cloud not received before timeout", file=sys.stderr)
                return 1
            time.sleep(args.interval)
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
