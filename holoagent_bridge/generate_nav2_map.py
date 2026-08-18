#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image

try:
    from holoagent_bridge.prepare_reloc_map import indexed_files, read_ascii_pcd, read_mapping, transform_point
except ModuleNotFoundError:
    from prepare_reloc_map import indexed_files, read_ascii_pcd, read_mapping, transform_point


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a Nav2 occupancy map from real pose-transformed FAST-LIVO keyframes.")
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=0.5)
    parser.add_argument(
        "--robot-radius",
        type=float,
        default=0.3,
        help="Clear this radius around every real keyframe pose; the robot physically traversed these cells.",
    )
    parser.add_argument("--min-obstacle-z", type=float, default=-0.8)
    parser.add_argument("--max-obstacle-z", type=float, default=0.3)
    return parser.parse_args()


def bresenham(x0: int, y0: int, x1: int, y1: int):
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def build_grid(
    map_dir: Path,
    resolution: float,
    padding: float,
    min_z: float,
    max_z: float,
    robot_radius: float = 0.3,
):
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be finite and positive")
    if not math.isfinite(robot_radius) or robot_radius < 0:
        raise ValueError("robot_radius must be finite and non-negative")
    poses = read_mapping(map_dir / "mapping.txt")
    clouds = indexed_files(map_dir / "keyframe_cloud", ".pcd")
    if len(poses) != len(clouds):
        raise ValueError(f"keyframe/pose count mismatch: poses={len(poses)} clouds={len(clouds)}")
    frames = []
    xs = [pose.x for pose in poses]
    ys = [pose.y for pose in poses]
    for path, pose in zip(clouds, poses, strict=True):
        _fields, points = read_ascii_pcd(path)
        transformed = [transform_point(point, pose) for point in points]
        frames.append((pose, transformed))
        xs.extend(point[0] for point in transformed)
        ys.extend(point[1] for point in transformed)
    if not frames or not xs:
        raise ValueError("map contains no real keyframe points")
    origin_x, origin_y = min(xs) - padding, min(ys) - padding
    width = math.ceil((max(xs) + padding - origin_x) / resolution) + 1
    height = math.ceil((max(ys) + padding - origin_y) / resolution) + 1
    cells = bytearray([205]) * (width * height)

    def cell(x: float, y: float) -> tuple[int, int]:
        return int((x - origin_x) / resolution), int((y - origin_y) / resolution)

    occupied: set[int] = set()
    for pose, points in frames:
        start_x, start_y = cell(pose.x, pose.y)
        for point in points:
            end_x, end_y = cell(point[0], point[1])
            ray = list(bresenham(start_x, start_y, end_x, end_y))
            for x, y in ray[:-1]:
                index = y * width + x
                if index not in occupied:
                    cells[index] = 254
            endpoint = end_y * width + end_x
            if min_z <= point[2] <= max_z:
                occupied.add(endpoint)
                cells[endpoint] = 0
            elif endpoint not in occupied:
                cells[endpoint] = 254

    # A mapped keyframe is direct free-space evidence: the physical robot occupied
    # that pose. Clear its footprint after all rays so a distant endpoint from a
    # different frame cannot make the recorded trajectory impassable.
    clear_radius = math.ceil(robot_radius / resolution)
    for pose in poses:
        center_x, center_y = cell(pose.x, pose.y)
        for dy in range(-clear_radius, clear_radius + 1):
            for dx in range(-clear_radius, clear_radius + 1):
                if (dx * resolution) ** 2 + (dy * resolution) ** 2 > robot_radius**2:
                    continue
                x, y = center_x + dx, center_y + dy
                if not (0 <= x < width and 0 <= y < height):
                    continue
                index = y * width + x
                occupied.discard(index)
                cells[index] = 254
    return cells, width, height, origin_x, origin_y, len(occupied)


def main() -> int:
    args = parse_args()
    map_dir = args.map_dir.resolve()
    cells, width, height, origin_x, origin_y, occupied = build_grid(
        map_dir,
        args.resolution,
        args.padding,
        args.min_obstacle_z,
        args.max_obstacle_z,
        args.robot_radius,
    )
    rows = [cells[y * width : (y + 1) * width] for y in range(height - 1, -1, -1)]
    image_path = map_dir / "grid_map.pgm"
    Image.frombytes("L", (width, height), b"".join(rows)).save(image_path)
    (map_dir / "grid_map.yaml").write_text(
        f"image: grid_map.pgm\nmode: trinary\nresolution: {args.resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    print(f"saved {image_path} and {map_dir / 'grid_map.yaml'}")
    print(f"size={width}x{height} occupied_cells={occupied} origin=({origin_x:.3f},{origin_y:.3f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
