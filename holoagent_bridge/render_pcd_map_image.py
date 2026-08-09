#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw

try:
    from holoagent_bridge.prepare_reloc_map import (
        indexed_files,
        read_ascii_pcd,
        read_mapping as read_mapping_poses,
        transform_point,
    )
except ModuleNotFoundError:
    from prepare_reloc_map import (  # type: ignore[no-redef]
        indexed_files,
        read_ascii_pcd,
        read_mapping as read_mapping_poses,
        transform_point,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render an ASCII PCD map and optional FAST-LIVO trajectory to a PNG.")
    parser.add_argument("map_dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--size", type=int, default=1600)
    parser.add_argument("--padding", type=int, default=80)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_dir = args.map_dir.resolve()
    output = args.output or (map_dir / "map_topdown.png")
    cloud = read_keyframe_clouds(map_dir)
    if not cloud:
        cloud = read_pcd_xyz(map_dir / "cloudGlobal.pcd")
    poses = read_trajectory(map_dir / "mapping.txt")
    if not cloud:
        raise SystemExit(f"no points found in {map_dir / 'cloudGlobal.pcd'}")

    xs = [p[0] for p in cloud] + [p[0] for p in poses]
    ys = [p[1] for p in cloud] + [p[1] for p in poses]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    draw_size = max(1, args.size - args.padding * 2)
    scale = draw_size / max(span_x, span_y)

    def project(x: float, y: float) -> tuple[int, int]:
        px = args.padding + int((x - min_x) * scale)
        py = args.size - args.padding - int((y - min_y) * scale)
        return px, py

    image = Image.new("RGB", (args.size, args.size), (250, 250, 247))
    draw = ImageDraw.Draw(image)
    for x, y, z in cloud:
        color = height_color(z)
        px, py = project(x, y)
        draw.point((px, py), fill=color)

    if len(poses) >= 2:
        path = [project(x, y) for x, y, _z in poses]
        draw.line(path, fill=(210, 60, 48), width=5)
        r = 7
        for px, py in path:
            draw.ellipse((px - r, py - r, px + r, py + r), fill=(210, 60, 48))

    draw.rectangle((0, 0, args.size - 1, args.size - 1), outline=(40, 40, 40), width=2)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)
    print(f"saved {output}")
    print(f"points={len(cloud)} poses={len(poses)} bounds=({min_x:.3f},{min_y:.3f})..({max_x:.3f},{max_y:.3f})")
    return 0


def read_pcd_xyz(path: Path) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if not path.is_file():
        return points
    data = False
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not data:
            if line.strip().lower().startswith("data"):
                data = True
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
        except ValueError:
            continue
    return points


def read_trajectory(path: Path) -> list[tuple[float, float, float]]:
    poses: list[tuple[float, float, float]] = []
    if not path.is_file():
        return poses
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            poses.append((float(parts[1]), float(parts[2]), float(parts[3])))
        except ValueError:
            continue
    return poses


def read_keyframe_clouds(map_dir: Path) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    keyframe_dir = map_dir / "keyframe_cloud"
    if not keyframe_dir.is_dir():
        return points
    cloud_paths = indexed_files(keyframe_dir, ".pcd")
    poses = read_mapping_poses(map_dir / "mapping.txt")
    if len(cloud_paths) != len(poses):
        raise ValueError(
            f"keyframe/pose count mismatch: clouds={len(cloud_paths)} poses={len(poses)}"
        )
    for path, pose in zip(cloud_paths, poses, strict=True):
        _fields, cloud = read_ascii_pcd(path)
        for point in cloud:
            transformed = transform_point(point, pose)
            points.append((transformed[0], transformed[1], transformed[2]))
    return points


def height_color(z: float) -> tuple[int, int, int]:
    if z < -0.5:
        return (68, 92, 130)
    if z < 0.2:
        return (72, 126, 132)
    if z < 0.8:
        return (82, 150, 99)
    return (160, 115, 65)


if __name__ == "__main__":
    raise SystemExit(main())
