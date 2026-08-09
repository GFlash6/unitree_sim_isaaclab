#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import NamedTuple


REQUIRED = [
    "singlesession_posegraph.g2o",
    "cloudGlobal.pcd",
    "keyframe_cloud",
    "keyframe_scancontext",
]


class Pose(NamedTuple):
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare a HoloAgent online_relo map from a real FAST-LIVO map output.")
    parser.add_argument("map_dir", type=Path)
    parser.add_argument(
        "--rebuild-global",
        action="store_true",
        help="Rebuild cloudGlobal from pose-transformed keyframe clouds.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_dir = args.map_dir.resolve()
    mapping_path = map_dir / "mapping.txt"
    keyframe_pose_path = map_dir / "keyframe_pose.txt"
    if not mapping_path.is_file():
        raise SystemExit(f"missing real mapping trajectory: {mapping_path}")

    try:
        poses = read_mapping(mapping_path)
        keyframe_clouds = indexed_files(map_dir / "keyframe_cloud", ".pcd")
        scancontexts = indexed_files(map_dir / "keyframe_scancontext", ".scd")
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if len(keyframe_clouds) != len(poses) or len(scancontexts) != len(poses):
        raise SystemExit(
            "map cardinality mismatch: "
            f"poses={len(poses)} clouds={len(keyframe_clouds)} "
            f"scancontexts={len(scancontexts)}"
        )

    rows = [
        f"{idx} {idx} {pose.x} {pose.y} {pose.z} "
        f"{pose.qw} {pose.qx} {pose.qy} {pose.qz}"
        for idx, pose in enumerate(poses)
    ]

    if args.rebuild_global or not (map_dir / "cloudGlobal.pcd").is_file():
        points = []
        fields = None
        for cloud_path, pose in zip(keyframe_clouds, poses, strict=True):
            cloud_fields, cloud_points = read_ascii_pcd(cloud_path)
            if fields is not None and cloud_fields != fields:
                raise SystemExit(f"inconsistent PCD fields in {cloud_path}")
            fields = cloud_fields
            points.extend(transform_point(point, pose) for point in cloud_points)
        if not points:
            raise SystemExit(f"keyframe clouds contain no real points: {map_dir / 'keyframe_cloud'}")
        write_ascii_pcd(map_dir / "cloudGlobal.pcd", fields or ["x", "y", "z", "intensity"], points)
        write_ascii_ply(map_dir / "cloudGlobal.ply", points)

    keyframe_pose_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    missing = [name for name in REQUIRED if not (map_dir / name).exists()]
    if missing:
        raise SystemExit(f"map is incomplete after keyframe_pose export; missing: {', '.join(missing)}")

    print(f"prepared {keyframe_pose_path}")
    print(f"keyframes={len(rows)} keyframe_clouds={len(keyframe_clouds)} scancontexts={len(scancontexts)}")
    return 0


def indexed_files(directory: Path, extension: str) -> list[Path]:
    if not directory.is_dir():
        raise ValueError(f"missing directory: {directory}")
    files = sorted(directory.glob(f"*{extension}"))
    if not files:
        raise ValueError(f"no {extension} files in {directory}")
    try:
        indexes = [int(path.stem) for path in files]
    except ValueError as exc:
        raise ValueError(f"non-numeric keyframe filename in {directory}") from exc
    if indexes != list(range(len(files))):
        raise ValueError(
            f"keyframe indexes must be contiguous from zero in {directory}: {indexes}"
        )
    return files


def read_mapping(path: Path) -> list[Pose]:
    poses: list[Pose] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError(f"invalid mapping row {line_number} in {path}")
        try:
            index = int(parts[0])
            values = [float(value) for value in parts[1:]]
        except ValueError as exc:
            raise ValueError(f"non-numeric mapping row {line_number} in {path}") from exc
        if index != len(poses) or not all(math.isfinite(value) for value in values):
            raise ValueError(f"invalid mapping index or non-finite pose at row {line_number}")
        pose = Pose(*values)
        norm = math.sqrt(pose.qx**2 + pose.qy**2 + pose.qz**2 + pose.qw**2)
        if not 0.99 <= norm <= 1.01:
            raise ValueError(f"non-unit quaternion at mapping row {line_number}: norm={norm}")
        poses.append(pose)
    if not poses:
        raise ValueError(f"no keyframe poses found in {path}")
    return poses


def transform_point(point: list[float], pose: Pose) -> list[float]:
    if len(point) < 3 or not all(math.isfinite(value) for value in point):
        raise ValueError("keyframe PCD contains an invalid point")
    px, py, pz = point[:3]
    qx, qy, qz, qw = pose.qx, pose.qy, pose.qz, pose.qw
    # Quaternion-vector rotation expanded as R(q) * p.
    rx = (1 - 2 * (qy * qy + qz * qz)) * px + 2 * (qx * qy - qz * qw) * py + 2 * (qx * qz + qy * qw) * pz
    ry = 2 * (qx * qy + qz * qw) * px + (1 - 2 * (qx * qx + qz * qz)) * py + 2 * (qy * qz - qx * qw) * pz
    rz = 2 * (qx * qz - qy * qw) * px + 2 * (qy * qz + qx * qw) * py + (1 - 2 * (qx * qx + qy * qy)) * pz
    return [rx + pose.x, ry + pose.y, rz + pose.z, *point[3:]]


def read_ascii_pcd(path: Path) -> tuple[list[str], list[list[float]]]:
    fields: list[str] = []
    points: list[list[float]] = []
    data = False
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        lower = stripped.lower()
        if not data:
            if lower.startswith("fields "):
                fields = stripped.split()[1:]
            if lower.startswith("data"):
                data = True
            continue
        parts = stripped.split()
        if len(parts) < 3:
            continue
        try:
            points.append([float(value) for value in parts])
        except ValueError:
            continue
    return fields, points


def write_ascii_pcd(path: Path, fields: list[str], points: list[list[float]]) -> None:
    field_count = len(fields)
    normalized = [point[:field_count] for point in points if len(point) >= field_count]
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS " + " ".join(fields),
                "SIZE " + " ".join(["4"] * field_count),
                "TYPE " + " ".join(["F"] * field_count),
                "COUNT " + " ".join(["1"] * field_count),
                f"WIDTH {len(normalized)}",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                f"POINTS {len(normalized)}",
                "DATA ascii",
                *(" ".join(f"{value:.6f}" for value in point) for point in normalized),
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def write_ascii_ply(path: Path, points: list[list[float]]) -> None:
    xyz = [point[:3] for point in points if len(point) >= 3]
    path.write_text(
        "\n".join(
            [
                "ply",
                "format ascii 1.0",
                f"element vertex {len(xyz)}",
                "property float x",
                "property float y",
                "property float z",
                "end_header",
                *(" ".join(f"{value:.6f}" for value in point) for point in xyz),
            ]
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    raise SystemExit(main())
