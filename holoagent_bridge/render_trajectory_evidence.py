#!/usr/bin/env python3
"""Render map/trajectory evidence from recorded real-run pose streams."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--transform", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def rotation_matrix(wxyz: list[float]) -> np.ndarray:
    w, x, y, z = wxyz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def main() -> int:
    args = parse_args()
    ground_truth = np.loadtxt(args.ground_truth)
    odometry = np.loadtxt(args.odometry)
    if ground_truth.ndim != 2 or odometry.ndim != 2:
        raise SystemExit("pose recordings are empty")

    transform = json.loads(args.transform.read_text(encoding="utf-8"))
    rotation = rotation_matrix(transform["rotation_wxyz"])
    translation = np.asarray(transform["translation"], dtype=np.float64)
    truth_map = (rotation @ ground_truth[:, 1:4].T).T + translation

    start = max(ground_truth[0, 0], odometry[0, 0])
    end = min(ground_truth[-1, 0], odometry[-1, 0])
    mask = (ground_truth[:, 0] >= start) & (ground_truth[:, 0] <= end)
    common_time = ground_truth[mask, 0]
    common_truth = truth_map[mask]
    common_odom = np.column_stack(
        [np.interp(common_time, odometry[:, 0], odometry[:, axis]) for axis in (1, 2, 3)]
    )
    error_xy = np.linalg.norm(common_truth[:, :2] - common_odom[:, :2], axis=1)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    map_config = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    occupancy = np.asarray(Image.open(args.map_yaml.parent / map_config["image"]))
    resolution = float(map_config["resolution"])
    origin_x, origin_y = map(float, map_config["origin"][:2])
    height, width = occupancy.shape
    extent = [
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    ]

    plt.figure(figsize=(8, 10))
    plt.imshow(np.flipud(occupancy), cmap="gray", origin="lower", extent=extent)
    plt.plot(truth_map[:, 0], truth_map[:, 1], "#00a878", linewidth=2, label="Isaac ground truth → map")
    plt.plot(odometry[:, 1], odometry[:, 2], "#e63946", linewidth=1.5, label="FAST-LIVO + NDT odometry")
    plt.scatter(truth_map[0, 0], truth_map[0, 1], c="#0077b6", marker="o", s=55, label="start")
    plt.scatter(truth_map[-1, 0], truth_map[-1, 1], c="#ffb703", marker="*", s=100, label="end")
    plt.xlabel("Map X (m)")
    plt.ylabel("Map Y (m)")
    plt.title("Recorded navigation trajectory on Nav2 occupancy map")
    plt.axis("equal")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(args.output_dir / "map_trajectory.png", dpi=180)
    plt.close()

    plt.figure(figsize=(9, 4))
    plt.plot(common_time - common_time[0], error_xy, color="#6a4c93")
    plt.xlabel("Simulation time since overlap start (s)")
    plt.ylabel("XY error (m)")
    plt.title("Localization error against transformed Isaac ground truth")
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(args.output_dir / "localization_error.png", dpi=180)
    plt.close()

    truth_displacement = np.linalg.norm(truth_map[-1, :2] - truth_map[0, :2])
    odom_displacement = np.linalg.norm(odometry[-1, 1:3] - odometry[0, 1:3])
    summary = {
        "ground_truth_samples": int(len(ground_truth)),
        "odometry_samples": int(len(odometry)),
        "overlap_samples": int(len(common_time)),
        "overlap_duration_sec": float(end - start),
        "ground_truth_xy_displacement_m": float(truth_displacement),
        "odometry_xy_displacement_m": float(odom_displacement),
        "xy_error_rmse_m": float(np.sqrt(np.mean(error_xy**2))),
        "xy_error_mean_m": float(np.mean(error_xy)),
        "xy_error_max_m": float(np.max(error_xy)),
        "transform_source": str(args.transform),
        "map_source": str(args.map_yaml),
    }
    (args.output_dir / "trajectory_metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
