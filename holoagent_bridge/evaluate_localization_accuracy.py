#!/usr/bin/env python3
"""Compare localization output with independent IsaacLab ground truth."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np


class Trajectory(NamedTuple):
    timestamps_s: np.ndarray
    positions: np.ndarray
    yaws: np.ndarray


class AccuracyReport(NamedTuple):
    ok: bool
    errors: tuple[str, ...]
    sample_count: int
    start_time_s: float
    end_time_s: float
    translation_rmse_m: float
    final_translation_error_m: float
    yaw_rmse_rad: float
    final_yaw_error_rad: float
    max_error_jump_m: float
    alignment_yaw_rad: float
    alignment_translation_xyz: tuple[float, float, float]


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate FAST-LIVO/relocalization against IsaacLab ground truth.")
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("estimate", type=Path, help="Eight-column timestamp/position/quaternion trajectory.")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--max-translation-rmse", type=positive_float, default=0.20)
    parser.add_argument("--max-final-translation-error", type=positive_float, default=0.30)
    parser.add_argument("--max-yaw-rmse", type=positive_float, default=0.15)
    parser.add_argument("--max-final-yaw-error", type=positive_float, default=0.20)
    parser.add_argument("--max-error-jump", type=positive_float, default=0.25)
    return parser.parse_args(argv)


def wrap_angle(values):
    return (np.asarray(values) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_xyzw_to_yaw(quaternion: np.ndarray) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_trajectory(path: str | Path) -> Trajectory:
    rows: list[list[float]] = []
    for line_number, raw in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 fields, got {len(fields)}")
        row = [float(value) for value in fields]
        if not np.isfinite(row).all():
            raise ValueError(f"{path}:{line_number}: values must be finite")
        quaternion = np.asarray(row[4:8], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if not 0.99 <= norm <= 1.01:
            raise ValueError(f"{path}:{line_number}: invalid quaternion norm {norm}")
        rows.append(row)
    if len(rows) < 2:
        raise ValueError(f"{path}: trajectory requires at least two poses")
    data = np.asarray(rows, dtype=np.float64)
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    yaws = np.asarray([quaternion_xyzw_to_yaw(row[4:8]) for row in data], dtype=np.float64)
    return Trajectory(data[:, 0], data[:, 1:4], yaws)


def interpolate_trajectory(source: Trajectory, query_timestamps_s: np.ndarray) -> Trajectory:
    query = np.asarray(query_timestamps_s, dtype=np.float64)
    if query.ndim != 1 or query.size == 0:
        raise ValueError("query timestamps must be a non-empty vector")
    if query[0] < source.timestamps_s[0] or query[-1] > source.timestamps_s[-1]:
        raise ValueError("query timestamps fall outside the source trajectory")
    positions = np.column_stack(
        [np.interp(query, source.timestamps_s, source.positions[:, axis]) for axis in range(3)]
    )
    unwrapped_yaw = np.unwrap(source.yaws)
    yaws = wrap_angle(np.interp(query, source.timestamps_s, unwrapped_yaw))
    return Trajectory(query, positions, yaws)


def evaluate(
    ground_truth: Trajectory,
    estimate: Trajectory,
    *,
    max_translation_rmse_m: float = 0.20,
    max_final_translation_error_m: float = 0.30,
    max_yaw_rmse_rad: float = 0.15,
    max_final_yaw_error_rad: float = 0.20,
    max_error_jump_m: float = 0.25,
) -> AccuracyReport:
    in_range = (estimate.timestamps_s >= ground_truth.timestamps_s[0]) & (
        estimate.timestamps_s <= ground_truth.timestamps_s[-1]
    )
    timestamps = estimate.timestamps_s[in_range]
    positions = estimate.positions[in_range]
    yaws = estimate.yaws[in_range]
    if timestamps.size < 2:
        raise ValueError("fewer than two estimate poses overlap ground truth")
    reference = interpolate_trajectory(ground_truth, timestamps)

    alignment_yaw = float(wrap_angle(yaws[0] - reference.yaws[0]))
    cosine = math.cos(alignment_yaw)
    sine = math.sin(alignment_yaw)
    rotation_xy = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    aligned_positions = reference.positions.copy()
    aligned_positions[:, :2] = reference.positions[:, :2] @ rotation_xy.T
    translation = positions[0] - aligned_positions[0]
    aligned_positions += translation
    aligned_yaws = wrap_angle(reference.yaws + alignment_yaw)

    error_vectors = positions - aligned_positions
    translation_errors = np.linalg.norm(error_vectors, axis=1)
    yaw_errors = np.abs(wrap_angle(yaws - aligned_yaws))
    error_jumps = np.linalg.norm(np.diff(error_vectors, axis=0), axis=1)
    translation_rmse = float(np.sqrt(np.mean(translation_errors**2)))
    final_translation = float(translation_errors[-1])
    yaw_rmse = float(np.sqrt(np.mean(yaw_errors**2)))
    final_yaw = float(yaw_errors[-1])
    maximum_jump = float(np.max(error_jumps)) if error_jumps.size else 0.0

    errors: list[str] = []
    if translation_rmse > max_translation_rmse_m:
        errors.append(f"translation RMSE {translation_rmse:.4f} m exceeds {max_translation_rmse_m:.4f} m")
    if final_translation > max_final_translation_error_m:
        errors.append(
            f"final translation error {final_translation:.4f} m exceeds {max_final_translation_error_m:.4f} m"
        )
    if yaw_rmse > max_yaw_rmse_rad:
        errors.append(f"yaw RMSE {yaw_rmse:.4f} rad exceeds {max_yaw_rmse_rad:.4f} rad")
    if final_yaw > max_final_yaw_error_rad:
        errors.append(f"final yaw error {final_yaw:.4f} rad exceeds {max_final_yaw_error_rad:.4f} rad")
    if maximum_jump > max_error_jump_m:
        errors.append(f"localization error jump {maximum_jump:.4f} m exceeds {max_error_jump_m:.4f} m")

    return AccuracyReport(
        ok=not errors,
        errors=tuple(errors),
        sample_count=int(timestamps.size),
        start_time_s=float(timestamps[0]),
        end_time_s=float(timestamps[-1]),
        translation_rmse_m=translation_rmse,
        final_translation_error_m=final_translation,
        yaw_rmse_rad=yaw_rmse,
        final_yaw_error_rad=final_yaw,
        max_error_jump_m=maximum_jump,
        alignment_yaw_rad=alignment_yaw,
        alignment_translation_xyz=tuple(float(value) for value in translation),
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    report = evaluate(
        read_trajectory(args.ground_truth),
        read_trajectory(args.estimate),
        max_translation_rmse_m=args.max_translation_rmse,
        max_final_translation_error_m=args.max_final_translation_error,
        max_yaw_rmse_rad=args.max_yaw_rmse,
        max_final_yaw_error_rad=args.max_final_yaw_error,
        max_error_jump_m=args.max_error_jump,
    )
    payload = report._asdict()
    output = json.dumps(payload, indent=2)
    print(output)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(output + "\n", encoding="utf-8")
    return 0 if report.ok else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
