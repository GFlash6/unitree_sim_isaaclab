from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "holoagent_bridge" / "evaluate_localization_accuracy.py"
SPEC = importlib.util.spec_from_file_location("evaluate_localization_accuracy", SCRIPT)
assert SPEC and SPEC.loader
evaluator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluator)


def trajectory(times, positions, yaws):
    return evaluator.Trajectory(
        np.asarray(times, dtype=float),
        np.asarray(positions, dtype=float),
        np.asarray(yaws, dtype=float),
    )


def test_interpolation_handles_yaw_wrap() -> None:
    source = trajectory(
        [0, 1, 2],
        [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
        np.deg2rad([179, -179, -177]),
    )
    result = evaluator.interpolate_trajectory(source, np.array([0.5, 1.5]))
    np.testing.assert_allclose(result.positions[:, 0], [0.5, 1.5])
    assert abs(abs(result.yaws[0]) - math.pi) < 1e-9
    assert abs(result.yaws[1] - math.radians(-178)) < 1e-9


def test_initial_rigid_alignment_does_not_count_map_origin_offset() -> None:
    ground_truth = trajectory([0, 1, 2], [[0, 0, 0], [1, 0, 0], [2, 0, 0]], [0, 0, 0])
    estimate = trajectory(
        [0, 1, 2],
        [[5, -2, 1], [5, -1, 1], [5, 0, 1]],
        [math.pi / 2] * 3,
    )
    report = evaluator.evaluate(ground_truth, estimate)
    assert report.ok
    assert report.translation_rmse_m < 1e-12
    assert report.yaw_rmse_rad < 1e-12


def test_metrics_measure_drift_after_initial_alignment() -> None:
    ground_truth = trajectory([0, 1, 2], [[0, 0, 0], [1, 0, 0], [2, 0, 0]], [0, 0, 0])
    estimate = trajectory([0, 1, 2], [[0, 0, 0], [1.1, 0, 0], [2.2, 0, 0]], [0, 0.05, 0.1])
    report = evaluator.evaluate(ground_truth, estimate)
    assert abs(report.translation_rmse_m - math.sqrt((0.1**2 + 0.2**2) / 3)) < 1e-12
    assert abs(report.final_translation_error_m - 0.2) < 1e-12
    assert abs(report.final_yaw_error_rad - 0.1) < 1e-12
    assert abs(report.max_error_jump_m - 0.1) < 1e-12


def test_threshold_and_jump_failures_are_reported() -> None:
    ground_truth = trajectory([0, 1, 2], [[0, 0, 0], [0.1, 0, 0], [0.2, 0, 0]], [0, 0, 0])
    estimate = trajectory([0, 1, 2], [[0, 0, 0], [1.0, 0, 0], [1.1, 0, 0]], [0, 0.5, 0.5])
    report = evaluator.evaluate(
        ground_truth,
        estimate,
        max_translation_rmse_m=0.2,
        max_final_translation_error_m=0.2,
        max_yaw_rmse_rad=0.2,
        max_final_yaw_error_rad=0.2,
        max_error_jump_m=0.5,
    )
    assert not report.ok
    assert any("RMSE" in error for error in report.errors)
    assert any("jump" in error for error in report.errors)

