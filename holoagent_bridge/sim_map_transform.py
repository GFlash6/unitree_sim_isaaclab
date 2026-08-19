"""Validated rigid transform between Isaac world and the navigation map."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def quaternion_inverse(quaternion: np.ndarray) -> np.ndarray:
    norm_squared = float(np.dot(quaternion, quaternion))
    if not math.isfinite(norm_squared) or norm_squared < 1e-12:
        raise ValueError("invalid zero quaternion")
    result = quaternion.astype(np.float64, copy=True)
    result[1:] *= -1.0
    return result / norm_squared


def quaternion_rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.concatenate(([0.0], np.asarray(vector, dtype=np.float64)))
    return quaternion_multiply(
        quaternion_multiply(quaternion, pure), quaternion_inverse(quaternion)
    )[1:]


def validated_pose(pose, label: str) -> np.ndarray:
    value = np.array(pose, dtype=np.float64, copy=True)
    if value.shape != (7,) or not np.isfinite(value).all():
        raise ValueError(f"{label} must contain 7 finite wxyz pose values")
    norm = float(np.linalg.norm(value[3:]))
    if not 0.99 <= norm <= 1.01:
        raise ValueError(f"{label} quaternion is not normalized")
    value[3:] /= norm
    return value


def align_sim_to_map(robot_sim, robot_map) -> np.ndarray:
    """Calculate map <- sim from one simultaneous robot pose pair."""
    sim = validated_pose(robot_sim, "robot_sim")
    map_pose = validated_pose(robot_map, "robot_map")
    rotation = quaternion_multiply(map_pose[3:], quaternion_inverse(sim[3:]))
    rotation /= np.linalg.norm(rotation)
    translation = map_pose[:3] - quaternion_rotate(rotation, sim[:3])
    return np.concatenate((translation, rotation))


def sim_point_to_map(point_sim, sim_to_map) -> np.ndarray:
    point = np.asarray(point_sim, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("point_sim must contain 3 finite values")
    transform = validated_pose(sim_to_map, "sim_to_map")
    return transform[:3] + quaternion_rotate(transform[3:], point)


def load_sim_to_map(path: Path) -> np.ndarray:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("parent_frame") != "map" or document.get("child_frame") != "sim_world":
        raise ValueError("sim-to-map artifact must describe map <- sim_world")
    pose = [*document["translation"], *document["rotation_wxyz"]]
    return validated_pose(pose, "sim_to_map")
