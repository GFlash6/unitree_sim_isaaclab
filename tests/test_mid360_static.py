#!/usr/bin/env python3
from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_lidar_config_is_exported() -> None:
    cfg = source("tasks/common_config/lidar_configs.py")
    tree = ast.parse(cfg)
    classes = {node.name for node in tree.body if isinstance(node, ast.ClassDef)}
    assert "LidarPresets" in classes

    init_py = source("tasks/common_config/__init__.py")
    assert "LidarPresets" in init_py


def test_g1_task_scenes_mount_mid360() -> None:
    for path in sorted((ROOT / "tasks/g1_tasks").glob("**/*env_cfg.py")) + sorted(
        (ROOT / "tasks/g1_tasks").glob("**/*joint_env_cfg.py")
    ) + sorted((ROOT / "tasks/g1_tasks").glob("**/*hw_env_cfg.py")):
        text = path.read_text(encoding="utf-8")
        if "front_camera = CameraPresets.g1_front_camera()" not in text:
            continue
        assert "LidarPresets" in text, path
        assert "mid360 = LidarPresets.g1_mid360()" in text, path


def test_mid360_observation_is_exported() -> None:
    obs = source("tasks/common_observations/mid360_state.py")
    assert "def get_mid360_points" in obs

    g1_obs_files = sorted((ROOT / "tasks/g1_tasks").glob("**/mdp/observations.py"))
    assert g1_obs_files
    for path in g1_obs_files:
        text = path.read_text(encoding="utf-8")
        assert "get_mid360_points" in text, path


if __name__ == "__main__":
    test_lidar_config_is_exported()
    test_g1_task_scenes_mount_mid360()
    test_mid360_observation_is_exported()
