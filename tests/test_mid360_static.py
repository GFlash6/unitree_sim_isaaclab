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
    assert "MultiMeshRayCasterCfg" in cfg
    assert "/World/envs/env_.*/Robot/mid360_link" in cfg
    assert "/World/envs/env_.*/Robot/mid360_link/mid360" not in cfg
    assert "{ENV_REGEX_NS}/Room" in cfg
    assert "{ENV_REGEX_NS}/PackingTable_1" in cfg
    assert "{ENV_REGEX_NS}/PackingTable_2" in cfg
    assert "{ENV_REGEX_NS}/Object" in cfg
    assert "paths.split(\",\")" in cfg
    assert "MID360_MESH_PRIM_PATHS" in cfg

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
        assert "mid360 = LidarPresets.g1_mid360(" in text or "mid360 = LidarPresets.g1_mid360()" in text, path


def test_mid360_observation_is_exported() -> None:
    obs = source("tasks/common_observations/mid360_state.py")
    assert "def get_mid360_points" in obs
    assert "quat_apply_inverse" in obs
    assert "sensor.data.pos_w" in obs
    assert "sensor.data.quat_w" in obs
    assert "finite_points_w - sensor_pos_w" in obs

    g1_obs_files = sorted((ROOT / "tasks/g1_tasks").glob("**/mdp/observations.py"))
    assert g1_obs_files
    for path in g1_obs_files:
        text = path.read_text(encoding="utf-8")
        assert "get_mid360_points" in text, path


def test_mid360_shared_memory_is_not_resource_tracked() -> None:
    shm = source("tools/pointcloud_shared_memory_utils.py")
    assert "resource_tracker.unregister" in shm


def test_sim_main_exports_mid360_every_loop() -> None:
    sim_main = source("sim_main.py")
    assert "from tasks.common_observations.mid360_state import get_mid360_points" in sim_main
    assert "controller.step()\n                get_mid360_points(env)" in sim_main


def test_wholebody_tasks_use_default_multi_mesh_lidar() -> None:
    for path in sorted((ROOT / "tasks/g1_tasks").glob("move_cylinder_g1_29dof_*_wholebody/*_hw_env_cfg.py")):
        text = path.read_text(encoding="utf-8")
        assert "mid360 = LidarPresets.g1_mid360()" in text, path


def test_g1_observation_exports_real_imu_with_simulator_timestamp() -> None:
    obs = source("tasks/common_observations/g1_29dof_state.py")
    assert "from tools.imu_shared_memory_utils import ImuWriter" in obs
    assert "imu_timestamp_ns = int(float(env.sim.current_time) * 1_000_000_000)" in obs
    assert "imu_sample = imu_data[0].contiguous().cpu().numpy()" in obs
    assert "_imu_writer.write_sample(" in obs
    assert "imu_sample[3:7]" in obs
    assert "imu_sample[7:10]" in obs
    assert "imu_sample[10:13]" in obs
    assert obs.count("get_robot_imu_data(env)") == 1
    assert '"imu_timestamp_ns": 0' in obs
    assert 'if imu_timestamp_ns > _obs_cache["imu_timestamp_ns"]:' in obs
    assert '_obs_cache["imu_sample"] = imu_sample' in obs


if __name__ == "__main__":
    test_lidar_config_is_exported()
    test_g1_task_scenes_mount_mid360()
    test_mid360_observation_is_exported()
    test_sim_main_exports_mid360_every_loop()
    test_wholebody_tasks_use_default_multi_mesh_lidar()
