#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_mid360_bridge_has_no_synthetic_cloud_path() -> None:
    source = (ROOT / "test" / "mid360_to_ros2_topic.py").read_text(encoding="utf-8")
    assert "--fake" not in source
    assert "fake_points" not in source
    assert "PointCloudReader()" in source


def test_mid360_bridge_only_publishes_raw_sensor_topic() -> None:
    bridge = load_module(ROOT / "test" / "mid360_to_ros2_topic.py", "mid360_to_ros2_topic")
    args = bridge.parse_args([])
    assert args.topic == "/mid360/points"
    assert not hasattr(args, "reloc_topic")


def test_cmd_vel_command_clamps_and_preserves_height() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args(["--max-x", "0.3", "--max-y", "0.2", "--max-yaw", "0.4", "--height", "0.75"])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=1.0, y=-1.0),
        angular=SimpleNamespace(z=2.0),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.3, -0.2, 0.4, 0.75]


def test_cmd_vel_command_goes_zero_when_stale() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args(["--stale-timeout", "0.5", "--height", "0.8"])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.2, y=0.1),
        angular=SimpleNamespace(z=0.3),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.0)

    assert command == [0.0, 0.0, 0.0, 0.8]


def test_documented_sim_command_selects_wholebody_policy_provider() -> None:
    readme = (ROOT / "holoagent_bridge" / "README.md").read_text(encoding="utf-8")
    assert "--action_source dds_wholebody" in readme


def test_mapping_sequence_streams_real_isaaclab_mid360_motion() -> None:
    source = (ROOT / "holoagent_bridge" / "stream_mid360_mapping_sequence.py").read_text(encoding="utf-8")
    assert "write_root_pose_to_sim" in source
    assert "write_root_velocity_to_sim" in source
    assert "get_mid360_points(env)" in source
    assert "PointCloudReader()" in source
    assert "fake" not in source.lower()
    assert "synthetic" not in source.lower()


def test_fast_livo_save_map_is_synchronous_and_repeatable() -> None:
    source = (
        ROOT
        / "HoloAgent"
        / "agentic_robot"
        / "core"
        / "src"
        / "fast_livo"
        / "src"
        / "LIVMapper.cpp"
    ).read_text(encoding="utf-8")
    callback = source[source.index("auto saveMapService") : source.index("srvSaveMap =")]
    save = source[source.index("bool LIVMapper::saveKeyFrame") : source.index("void LIVMapper::run")]

    assert "fork(" not in callback
    assert "res->success = saveKeyFrame" in callback
    assert "ScanContext::SCManager save_sc_manager" in save
    assert "save_sc_manager.polarcontexts_.size() - 1" in save
    assert "VoxelGrid<PointTypeXYZI>" not in save
    assert "voxel_points.try_emplace(key, point)" in save
    assert '"Map validation result: %s"' in save


def test_relocalization_external_topics_use_ros_time() -> None:
    source = (
        ROOT
        / "HoloAgent"
        / "agentic_robot"
        / "core"
        / "src"
        / "fast_livo"
        / "include"
        / "online-relo"
        / "pose_estimator.cpp"
    ).read_text(encoding="utf-8")
    cloud_publish = source[source.index("bool pose_estimator::relocalization") : source.index("bool pose_estimator::easyToRelo")]
    odom_publish = source[source.index("void pose_estimator::publish_odometry(", source.index("void pose_estimator::publish_odometry(") + 1) : source.index("void pose_estimator::publish_path")]

    assert 'publishCloud(pubRelocBodyCloud, cloudInBody, publish_stamp, "base_link")' in cloud_publish
    assert "currentCloudTime * 1e9" not in cloud_publish
    assert "odomAftMapped.header.stamp = stamp;" in source
    assert "odomAftMapped.header.stamp = this->node->now();" not in odom_publish


def test_nav2_configuration_uses_wall_time_consistently() -> None:
    paths = [
        ROOT / "HoloAgent" / "agentic_robot" / "core" / "src" / "nav_bringup" / "param" / "g1.yaml",
        ROOT / "HoloAgent" / "robots" / "unitree" / "config" / "nav_params.yaml",
    ]
    for path in paths:
        source = path.read_text(encoding="utf-8").lower()
        assert "use_sim_time: true" not in source, path
