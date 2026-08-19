#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]


def run_skill_script(relative_path: str, *args: str) -> str:
    result = subprocess.run(
        [sys.executable, str(ROOT / relative_path), *args, "--dry-run"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_mid360_bridge_has_no_synthetic_cloud_path() -> None:
    source = (ROOT / "holoagent_bridge" / "mid360_to_ros2_topic.py").read_text(encoding="utf-8")
    assert "--fake" not in source
    assert "fake_points" not in source
    assert "PointCloudReader()" in source


def test_mid360_bridge_only_publishes_raw_sensor_topic() -> None:
    bridge = load_module(
        ROOT / "holoagent_bridge" / "mid360_to_ros2_topic.py",
        "mid360_to_ros2_topic",
    )
    args = bridge.parse_args([])
    assert args.topic == "sensors/lidar/points"
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


def test_cmd_vel_pure_positive_yaw_adds_lateral_turn_assist() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args([])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.8),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.0, 0.3, 0.8, 0.8]


def test_cmd_vel_pure_negative_yaw_adds_backward_turn_assist() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args([])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=-0.8),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [-0.3, 0.0, -0.8, 0.8]


def test_cmd_vel_high_positive_yaw_uses_measured_policy_working_point() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args([])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.16, y=0.0),
        angular=SimpleNamespace(z=0.8),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.0, 0.3, 0.8, 0.8]


def test_cmd_vel_low_yaw_uses_measured_minimum_effective_forward_command() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args([])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.16, y=0.0),
        angular=SimpleNamespace(z=0.4),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.6, 0.0, 0.4, 0.8]


def test_cmd_vel_minimum_effective_forward_can_be_disabled() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args(["--min-effective-forward", "0"])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.16, y=0.0),
        angular=SimpleNamespace(z=0.4),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.16, 0.0, 0.4, 0.8]


def test_cmd_vel_turn_assist_can_be_disabled() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args(["--turn-assist-speed", "0"])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.8),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.0, 0.0, 0.8, 0.8]


def test_cmd_vel_turn_assist_ignores_floating_point_yaw_noise() -> None:
    bridge = load_module(ROOT / "holoagent_bridge" / "cmd_vel_to_unitree_dds.py", "cmd_vel_to_unitree_dds")
    args = bridge.parse_args([])
    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=-5e-16),
    )

    command = bridge.command_from_twist(msg, args, now=10.0, last_stamp=9.9)

    assert command == [0.0, 0.0, -5e-16, 0.8]


def test_documented_sim_command_selects_wholebody_policy_provider() -> None:
    readme = (ROOT / "holoagent_bridge" / "README.md").read_text(encoding="utf-8")
    assert "--action_source dds_wholebody" in readme


def test_documented_motion_command_is_sustained() -> None:
    readme = (ROOT / "holoagent_bridge" / "README.md").read_text(encoding="utf-8")
    assert "ros2 topic pub -r 10" in readme
    assert "ros2 topic pub --once /cmd_vel" not in readme
    assert "0.5 seconds" in readme


def test_holoagent_navigation_skills_match_robot_bridge_contract() -> None:
    base = "HoloAgent/agentic_robot/agentOS/holoagent_skills/skills"
    semantic = run_skill_script(
        f"{base}/sem-nav-skill/scripts/semantic_nav.py",
        "--robot-url", "http://robot:8000", "--cmd", "1F,lab,charger",
    )
    relative = run_skill_script(
        f"{base}/rel-move-skill/scripts/relative_move.py",
        "--robot-url", "http://robot:8000", "--cmd", "1.0,0.0,90",
    )

    assert "http://robot:8000/api/semantic_nav" in semantic
    assert '"cmd": "1F,lab,charger"' in semantic
    assert "http://robot:8000/api/relative_nav" in relative
    assert '"cmd": "1.0,0.0,90"' in relative


def test_holoagent_agent_polls_action_results_directly() -> None:
    source = (
        ROOT / "HoloAgent" / "agentic_robot" / "agentOS" / "sandbox_test" /
        "long_horizon_text_runner.py"
    ).read_text(encoding="utf-8")

    assert 'accepted.get("goal_id", "")' in source
    assert '/api/tasks/{goal_id}' in source
    assert 'state == "succeeded"' in source
    assert "视觉语义模型的开放词汇查询使用英文" in source


def test_holoagent_nav_executor_reports_terminal_failures_and_cancel() -> None:
    source = (
        ROOT / "HoloAgent" / "agentic_robot" / "core" / "src" /
        "navigation" / "nav_executor" / "nav_executor" / "pubpose.py"
    ).read_text(encoding="utf-8")

    assert 'publish_waypoint_reached("nav_canceled")' in source
    assert 'else "nav_failed"' in source
    assert "msg.data.startswith('custom_one_point_1_')" not in source
    assert "tf_transformations" not in source
    assert "math.sin(yaw / 2.0)" in source
    assert "declare_parameter('robot_name', 'unitree')" in source
    assert "self._start_single_pose(msg, self.goal_status_pub)" in source


def test_holoagent_semantic_goal_publishes_valid_orientation() -> None:
    source = (
        ROOT / "HoloAgent" / "agentic_robot" / "core" / "src" /
        "navigation" / "semantic_goal" / "semantic_goal" /
        "semantic_goal_node.py"
    ).read_text(encoding="utf-8")

    assert "goal.pose.orientation.z = float(np.sin(yaw / 2.0))" in source
    assert "goal.pose.orientation.w = float(np.cos(yaw / 2.0))" in source


def test_robot_bridge_reports_configured_robot_id() -> None:
    source_path = (
        ROOT / "HoloAgent" / "agentic_robot" / "services" / "src" /
        "robot_bridge" / "robot_bridge" / "robot_bridge_node.py"
    )
    config = (
        ROOT / "HoloAgent" / "agentic_robot" / "services" / "src" /
        "robot_bridge" / "config" / "bridge_config.yaml"
    ).read_text(encoding="utf-8")
    source = source_path.read_text(encoding="utf-8")

    assert 'robot_id: "${ROBOT_ID}"' in config
    assert 'if var.startswith("msg."):' in source
    assert "request.path_params" in source
    assert "**path_kwargs" not in source
    assert "rclpy.shutdown()" in source
    assert "ros_thread.join(timeout=2.0)" in source
    assert source.index("rclpy.shutdown()") < source.index("ros_node.destroy_node()")


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
    empty_guard = save.index("if (surfCloudKeyFrames.empty()")
    assert empty_guard < save.index("fsmkdir(save_dir)")


def test_documented_mapping_launch_enables_keyframe_collection() -> None:
    readme = (ROOT / "holoagent_bridge" / "README.md").read_text(encoding="utf-8")
    mapping_launch = readme[
        readme.index("To build a new relocation map") :
        readme.index("Before starting FAST-LIVO")
    ]
    assert "--params-file holoagent_bridge/fast_livo_mid360_mapping_sim.yaml" in mapping_launch


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
    assert "currentCloudTime * 1e9" in cloud_publish
    assert "odomAftMapped.header.stamp = stamp;" in source
    assert "odomAftMapped.header.stamp = this->node->now();" not in odom_publish


def test_relocalization_uses_measured_lidar_to_body_extrinsic() -> None:
    config = (ROOT / "holoagent_bridge" / "fast_livo_mid360_reloc_sim.yaml").read_text(encoding="utf-8")
    assert "extrinsic_T: [0.0398735, 0.00227, 0.26826]" in config
    assert "0.999194395, 0.0, 0.040131795" in config


def test_relocalization_secondary_search_uses_its_own_distances() -> None:
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
    easy_relo = source[source.index("bool pose_estimator::easyToRelo") : source.index("bool pose_estimator::globalRelo")]
    assert "!disVec_copy.empty()" in easy_relo
    assert "disVec_copy[0] <= searchDis * 2.0" in easy_relo
    assert "disVec[0] <= searchDis * 2.0" not in easy_relo


def test_nav2_configuration_uses_wall_time_consistently() -> None:
    paths = [
        ROOT / "HoloAgent" / "agentic_robot" / "core" / "src" / "nav_bringup" / "param" / "g1.yaml",
        ROOT / "HoloAgent" / "robots" / "unitree" / "config" / "nav_params.yaml",
    ]
    for path in paths:
        source = path.read_text(encoding="utf-8").lower()
        assert "use_sim_time: true" not in source, path


def test_relocalization_publishes_measured_body_twist() -> None:
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

    assert 'declare_parameter<double>("relo.twist_window_duration"' in source
    assert "twist_pose_window_.front().second.inverse() * pose_in_odom" in source
    assert "currentCloudTime - twist_pose_window_.front().first" in source
    assert "odomAftMapped.twist.twist.linear.x" in source
    assert "odomAftMapped.twist.twist.angular.z" in source


def test_nav2_consumes_validated_relocalization_odometry() -> None:
    config = (
        ROOT
        / "HoloAgent"
        / "agentic_robot"
        / "core"
        / "src"
        / "nav_bringup"
        / "param"
        / "g1.yaml"
    ).read_text(encoding="utf-8")
    assert "odom_topic: localization/odom" in config
    assert "odom_topic: /odom" not in config
    controller_config = config[
        config.index("controller_server:") : config.index("controller_server_rclcpp_node:")
    ]
    assert "odom_topic: localization/odom" in controller_config


def test_nav2_obstacle_layer_rejects_simulated_ground_returns() -> None:
    config = (
        ROOT
        / "HoloAgent"
        / "agentic_robot"
        / "core"
        / "src"
        / "nav_bringup"
        / "param"
        / "g1.yaml"
    ).read_text(encoding="utf-8")
    assert "min_obstacle_height: -0.8" not in config
    assert config.count("min_obstacle_height: -0.3") >= 4


def test_nav2_yaw_command_reaches_policy_effective_range() -> None:
    config = (
        ROOT
        / "HoloAgent"
        / "agentic_robot"
        / "core"
        / "src"
        / "nav_bringup"
        / "param"
        / "g1.yaml"
    ).read_text(encoding="utf-8")
    assert "max_vel_theta: 0.8" in config
    assert "min_speed_theta: 0.8" in config


def test_wholebody_reset_all_is_not_downgraded_to_object_only() -> None:
    source = (ROOT / "sim_main.py").read_text(encoding="utf-8")
    reset_logic = source[source.index("if reset_pose_cmd is not None:") : source.index("else:", source.index("if reset_pose_cmd is not None:"))]
    assert "if reset_category == '1':" in reset_logic
    assert "elif reset_category == '2':" in reset_logic
    assert 'env_cfg.event_manager.trigger("reset_all_self", env)' in reset_logic


def test_wholebody_policy_holds_fresh_dds_commands_between_messages() -> None:
    receiver = (ROOT / "dds" / "commands_dds.py").read_text(encoding="utf-8")
    provider = (ROOT / "action_provider" / "action_provider_wh_dds.py").read_text(encoding="utf-8")

    assert '"received_at_monotonic_ns": time.monotonic_ns()' in receiver
    assert "wholebody_command_timeout" in provider
    assert "time.monotonic_ns()" in provider
    assert "write_run_command([0.0,0,0,0.8])" not in provider


def test_run_command_dds_records_receive_time() -> None:
    receiver = load_module(ROOT / "dds" / "commands_dds.py", "commands_dds")
    writes = []
    instance = receiver.RunCommandDDS.__new__(receiver.RunCommandDDS)
    instance.node_name = "test"
    instance.output_shm = SimpleNamespace(write_data=writes.append)

    instance.dds_subscriber(SimpleNamespace(data="[0.0, 0.0, 0.8, 0.8]"))

    assert writes[0]["run_command"] == "[0.0, 0.0, 0.8, 0.8]"
    assert writes[0]["received_at_monotonic_ns"] > 0
