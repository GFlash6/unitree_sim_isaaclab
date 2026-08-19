from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "HoloAgent/agentic_robot/core/src"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_stable_ros_interfaces_cover_long_running_tasks_and_health() -> None:
    interfaces = CORE / "holoagent_interfaces"
    assert (interfaces / "action/NavigateToObject.action").is_file()
    assert (interfaces / "action/ExecuteArmSkill.action").is_file()
    assert (interfaces / "srv/QueryObject.srv").is_file()
    status = (interfaces / "msg/LocalizationStatus.msg").read_text()
    assert "std_msgs/Header header" in status
    assert "bool localized" in status


def test_rgbd_robot_io_has_no_localization_dependency() -> None:
    source = (ROOT / "holoagent_bridge/isaac_rgbd_pose_bridge.py").read_text()
    assert "Odometry" not in source
    assert "PoseStamped" not in source
    assert '"sensors/front/camera_info"' in source
    assert "TransformBroadcaster" in source
    assert "camera_pose_in_base" in source
    camera_state = (ROOT / "tasks/common_observations/camera_state.py").read_text()
    assert 'body_names.index("imu_in_torso")' in camera_state
    assert "root_link_pos_w" not in camera_state


def test_semantic_navigation_depends_on_ros_services_not_http() -> None:
    source = (
        CORE
        / "navigation/semantic_goal/semantic_goal/semantic_goal_node.py"
    ).read_text()
    assert "QueryObject" in source
    assert "NavigateToObject" in source
    assert "urlopen" not in source
    assert "HMSG_QUERY_URL" not in source
    hmsg = (
        ROOT / "HoloAgent/agentic_robot/fsr_vln/scripts/hmsg_query_server.py"
    ).read_text()
    assert "MultiImageReader" not in hmsg
    assert "shared_memory" not in hmsg
    assert "load_sim_to_map" in hmsg


def test_online_semantic_mapping_uses_camera_info_and_tf() -> None:
    source = (
        ROOT
        / "HoloAgent/agentic_robot/fsr_vln/ovo/entities/semantic_mapping_online.py"
    ).read_text()
    assert "TransformListener" in source
    assert "lookup_transform" in source
    assert "CameraInfo" in source
    assert "PoseStamped" not in source
    assert "pcd_object_ids.detach().cpu().numpy().reshape(-1)" in source
    assert '"frame_id": self.world_frame' in source


def test_canonical_localization_graph_uses_one_sim_clock() -> None:
    launch = (
        CORE / "fast_livo/launch/isaac_localization.launch.py"
    ).read_text()
    assert '"lio/odom"' in launch
    assert '"localization/odom"' in launch
    assert '"perception/obstacles"' in launch
    assert '"use_sim_time": True' in launch
    imu = (ROOT / "holoagent_bridge/imu_to_ros2_topic.py").read_text()
    assert '"/clock"' in imu
    relo = (
        CORE / "fast_livo/include/online-relo/pose_estimator.cpp"
    ).read_text()
    assert "currentCloudTime * 1e9" in relo
    validator = (ROOT / "holoagent_bridge/validate_relocalization.py").read_text()
    assert '"/localization/odom"' in validator
    assert '"/perception/obstacles"' in validator


def test_semantic_http_result_rejects_wrong_frame() -> None:
    backend = load(
        CORE / "semantic_map_bridge/semantic_map_bridge/http_backend.py",
        "semantic_http_backend",
    )
    parsed = backend.parse_query_result(
        {"center_map": [1, 2, 3], "score": 0.8, "frame_id": "map"}
    )
    assert parsed["found"]
    with pytest.raises(ValueError, match="unsupported frame"):
        backend.parse_query_result(
            {"center_map": [1, 2, 3], "score": 0.8, "frame_id": "odom"}
        )


def test_localization_health_has_explicit_stages() -> None:
    health = load(
        CORE / "localization_monitor/localization_monitor/health.py",
        "localization_health",
    )
    assert health.localization_state(None, float("nan"), 0, 3) == "INITIALIZING"
    assert health.localization_state(False, 1.0, 1, 3) == "DEGRADED"
    assert health.localization_state(False, 1.0, 3, 3) == "LOST"
    assert health.localization_state(True, 0.1, 0, 3) == "TRACKING"


def test_arm_result_protocol_is_task_correlatable() -> None:
    protocol = load(
        CORE / "manipulation/manipulation/protocol.py", "arm_protocol"
    )
    assert protocol.parse_arm_result("arm_finish:wave") == (True, "wave", "")
    assert protocol.parse_arm_result("arm_failed:wave:7") == (False, "wave", "7")
    assert protocol.parse_arm_result("nav_finish") is None


def test_arm_action_rejects_missing_device_adapter() -> None:
    source = (
        CORE / "manipulation/manipulation/arm_skill_server.py"
    ).read_text()
    assert "command_publisher.get_subscription_count() == 0" in source


def test_gateway_action_endpoints_are_real_actions() -> None:
    config = (
        ROOT
        / "HoloAgent/agentic_robot/services/src/robot_bridge/config/bridge_config.yaml"
    ).read_text()
    node = (
        ROOT
        / "HoloAgent/agentic_robot/services/src/robot_bridge/robot_bridge/robot_bridge_node.py"
    ).read_text()
    assert "holoagent_interfaces/NavigateToObject" in config
    assert "holoagent_interfaces/ExecuteArmSkill" in config
    assert "send_goal_async" in node
    assert '"action calls not yet implemented"' not in node
