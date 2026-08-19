import importlib.util
import json
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).parents[1]
    / "HoloAgent/agentic_robot/fsr_vln/scripts/hmsg_query_server.py"
)
SPEC = importlib.util.spec_from_file_location("hmsg_query_server", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_sim_point_to_map_uses_explicit_alignment_artifact():
    half = np.sqrt(0.5)
    point_sim = np.array([2.0, 1.0, 0.5])
    robot_sim = np.array([1.0, 1.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    robot_map = np.array([10.0, 20.0, 0.5, half, 0.0, 0.0, half])

    transform = MODULE.align_sim_to_map(robot_sim, robot_map)
    result = MODULE.sim_point_to_map(point_sim, transform)

    np.testing.assert_allclose(result, [10.0, 21.0, 0.5], atol=1e-7)


def test_checked_in_alignment_maps_calibration_pose_to_map_origin():
    root = Path(__file__).parents[1]
    artifact = (
        root
        / "holoagent_bridge/semantic_maps/isaac_live_20260810_0055/sim_to_map.json"
    )
    transform = MODULE.load_sim_to_map(artifact)
    document = json.loads(artifact.read_text())
    robot_sim = document["calibration"]["robot_sim_pose_wxyz"]

    result = MODULE.sim_point_to_map(np.asarray(robot_sim[:3]), transform)

    np.testing.assert_allclose(result, [0.0, 0.0, 0.0], atol=1e-7)


def test_refined_anchor_atomically_overwrites_previous_position(tmp_path: Path):
    path = tmp_path / "semantic_anchors.json"
    store = MODULE.AnchorStore(path, min_observations=3)

    first = MODULE.AnchorUpdateRequest(
        object_id="0_0_2",
        center_map=[1.0, 2.0, 0.5],
        score=0.4,
        observation_count=3,
        source_timestamp_ms=1000,
    )
    second = MODULE.AnchorUpdateRequest(
        object_id="0_0_2",
        center_map=[3.0, 4.0, 0.5],
        score=0.6,
        observation_count=5,
        source_timestamp_ms=2000,
    )

    store.update(first)
    store.update(second)

    np.testing.assert_allclose(store.get("0_0_2")["center_map"], [3.0, 4.0, 0.5])
    document = json.loads(path.read_text(encoding="utf-8"))
    assert document["version"] == 1
    assert document["anchors"]["0_0_2"]["observation_count"] == 5
    assert not path.with_suffix(".json.tmp").exists()
