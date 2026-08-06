from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_sim_main_exports_real_imu_pose_after_control_step() -> None:
    source = (ROOT / "sim_main.py").read_text(encoding="utf-8")
    assert "from tools.ground_truth_shared_memory_utils import GroundTruthWriter" in source
    assert "_ground_truth_writer = GroundTruthWriter()" in source
    assert 'imu_body_index = env.scene["robot"].data.body_names.index("imu_in_torso")' in source
    assert 'imu_pose = env.scene["robot"].data.body_link_pose_w[0, imu_body_index]' in source
    assert "ground_truth_timestamp_ns = int(float(env.sim.current_time) * 1_000_000_000)" in source
    assert "_ground_truth_writer.write_pose(" in source
    assert source.index("controller.step()") < source.index("_ground_truth_writer.write_pose(")


def test_diagnostic_stream_exports_observed_root_pose_not_command_buffer() -> None:
    source = (ROOT / "holoagent_bridge/stream_mid360_mapping_sequence.py").read_text(encoding="utf-8")
    assert "GroundTruthWriter" in source
    assert "observed_pose = robot.data.root_link_pose_w[0]" in source
    assert "ground_truth_writer.write_pose(" in source
    assert "observed_pose_sample[:3]" in source
    assert "observed_pose_sample[3:7]" in source
