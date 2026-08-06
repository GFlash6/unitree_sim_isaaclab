from __future__ import annotations

import uuid

import numpy as np

from tools.ground_truth_shared_memory_utils import GroundTruthReader, GroundTruthWriter


def shm_name() -> str:
    return f"test_isaac_ground_truth_{uuid.uuid4().hex}"


def test_round_trip_and_fresh_sample_suppression() -> None:
    name = shm_name()
    writer = GroundTruthWriter(name)
    reader = GroundTruthReader(name)
    try:
        assert writer.write_pose(123, [1, 2, 3], [1, 0, 0, 0])
        sample = reader.read_pose()
        assert sample is not None
        assert sample.timestamp_ns == 123
        np.testing.assert_allclose(sample.position, [1, 2, 3])
        np.testing.assert_allclose(sample.quaternion_wxyz, [1, 0, 0, 0])
        assert reader.read_pose() is None
    finally:
        reader.close()
        writer.shm.unlink()
        writer.close()


def test_invalid_pose_is_rejected() -> None:
    name = shm_name()
    writer = GroundTruthWriter(name)
    try:
        assert not writer.write_pose(0, [1, 2, 3], [1, 0, 0, 0])
        assert not writer.write_pose(1, [np.nan, 2, 3], [1, 0, 0, 0])
        assert not writer.write_pose(1, [1, 2, 3], [2, 0, 0, 0])
    finally:
        writer.shm.unlink()
        writer.close()


def test_writer_rejects_duplicate_or_regressive_source_time() -> None:
    name = shm_name()
    writer = GroundTruthWriter(name)
    try:
        assert writer.write_pose(10, [0, 0, 0], [1, 0, 0, 0])
        assert not writer.write_pose(10, [0, 0, 0], [1, 0, 0, 0])
        assert not writer.write_pose(9, [0, 0, 0], [1, 0, 0, 0])
        assert writer.write_pose(11, [0, 0, 0], [1, 0, 0, 0])
    finally:
        writer.shm.unlink()
        writer.close()
