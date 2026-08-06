from __future__ import annotations

import threading
import uuid
from multiprocessing import shared_memory

import numpy as np

from tools.pointcloud_shared_memory_utils import PointCloudReader, PointCloudWriter


def _name() -> str:
    return f"mid360_test_{uuid.uuid4().hex}"


def _unlink(name: str) -> None:
    try:
        shm = shared_memory.SharedMemory(name=name)
    except FileNotFoundError:
        return
    shm.close()
    shm.unlink()


def test_reader_returns_each_sequence_once_with_source_timestamp() -> None:
    name = _name()
    writer = PointCloudWriter(name, max_points=8)
    reader = PointCloudReader(name)
    try:
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        assert writer.write_points(points, timestamp_ns=123456789)
        np.testing.assert_array_equal(reader.read_points(), points)
        assert reader.last_timestamp_ns == 123456789
        assert reader.read_points() is None

        assert not writer.write_points(points + 1, timestamp_ns=123456789)
        assert not writer.write_points(points + 1, timestamp_ns=123456788)
        assert writer.write_points(points + 1, timestamp_ns=123456790)
        np.testing.assert_array_equal(reader.read_points(), points + 1)
    finally:
        reader.close()
        writer.close()
        _unlink(name)


def test_concurrent_reads_never_return_torn_frames() -> None:
    name = _name()
    writer = PointCloudWriter(name, max_points=1024)
    reader = PointCloudReader(name)
    failures: list[np.ndarray] = []

    def produce() -> None:
        for value in range(1, 250):
            frame = np.full((1024, 3), value, dtype=np.float32)
            assert writer.write_points(frame, timestamp_ns=value)

    thread = threading.Thread(target=produce)
    thread.start()
    while thread.is_alive():
        frame = reader.read_points()
        if frame is not None and not np.all(frame == frame[0, 0]):
            failures.append(frame)
    thread.join()
    try:
        assert not failures
    finally:
        reader.close()
        writer.close()
        _unlink(name)


def test_writer_restart_continues_sequence_for_existing_reader() -> None:
    name = _name()
    first_writer = PointCloudWriter(name, max_points=8)
    reader = PointCloudReader(name)
    try:
        assert first_writer.write_points(np.zeros((1, 3), np.float32), timestamp_ns=1)
        assert reader.read_points() is not None
        first_writer.close()
        restarted_writer = PointCloudWriter(name, max_points=8)
        try:
            assert restarted_writer.write_points(np.ones((1, 3), np.float32), timestamp_ns=2)
            np.testing.assert_array_equal(reader.read_points(), np.ones((1, 3), np.float32))
        finally:
            restarted_writer.close()
    finally:
        reader.close()
        _unlink(name)
