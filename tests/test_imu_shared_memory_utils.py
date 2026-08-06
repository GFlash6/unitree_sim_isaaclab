from __future__ import annotations

import ctypes
import uuid
from multiprocessing import shared_memory

import numpy as np

from tools.imu_shared_memory_utils import ImuHeader, ImuReader, ImuWriter


def shm_name() -> str:
    return f"test_isaac_imu_{uuid.uuid4().hex}"


def unlink(name: str) -> None:
    try:
        shm = shared_memory.SharedMemory(name=name)
    except FileNotFoundError:
        return
    shm.close()
    shm.unlink()


def test_complete_real_imu_sample_round_trips_once() -> None:
    name = shm_name()
    writer = ImuWriter(name)
    reader = ImuReader(name)
    try:
        assert writer.write_sample(
            123456789,
            [1.0, 0.0, 0.0, 0.0],
            [0.1, -0.2, 9.81],
            [0.01, 0.02, -0.03],
        )
        sample = reader.read_sample()
        assert sample is not None
        assert sample.sequence == 2
        assert sample.timestamp_ns == 123456789
        np.testing.assert_allclose(sample.quaternion_wxyz, [1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(sample.linear_acceleration, [0.1, -0.2, 9.81])
        np.testing.assert_allclose(sample.angular_velocity, [0.01, 0.02, -0.03])
        assert reader.read_sample() is None
    finally:
        reader.close()
        writer.close()
        unlink(name)


def test_partial_odd_sequence_record_is_never_returned() -> None:
    name = shm_name()
    writer = ImuWriter(name)
    reader = ImuReader(name)
    try:
        assert writer.write_sample(10, [1, 0, 0, 0], [0, 0, 9.81], [0, 0, 0])
        header = ImuHeader.from_buffer_copy(bytes(writer.shm.buf[: ctypes.sizeof(ImuHeader)]))
        header.sequence += 1
        writer.shm.buf[: ctypes.sizeof(ImuHeader)] = ctypes.string_at(
            ctypes.byref(header), ctypes.sizeof(ImuHeader)
        )
        assert reader.read_sample() is None
    finally:
        reader.close()
        writer.close()
        unlink(name)


def test_invalid_or_non_finite_measurements_are_rejected() -> None:
    name = shm_name()
    writer = ImuWriter(name)
    try:
        assert not writer.write_sample(1, [1, 0, 0], [0, 0, 9.81], [0, 0, 0])
        assert not writer.write_sample(1, [1, 0, 0, 0], [0, np.nan, 9.81], [0, 0, 0])
        assert not writer.write_sample(1, [2, 0, 0, 0], [0, 0, 9.81], [0, 0, 0])
        assert not writer.write_sample(0, [1, 0, 0, 0], [0, 0, 9.81], [0, 0, 0])
    finally:
        writer.close()
        unlink(name)
