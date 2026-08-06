"""Lock-free shared-memory transport for real IsaacLab IMU samples."""

from __future__ import annotations

import ctypes
import math
from multiprocessing import resource_tracker, shared_memory
from typing import NamedTuple, Optional, Sequence

import numpy as np


IMU_SHM_NAME = "isaac_imu_state_shm"
IMU_SHM_MAGIC = 0x4953414143494D55  # "ISAACIMU"
IMU_SHM_VERSION = 1


def _untrack(shm: shared_memory.SharedMemory) -> None:
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


class ImuHeader(ctypes.LittleEndianStructure):
    _fields_ = [
        ("magic", ctypes.c_uint64),
        ("version", ctypes.c_uint32),
        ("header_size", ctypes.c_uint32),
        ("sequence", ctypes.c_uint64),
        ("timestamp_ns", ctypes.c_uint64),
        ("quaternion_wxyz", ctypes.c_float * 4),
        ("linear_acceleration", ctypes.c_float * 3),
        ("angular_velocity", ctypes.c_float * 3),
    ]


class ImuSample(NamedTuple):
    sequence: int
    timestamp_ns: int
    quaternion_wxyz: np.ndarray
    linear_acceleration: np.ndarray
    angular_velocity: np.ndarray


def _vector(values: Sequence[float], size: int) -> Optional[np.ndarray]:
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (size,) or not np.isfinite(result).all():
        return None
    return result


class ImuWriter:
    def __init__(self, shm_name: str = IMU_SHM_NAME):
        self.shm_name = shm_name
        self._sequence = 0
        size = ctypes.sizeof(ImuHeader)
        try:
            self.shm = shared_memory.SharedMemory(name=shm_name)
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(create=True, size=size, name=shm_name)
        _untrack(self.shm)
        if self.shm.size < size:
            actual = self.shm.size
            self.shm.close()
            raise ValueError(f"shared memory {shm_name!r} is {actual} bytes; at least {size} required")
        existing = ImuHeader.from_buffer_copy(bytes(self.shm.buf[:size]))
        if (
            existing.magic == IMU_SHM_MAGIC
            and existing.version == IMU_SHM_VERSION
            and existing.sequence % 2 == 0
        ):
            self._sequence = existing.sequence

    def write_sample(
        self,
        timestamp_ns: int,
        quaternion_wxyz: Sequence[float],
        linear_acceleration: Sequence[float],
        angular_velocity: Sequence[float],
    ) -> bool:
        quaternion = _vector(quaternion_wxyz, 4)
        acceleration = _vector(linear_acceleration, 3)
        angular = _vector(angular_velocity, 3)
        if timestamp_ns <= 0 or quaternion is None or acceleration is None or angular is None:
            return False
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or not 0.99 <= norm <= 1.01:
            return False
        self._sequence += 2
        header = ImuHeader(
            magic=IMU_SHM_MAGIC,
            version=IMU_SHM_VERSION,
            header_size=ctypes.sizeof(ImuHeader),
            sequence=self._sequence,
            timestamp_ns=int(timestamp_ns),
            quaternion_wxyz=tuple(float(value) for value in quaternion),
            linear_acceleration=tuple(float(value) for value in acceleration),
            angular_velocity=tuple(float(value) for value in angular),
        )
        writing = ImuHeader.from_buffer_copy(header)
        writing.sequence -= 1
        size = ctypes.sizeof(ImuHeader)
        self.shm.buf[:size] = ctypes.string_at(ctypes.byref(writing), size)
        self.shm.buf[:size] = ctypes.string_at(ctypes.byref(header), size)
        return True

    def close(self) -> None:
        self.shm.close()


class ImuReader:
    def __init__(self, shm_name: str = IMU_SHM_NAME):
        self.shm_name = shm_name
        self.shm: Optional[shared_memory.SharedMemory] = None
        self.last_sequence = 0
        self.last_timestamp_ns = 0

    def read_sample(self) -> Optional[ImuSample]:
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=self.shm_name)
                _untrack(self.shm)
            except FileNotFoundError:
                return None
        size = ctypes.sizeof(ImuHeader)
        for _ in range(3):
            header = ImuHeader.from_buffer_copy(bytes(self.shm.buf[:size]))
            if (
                header.magic != IMU_SHM_MAGIC
                or header.version != IMU_SHM_VERSION
                or header.header_size != size
                or header.sequence % 2 != 0
                or header.sequence <= self.last_sequence
                or header.timestamp_ns <= 0
            ):
                return None
            confirmed = ImuHeader.from_buffer_copy(bytes(self.shm.buf[:size]))
            if confirmed.sequence != header.sequence or confirmed.sequence % 2:
                continue
            quaternion = np.array(header.quaternion_wxyz, dtype=np.float32)
            acceleration = np.array(header.linear_acceleration, dtype=np.float32)
            angular = np.array(header.angular_velocity, dtype=np.float32)
            if not all(np.isfinite(value).all() for value in (quaternion, acceleration, angular)):
                return None
            self.last_sequence = header.sequence
            self.last_timestamp_ns = header.timestamp_ns
            return ImuSample(
                header.sequence,
                header.timestamp_ns,
                quaternion,
                acceleration,
                angular,
            )
        return None

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None
