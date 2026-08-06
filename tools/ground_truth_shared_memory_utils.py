"""Shared-memory transport for timestamped IsaacLab root ground truth."""

from __future__ import annotations

import ctypes
import math
from multiprocessing import resource_tracker, shared_memory
from typing import NamedTuple, Optional, Sequence

import numpy as np


GROUND_TRUTH_SHM_NAME = "isaac_ground_truth_shm"
GROUND_TRUTH_SHM_MAGIC = 0x4953414143475450  # "ISAACGTP"
GROUND_TRUTH_SHM_VERSION = 1


def _untrack(shm: shared_memory.SharedMemory) -> None:
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


class GroundTruthHeader(ctypes.LittleEndianStructure):
    _fields_ = [
        ("magic", ctypes.c_uint64),
        ("version", ctypes.c_uint32),
        ("header_size", ctypes.c_uint32),
        ("sequence", ctypes.c_uint64),
        ("timestamp_ns", ctypes.c_uint64),
        ("position", ctypes.c_double * 3),
        ("quaternion_wxyz", ctypes.c_double * 4),
    ]


class GroundTruthSample(NamedTuple):
    sequence: int
    timestamp_ns: int
    position: np.ndarray
    quaternion_wxyz: np.ndarray


def _vector(values: Sequence[float], size: int) -> Optional[np.ndarray]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (size,) or not np.isfinite(result).all():
        return None
    return result


class GroundTruthWriter:
    def __init__(self, shm_name: str = GROUND_TRUTH_SHM_NAME):
        self.shm_name = shm_name
        self._sequence = 0
        self._last_timestamp_ns = 0
        size = ctypes.sizeof(GroundTruthHeader)
        try:
            self.shm = shared_memory.SharedMemory(name=shm_name)
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(create=True, size=size, name=shm_name)
        _untrack(self.shm)
        if self.shm.size < size:
            actual = self.shm.size
            self.shm.close()
            raise ValueError(f"shared memory {shm_name!r} is {actual} bytes; at least {size} required")
        existing = GroundTruthHeader.from_buffer_copy(bytes(self.shm.buf[:size]))
        if (
            existing.magic == GROUND_TRUTH_SHM_MAGIC
            and existing.version == GROUND_TRUTH_SHM_VERSION
            and existing.sequence % 2 == 0
        ):
            self._sequence = existing.sequence

    def write_pose(
        self,
        timestamp_ns: int,
        position: Sequence[float],
        quaternion_wxyz: Sequence[float],
    ) -> bool:
        position_array = _vector(position, 3)
        quaternion = _vector(quaternion_wxyz, 4)
        if timestamp_ns <= self._last_timestamp_ns or position_array is None or quaternion is None:
            return False
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or not 0.99 <= norm <= 1.01:
            return False
        self._sequence += 2
        header = GroundTruthHeader(
            magic=GROUND_TRUTH_SHM_MAGIC,
            version=GROUND_TRUTH_SHM_VERSION,
            header_size=ctypes.sizeof(GroundTruthHeader),
            sequence=self._sequence,
            timestamp_ns=int(timestamp_ns),
            position=tuple(float(value) for value in position_array),
            quaternion_wxyz=tuple(float(value) for value in quaternion),
        )
        writing = GroundTruthHeader.from_buffer_copy(header)
        writing.sequence -= 1
        size = ctypes.sizeof(GroundTruthHeader)
        self.shm.buf[:size] = ctypes.string_at(ctypes.byref(writing), size)
        self.shm.buf[:size] = ctypes.string_at(ctypes.byref(header), size)
        self._last_timestamp_ns = int(timestamp_ns)
        return True

    def close(self) -> None:
        self.shm.close()


class GroundTruthReader:
    def __init__(self, shm_name: str = GROUND_TRUTH_SHM_NAME):
        self.shm_name = shm_name
        self.shm: Optional[shared_memory.SharedMemory] = None
        self.last_sequence = 0

    def read_pose(self) -> Optional[GroundTruthSample]:
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=self.shm_name)
                _untrack(self.shm)
            except FileNotFoundError:
                return None
        size = ctypes.sizeof(GroundTruthHeader)
        for _ in range(3):
            header = GroundTruthHeader.from_buffer_copy(bytes(self.shm.buf[:size]))
            if (
                header.magic != GROUND_TRUTH_SHM_MAGIC
                or header.version != GROUND_TRUTH_SHM_VERSION
                or header.header_size != size
                or header.sequence % 2 != 0
                or header.sequence <= self.last_sequence
                or header.timestamp_ns <= 0
            ):
                return None
            confirmed = GroundTruthHeader.from_buffer_copy(bytes(self.shm.buf[:size]))
            if confirmed.sequence != header.sequence or confirmed.sequence % 2:
                continue
            position = np.array(header.position, dtype=np.float64)
            quaternion = np.array(header.quaternion_wxyz, dtype=np.float64)
            if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
                return None
            self.last_sequence = header.sequence
            return GroundTruthSample(header.sequence, header.timestamp_ns, position, quaternion)
        return None

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None
