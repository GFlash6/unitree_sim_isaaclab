# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Shared memory helpers for MID360 point clouds.
"""

from __future__ import annotations

import ctypes
import time
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
from typing import Optional

import numpy as np


MID360_SHM_NAME = "isaac_mid360_points_shm"
MID360_MAX_POINTS = 20000
MID360_SHM_MAGIC = 0x4D49443336305348  # "MID360SH"
MID360_SHM_VERSION = 2


def _untrack_shared_memory(shm: shared_memory.SharedMemory) -> None:
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


class PointCloudHeader(ctypes.LittleEndianStructure):
    _fields_ = [
        ("magic", ctypes.c_uint64),
        ("version", ctypes.c_uint32),
        ("header_size", ctypes.c_uint32),
        ("sequence", ctypes.c_uint64),
        ("timestamp_ns", ctypes.c_uint64),
        ("point_count", ctypes.c_uint32),
        ("channels", ctypes.c_uint32),
        ("data_size", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
    ]


class PointCloudWriter:
    def __init__(self, shm_name: str = MID360_SHM_NAME, max_points: int = MID360_MAX_POINTS):
        self.shm_name = shm_name
        self.max_points = int(max_points)
        self._sequence = 0
        self._last_timestamp_ns = 0
        required_size = ctypes.sizeof(PointCloudHeader) + self.max_points * 3 * 4
        try:
            self.shm = shared_memory.SharedMemory(name=shm_name)
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(create=True, size=required_size, name=shm_name)
        _untrack_shared_memory(self.shm)
        if self.shm.size < required_size:
            actual_size = self.shm.size
            self.shm.close()
            raise ValueError(
                f"shared memory {shm_name!r} is {actual_size} bytes; "
                f"at least {required_size} bytes are required"
            )
        header_size = ctypes.sizeof(PointCloudHeader)
        existing = PointCloudHeader.from_buffer_copy(bytes(self.shm.buf[:header_size]))
        if (
            existing.magic == MID360_SHM_MAGIC
            and existing.version == MID360_SHM_VERSION
            and existing.sequence % 2 == 0
        ):
            self._sequence = existing.sequence

    def write_points(self, points: np.ndarray, timestamp_ns: int | None = None) -> bool:
        source_timestamp_ns = time.time_ns() if timestamp_ns is None else int(timestamp_ns)
        if source_timestamp_ns <= self._last_timestamp_ns:
            return False
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            return False
        points = np.ascontiguousarray(points[: self.max_points])
        payload = points.tobytes()
        self._sequence += 2
        header = PointCloudHeader(
            magic=MID360_SHM_MAGIC,
            version=MID360_SHM_VERSION,
            header_size=ctypes.sizeof(PointCloudHeader),
            sequence=self._sequence,
            timestamp_ns=source_timestamp_ns,
            point_count=points.shape[0],
            channels=3,
            data_size=len(payload),
        )
        header_size = ctypes.sizeof(PointCloudHeader)
        if header_size + len(payload) > self.shm.size:
            return False
        writing_header = PointCloudHeader.from_buffer_copy(header)
        writing_header.sequence -= 1
        self.shm.buf[:header_size] = ctypes.string_at(
            ctypes.byref(writing_header), header_size
        )
        self.shm.buf[header_size : header_size + len(payload)] = payload
        self.shm.buf[:header_size] = ctypes.string_at(ctypes.byref(header), header_size)
        self._last_timestamp_ns = source_timestamp_ns
        return True

    def close(self) -> None:
        self.shm.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


class PointCloudReader:
    def __init__(self, shm_name: str = MID360_SHM_NAME):
        self.shm_name = shm_name
        self.shm: Optional[shared_memory.SharedMemory] = None
        self.last_sequence = 0
        self.last_timestamp_ns = 0

    def read_points(self) -> Optional[np.ndarray]:
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=self.shm_name)
                _untrack_shared_memory(self.shm)
            except FileNotFoundError:
                return None

        expected_header_size = ctypes.sizeof(PointCloudHeader)
        for _ in range(3):
            header = PointCloudHeader.from_buffer_copy(
                bytes(self.shm.buf[:expected_header_size])
            )
            if (
                header.magic != MID360_SHM_MAGIC
                or header.version != MID360_SHM_VERSION
                or header.header_size != expected_header_size
                or header.sequence % 2 != 0
                or header.sequence <= self.last_sequence
                or header.point_count == 0
                or header.channels != 3
                or header.data_size != header.point_count * header.channels * 4
            ):
                return None
            start = expected_header_size
            end = start + header.data_size
            if end > self.shm.size:
                return None
            payload = bytes(self.shm.buf[start:end])
            confirmed = PointCloudHeader.from_buffer_copy(
                bytes(self.shm.buf[:expected_header_size])
            )
            if confirmed.sequence != header.sequence or confirmed.sequence % 2:
                continue
            data = np.frombuffer(payload, dtype=np.float32).reshape(
                header.point_count, header.channels
            )
            self.last_sequence = header.sequence
            self.last_timestamp_ns = header.timestamp_ns
            return data
        return None

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None
