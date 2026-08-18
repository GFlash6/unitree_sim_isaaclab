from multiprocessing import shared_memory
from uuid import uuid4

import numpy as np

from tools import shared_memory_utils


def test_float32_depth_round_trip(monkeypatch):
    prefix = f"holoagent_test_{uuid4().hex}"
    monkeypatch.setattr(
        shared_memory_utils, "get_shm_name", lambda name: f"{prefix}_{name}")
    writer = shared_memory_utils.MultiImageWriter()
    reader = shared_memory_utils.MultiImageReader()
    depth = np.linspace(0.1, 4.0, 480 * 640, dtype=np.float32).reshape(480, 640)

    try:
        assert writer.write_images({"head_depth": depth})
        received = reader.read_single_image("head_depth")
        assert received is not None
        assert received.dtype == np.float32
        np.testing.assert_array_equal(received, depth)
    finally:
        reader.close()
        writer.close()
        try:
            shm = shared_memory.SharedMemory(name=f"{prefix}_head_depth", track=False)
            shm.unlink()
            shm.close()
        except TypeError:
            # The read handle was deliberately removed from Python 3.10's
            # process-wide resource tracker, so unlink directly in this test.
            from multiprocessing import shared_memory as shm_module

            shm_module._posixshmem.shm_unlink(f"/{prefix}_head_depth")
