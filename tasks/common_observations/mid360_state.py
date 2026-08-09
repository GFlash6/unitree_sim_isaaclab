# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
MID360 point cloud observation and shared-memory export.
"""

from __future__ import annotations

import os

from typing import TYPE_CHECKING

import torch
from isaaclab.utils.math import quat_apply_inverse

from tools.pointcloud_shared_memory_utils import PointCloudWriter

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


_writer = PointCloudWriter()
_return_placeholder = None
_debug_counter = 0


def get_mid360_points(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Read MID360 ray hits and publish finite sensor-frame points to shared memory."""
    global _return_placeholder, _debug_counter
    if _return_placeholder is None:
        _return_placeholder = torch.zeros((1, 1), device=env.device)

    if "mid360" not in env.scene.keys():
        if os.getenv("MID360_DEBUG"):
            print("[mid360] sensor 'mid360' not found in scene", flush=True)
        return _return_placeholder

    sensor = env.scene["mid360"]
    try:
        sensor.update(getattr(env, "physics_dt", 0.02), force_recompute=True)
    except Exception as exc:
        if os.getenv("MID360_DEBUG"):
            print(f"[mid360] sensor update failed: {exc}", flush=True)

    points_w = sensor.data.ray_hits_w[0]
    finite_mask = torch.isfinite(points_w).all(dim=-1)
    finite_points_w = points_w[finite_mask]
    sensor_pos_w = sensor.data.pos_w[0].unsqueeze(0)
    sensor_quat_w = sensor.data.quat_w[0].unsqueeze(0).expand(finite_points_w.shape[0], -1)
    finite_points = quat_apply_inverse(sensor_quat_w, finite_points_w - sensor_pos_w)
    if finite_points.numel() > 0:
        sim_time_ns = int(float(env.sim.current_time) * 1_000_000_000)
        _writer.write_points(
            finite_points.detach().cpu().numpy(), timestamp_ns=sim_time_ns
        )
    if os.getenv("MID360_DEBUG"):
        _debug_counter += 1
        if _debug_counter == 1 or _debug_counter % 50 == 0:
            print(
                f"[mid360] call={_debug_counter} finite={finite_points.shape[0]} total={points_w.shape[0]}",
                flush=True,
            )

    return _return_placeholder
