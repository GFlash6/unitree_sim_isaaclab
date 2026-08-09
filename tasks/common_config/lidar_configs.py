# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Reusable lidar configurations.
"""

import os

from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass


def _mesh_prim_paths(fallback: str | None = None) -> list[str | MultiMeshRayCasterCfg.RaycastTargetCfg]:
    paths = os.getenv("MID360_MESH_PRIM_PATHS", "").strip()
    if paths:
        parsed = [path.strip() for path in paths.split(",") if path.strip()]
        return parsed or _default_mesh_prim_paths()
    path = os.getenv("MID360_MESH_PRIM_PATH", "").strip()
    if path:
        return [path]
    if fallback:
        return [fallback]
    return _default_mesh_prim_paths()


def _default_mesh_prim_paths() -> list[MultiMeshRayCasterCfg.RaycastTargetCfg]:
    target = MultiMeshRayCasterCfg.RaycastTargetCfg
    return [
        target(prim_expr="{ENV_REGEX_NS}/Room", is_shared=True, track_mesh_transforms=False),
        target(prim_expr="{ENV_REGEX_NS}/PackingTable_1", is_shared=True, track_mesh_transforms=False),
        target(prim_expr="{ENV_REGEX_NS}/PackingTable_2", is_shared=True, track_mesh_transforms=False),
        target(prim_expr="{ENV_REGEX_NS}/Object", track_mesh_transforms=True),
    ]


@configclass
class LidarPresets:
    """Lidar sensor presets."""

    @classmethod
    def g1_mid360(cls, mesh_prim_path: str | None = None) -> MultiMeshRayCasterCfg:
        """MID360-style ray caster mounted on the G1 head."""
        return cls._mid360("/World/envs/env_.*/Robot/mid360_link", mesh_prim_path=mesh_prim_path)

    @classmethod
    def h12_mid360(cls, mesh_prim_path: str | None = None) -> MultiMeshRayCasterCfg:
        """MID360-style ray caster mounted on the H1-2 head."""
        return cls._mid360("/World/envs/env_.*/Robot/camera_link/mid360", mesh_prim_path=mesh_prim_path)

    @classmethod
    def _mid360(cls, prim_path: str, mesh_prim_path: str | None = None) -> MultiMeshRayCasterCfg:
        return MultiMeshRayCasterCfg(
            prim_path=prim_path,
            update_period=0.05,
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
            mesh_prim_paths=_mesh_prim_paths(mesh_prim_path),
            ray_alignment="yaw",
            pattern_cfg=patterns.LidarPatternCfg(
                channels=32,
                vertical_fov_range=(-52.0, 7.0),
                horizontal_fov_range=(-180.0, 180.0),
                horizontal_res=1.0,
            ),
            debug_vis=False,
        )
