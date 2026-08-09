#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import torch
from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Run a real IsaacLab + MID360 smoke test without sim_main process cleanup.")
parser.add_argument("--task", default="Isaac-Move-Cylinder-G129-Dex1-Wholebody")
parser.add_argument("--steps", type=int, default=80)
parser.add_argument("--read-timeout", type=float, default=10.0)
parser.add_argument("--print-stats", action="store_true", help="Print real MID360 point cloud statistics.")
AppLauncher.add_app_launcher_args(parser)


def main() -> int:
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.environ["PROJECT_ROOT"] = project_root
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import gymnasium as gym
    import tasks  # noqa: F401
    from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
    from tasks.common_observations.mid360_state import get_mid360_points
    from tools.pointcloud_shared_memory_utils import PointCloudReader

    env = None
    reader = PointCloudReader()
    try:
        print(f"[smoke] creating task={args.task} device={args.device}", flush=True)
        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
        env_cfg.env_name = args.task
        env = gym.make(args.task, cfg=env_cfg).unwrapped
        env.seed(42)
        sensors = list(getattr(env.scene, "sensors", {}).keys())
        print(f"[smoke] sensors={sensors}", flush=True)
        if "mid360" not in env.scene.keys():
            raise RuntimeError("mid360 sensor is not present in the scene")
        sensor = env.scene["mid360"]
        print(f"[smoke] mid360_type={sensor.__class__.__name__}", flush=True)
        print(f"[smoke] mid360_mesh_targets={sensor.cfg.mesh_prim_paths}", flush=True)

        env.sim.reset()
        env.reset()
        action_shape = tuple(env.action_space.shape)
        print(f"[smoke] action_space={action_shape}", flush=True)
        action = torch.zeros((env.num_envs, action_shape[-1]), device=env.device)

        for step in range(max(1, args.steps)):
            env.step(action)
            get_mid360_points(env)
            points = reader.read_points()
            if points is not None and points.shape[0] > 0:
                print(f"[smoke] mid360_points={points.shape[0]} step={step}", flush=True)
                if args.print_stats:
                    print(_point_stats(points), flush=True)
                return 0
            if not simulation_app.is_running():
                raise RuntimeError("simulation app stopped before MID360 points were produced")

        deadline = time.monotonic() + args.read_timeout
        while time.monotonic() < deadline:
            points = reader.read_points()
            if points is not None and points.shape[0] > 0:
                print(f"[smoke] mid360_points={points.shape[0]}", flush=True)
                if args.print_stats:
                    print(_point_stats(points), flush=True)
                return 0
            time.sleep(0.1)
        raise RuntimeError("MID360 shared-memory point cloud was not produced before timeout")
    except BaseException as exc:
        print(f"[smoke] exception={type(exc).__name__}: {exc!r}", flush=True)
        raise
    finally:
        reader.close()
        if env is not None:
            env.close()
        simulation_app.close()


def _point_stats(points: np.ndarray) -> str:
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    means = np.mean(points, axis=0)
    return (
        "[smoke] point_stats="
        f"min={mins.tolist()} max={maxs.tolist()} mean={means.tolist()} "
        f"z_span={float(maxs[2] - mins[2]):.6f}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
