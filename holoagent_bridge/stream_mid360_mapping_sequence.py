#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import torch
from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Stream real IsaacLab MID360 raycasts while moving the robot root pose.")
parser.add_argument("--task", default="Isaac-Move-Cylinder-G129-Dex1-Wholebody")
parser.add_argument("--steps", type=int, default=300)
parser.add_argument("--rate", type=float, default=5.0)
parser.add_argument("--x-distance", type=float, default=1.5)
parser.add_argument("--y-amplitude", type=float, default=0.25)
parser.add_argument("--yaw-amplitude", type=float, default=0.35)
parser.add_argument("--warmup-steps", type=int, default=20)
parser.add_argument("--print-every", type=int, default=25)
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
    from tools.ground_truth_shared_memory_utils import GroundTruthWriter
    from tools.pointcloud_shared_memory_utils import PointCloudReader

    env = None
    reader = PointCloudReader()
    ground_truth_writer = GroundTruthWriter()
    try:
        if args.steps <= 0:
            raise ValueError("--steps must be positive")
        if args.rate <= 0:
            raise ValueError("--rate must be positive")

        print(f"[stream] creating task={args.task} device={args.device}", flush=True)
        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
        env_cfg.env_name = args.task
        env = gym.make(args.task, cfg=env_cfg).unwrapped
        env.seed(42)
        env.sim.reset()
        env.reset()

        if "mid360" not in env.scene.keys():
            raise RuntimeError("mid360 sensor is not present in the scene")
        if "robot" not in env.scene.keys():
            raise RuntimeError("robot articulation is not present in the scene")

        sensor = env.scene["mid360"]
        robot = env.scene["robot"]
        print(f"[stream] mid360_type={sensor.__class__.__name__}", flush=True)
        print(f"[stream] mid360_mesh_targets={sensor.cfg.mesh_prim_paths}", flush=True)

        action = torch.zeros((env.num_envs, env.action_space.shape[-1]), device=env.device)
        for _ in range(max(0, args.warmup_steps)):
            env.step(action)

        base_pose = robot.data.root_link_pose_w.clone()
        base_z = float(base_pose[0, 2].item())
        period = 1.0 / args.rate
        last_points = None
        started = time.monotonic()
        for step in range(args.steps):
            if not simulation_app.is_running():
                raise RuntimeError("simulation app stopped while streaming")

            pose = pose_for_step(base_pose, step, args.steps, args.x_distance, args.y_amplitude, args.yaw_amplitude)
            robot.write_root_pose_to_sim(pose)
            robot.write_root_velocity_to_sim(torch.zeros((env.num_envs, 6), device=env.device))
            env.scene.write_data_to_sim()
            env.sim.step(render=False)
            env.scene.update(dt=getattr(env, "physics_dt", 0.02))
            get_mid360_points(env)
            observed_pose = robot.data.root_link_pose_w[0]
            observed_pose_sample = observed_pose.contiguous().cpu().numpy()
            ground_truth_writer.write_pose(
                int(float(env.sim.current_time) * 1_000_000_000),
                observed_pose_sample[:3],
                observed_pose_sample[3:7],
            )

            points = reader.read_points()
            if points is not None and points.shape[0] > 0:
                last_points = points
            if step == 0 or (step + 1) % max(1, args.print_every) == 0:
                xyz = pose[0, :3].detach().cpu().tolist()
                yaw = yaw_for_step(step, args.steps, args.yaw_amplitude)
                point_text = "none" if last_points is None else str(last_points.shape[0])
                print(
                    f"[stream] step={step + 1}/{args.steps} pose_xyz={xyz} base_z={base_z:.3f} "
                    f"yaw={yaw:.4f} points={point_text}",
                    flush=True,
                )
            sleep_until = started + (step + 1) * period
            delay = sleep_until - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        if last_points is None:
            raise RuntimeError("no MID360 shared-memory point cloud was produced")
        print(f"[stream] done points={last_points.shape[0]} stats={point_stats(last_points)}", flush=True)
        return 0
    finally:
        ground_truth_writer.close()
        reader.close()
        if env is not None:
            env.close()
        simulation_app.close()


def pose_for_step(
    base_pose: torch.Tensor,
    step: int,
    total_steps: int,
    x_distance: float,
    y_amplitude: float,
    yaw_amplitude: float,
) -> torch.Tensor:
    progress = 0.0 if total_steps <= 1 else step / float(total_steps - 1)
    centered = progress - 0.5
    yaw = yaw_for_step(step, total_steps, yaw_amplitude)
    pose = base_pose.clone()
    pose[:, 0] = base_pose[:, 0] + centered * x_distance
    pose[:, 1] = base_pose[:, 1] + math.sin(progress * 2.0 * math.pi) * y_amplitude
    pose[:, 3] = math.cos(yaw * 0.5)
    pose[:, 4] = 0.0
    pose[:, 5] = 0.0
    pose[:, 6] = math.sin(yaw * 0.5)
    return pose


def yaw_for_step(step: int, total_steps: int, yaw_amplitude: float) -> float:
    progress = 0.0 if total_steps <= 1 else step / float(total_steps - 1)
    return math.sin(progress * 2.0 * math.pi) * yaw_amplitude


def point_stats(points: np.ndarray) -> str:
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    span = maxs - mins
    return f"min={mins.tolist()} max={maxs.tolist()} span={span.tolist()}"


if __name__ == "__main__":
    raise SystemExit(main())
