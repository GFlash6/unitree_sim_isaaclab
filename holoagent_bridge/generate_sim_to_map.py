#!/usr/bin/env python3
"""Create a versioned map <- sim_world calibration from a simultaneous pose pair."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from holoagent_bridge.sim_map_transform import align_sim_to_map


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--robot-sim", nargs=7, type=float, required=True)
    parser.add_argument("--robot-map", nargs=7, type=float, required=True)
    parser.add_argument("--source", required=True)
    args = parser.parse_args()
    transform = align_sim_to_map(args.robot_sim, args.robot_map)
    document = {
        "version": 1,
        "parent_frame": "map",
        "child_frame": "sim_world",
        "translation": transform[:3].tolist(),
        "rotation_wxyz": transform[3:].tolist(),
        "calibration": {
            "robot_sim_pose_wxyz": args.robot_sim,
            "robot_map_pose_wxyz": args.robot_map,
            "source": args.source,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(temporary, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
