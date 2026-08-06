#!/usr/bin/env python3
"""Record fresh IsaacLab ground-truth poses for offline validation only."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.ground_truth_shared_memory_utils import GroundTruthReader, GroundTruthSample


class TimestampGuard:
    def __init__(self) -> None:
        self.last_timestamp_ns = 0

    def check(self, timestamp_ns: int) -> None:
        if timestamp_ns <= self.last_timestamp_ns:
            raise ValueError(
                f"ground-truth timestamp did not increase: {timestamp_ns} <= {self.last_timestamp_ns}"
            )
        self.last_timestamp_ns = timestamp_ns


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record real IsaacLab root poses from shared memory.")
    parser.add_argument("output", type=Path)
    parser.add_argument("--poll-rate", type=positive_float, default=200.0)
    parser.add_argument("--duration", type=nonnegative_float, default=0.0, help="Seconds; zero records until interrupted.")
    return parser.parse_args(argv)


def format_sample(sample: GroundTruthSample) -> str:
    position = sample.position
    quaternion = sample.quaternion_wxyz
    timestamp_s = sample.timestamp_ns * 1e-9
    return (
        f"{timestamp_s:.9f} "
        f"{position[0]:.9f} {position[1]:.9f} {position[2]:.9f} "
        f"{quaternion[1]:.9f} {quaternion[2]:.9f} {quaternion[3]:.9f} {quaternion[0]:.9f}"
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    reader = GroundTruthReader()
    guard = TimestampGuard()
    period = 1.0 / args.poll_rate
    started = time.monotonic()
    count = 0
    try:
        with args.output.open("w", encoding="utf-8", buffering=1) as output:
            while args.duration == 0.0 or time.monotonic() - started < args.duration:
                sample = reader.read_pose()
                if sample is not None:
                    try:
                        guard.check(sample.timestamp_ns)
                    except ValueError as exc:
                        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
                        return 2
                    output.write(format_sample(sample) + "\n")
                    count += 1
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
    print(f"recorded_ground_truth_poses={count} output={args.output}", flush=True)
    return 0 if count >= 2 else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
