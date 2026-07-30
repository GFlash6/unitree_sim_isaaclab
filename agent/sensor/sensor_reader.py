#!/usr/bin/env python3
"""
Read Unitree sim robot sensor data from DDS.

Subscribed topics:
  - rt/lowstate       unitree_hg/LowState_       joint state and IMU
  - rt/sim_state      std_msgs/String_ JSON      simulator state
  - rt/rewards_state  std_msgs/String_ JSON      reward/debug state
"""

from __future__ import annotations

import argparse
import ast
import json
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

ChannelFactoryInitialize: Any = None
ChannelSubscriber: Any = None
LowState_: Any = None
String_: Any = None


@dataclass(frozen=True)
class SensorSnapshot:
    lowstate: dict[str, Any]
    sim_state: dict[str, Any]
    rewards_state: dict[str, Any]
    age: dict[str, Optional[float]]


@dataclass
class SensorState:
    lowstate: dict[str, Any] = field(default_factory=dict)
    sim_state: dict[str, Any] = field(default_factory=dict)
    rewards_state: dict[str, Any] = field(default_factory=dict)
    last_lowstate_at: float = 0.0
    last_sim_state_at: float = 0.0
    last_rewards_state_at: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_lowstate(self, msg: Any) -> None:
        with self.lock:
            self.lowstate = lowstate_to_dict(msg)
            self.last_lowstate_at = time.monotonic()

    def update_sim_state(self, msg: Any) -> None:
        with self.lock:
            self.sim_state = parse_json_payload(getattr(msg, "data", ""))
            self.last_sim_state_at = time.monotonic()

    def update_rewards_state(self, msg: Any) -> None:
        with self.lock:
            self.rewards_state = parse_json_payload(getattr(msg, "data", ""))
            self.last_rewards_state_at = time.monotonic()

    def snapshot(self) -> SensorSnapshot:
        now = time.monotonic()
        with self.lock:
            return SensorSnapshot(
                lowstate=dict(self.lowstate),
                sim_state=dict(self.sim_state),
                rewards_state=dict(self.rewards_state),
                age={
                    "lowstate": age_seconds(now, self.last_lowstate_at),
                    "sim_state": age_seconds(now, self.last_sim_state_at),
                    "rewards_state": age_seconds(now, self.last_rewards_state_at),
                },
            )


class SensorReader:
    def __init__(self, queue_size: int = 10) -> None:
        load_unitree_sdk()
        self.state = SensorState()

        ChannelFactoryInitialize(1)
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sim_state_sub = ChannelSubscriber("rt/sim_state", String_)
        self.rewards_sub = ChannelSubscriber("rt/rewards_state", String_)

        self.lowstate_sub.Init(self.state.update_lowstate, queue_size)
        self.sim_state_sub.Init(self.state.update_sim_state, queue_size)
        self.rewards_sub.Init(self.state.update_rewards_state, queue_size)

    def snapshot(self) -> SensorSnapshot:
        return self.state.snapshot()


def lowstate_to_dict(msg: Any) -> dict[str, Any]:
    imu = getattr(msg, "imu_state", None)
    motor_state = list(getattr(msg, "motor_state", []) or [])
    return {
        "tick": int(getattr(msg, "tick", 0)),
        "mode_machine": int(getattr(msg, "mode_machine", 0)),
        "imu": {
            "quaternion_xyzw": float_list(getattr(imu, "quaternion", [])),
            "accelerometer": float_list(getattr(imu, "accelerometer", [])),
            "gyroscope": float_list(getattr(imu, "gyroscope", [])),
            "rpy": float_list(getattr(imu, "rpy", [])),
        },
        "joints": {
            "q": [float(getattr(m, "q", 0.0)) for m in motor_state],
            "dq": [float(getattr(m, "dq", 0.0)) for m in motor_state],
            "tau_est": [float(getattr(m, "tau_est", 0.0)) for m in motor_state],
        },
    }


def parse_json_payload(data: str) -> dict[str, Any]:
    if not data:
        return {}
    try:
        value = json.loads(data)
    except json.JSONDecodeError:
        try:
            value = ast.literal_eval(data)
        except (SyntaxError, ValueError):
            return {"raw": data}
    return value if isinstance(value, dict) else {"value": value}


def snapshot_to_dict(snapshot: SensorSnapshot) -> dict[str, Any]:
    return {
        "lowstate": snapshot.lowstate,
        "sim_state": snapshot.sim_state,
        "rewards_state": snapshot.rewards_state,
        "age": snapshot.age,
    }


def float_list(values: Any) -> list[float]:
    return [float(v) for v in (values or [])]


def age_seconds(now: float, timestamp: float) -> Optional[float]:
    if timestamp <= 0.0:
        return None
    return now - timestamp


def load_unitree_sdk() -> None:
    global ChannelFactoryInitialize
    global ChannelSubscriber
    global LowState_
    global String_

    if ChannelFactoryInitialize is not None:
        return

    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize as _ChannelFactoryInitialize,
            ChannelSubscriber as _ChannelSubscriber,
        )
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_ as _String
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as _LowState
    except ImportError as exc:  # pragma: no cover - depends on Unitree runtime.
        raise SystemExit(
            "unitree_sdk2py is not importable. Run inside the Unitree sim environment, "
            "for example: conda activate unitree_sim_env"
        ) from exc

    ChannelFactoryInitialize = _ChannelFactoryInitialize
    ChannelSubscriber = _ChannelSubscriber
    LowState_ = _LowState
    String_ = _String


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read robot sensor data from Unitree sim DDS.")
    parser.add_argument("--interval", type=positive_float, default=0.2, help="Print interval in seconds.")
    parser.add_argument("--duration", type=positive_float, default=None, help="Run seconds; omit for unlimited.")
    parser.add_argument("--once", action="store_true", help="Print one snapshot after the first lowstate arrives.")
    parser.add_argument("--timeout", type=positive_float, default=10.0, help="Wait seconds for --once.")
    parser.add_argument("--jsonl", type=Path, default=None, help="Optional JSONL output path.")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    reader = SensorReader()
    stop_event = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    output = args.jsonl.open("a", encoding="utf-8") if args.jsonl else None
    start = time.monotonic()
    try:
        while not stop_event.is_set():
            snapshot = reader.snapshot()
            if not args.once or snapshot.lowstate:
                line = json.dumps(snapshot_to_dict(snapshot), ensure_ascii=False)
                print(line, flush=True)
                if output:
                    output.write(line + "\n")
                    output.flush()
                if args.once:
                    return 0

            if args.duration is not None and time.monotonic() - start >= args.duration:
                return 0
            if args.once and time.monotonic() - start >= args.timeout:
                print("lowstate not received before timeout", file=sys.stderr)
                return 1
            time.sleep(args.interval)
    finally:
        if output:
            output.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
