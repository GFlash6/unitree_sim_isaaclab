#!/usr/bin/env python3
"""
Free-exploration agent for Unitree G1 in unitree_sim_isaaclab.

The agent uses the simulator's high-level Wholebody DDS command interface:
  - publish:   rt/run_command/cmd       std_msgs/String_ "[x, y, yaw, height]"
  - subscribe: rt/lowstate              unitree_hg/LowState_
  - subscribe: rt/sim_state             std_msgs/String_ JSON
  - subscribe: rt/rewards_state         std_msgs/String_ JSON

It intentionally avoids rt/lowcmd joint control because that path needs CRC and
has a larger safety surface. Start the simulator with a Wholebody task, for
example Isaac-Move-Cylinder-G129-Dex1-Wholebody.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import random
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

ChannelFactoryInitialize: Any = None
ChannelPublisher: Any = None
ChannelSubscriber: Any = None
String_: Any = None
MotorCmds_: Any = None
LowState_: Any = None
unitree_go_msg_dds__MotorCmd_: Any = None


Command = tuple[float, float, float, float]


@dataclass
class AgentConfig:
    duration: Optional[float]
    tick_hz: float
    segment_min: float
    segment_max: float
    x_min: float
    x_max: float
    y_abs: float
    yaw_abs: float
    height: float
    seed: Optional[int]
    stuck_window: float
    progress_epsilon: float
    reset_on_fall: bool
    enable_grippers: bool
    gripper_period: float
    verbose: bool


@dataclass
class SharedState:
    lowstate: Optional[Any] = None
    sim_state: dict[str, Any] = field(default_factory=dict)
    rewards_state: dict[str, Any] = field(default_factory=dict)
    last_lowstate_at: float = 0.0
    last_sim_state_at: float = 0.0
    last_rewards_state_at: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_lowstate(self, msg: Any) -> None:
        with self.lock:
            self.lowstate = msg
            self.last_lowstate_at = time.monotonic()

    def update_sim_state(self, msg: Any) -> None:
        parsed = parse_json_payload(getattr(msg, "data", ""))
        with self.lock:
            self.sim_state = parsed
            self.last_sim_state_at = time.monotonic()

    def update_rewards_state(self, msg: Any) -> None:
        parsed = parse_json_payload(getattr(msg, "data", ""))
        with self.lock:
            self.rewards_state = parsed
            self.last_rewards_state_at = time.monotonic()

    def snapshot(self) -> "StateSnapshot":
        with self.lock:
            return StateSnapshot(
                lowstate=self.lowstate,
                sim_state=dict(self.sim_state),
                rewards_state=dict(self.rewards_state),
                last_lowstate_at=self.last_lowstate_at,
                last_sim_state_at=self.last_sim_state_at,
                last_rewards_state_at=self.last_rewards_state_at,
            )


@dataclass(frozen=True)
class StateSnapshot:
    lowstate: Optional[Any]
    sim_state: dict[str, Any]
    rewards_state: dict[str, Any]
    last_lowstate_at: float
    last_sim_state_at: float
    last_rewards_state_at: float


class G1FreeExploreAgent:
    def __init__(self, config: AgentConfig) -> None:
        load_unitree_sdk()
        self.config = config
        self.state = SharedState()
        self.rng = random.Random(config.seed)
        self.stop_event = threading.Event()
        self.current_cmd: Command = (0.0, 0.0, 0.0, config.height)
        self.next_segment_at = 0.0
        self.last_gripper_at = 0.0
        self.gripper_open = False
        self.best_score: Optional[float] = None
        self.last_progress_at = time.monotonic()

        ChannelFactoryInitialize(1)

        self.move_pub = ChannelPublisher("rt/run_command/cmd", String_)
        self.reset_pub = ChannelPublisher("rt/reset_pose/cmd", String_)
        self.left_gripper_pub = ChannelPublisher("rt/dex1/left/cmd", MotorCmds_)
        self.right_gripper_pub = ChannelPublisher("rt/dex1/right/cmd", MotorCmds_)
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sim_state_sub = ChannelSubscriber("rt/sim_state", String_)
        self.rewards_sub = ChannelSubscriber("rt/rewards_state", String_)

        self.move_pub.Init()
        self.reset_pub.Init()
        self.left_gripper_pub.Init()
        self.right_gripper_pub.Init()
        self.lowstate_sub.Init(self.state.update_lowstate, 10)
        self.sim_state_sub.Init(self.state.update_sim_state, 10)
        self.rewards_sub.Init(self.state.update_rewards_state, 10)

    def run(self) -> None:
        signal.signal(signal.SIGINT, self._request_stop)
        signal.signal(signal.SIGTERM, self._request_stop)

        start = time.monotonic()
        tick_sleep = 1.0 / max(self.config.tick_hz, 1.0)
        self.wait_for_lowstate(timeout=10.0)
        self.schedule_next_segment(force=True)

        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                if self.config.duration is not None and now - start >= self.config.duration:
                    break

                snap = self.state.snapshot()
                if self.config.reset_on_fall and looks_fallen(snap.lowstate):
                    self.log("fall detected; stopping and resetting pose")
                    self.stop_motion()
                    self.reset_pose()
                    time.sleep(1.0)
                    self.schedule_next_segment(force=True)
                    continue

                self.update_progress(snap, now)
                if now >= self.next_segment_at or self.is_stuck(now):
                    self.schedule_next_segment(force=True)

                self.publish_motion(self.current_cmd)
                self.maybe_move_grippers(now)
                time.sleep(tick_sleep)
        finally:
            self.stop_motion()
            if self.config.enable_grippers:
                self.write_grippers(open_gripper=True)
            self.log("agent stopped")

    def wait_for_lowstate(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.stop_event.is_set():
            if self.state.snapshot().lowstate is not None:
                self.log("received lowstate")
                return
            time.sleep(0.05)
        self.log("lowstate not received yet; continuing with command-only exploration")

    def schedule_next_segment(self, force: bool = False) -> None:
        if not force:
            return
        self.current_cmd = self.pick_command()
        segment = self.rng.uniform(self.config.segment_min, self.config.segment_max)
        self.next_segment_at = time.monotonic() + segment
        self.log(f"cmd={format_cmd(self.current_cmd)} segment={segment:.1f}s")

    def pick_command(self) -> Command:
        choices = [
            self.forward_arc,
            self.forward_arc,
            self.scan_turn,
            self.side_step,
            self.short_reverse,
        ]
        return self.rng.choice(choices)()

    def forward_arc(self) -> Command:
        x = self.rng.uniform(max(0.05, self.config.x_min), self.config.x_max)
        y = self.rng.uniform(-0.15, 0.15)
        yaw = self.rng.uniform(-0.45, 0.45)
        return self.clamp_command((x, y, yaw, self.config.height))

    def scan_turn(self) -> Command:
        yaw = self.rng.choice([-1.0, 1.0]) * self.rng.uniform(0.45, self.config.yaw_abs)
        return self.clamp_command((0.0, 0.0, yaw, self.config.height))

    def side_step(self) -> Command:
        y = self.rng.choice([-1.0, 1.0]) * self.rng.uniform(0.12, self.config.y_abs)
        yaw = self.rng.uniform(-0.25, 0.25)
        return self.clamp_command((0.05, y, yaw, self.config.height))

    def short_reverse(self) -> Command:
        x = self.rng.uniform(self.config.x_min, min(-0.05, self.config.x_max))
        yaw = self.rng.uniform(-0.35, 0.35)
        return self.clamp_command((x, 0.0, yaw, self.config.height))

    def clamp_command(self, cmd: Command) -> Command:
        x, y, yaw, height = cmd
        return (
            clamp(x, self.config.x_min, self.config.x_max),
            clamp(y, -self.config.y_abs, self.config.y_abs),
            clamp(yaw, -self.config.yaw_abs, self.config.yaw_abs),
            height,
        )

    def update_progress(self, snap: StateSnapshot, now: float) -> None:
        score = extract_reward_score(snap.rewards_state)
        if score is None:
            return
        if self.best_score is None or score > self.best_score + self.config.progress_epsilon:
            self.best_score = score
            self.last_progress_at = now
            self.log(f"reward progress score={score:.4f}")

    def is_stuck(self, now: float) -> bool:
        if self.best_score is None:
            return False
        if now - self.last_progress_at < self.config.stuck_window:
            return False
        self.log("no reward progress; changing primitive")
        self.last_progress_at = now
        return True

    def maybe_move_grippers(self, now: float) -> None:
        if not self.config.enable_grippers:
            return
        if now - self.last_gripper_at < self.config.gripper_period:
            return
        self.last_gripper_at = now
        self.gripper_open = not self.gripper_open
        self.write_grippers(open_gripper=self.gripper_open)

    def write_grippers(self, open_gripper: bool) -> None:
        q = 5.4 if open_gripper else 0.0
        msg = make_gripper_cmd(q)
        self.left_gripper_pub.Write(msg)
        self.right_gripper_pub.Write(msg)
        self.log(f"grippers={'open' if open_gripper else 'closed'}")

    def publish_motion(self, cmd: Command) -> None:
        self.move_pub.Write(String_(data=format_cmd(cmd)))

    def stop_motion(self) -> None:
        self.publish_motion((0.0, 0.0, 0.0, self.config.height))

    def reset_pose(self) -> None:
        self.reset_pub.Write(String_(data="1"))

    def _request_stop(self, _signum: int, _frame: object) -> None:
        self.stop_event.set()

    def log(self, msg: str) -> None:
        if self.config.verbose:
            print(f"[g1-free-explore] {msg}", flush=True)


def make_gripper_cmd(q: float) -> Any:
    msg = MotorCmds_()
    cmd = unitree_go_msg_dds__MotorCmd_()
    cmd.mode = 1
    cmd.q = q
    cmd.dq = 0.0
    cmd.tau = 0.0
    cmd.kp = 1.0
    cmd.kd = 0.0
    msg.cmds.append(cmd)
    return msg


def load_unitree_sdk() -> None:
    global ChannelFactoryInitialize
    global ChannelPublisher
    global ChannelSubscriber
    global String_
    global MotorCmds_
    global LowState_
    global unitree_go_msg_dds__MotorCmd_

    if ChannelFactoryInitialize is not None:
        return

    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize as _ChannelFactoryInitialize,
            ChannelPublisher as _ChannelPublisher,
            ChannelSubscriber as _ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__MotorCmd_ as _MotorCmd,
        )
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_ as _String
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_ as _MotorCmds
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as _LowState
    except ImportError as exc:  # pragma: no cover - depends on Unitree runtime.
        raise SystemExit(
            "unitree_sdk2py is not importable. Run this inside the Unitree sim "
            "environment, for example: conda activate unitree_sim_env"
        ) from exc

    ChannelFactoryInitialize = _ChannelFactoryInitialize
    ChannelPublisher = _ChannelPublisher
    ChannelSubscriber = _ChannelSubscriber
    String_ = _String
    MotorCmds_ = _MotorCmds
    LowState_ = _LowState
    unitree_go_msg_dds__MotorCmd_ = _MotorCmd


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


def extract_reward_score(rewards: dict[str, Any]) -> Optional[float]:
    if not rewards:
        return None
    preferred_keys = ("total_reward", "reward", "score", "episode_reward", "progress")
    for key in preferred_keys:
        value = rewards.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    numeric_values = [float(v) for v in rewards.values() if isinstance(v, (int, float))]
    if not numeric_values:
        return None
    return sum(numeric_values)


def looks_fallen(lowstate: Optional[Any]) -> bool:
    if lowstate is None:
        return False
    imu = getattr(lowstate, "imu_state", None)
    if imu is None:
        return False

    rpy = getattr(imu, "rpy", None)
    if rpy and len(rpy) >= 2:
        roll = abs(float(rpy[0]))
        pitch = abs(float(rpy[1]))
        return roll > 0.9 or pitch > 0.9

    quat = getattr(imu, "quaternion", None)
    if quat and len(quat) >= 4:
        roll, pitch = quat_to_roll_pitch(quat)
        return abs(roll) > 0.9 or abs(pitch) > 0.9

    return False


def quat_to_roll_pitch(quat: Any) -> tuple[float, float]:
    w, x, y, z = [float(quat[i]) for i in range(4)]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def format_cmd(cmd: Command) -> str:
    x, y, yaw, height = cmd
    return f"[{x:.3f}, {y:.3f}, {yaw:.3f}, {height:.3f}]"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> AgentConfig:
    parser = argparse.ArgumentParser(
        description="Run a free-exploration agent for Unitree G1 Wholebody simulation."
    )
    parser.add_argument("--duration", type=float, default=None, help="Run seconds; omit for unlimited.")
    parser.add_argument("--tick-hz", type=positive_float, default=20.0, help="DDS publish rate.")
    parser.add_argument("--segment-min", type=positive_float, default=1.5, help="Minimum primitive duration.")
    parser.add_argument("--segment-max", type=positive_float, default=4.0, help="Maximum primitive duration.")
    parser.add_argument("--x-min", type=float, default=-0.25, help="Minimum forward velocity.")
    parser.add_argument("--x-max", type=float, default=0.55, help="Maximum forward velocity.")
    parser.add_argument("--y-abs", type=positive_float, default=0.30, help="Absolute lateral velocity limit.")
    parser.add_argument("--yaw-abs", type=positive_float, default=0.90, help="Absolute yaw velocity limit.")
    parser.add_argument("--height", type=float, default=0.8, help="Wholebody height command.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for repeatability.")
    parser.add_argument("--stuck-window", type=positive_float, default=8.0, help="Seconds without reward progress before changing primitive.")
    parser.add_argument("--progress-epsilon", type=float, default=1e-4, help="Reward improvement threshold.")
    parser.add_argument("--no-reset-on-fall", action="store_true", help="Do not reset when IMU looks fallen.")
    parser.add_argument("--enable-grippers", action="store_true", help="Also pulse Dex1 grippers.")
    parser.add_argument("--gripper-period", type=positive_float, default=6.0, help="Seconds between gripper toggles.")
    parser.add_argument("--quiet", action="store_true", help="Reduce logs.")
    args = parser.parse_args(argv)

    if args.segment_max < args.segment_min:
        parser.error("--segment-max must be >= --segment-min")
    if args.x_max < args.x_min:
        parser.error("--x-max must be >= --x-min")

    return AgentConfig(
        duration=args.duration,
        tick_hz=args.tick_hz,
        segment_min=args.segment_min,
        segment_max=args.segment_max,
        x_min=args.x_min,
        x_max=args.x_max,
        y_abs=args.y_abs,
        yaw_abs=args.yaw_abs,
        height=args.height,
        seed=args.seed,
        stuck_window=args.stuck_window,
        progress_epsilon=args.progress_epsilon,
        reset_on_fall=not args.no_reset_on_fall,
        enable_grippers=args.enable_grippers,
        gripper_period=args.gripper_period,
        verbose=not args.quiet,
    )


def main(argv: list[str]) -> int:
    config = parse_args(argv)
    agent = G1FreeExploreAgent(config)
    agent.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
