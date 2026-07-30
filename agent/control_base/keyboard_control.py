#!/usr/bin/env python3
"""
Human-as-agent controller for Unitree G1 in unitree_sim_isaaclab.

This script keeps the same DDS actuator layer as g1_base_control.py, but
the decision source is a human operator. It supports:
  - keyboard mode: press keys continuously to publish Wholebody commands
  - command mode: type atomic skill names such as forward, turn_left, open
"""

from __future__ import annotations

import argparse
import os
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from g1_base_control import (
    Command,
    String_,
    format_cmd,
    load_unitree_sdk,
    make_gripper_cmd,
)


ChannelFactoryInitialize: Any = None
ChannelPublisher: Any = None
MotorCmds_: Any = None


@dataclass
class HumanConfig:
    mode: str
    tick_hz: float
    x_speed: float
    y_speed: float
    yaw_speed: float
    height: float
    no_grippers: bool
    verbose: bool


class HumanG1Agent:
    def __init__(self, config: HumanConfig) -> None:
        bind_unitree_symbols()
        self.config = config
        self.stop_event = threading.Event()
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.current_cmd: Command = (0.0, 0.0, 0.0, config.height)
        self.gripper_open = True

        ChannelFactoryInitialize(1)
        self.move_pub = ChannelPublisher("rt/run_command/cmd", String_)
        self.reset_pub = ChannelPublisher("rt/reset_pose/cmd", String_)
        self.left_gripper_pub = ChannelPublisher("rt/dex1/left/cmd", MotorCmds_)
        self.right_gripper_pub = ChannelPublisher("rt/dex1/right/cmd", MotorCmds_)

        self.move_pub.Init()
        self.reset_pub.Init()
        self.left_gripper_pub.Init()
        self.right_gripper_pub.Init()

    def run(self) -> None:
        signal.signal(signal.SIGINT, self.request_stop)
        signal.signal(signal.SIGTERM, self.request_stop)

        if self.config.mode == "keyboard":
            self.print_keyboard_help()
            self.run_keyboard()
        else:
            self.print_command_help()
            self.run_command_prompt()

    def run_keyboard(self) -> None:
        if os.name != "nt":
            raise SystemExit("keyboard mode currently supports Windows terminals only")

        import msvcrt

        tick_sleep = 1.0 / max(self.config.tick_hz, 1.0)
        try:
            while not self.stop_event.is_set():
                while msvcrt.kbhit():
                    raw = msvcrt.getwch()
                    if raw in ("\x00", "\xe0"):
                        raw = msvcrt.getwch()
                    self.handle_key(raw.lower())
                self.publish_motion(self.current_cmd)
                time.sleep(tick_sleep)
        finally:
            self.stop_motion()

    def run_command_prompt(self) -> None:
        input_thread = threading.Thread(target=self.read_stdin_loop, daemon=True)
        input_thread.start()
        tick_sleep = 1.0 / max(self.config.tick_hz, 1.0)

        try:
            while not self.stop_event.is_set():
                self.drain_command_queue()
                self.publish_motion(self.current_cmd)
                time.sleep(tick_sleep)
        finally:
            self.stop_motion()

    def read_stdin_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = input("> ")
            except EOFError:
                self.stop_event.set()
                return
            self.command_queue.put(line.strip())

    def drain_command_queue(self) -> None:
        while True:
            try:
                line = self.command_queue.get_nowait()
            except queue.Empty:
                return
            self.handle_text_command(line)

    def handle_key(self, key: str) -> None:
        keymap = {
            "w": "forward",
            "s": "back",
            "a": "left",
            "d": "right",
            "z": "turn_left",
            "x": "turn_right",
            " ": "stop",
            "r": "reset",
            "o": "open",
            "p": "close",
            "q": "quit",
        }
        command = keymap.get(key)
        if command:
            self.handle_text_command(command)

    def handle_text_command(self, line: str) -> None:
        if not line:
            return
        parts = line.split()
        name = parts[0].lower()
        args = parts[1:]

        if name in ("q", "quit", "exit"):
            self.stop_event.set()
            return
        if name in ("help", "?"):
            self.print_command_help()
            return
        if name == "raw":
            self.current_cmd = self.parse_raw_command(args)
        elif name in COMMAND_BUILDERS:
            self.current_cmd = COMMAND_BUILDERS[name](self.config)
        elif name == "stop":
            self.current_cmd = (0.0, 0.0, 0.0, self.config.height)
        elif name == "hold":
            self.hold_current_command(args)
        elif name == "reset":
            self.stop_motion()
            self.reset_pub.Write(String_(data="1"))
            self.log("reset sent")
        elif name == "open":
            self.write_grippers(open_gripper=True)
        elif name == "close":
            self.write_grippers(open_gripper=False)
        elif name == "toggle":
            self.write_grippers(open_gripper=not self.gripper_open)
        else:
            self.log(f"unknown command: {line}")
            return

        self.log(f"motion={format_cmd(self.current_cmd)}")

    def hold_current_command(self, args: list[str]) -> None:
        if len(args) != 1:
            self.log("usage: hold <seconds>")
            return
        seconds = max(0.0, float(args[0]))
        deadline = time.monotonic() + seconds
        tick_sleep = 1.0 / max(self.config.tick_hz, 1.0)
        self.log(f"holding {format_cmd(self.current_cmd)} for {seconds:.1f}s")
        while time.monotonic() < deadline and not self.stop_event.is_set():
            self.publish_motion(self.current_cmd)
            time.sleep(tick_sleep)

    def parse_raw_command(self, args: list[str]) -> Command:
        if len(args) not in (3, 4):
            self.log("usage: raw <x_vel> <y_vel> <yaw_vel> [height]")
            return self.current_cmd
        x = float(args[0])
        y = float(args[1])
        yaw = float(args[2])
        height = float(args[3]) if len(args) == 4 else self.config.height
        return (x, y, yaw, height)

    def write_grippers(self, open_gripper: bool) -> None:
        if self.config.no_grippers:
            self.log("grippers disabled by --no-grippers")
            return
        q = 5.4 if open_gripper else 0.0
        msg = make_gripper_cmd(q)
        self.left_gripper_pub.Write(msg)
        self.right_gripper_pub.Write(msg)
        self.gripper_open = open_gripper
        self.log(f"grippers={'open' if open_gripper else 'closed'}")

    def publish_motion(self, cmd: Command) -> None:
        self.move_pub.Write(String_(data=format_cmd(cmd)))

    def stop_motion(self) -> None:
        self.publish_motion((0.0, 0.0, 0.0, self.config.height))

    def request_stop(self, _signum: int, _frame: Optional[object]) -> None:
        self.stop_event.set()

    def print_keyboard_help(self) -> None:
        print(
            "Keyboard human agent:\n"
            "  W/S forward/back, A/D left/right, Z/X turn, Space stop\n"
            "  O/P open/close grippers, R reset, Q quit",
            flush=True,
        )

    def print_command_help(self) -> None:
        print(
            "Command human agent:\n"
            "  forward, back, left, right, turn_left, turn_right, stop\n"
            "  open, close, toggle, reset, raw <x> <y> <yaw> [height], hold <seconds>, quit",
            flush=True,
        )

    def log(self, msg: str) -> None:
        if self.config.verbose:
            print(f"[g1-human-agent] {msg}", flush=True)


def bind_unitree_symbols() -> None:
    global ChannelFactoryInitialize
    global ChannelPublisher
    global MotorCmds_
    global String_

    load_unitree_sdk()

    import g1_base_control as sdk

    ChannelFactoryInitialize = sdk.ChannelFactoryInitialize
    ChannelPublisher = sdk.ChannelPublisher
    MotorCmds_ = sdk.MotorCmds_
    String_ = sdk.String_


def forward(config: HumanConfig) -> Command:
    return (config.x_speed, 0.0, 0.0, config.height)


def back(config: HumanConfig) -> Command:
    return (-config.x_speed, 0.0, 0.0, config.height)


def left(config: HumanConfig) -> Command:
    return (0.0, config.y_speed, 0.0, config.height)


def right(config: HumanConfig) -> Command:
    return (0.0, -config.y_speed, 0.0, config.height)


def turn_left(config: HumanConfig) -> Command:
    return (0.0, 0.0, config.yaw_speed, config.height)


def turn_right(config: HumanConfig) -> Command:
    return (0.0, 0.0, -config.yaw_speed, config.height)


COMMAND_BUILDERS = {
    "forward": forward,
    "w": forward,
    "back": back,
    "s": back,
    "left": left,
    "a": left,
    "right": right,
    "d": right,
    "turn_left": turn_left,
    "tl": turn_left,
    "turn_right": turn_right,
    "tr": turn_right,
}


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> HumanConfig:
    parser = argparse.ArgumentParser(
        description="Human-as-agent controller for Unitree G1 Wholebody simulation."
    )
    parser.add_argument(
        "--mode",
        choices=("keyboard", "command"),
        default="keyboard",
        help="Human input mode.",
    )
    parser.add_argument("--tick-hz", type=positive_float, default=20.0, help="DDS publish rate.")
    parser.add_argument("--x-speed", type=positive_float, default=0.25, help="Forward/back speed.")
    parser.add_argument("--y-speed", type=positive_float, default=0.20, help="Left/right speed.")
    parser.add_argument("--yaw-speed", type=positive_float, default=0.50, help="Turn speed.")
    parser.add_argument("--height", type=float, default=0.8, help="Wholebody height command.")
    parser.add_argument("--no-grippers", action="store_true", help="Disable Dex1 gripper commands.")
    parser.add_argument("--quiet", action="store_true", help="Reduce logs.")
    args = parser.parse_args(argv)

    return HumanConfig(
        mode=args.mode,
        tick_hz=args.tick_hz,
        x_speed=args.x_speed,
        y_speed=args.y_speed,
        yaw_speed=args.yaw_speed,
        height=args.height,
        no_grippers=args.no_grippers,
        verbose=not args.quiet,
    )


def main(argv: list[str]) -> int:
    config = parse_args(argv)
    agent = HumanG1Agent(config)
    agent.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
