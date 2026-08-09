#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time


def bounded_float(value: str) -> float:
    return float(value)


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
    parser = argparse.ArgumentParser(description="Forward ROS2 /cmd_vel to the Unitree sim wholebody DDS run-command topic.")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="ROS2 geometry_msgs/Twist topic to subscribe.")
    parser.add_argument("--dds-topic", default="rt/run_command/cmd", help="Unitree DDS String_ topic consumed by sim_main wholebody tasks.")
    parser.add_argument("--dds-domain", type=int, default=1, help="Unitree DDS channel; sim_main uses 1.")
    parser.add_argument("--height", type=bounded_float, default=0.8, help="Fourth run-command value expected by the locomotion policy.")
    parser.add_argument("--max-x", type=positive_float, default=0.6, help="Absolute x velocity limit in m/s.")
    parser.add_argument("--max-y", type=positive_float, default=0.5, help="Absolute y velocity limit in m/s.")
    parser.add_argument("--max-yaw", type=positive_float, default=1.57, help="Absolute yaw velocity limit in rad/s.")
    parser.add_argument(
        "--turn-assist-speed",
        type=nonnegative_float,
        default=0.3,
        help="Measured policy translation used for high-yaw commands; zero disables it.",
    )
    parser.add_argument(
        "--turn-assist-yaw-threshold",
        type=positive_float,
        default=0.75,
        help="Absolute yaw rate at which the measured policy turn command replaces planar input.",
    )
    parser.add_argument("--publish-hz", type=positive_float, default=20.0, help="DDS publish frequency while ROS2 is alive.")
    parser.add_argument("--stale-timeout", type=positive_float, default=0.5, help="Seconds before stale /cmd_vel is replaced by zero velocity.")
    return parser.parse_args(argv)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def command_from_twist(msg, args: argparse.Namespace, now: float, last_stamp: float) -> list[float]:
    stale = now - last_stamp > args.stale_timeout
    if stale:
        return [0.0, 0.0, 0.0, float(args.height)]
    x = clamp(msg.linear.x, args.max_x)
    y = clamp(msg.linear.y, args.max_y)
    yaw = clamp(msg.angular.z, args.max_yaw)
    if abs(yaw) >= args.turn_assist_yaw_threshold and args.turn_assist_speed > 0.0:
        if yaw > 0.0:
            x = 0.0
            y = min(args.turn_assist_speed, args.max_y)
        else:
            x = -min(args.turn_assist_speed, args.max_x)
            y = 0.0
    return [x, y, yaw, float(args.height)]


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    except ImportError as exc:
        raise SystemExit("ROS2 and unitree_sdk2py must be importable before starting this bridge.") from exc

    ChannelFactoryInitialize(args.dds_domain)
    publisher = ChannelPublisher(args.dds_topic, String_)
    publisher.Init()

    rclpy.init()
    node = Node("cmd_vel_to_unitree_dds")
    latest = Twist()
    last_stamp = 0.0
    last_log = 0.0

    def on_twist(msg: Twist) -> None:
        nonlocal latest, last_stamp
        latest = msg
        last_stamp = time.monotonic()

    def publish() -> None:
        nonlocal last_log
        command = command_from_twist(latest, args, time.monotonic(), last_stamp)
        publisher.Write(String_(data=str(command)))
        now = time.monotonic()
        if now - last_log >= 1.0:
            node.get_logger().info(f"published DDS run command {command} to {args.dds_topic}")
            last_log = now

    node.create_subscription(Twist, args.cmd_vel_topic, on_twist, 10)
    timer = node.create_timer(1.0 / args.publish_hz, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
