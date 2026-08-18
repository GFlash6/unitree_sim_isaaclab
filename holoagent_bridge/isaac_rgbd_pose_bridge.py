#!/usr/bin/env python3
"""Publish synchronized IsaacLab RGB, metric depth, and map-frame camera pose."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from tools.shared_memory_utils import MultiImageReader


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ])


def quat_inverse(quaternion: np.ndarray) -> np.ndarray:
    norm_squared = float(np.dot(quaternion, quaternion))
    if not math.isfinite(norm_squared) or norm_squared < 1e-12:
        raise ValueError("invalid zero quaternion")
    result = quaternion.copy()
    result[1:] *= -1.0
    return result / norm_squared


def quat_rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.concatenate(([0.0], vector))
    return quat_multiply(
        quat_multiply(quaternion, pure), quat_inverse(quaternion)
    )[1:]


def camera_pose_in_map(
    camera_sim: np.ndarray, robot_sim: np.ndarray, robot_map: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Transfer the measured camera-to-robot transform into the localization map."""
    camera_pos, camera_quat = camera_sim[:3], camera_sim[3:]
    robot_pos, robot_quat = robot_sim[:3], robot_sim[3:]
    map_pos, map_quat = robot_map[:3], robot_map[3:]
    relative_pos = quat_rotate(quat_inverse(robot_quat), camera_pos - robot_pos)
    relative_quat = quat_multiply(quat_inverse(robot_quat), camera_quat)
    result_pos = map_pos + quat_rotate(map_quat, relative_pos)
    result_quat = quat_multiply(map_quat, relative_quat)
    result_quat /= np.linalg.norm(result_quat)
    return result_pos, result_quat


class IsaacRgbdPoseBridge(Node):
    def __init__(self, rate_hz: float, max_pose_age: float):
        super().__init__("isaac_rgbd_pose_bridge")
        self.reader = MultiImageReader()
        self.max_pose_age = max_pose_age
        self.latest_map_pose: np.ndarray | None = None
        self.latest_map_pose_received = 0.0
        self.last_frame_timestamp_ms = 0
        self.rgb_pub = self.create_publisher(Image, "/isaac/front/rgb", 10)
        self.depth_pub = self.create_publisher(Image, "/isaac/front/depth", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/isaac/front/pose", 10)
        self.create_subscription(Odometry, "/pose", self._pose_callback, 10)
        self.create_timer(1.0 / rate_hz, self._publish_frame)

    def _pose_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.latest_map_pose = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ], dtype=np.float64)
        self.latest_map_pose_received = time.monotonic()

    @staticmethod
    def _image_message(array: np.ndarray, encoding: str, stamp) -> Image:
        contiguous = np.ascontiguousarray(array)
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = "isaac_front_camera"
        message.height, message.width = contiguous.shape[:2]
        message.encoding = encoding
        message.is_bigendian = False
        message.step = contiguous.strides[0]
        message.data = contiguous.tobytes()
        return message

    def _publish_frame(self) -> None:
        if self.latest_map_pose is None:
            return
        if time.monotonic() - self.latest_map_pose_received > self.max_pose_age:
            return

        rgb = self.reader.read_single_image("head")
        depth = self.reader.read_single_image("head_depth")
        poses = self.reader.read_single_image("head_pose")
        timestamps = self.reader.last_timestamps
        frame_timestamp_ms = min(
            timestamps.get("head", 0),
            timestamps.get("head_depth", 0),
            timestamps.get("head_pose", 0),
        )
        if frame_timestamp_ms <= self.last_frame_timestamp_ms:
            return
        if rgb is None or depth is None or poses is None:
            return
        if rgb.shape != (480, 640, 3) or depth.shape != (480, 640) or poses.shape != (2, 7):
            self.get_logger().error(
                f"invalid RGB-D-pose shapes: rgb={rgb.shape} depth={depth.shape} poses={poses.shape}"
            )
            return
        if rgb.dtype != np.uint8 or depth.dtype != np.float32:
            self.get_logger().error(
                f"invalid RGB-D dtypes: rgb={rgb.dtype} depth={depth.dtype}"
            )
            return
        if not np.isfinite(depth).all() or not np.isfinite(poses).all():
            self.get_logger().error("non-finite Isaac RGB-D-pose frame rejected")
            return

        camera_pos, camera_quat = camera_pose_in_map(
            poses[0].astype(np.float64),
            poses[1].astype(np.float64),
            self.latest_map_pose,
        )
        stamp = self.get_clock().now().to_msg()
        self.rgb_pub.publish(self._image_message(rgb, "bgr8", stamp))
        self.depth_pub.publish(self._image_message(depth, "32FC1", stamp))
        pose_message = PoseStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = "map"
        pose_message.pose.position.x = float(camera_pos[0])
        pose_message.pose.position.y = float(camera_pos[1])
        pose_message.pose.position.z = float(camera_pos[2])
        pose_message.pose.orientation.w = float(camera_quat[0])
        pose_message.pose.orientation.x = float(camera_quat[1])
        pose_message.pose.orientation.y = float(camera_quat[2])
        pose_message.pose.orientation.z = float(camera_quat[3])
        self.pose_pub.publish(pose_message)
        self.last_frame_timestamp_ms = frame_timestamp_ms

    def destroy_node(self):
        self.reader.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--max-pose-age", type=float, default=1.0)
    args = parser.parse_args()
    if args.rate <= 0 or args.max_pose_age <= 0:
        parser.error("--rate and --max-pose-age must be positive")
    rclpy.init()
    node = IsaacRgbdPoseBridge(args.rate, args.max_pose_age)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
