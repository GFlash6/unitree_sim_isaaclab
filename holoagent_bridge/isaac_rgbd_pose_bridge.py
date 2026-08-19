#!/usr/bin/env python3
"""Publish Isaac RGB-D, calibration, and the measured base-to-camera TF."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from tools.shared_memory_utils import MultiImageReader


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def quat_inverse(quaternion: np.ndarray) -> np.ndarray:
    norm_squared = float(np.dot(quaternion, quaternion))
    if not math.isfinite(norm_squared) or norm_squared < 1e-12:
        raise ValueError("invalid zero quaternion")
    result = quaternion.copy()
    result[1:] *= -1.0
    return result / norm_squared


def quat_rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.concatenate(([0.0], vector))
    return quat_multiply(quat_multiply(quaternion, pure), quat_inverse(quaternion))[1:]


def camera_pose_in_base(
    camera_sim: np.ndarray, robot_sim: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return the measured camera pose relative to the simulated robot root."""
    camera_pos, camera_quat = camera_sim[:3], camera_sim[3:]
    robot_pos, robot_quat = robot_sim[:3], robot_sim[3:]
    relative_pos = quat_rotate(quat_inverse(robot_quat), camera_pos - robot_pos)
    relative_quat = quat_multiply(quat_inverse(robot_quat), camera_quat)
    relative_quat /= np.linalg.norm(relative_quat)
    return relative_pos, relative_quat


def timestamp_msg(timestamp_ms: int) -> Time:
    if timestamp_ms <= 0:
        raise ValueError("timestamp_ms must be positive")
    stamp = Time()
    stamp.sec, remainder_ms = divmod(int(timestamp_ms), 1000)
    stamp.nanosec = remainder_ms * 1_000_000
    return stamp


def camera_info_message(
    width: int, height: int, fx: float, fy: float, cx: float, cy: float, frame_id: str, stamp
) -> CameraInfo:
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.width = width
    message.height = height
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return message


class IsaacRgbdBridge(Node):
    def __init__(
        self,
        rate_hz: float,
        base_frame: str,
        camera_frame: str,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> None:
        super().__init__("isaac_rgbd_bridge")
        self.reader = MultiImageReader()
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.intrinsics = (fx, fy, cx, cy)
        self.last_frame_timestamp_ms = 0
        self.rgb_pub = self.create_publisher(Image, "sensors/front/rgb", 10)
        self.depth_pub = self.create_publisher(Image, "sensors/front/depth", 10)
        self.info_pub = self.create_publisher(
            CameraInfo, "sensors/front/camera_info", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / rate_hz, self._publish_frame)

    def _image_message(self, array: np.ndarray, encoding: str, stamp) -> Image:
        contiguous = np.ascontiguousarray(array)
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame
        message.height, message.width = contiguous.shape[:2]
        message.encoding = encoding
        message.is_bigendian = False
        message.step = contiguous.strides[0]
        message.data = contiguous.tobytes()
        return message

    def _publish_frame(self) -> None:
        rgb = self.reader.read_single_image("head")
        depth = self.reader.read_single_image("head_depth")
        poses = self.reader.read_single_image("head_pose")
        timestamps = self.reader.last_timestamps
        source_stamps = [
            timestamps.get("head", 0),
            timestamps.get("head_depth", 0),
            timestamps.get("head_pose", 0),
        ]
        frame_timestamp_ms = min(source_stamps)
        if frame_timestamp_ms <= self.last_frame_timestamp_ms:
            return
        if len(set(source_stamps)) != 1:
            self.get_logger().warning("rejected unsynchronized RGB-D-pose shared frame")
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

        camera_pos, camera_quat = camera_pose_in_base(
            poses[0].astype(np.float64), poses[1].astype(np.float64)
        )
        stamp = timestamp_msg(frame_timestamp_ms)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = float(camera_pos[0])
        transform.transform.translation.y = float(camera_pos[1])
        transform.transform.translation.z = float(camera_pos[2])
        transform.transform.rotation.w = float(camera_quat[0])
        transform.transform.rotation.x = float(camera_quat[1])
        transform.transform.rotation.y = float(camera_quat[2])
        transform.transform.rotation.z = float(camera_quat[3])
        self.tf_broadcaster.sendTransform(transform)

        self.rgb_pub.publish(self._image_message(rgb, "bgr8", stamp))
        self.depth_pub.publish(self._image_message(depth, "32FC1", stamp))
        self.info_pub.publish(
            camera_info_message(
                640, 480, *self.intrinsics, self.camera_frame, stamp
            )
        )
        self.last_frame_timestamp_ms = frame_timestamp_ms

    def destroy_node(self):
        self.reader.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-frame", default="front_camera_optical_frame")
    parser.add_argument("--fx", type=float, default=243.2)
    parser.add_argument("--fy", type=float, default=243.2)
    parser.add_argument("--cx", type=float, default=319.5)
    parser.add_argument("--cy", type=float, default=239.5)
    args = parser.parse_args()
    if args.rate <= 0 or min(args.fx, args.fy) <= 0:
        parser.error("--rate, --fx and --fy must be positive")
    rclpy.init()
    node = IsaacRgbdBridge(
        args.rate,
        args.base_frame,
        args.camera_frame,
        args.fx,
        args.fy,
        args.cx,
        args.cy,
    )
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
