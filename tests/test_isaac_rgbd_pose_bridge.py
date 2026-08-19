import numpy as np

from holoagent_bridge.isaac_rgbd_pose_bridge import (
    camera_info_message,
    camera_pose_in_base,
    timestamp_msg,
)


def test_camera_pose_in_base_preserves_measured_relative_transform():
    camera_sim = np.array([2.0, 1.0, 1.5, 1.0, 0.0, 0.0, 0.0])
    robot_sim = np.array([1.0, 1.0, 0.5, 1.0, 0.0, 0.0, 0.0])

    position, quaternion = camera_pose_in_base(camera_sim, robot_sim)

    np.testing.assert_allclose(position, [1.0, 0.0, 1.0], atol=1e-7)
    np.testing.assert_allclose(quaternion, [1.0, 0.0, 0.0, 0.0], atol=1e-7)


def test_shared_timestamp_is_preserved_in_ros_time():
    stamp = timestamp_msg(12_345)
    assert stamp.sec == 12
    assert stamp.nanosec == 345_000_000


def test_camera_info_has_valid_identity_rectification_matrix():
    message = camera_info_message(640, 480, 243.2, 243.2, 319.5, 239.5, "camera", timestamp_msg(1))

    np.testing.assert_array_equal(
        message.r, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
