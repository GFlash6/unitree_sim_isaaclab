import numpy as np

from holoagent_bridge.isaac_rgbd_pose_bridge import camera_pose_in_map


def test_camera_pose_in_map_preserves_measured_relative_transform():
    half = np.sqrt(0.5)
    camera_sim = np.array([2.0, 1.0, 1.5, 1.0, 0.0, 0.0, 0.0])
    robot_sim = np.array([1.0, 1.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    robot_map = np.array([10.0, 20.0, 0.5, half, 0.0, 0.0, half])

    position, quaternion = camera_pose_in_map(camera_sim, robot_sim, robot_map)

    np.testing.assert_allclose(position, [10.0, 21.0, 1.5], atol=1e-7)
    np.testing.assert_allclose(quaternion, [half, 0.0, 0.0, half], atol=1e-7)
