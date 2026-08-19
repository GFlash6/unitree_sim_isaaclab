import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).parents[1]
    / "HoloAgent/agentic_robot/core/src/navigation/semantic_goal/semantic_goal/geometry.py"
)
SPEC = importlib.util.spec_from_file_location("semantic_goal_geometry", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_standoff_goal_stops_before_object_and_faces_it():
    goal, yaw = MODULE.standoff_goal(
        np.array([0.0, 0.0]), np.array([3.0, 4.0]), 1.0)

    np.testing.assert_allclose(goal, [2.4, 3.2], atol=1e-7)
    assert np.isclose(yaw, np.arctan2(4.0, 3.0))
