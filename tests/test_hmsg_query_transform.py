import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).parents[1]
    / "HoloAgent/agentic_robot/fsr_vln/scripts/hmsg_query_server.py"
)
SPEC = importlib.util.spec_from_file_location("hmsg_query_server", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_sim_point_to_map_uses_live_robot_alignment():
    half = np.sqrt(0.5)
    point_sim = np.array([2.0, 1.0, 0.5])
    robot_sim = np.array([1.0, 1.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    robot_map = np.array([10.0, 20.0, 0.5, half, 0.0, 0.0, half])

    result = MODULE.sim_point_to_map(point_sim, robot_sim, robot_map)

    np.testing.assert_allclose(result, [10.0, 21.0, 0.5], atol=1e-7)
