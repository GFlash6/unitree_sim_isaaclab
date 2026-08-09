from pathlib import Path

from holoagent_bridge.generate_nav2_map import bresenham, build_grid, parse_args


def test_default_obstacle_floor_matches_runtime_costmap(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr("sys.argv", ["generate_nav2_map.py", str(tmp_path)])

    args = parse_args()

    assert args.min_obstacle_z == -0.3


def test_bresenham_includes_real_ray_origin_and_endpoint() -> None:
    assert list(bresenham(1, 1, 4, 2)) == [(1, 1), (2, 1), (3, 2), (4, 2)]


def test_real_keyframe_footprint_is_cleared_after_obstacle_marking(tmp_path: Path) -> None:
    (tmp_path / "keyframe_cloud").mkdir()
    (tmp_path / "mapping.txt").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")
    (tmp_path / "keyframe_cloud" / "000000.pcd").write_text(
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
        "0 0 0 1\n1 0 0 1\n",
        encoding="utf-8",
    )

    cells, width, _height, origin_x, origin_y, occupied = build_grid(
        tmp_path, resolution=0.1, padding=0.5, min_z=-0.8, max_z=1.5, robot_radius=0.3
    )

    pose_x = int((0.0 - origin_x) / 0.1)
    pose_y = int((0.0 - origin_y) / 0.1)
    endpoint_x = int((1.0 - origin_x) / 0.1)
    assert cells[pose_y * width + pose_x] == 254
    assert cells[pose_y * width + endpoint_x] == 0
    assert occupied == 1
