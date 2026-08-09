from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "holoagent_bridge" / "render_pcd_map_image.py"
spec = importlib.util.spec_from_file_location("render_pcd_map_image", SCRIPT)
render = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(render)


def test_keyframe_points_are_transformed_into_map_frame(tmp_path: Path) -> None:
    keyframes = tmp_path / "keyframe_cloud"
    keyframes.mkdir()
    (tmp_path / "mapping.txt").write_text(
        "0 10 20 30 0 0 0 1\n",
        encoding="utf-8",
    )
    (keyframes / "000000.pcd").write_text(
        "\n".join(
            [
                "FIELDS x y z intensity",
                "POINTS 1",
                "DATA ascii",
                "1 2 3 0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    assert render.read_keyframe_clouds(tmp_path) == [(11.0, 22.0, 33.0)]
