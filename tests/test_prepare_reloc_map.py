from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "holoagent_bridge" / "prepare_reloc_map.py"
spec = importlib.util.spec_from_file_location("prepare_reloc_map", SCRIPT)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)


def test_transform_point_applies_keyframe_rotation_and_translation() -> None:
    # 90 degrees around Z, followed by translation (10, 20, 30).
    pose = module.Pose(10.0, 20.0, 30.0, 0.0, 0.0, 2**-0.5, 2**-0.5)
    point = module.transform_point([1.0, 0.0, 2.0, 7.0], pose)

    assert point[:3] == [10.0, 21.0, 32.0]
    assert point[3] == 7.0


def test_indexed_files_requires_contiguous_numeric_names(tmp_path: Path) -> None:
    (tmp_path / "000000.pcd").write_text("", encoding="utf-8")
    (tmp_path / "000002.pcd").write_text("", encoding="utf-8")

    try:
        module.indexed_files(tmp_path, ".pcd")
    except ValueError as exc:
        assert "contiguous" in str(exc)
    else:
        raise AssertionError("non-contiguous keyframe files were accepted")
