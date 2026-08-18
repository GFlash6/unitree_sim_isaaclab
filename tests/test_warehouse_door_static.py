from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_warehouse_gate_door_is_removed() -> None:
    asset = (
        ROOT / "assets/objects/small_warehouse/small_warehouse_no_door.usda"
    ).read_text(encoding="utf-8")
    scene = (
        ROOT / "tasks/common_scene/base_scene_pickplace_cylindercfg_wholebody.py"
    ).read_text(encoding="utf-8")

    assert 'over "door" (' in asset
    assert "active = false" in asset
    assert "door_hinge" not in asset
    assert "small_warehouse_no_door.usda" in scene
