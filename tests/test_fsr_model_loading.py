import importlib.util
import base64
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest


FSR_ROOT = Path(__file__).parents[1] / "HoloAgent/agentic_robot/fsr_vln"


def _load_module(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, FSR_ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


clip_utils = _load_module("fsr_clip_utils", "ovo/utils/clip_utils.py")
segment_utils = _load_module("fsr_segment_utils", "ovo/utils/segment_utils.py")


def test_resolve_siglip_snapshot_requires_complete_local_snapshot(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        clip_utils.resolve_siglip_snapshot(tmp_path)

    (tmp_path / "open_clip_model.safetensors").touch()
    (tmp_path / "tokenizer.json").touch()
    assert clip_utils.resolve_siglip_snapshot(tmp_path) == (
        tmp_path / "open_clip_model.safetensors",
        tmp_path,
    )


def test_resolve_sam3_checkpoint_accepts_file_or_snapshot_directory(tmp_path: Path):
    checkpoint = tmp_path / "sam3.pt"
    checkpoint.touch()
    assert segment_utils.resolve_sam3_checkpoint(checkpoint) == checkpoint
    assert segment_utils.resolve_sam3_checkpoint(tmp_path) == checkpoint

    checkpoint.unlink()
    with pytest.raises(FileNotFoundError):
        segment_utils.resolve_sam3_checkpoint(tmp_path)


def test_sam3_service_generator_filters_real_response_by_score_and_area():
    image = np.zeros((4, 5, 3), dtype=np.uint8)
    accepted = np.zeros((4, 5), dtype=np.uint8)
    accepted[1:3, 1:4] = 1
    too_small = np.zeros((4, 5), dtype=np.uint8)
    too_small[0, 0] = 1
    response = {
        "results": [
            {
                "mask_base64": base64.b64encode(accepted.tobytes()).decode(),
                "shape": [4, 5],
                "box": [1, 1, 4, 3],
                "score": 0.9,
                "label": "object",
            },
            {
                "mask_base64": base64.b64encode(too_small.tobytes()).decode(),
                "shape": [4, 5],
                "box": [0, 0, 1, 1],
                "score": 0.8,
                "label": "object",
            },
        ]
    }

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return json.dumps(response).encode()

    generator = segment_utils.Sam3ServiceMaskGenerator({
        "sam_prompts": ["object"],
        "confidence_threshold": 0.1,
        "min_mask_region_area": 2,
        "max_masks": 10,
    })
    with patch.object(segment_utils.request, "urlopen", return_value=FakeResponse()):
        masks = generator.generate(image)

    assert len(masks) == 1
    assert masks[0]["area"] == 6
    assert masks[0]["bbox"] == [1.0, 1.0, 3.0, 2.0]
    assert np.array_equal(masks[0]["segmentation"], accepted.astype(bool))


def test_sam3_service_generator_does_not_turn_failure_into_empty_success():
    generator = segment_utils.Sam3ServiceMaskGenerator({
        "sam_prompts": ["object"],
    })
    with patch.object(segment_utils.request, "urlopen", side_effect=OSError("offline")):
        with pytest.raises(RuntimeError, match="SAM3 service request failed"):
            generator.generate(np.zeros((2, 2, 3), dtype=np.uint8))


def test_mask_nms_handles_fewer_than_three_low_score_masks():
    import torch

    masks = torch.zeros((1, 4, 4), dtype=torch.bool)
    masks[0, 1:3, 1:3] = True
    selected = segment_utils.mask_nms(
        masks,
        torch.tensor([0.01]),
        score_thr=0.5)
    assert selected.tolist() == [0]
