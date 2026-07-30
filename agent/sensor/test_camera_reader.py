#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from read_cameras import frame_path, parse_args, parse_camera_names


def test_parse_camera_names() -> None:
    assert parse_camera_names("head,left,right") == ["head", "left", "right"]
    assert parse_camera_names(" head, left ,, right ") == ["head", "left", "right"]


def test_frame_path() -> None:
    path = frame_path(Path("frames"), "head", 3, "png")
    assert path == Path("frames/head_000003.png")


def test_parse_show_args() -> None:
    args = parse_args(["--show", "--window-ms", "10"])
    assert args.show is True
    assert args.window_ms == 10


if __name__ == "__main__":
    test_parse_camera_names()
    test_frame_path()
    test_parse_show_args()
