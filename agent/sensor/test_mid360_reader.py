#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from read_mid360 import cloud_path, parse_args, positive_int


def test_cloud_path() -> None:
    path = cloud_path(Path("clouds"), 3)
    assert path == Path("clouds/mid360_000003.npy")


def test_positive_int() -> None:
    assert positive_int("4") == 4


def test_parse_once_args() -> None:
    args = parse_args(["--once", "--max-points", "128"])
    assert args.once is True
    assert args.max_points == 128


if __name__ == "__main__":
    test_cloud_path()
    test_positive_int()
    test_parse_once_args()
