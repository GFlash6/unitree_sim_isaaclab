#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.shared_memory_utils import MultiImageReader


def parse_camera_names(value: str) -> list[str]:
    return [name.strip() for name in value.split(",") if name.strip()]


def frame_path(output_dir: Path, camera_name: str, frame_index: int, image_format: str) -> Path:
    return output_dir / f"{camera_name}_{frame_index:06d}.{image_format}"


def save_frame(image, path: Path) -> bool:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(path), image))


def show_images(images: dict[str, object], window_ms: int) -> bool:
    import cv2

    for name, image in images.items():
        cv2.imshow(name, image)
    key = cv2.waitKey(window_ms) & 0xFF
    return key not in (27, ord("q"))


def read_named_images(reader: MultiImageReader, names: list[str]) -> dict[str, object]:
    images = {}
    for name in names:
        image = reader.read_single_image(name)
        if image is not None:
            images[name] = image
    return images


def print_summary(images: dict[str, object], frame_index: int) -> None:
    parts = []
    for name, image in images.items():
        shape = getattr(image, "shape", None)
        parts.append(f"{name}:{tuple(shape) if shape is not None else 'unknown'}")
    print(f"frame={frame_index} " + " ".join(parts), flush=True)


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read camera images from Unitree sim shared memory.")
    parser.add_argument("--names", default="head,left,right", help="Comma-separated camera names.")
    parser.add_argument("--output-dir", type=Path, default=Path("camera_frames"), help="Frame output directory.")
    parser.add_argument("--format", choices=("png", "jpg"), default="png", help="Saved image format.")
    parser.add_argument("--interval", type=positive_float, default=0.2, help="Read interval in seconds.")
    parser.add_argument("--duration", type=positive_float, default=None, help="Run seconds; omit for unlimited.")
    parser.add_argument("--once", action="store_true", help="Save one set of camera frames and exit.")
    parser.add_argument("--timeout", type=positive_float, default=10.0, help="Wait seconds for --once.")
    parser.add_argument("--no-save", action="store_true", help="Only print camera shapes.")
    parser.add_argument("--show", action="store_true", help="Show live OpenCV windows; press q or Esc to exit.")
    parser.add_argument("--window-ms", type=int, default=1, help="OpenCV waitKey milliseconds for --show.")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    names = parse_camera_names(args.names)
    if not names:
        print("no camera names requested", file=sys.stderr)
        return 2

    reader = MultiImageReader()
    start = time.monotonic()
    frame_index = 0
    try:
        while True:
            images = read_named_images(reader, names)
            if images:
                print_summary(images, frame_index)
                if args.show and not show_images(images, args.window_ms):
                    return 0
                if not args.no_save and not args.show:
                    for name, image in images.items():
                        path = frame_path(args.output_dir, name, frame_index, args.format)
                        if not save_frame(image, path):
                            print(f"failed to save {path}", file=sys.stderr)
                frame_index += 1
                if args.once:
                    return 0

            elapsed = time.monotonic() - start
            if args.duration is not None and elapsed >= args.duration:
                return 0
            if args.once and elapsed >= args.timeout:
                print("camera frames not received before timeout", file=sys.stderr)
                return 1
            time.sleep(args.interval)
    finally:
        if "args" in locals() and args.show:
            import cv2

            cv2.destroyAllWindows()
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
