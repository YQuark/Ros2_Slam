#!/usr/bin/env python3
"""Pad a nav2 occupancy map so planners have room near saved-map edges."""

import argparse
import math
import os
import re
import sys
from typing import Tuple


def read_token(handle) -> bytes:
    token = bytearray()
    while True:
        char = handle.read(1)
        if not char:
            raise ValueError("unexpected EOF while reading PGM header")
        if char == b"#":
            handle.readline()
            continue
        if char.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(char)


def read_pgm(path: str) -> Tuple[int, int, int, bytes]:
    with open(path, "rb") as handle:
        magic = read_token(handle)
        if magic != b"P5":
            raise ValueError(f"unsupported PGM magic {magic!r}; expected P5")
        width = int(read_token(handle))
        height = int(read_token(handle))
        maxval = int(read_token(handle))
        if maxval > 255:
            raise ValueError("only 8-bit PGM maps are supported")
        data = handle.read()

    expected = width * height
    if len(data) != expected:
        raise ValueError(f"PGM data size mismatch: got {len(data)}, expected {expected}")
    return width, height, maxval, data


def write_pgm(path: str, width: int, height: int, maxval: int, data: bytes) -> None:
    with open(path, "wb") as handle:
        handle.write(f"P5\n# Padded for Nav2 edge planning\n{width} {height}\n{maxval}\n".encode("ascii"))
        handle.write(data)


def parse_yaml_text(text: str) -> Tuple[str, float, Tuple[float, float, float]]:
    image_match = re.search(r"(?m)^image:\s*(.+?)\s*$", text)
    resolution_match = re.search(r"(?m)^resolution:\s*([-+0-9.eE]+)\s*$", text)
    origin_match = re.search(r"(?m)^origin:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]\s*$", text)

    if not image_match or not resolution_match or not origin_match:
        raise ValueError("map yaml must contain image, resolution, and origin")

    image = image_match.group(1).strip().strip('"').strip("'")
    resolution = float(resolution_match.group(1))
    origin = (
        float(origin_match.group(1)),
        float(origin_match.group(2)),
        float(origin_match.group(3)),
    )
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    return image, resolution, origin


def rewrite_origin(text: str, origin: Tuple[float, float, float]) -> str:
    replacement = f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]"
    return re.sub(
        r"(?m)^origin:\s*\[\s*[-+0-9.eE]+\s*,\s*[-+0-9.eE]+\s*,\s*[-+0-9.eE]+\s*\]\s*$",
        replacement,
        text,
        count=1,
    )


def resolve_image_path(yaml_path: str, image_value: str) -> str:
    if os.path.isabs(image_value):
        return image_value
    return os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", help="Path to a Nav2 map yaml file")
    parser.add_argument("--padding-m", type=float, default=1.0, help="Padding on each side in meters")
    parser.add_argument("--unknown-value", type=int, default=205, help="PGM value used for unknown padding")
    args = parser.parse_args()

    if args.padding_m <= 0.0:
        return 0
    if args.unknown_value < 0 or args.unknown_value > 255:
        raise ValueError("--unknown-value must be in 0..255")

    with open(args.map_yaml, "r", encoding="utf-8") as handle:
        yaml_text = handle.read()

    image_value, resolution, origin = parse_yaml_text(yaml_text)
    image_path = resolve_image_path(args.map_yaml, image_value)
    width, height, maxval, data = read_pgm(image_path)
    pad_cells = int(math.ceil(args.padding_m / resolution))
    if pad_cells <= 0:
        return 0

    new_width = width + 2 * pad_cells
    new_height = height + 2 * pad_cells
    fill = bytes([args.unknown_value])
    padded_rows = []
    empty_row = fill * new_width
    for _ in range(pad_cells):
        padded_rows.append(empty_row)
    for row_idx in range(height):
        row = data[row_idx * width : (row_idx + 1) * width]
        padded_rows.append(fill * pad_cells + row + fill * pad_cells)
    for _ in range(pad_cells):
        padded_rows.append(empty_row)

    new_origin = (
        origin[0] - pad_cells * resolution,
        origin[1] - pad_cells * resolution,
        origin[2],
    )

    write_pgm(image_path, new_width, new_height, maxval, b"".join(padded_rows))
    with open(args.map_yaml, "w", encoding="utf-8") as handle:
        handle.write(rewrite_origin(yaml_text, new_origin))

    print(
        f"padded map: {width}x{height} -> {new_width}x{new_height}, "
        f"origin ({origin[0]:.3f}, {origin[1]:.3f}) -> ({new_origin[0]:.3f}, {new_origin[1]:.3f}), "
        f"padding={pad_cells} cells"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"pad_nav_map.py: error: {exc}", file=sys.stderr)
        raise SystemExit(1)
