from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class MapMeta:
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    occupied_thresh: float
    free_thresh: float
    negate: int


@dataclass(frozen=True)
class GridMap:
    meta: MapMeta
    image_height: int
    image_width: int
    occupancy: np.ndarray


def load_map_metadata(yaml_path):
    yaml_path = Path(yaml_path)
    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    origin_x, origin_y, origin_yaw = data["origin"]
    return MapMeta(
        image_path=(yaml_path.parent / data["image"]).resolve(),
        resolution=float(data["resolution"]),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        origin_yaw=float(origin_yaw),
        occupied_thresh=float(data["occupied_thresh"]),
        free_thresh=float(data["free_thresh"]),
        negate=int(data.get("negate", 0)),
    )


def load_pgm_image(image_path):
    image_path = Path(image_path)
    with image_path.open("rb") as handle:
        magic = _next_non_comment_line(handle)
        if magic != b"P5":
            raise ValueError(f"Unsupported PGM format in {image_path}: expected P5, got {magic!r}")

        dims = _next_non_comment_line(handle).split()
        if len(dims) != 2:
            raise ValueError(f"Invalid PGM dimensions in {image_path}")
        width, height = int(dims[0]), int(dims[1])

        maxval = int(_next_non_comment_line(handle))
        if maxval > 255:
            raise ValueError(f"Unsupported PGM maxval {maxval} in {image_path}")

        image = np.frombuffer(handle.read(width * height), dtype=np.uint8)
        if image.size != width * height:
            raise ValueError(f"Unexpected PGM payload size in {image_path}")
        return image.reshape((height, width))


def load_occupancy_map(yaml_path):
    meta = load_map_metadata(yaml_path)
    image = load_pgm_image(meta.image_path)
    if meta.negate:
        image = 255 - image

    image_float = image.astype(np.float32) / 255.0
    occupancy = np.full(image.shape, -1, dtype=np.int8)
    occupied_cutoff = 1.0 - meta.occupied_thresh

    occupancy[image_float >= meta.free_thresh] = 0
    occupancy[image_float <= occupied_cutoff] = 1

    return GridMap(
        meta=meta,
        image_height=image.shape[0],
        image_width=image.shape[1],
        occupancy=occupancy,
    )


def world_to_fine_cell(x, y, grid_map):
    map_x = int((x - grid_map.meta.origin_x) / grid_map.meta.resolution)
    map_y = int((y - grid_map.meta.origin_y) / grid_map.meta.resolution)

    row = grid_map.image_height - 1 - map_y
    col = map_x

    if not (0 <= row < grid_map.image_height and 0 <= col < grid_map.image_width):
        raise ValueError(f"World coordinate ({x}, {y}) is outside the saved map bounds")

    return row, col


def fine_cell_to_world(row, col, grid_map):
    map_x = col + 0.5
    map_y = (grid_map.image_height - 1 - row) + 0.5

    x = grid_map.meta.origin_x + map_x * grid_map.meta.resolution
    y = grid_map.meta.origin_y + map_y * grid_map.meta.resolution
    return x, y


def _next_non_comment_line(handle):
    while True:
        line = handle.readline()
        if not line:
            raise ValueError("Unexpected end of file while reading PGM")

        stripped = line.strip()
        if stripped and not stripped.startswith(b"#"):
            return stripped
